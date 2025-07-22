import json
from pathlib import Path
import sys

from Class_and_functions import recover_mission_from_json, generateTMProblemPDDL

FAILURE_INFO_PATH = Path("/tmp/failure_info.json")
MISSION_PATH = Path("/tmp/world_info.json")
OUTPUT_PROBLEM_PATH = Path("/tmp/validation/failed_problem")  # .pddl will be appended
FAILURE_INDEX_MAPPING_PATH = Path("/tmp/failure_index.json")


def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)


def apply_failure_to_mission(mission, failure, failure_map):
    robot = failure["robot"]
    action = failure["action"]
    point = failure["point"]
    site = failure["site"]
    index = str(failure["failure_index"])

    failure_type = failure_map.get(action, {}).get(index, None)
    if failure_type is None:
        print(f"❌ Unknown failure type for [{action}] with index [{index}]")
        return mission

    print(f"✅ Applying failure: [{action}] index {index} → {failure_type}")

    # Get the robot and site objects
    target_robot = next((r for r in mission.robots if r.name == robot), None)
    target_site = next((s for s in mission.sites if s.name == site), None)

    # print(target_robot, target_site)

    if not target_robot:
        print(f"❌ Robot {robot} not found.")
        return mission
    if not target_site:
        print(f"❌ Site {site} not found.")
        return mission

    # Apply modifications
    if failure_type == "robot_unavailable":
        target_robot.available = False

    elif failure_type == "remove_goal_point":
        for site in mission.sites:
            for poi in site.poi:
                if poi.name == point:
                    poi.isgoal = False  # ❌ No longer considered a goal

    elif failure_type == "point_unswitchable":
        for site in mission.sites:
            for poi in site.poi:
                if poi.name == point and poi.typepoi == "transition":
                    poi.isswitchable = False

    elif failure_type == "robot_cannot_switch robot_waterconf":
        target_robot.medium = -1
        target_robot.canswitch = False

    elif failure_type == "robot_cannot_switch robot_airconf":
        target_robot.medium = 1
        target_robot.canswitch = False

    elif failure_type == "create_new_poi same_site":
        new_name = "auto_poi_" + str(len(target_site.poi))
        target_robot.site = target_site.name
        target_robot.poi = new_name
        target_site.poi.append(
            Poi(name=new_name, loc=target_robot.loc, typepoi="survey", mediums=[1])
        )

    elif failure_type == "create_new_poi new_site":
        new_name_site = "autosite_" + str(len(mission.sites))
        new_name_poi = "auto_cppoi"
        target_robot.site = new_name_site
        target_robot.poi = new_name_poi
        mission.sites.append(
            Site(poi=[Poi(name=new_name_poi, loc=target_robot.loc, typepoi="survey", mediums=[1])], robots=[target_robot], name=new_name_site, center=target_robot.loc, size=5)
        )

    elif failure_type == "robot_cannot_translate":
        target_robot.canrelay = False

    elif failure_type == "point_cannot_translate":
        for site in mission.sites:
            for poi in site.poi:
                if poi.name == point:
                    poi.isrelay = False

    elif failure_type == "robot_cannot_sample":
        target_robot.cansample = False

    elif failure_type == "create_new_poi sp_type":
        new_name = "auto_poi_" + str(len(target_site.poi))
        target_robot.poi = new_name
        target_site.poi.append(
            Poi(name=new_name, loc=target_robot.loc, typepoi="transition", mediums=[1,-1])
        )

    elif failure_type == "create_new_poi sp_type waterconf_only":
        new_name = "auto_poi_" + str(len(target_site.poi))
        target_robot.poi = new_name
        target_site.poi.append(
            Poi(name=new_name, loc=target_robot.loc, typepoi="survey", mediums=[-1])
        )

    elif failure_type == "create_new_poi obs_type":
        new_name = "auto_poi_" + str(len(target_site.poi))
        target_robot.poi = new_name
        target_site.poi.append(
            Poi(name=new_name, loc=target_robot.loc, typepoi="survey", mediums=[1])
        )
    return mission


def main():
    if not all(p.exists() for p in [FAILURE_INFO_PATH, MISSION_PATH, FAILURE_INDEX_MAPPING_PATH]):
        print("❌ Required input files missing.")
        sys.exit(1)

    failure = load_json(FAILURE_INFO_PATH)
    failure_map = load_json(FAILURE_INDEX_MAPPING_PATH)

    mission = recover_mission_from_json(MISSION_PATH)
    updated_mission = apply_failure_to_mission(mission, failure, failure_map)

    # 🔻 Remove unavailable robots from mission and sites
    unavailable_robot_names = [r.name for r in updated_mission.robots if not getattr(r, "available", True)]

    updated_mission.robots = [r for r in updated_mission.robots if getattr(r, "available", True)]
    for site in updated_mission.sites:
        site.robots = [r for r in site.robots if r.name not in unavailable_robot_names]

    # Save to new PDDL file
    OUTPUT_PROBLEM_PATH.parent.mkdir(parents=True, exist_ok=True)
    generateTMProblemPDDL(updated_mission, OUTPUT_PROBLEM_PATH)

    print(f"✅ Failure-applied problem saved at {OUTPUT_PROBLEM_PATH}.pddl")


if __name__ == "__main__":
    main()
