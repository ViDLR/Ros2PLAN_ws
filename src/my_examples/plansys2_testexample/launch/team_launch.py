from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_robot_nodes(context, *args, **kwargs):
    # Retrieve runtime-resolved LaunchConfiguration values
    robots_ids = context.launch_configurations.get('robots_ids', 'robot0')
    team_name = context.launch_configurations.get('team_name', 'team1')
    
    # Convert robots_ids into a list
    robots_ids_list = robots_ids.split(',')

    robot_nodes = []
    for robot_id in robots_ids_list:
        robot_nodes.extend([
            Node(
                package='plansys2_testexample',
                executable=f'{action}_action_node',
                name=f"{action}_action_node_{robot_id}",
                namespace=team_name,
                output='screen',
                parameters=[
                    {'specialized_arguments': [robot_id]},
                    {'team_name': team_name}  # Adding the team_name parameter
                ]
            )
            for action in [
                'landing', 'takeoff', 'change_site', 'navigation_air',
                'navigation_water', 'switch_airwater', 'switch_waterair',
                'translate_data', 'observe', 'observe_2r', 'sample'
            ]
        ])
        robot_nodes.append(
            Node(
                package='action_simulator',
                executable='action_simulator_node',
                name=f"simulator_{robot_id}",
                namespace=team_name,
                output='screen',
                parameters=[
                    {'robot_id': robot_id},
                    {'team_name': team_name}
                ]
            )
        )
    return robot_nodes


def generate_launch_description():
    # Declare Launch Arguments

    declare_robots_ids_cmd = DeclareLaunchArgument(
        'robots_ids',
        default_value='robot0',
        description='Comma-separated list of robot IDs'
    )

    declare_team_name_cmd = DeclareLaunchArgument(
        'team_name',
        default_value='team1',
        description='Team name to which the robot belongs'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/home/virgile/PHD/Ros2PLAN_ws/src/ros2_planning_system/plansys2_bringup/params/plansys2_params.yaml',
        description='Path to params file'
    )

    declare_action_bt_file_cmd = DeclareLaunchArgument(
        'action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'),
            'behavior_trees', 'plansys2_action_bt.xml'
        ),
        description='BT representing a PDDL action'
    )

    declare_start_action_bt_file_cmd = DeclareLaunchArgument(
        'start_action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'),
            'behavior_trees', 'plansys2_start_action_bt.xml'
        ),
        description='BT representing a PDDL start action'
    )

    declare_end_action_bt_file_cmd = DeclareLaunchArgument(
        'end_action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'),
            'behavior_trees', 'plansys2_end_action_bt.xml'
        ),
        description='BT representing a PDDL end action'
    )

    declare_bt_builder_plugin_cmd = DeclareLaunchArgument(
        'bt_builder_plugin',
        default_value='SimpleBTBuilder',
        description='Behavior tree builder plugin'
    )

    team_name = LaunchConfiguration("team_name")

    # Executor node
    executor_node = Node(
        package='plansys2_executor',
        executable='executor_node',
        name='executor',
        namespace=team_name,
        output='screen',
        parameters=[
            {'default_action_bt_xml_filename': LaunchConfiguration('action_bt_file')},
            {'default_start_action_bt_xml_filename': LaunchConfiguration('start_action_bt_file')},
            {'default_end_action_bt_xml_filename': LaunchConfiguration('end_action_bt_file')},
            {'bt_builder_plugin': LaunchConfiguration('bt_builder_plugin')},
            {'team_name': team_name},
            LaunchConfiguration('params_file')
        ]
    )

    # Lifecycle manager node for the team executor
    lifecycle_manager_cmd = Node(
        package='plansys2_lifecycle_manager',
        executable='lifecycle_manager_node',
        name=['lifecycle_manager_executor_', team_name],  # Unique name per team
        output='screen',
        parameters=[{
            'managed_nodes': [[team_name , '_executor']]
        }]
    )


    # Group Actions
    group_action = OpaqueFunction(function=generate_robot_nodes)

    # Launch description
    ld = LaunchDescription()

    # Add all declared arguments
    ld.add_action(declare_robots_ids_cmd)
    ld.add_action(declare_team_name_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_action_bt_file_cmd)
    ld.add_action(declare_start_action_bt_file_cmd)
    ld.add_action(declare_end_action_bt_file_cmd)
    ld.add_action(declare_bt_builder_plugin_cmd)

    # Add executor node and robot nodes
    ld.add_action(executor_node)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(group_action)

    return ld
