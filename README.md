# AMA: Adaptive Modular Architecture for Multi-Robot Planning and Execution

This repository contains the full source code of **AMA**, a modular and distributed architecture for robust mission execution in multi-robot, trans-media environments. AMA is built on top of [PlanSys2](https://github.com/IntelligentRoboticsLabs/ros2_planning_system) and extends it with symbolic-temporal supervision, distributed Behavior Tree execution, and structured fault-tolerant recovery.
---

## Architecture Overview

The AMA system is divided into two main components:

### AMA-PLAN
- Symbolic mission decomposition into linked subproblems and automated planning using PDDL2.1
- Robot clustering, coalition formation, and site path generation
- Generation of **execution-ready** plans with temporal dependencies encoded as `STN_teams`, a graphical dependency graph based on STN relations and sequential dependencies between subproblems

### AMA-EXEC
- Distributed execution using PlanSys2-based BTs
- **STN-CONTROLLER** for runtime synchronization and delay injection
- **DEM (Distributed Execution Manager)** for structured failure recovery and partial replanning

AMA enables multi-robot teams to coordinate asynchronously, recover from failures locally, and maintain temporal coherence during long-term missions.

---

## 📁 Repository Structure

```
Ros2PLAN_ws/
├── src/
│   ├── arms_plan_solver/             # AMA plugin for PlanSys2 planner, contains Python scripts and OPTIC binary (AMA-PLAN core)
│   ├── ros2_planning_system/         # Forked PlanSys2 core (as a Git submodule)
│   ├── action_simulator/             # AMA-EXEC core: Distributed Execution Manager (DEM), STNController, robot simulators
│   ├── my_examples/                  # Test launch files, TLCM node, and PlanSys2 action definitions
│   ├── optic_plan_solver/            # OPTIC planner backend (legacy/experimental)
│   └── user_visualization_interface/ # WIP visualization frontend (not yet integrated)
└── README.md
```

---

## 🚀 Getting Started

### 1. Prerequisites

- ROS 2 Humble
- Python ≥ 3.10
- Tested on Ubuntu 22.04

### 2. Clone & Build

```
git clone https://github.com/ViDLR/Ros2PLAN_ws.git
cd Ros2PLAN_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
Create a problem based on our domain in my_examples/plansys2_testexamples/pddl/) and constraints in the same folder.
place it there, to be uploaded by the DEM

# Update the DEM with your problem.pddl

# Plan and execute 
This launch file:
  Starts the PlanSys2 core (domain & problem experts, planner)
  Launches the Distributed Execution Manager (DEM)
  Parses the symbolic plan, decomposes it using AMA-PLAN
  Triggers distributed execution with STN-based monitoring


### Disclaimers 

This repository contains an **ongoing development version** of the AMA system.  
While the core architecture (AMA-PLAN and AMA-EXEC) is functional, some modules are still under active development.

Feel free to open issues or request guidance for integration.

---

## 👤 Author

**Virgile de La Rochefoucauld**  
PhD Researcher — LAAS-CNRS & Osaka University  
📧 virgile.dlr@protonmail.com  
