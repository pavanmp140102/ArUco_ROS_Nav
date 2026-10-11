# Documentation

Design and planning documents for the ArUco autonomous docking simulation (ROS 2 Jazzy + Gazebo Harmonic).

**Status:** design complete, implementation under way (see the milestone checklists in [plan/](plan/)). Documents describe the intended system and are updated as the matching code lands.

| Document | What it covers |
|---|---|
| [architecture.md](architecture.md) | Packages, data flow, coordinate frames, detector and controller design, and the reasons behind each design decision |
| [interfaces.md](interfaces.md) | Topics, the `/dock` action, message definitions, and node parameters with defaults |
| [world_and_robot.md](world_and_robot.md) | Robot dimensions, camera geometry, marker specification, and world layout |
| [setup.md](setup.md) | Required packages, optional tool111s, the environment the project was built on, GPU selection, and Docker prerequisites |
| [test_plan.md](test_plan.md) | Every test case with an ID, method, and pass criterion |
| [plan/README.md](plan/README.md) | Master plan: milestones, dependency order, requirement traceability, risks |
| [plan/](plan/) | One plan file per milestone (M0 to M8) with sub-task checklists. M8 is the Docker image for running on other systems |

## Reading order

1. [architecture.md](architecture.md) for the overall shape.
2. [plan/README.md](plan/README.md) to see what gets built in what order.
3. The milestone file you are about to work on, plus the test cases it names in [test_plan.md](test_plan.md).

## Conventions

Every function carries a description of what it does and how it serves the docking goal.

## Figures

The images in [images/](images/) are generated, not hand-drawn:

- `world_layout.png`, `robot_side_view.png`, `pipeline.png`: `python3 docs/images/generate_diagrams.py` (the world plan is read from `docking_world.sdf` and `markers.yaml`, so it follows the simulation).
- `gazebo_world.png`, `gazebo_robot.png`: rendered by extra static cameras in the headless simulator (same renderer as the GUI, without the window).
- `rviz.png`: a screenshot of RViz with `rviz/dockbot.rviz`.
- `camera_view.png`: one frame from `/camera/image_raw` with the robot 1.5 m in front of marker 4.
