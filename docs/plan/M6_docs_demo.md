# M6: Documentation and demo

**Goal:** someone with ROS 2 Jazzy installed can clone the repository, follow the README, and watch the robot dock, and the documents describe the system as built.

**Depends on:** M5.

## Sub-tasks

- [x] `README.md`: what the project is, prerequisites (apt packages), build, the one-command demo, how to send a goal to a different marker, how to run the tests, a screenshot or short clip.
- [x] Bring [../architecture.md](../architecture.md) in line with the code: no "planned" wording, the trajectory plot from M4 is in. An RViz screenshot taken during a dock (`docs/images/rviz_docking.png`) is left for later.
- [x] Bring [../interfaces.md](../interfaces.md) in line: final parameter defaults, any topics added or dropped.
- [x] Bring [../world_and_robot.md](../world_and_robot.md) in line: final obstacle layout, a top-down image of the world.
- [x] Add the results table to [../test_plan.md](../test_plan.md).
- [x] Update the README: status, real commands, where things live.
- [x] Audit function comments: every function, in C++ headers and in the Python launch and test files, says what it does and how it contributes to docking.
- [x] Bring [../setup.md](../setup.md) in line: confirmed package list, measured GPU numbers.
- [x] Demo script or documented command sequence: launch, dock to ID 4, then dock to ID 0.

## Gate

- MAN1: RViz shows the model, camera and debug images and the marker frames; the docking state is read from `/docking/state` (checked, see the test plan).
- MAN2: the README quick start works on a fresh clone in a clean shell.
- No document under `docs/` contradicts the code. Check by grepping each parameter default in `interfaces.md` against the YAML files.

## Quick start the README should end up with

```bash
# prerequisites: see docs/setup.md
cd ArUco_ROS_Nav
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch dockbot_bringup dock.launch.py world:=docking_world rviz:=true
# in a second terminal
ros2 action send_goal /dock dockbot_interfaces/action/Dock "{marker_id: 2}" --feedback
```
