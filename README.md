# ArUco_ROS_Nav

A ROS 2 Jazzy and Gazebo Harmonic simulation in which a differential-drive robot (`dockbot`) docks autonomously to a station identified by an ArUco marker, using only a camera-based pose estimate. There is no map, no external localisation, and no odometry in the control loop.

**Pipeline:** camera image → ArUco detector → marker pose relative to the robot → docking controller → `cmd_vel`. The robot stops with its front face 10 cm from the marker plane.

**Status:** under development, milestone by milestone. The workspace, interfaces, first Docker image, and the robot model with its Gazebo launch are in. The world and markers, detector, controller, and end-to-end tests are next. See [docs/plan/README.md](docs/plan/README.md).

## What it looks like

<p align="center">
  <img src="docs/images/gazebo_world.png" alt="Gazebo view of docking_world: a walled room with five ArUco markers, boxes, pillars and the robot" width="85%">
</p>

*`docking_world` in Gazebo: a 7 m x 7 m room with five docking markers (IDs 0 to 4), obstacles, and `dockbot` at its spawn point.*

| | |
|---|---|
| <img src="docs/images/gazebo_robot.png" alt="dockbot facing markers 0 and 1 on the east wall"> | <img src="docs/images/rviz.png" alt="RViz showing the robot model, TF frames, odometry and the live camera image"> |
| *`dockbot` facing markers 0 and 1, which sit side by side on purpose so the robot must pick the requested ID.* | *RViz: robot model, TF frames, odometry, and the live camera image (top left).* |

<p align="center">
  <img src="docs/images/pipeline.png" alt="Data flow: camera, aruco_detector, docking_controller, cmd_vel into the Gazebo diff drive" width="90%">
</p>

The robot is a small differential-drive base with a camera on a mast, 0.20 m behind its front face, so the whole marker stays in view at the 10 cm docking distance:

<p align="center">
  <img src="docs/images/robot_side_view.png" alt="Side view of dockbot at the docking position with dimensions" width="75%">
</p>

The world plan shows where each marker sits, which way its printed face points, and the corridor kept clear in front of it:

<p align="center">
  <img src="docs/images/world_layout.png" alt="Plan view of docking_world with marker IDs, facing directions, docking corridors and obstacles" width="65%">
</p>

What the robot's camera sees from 1.5 m in front of marker 4 (the detector will read this frame):

<p align="center">
  <img src="docs/images/camera_view.png" alt="Camera frame showing marker 4 on its stand" width="55%">
</p>

## Packages

All nodes are C++17 with `rclcpp`; every package is `ament_cmake`. Python is used only for launch files, tests, and the marker image generator.

| Package | Contents |
|---|---|
| `dockbot_interfaces` | `ArucoMarker`, `ArucoMarkerArray`, `DockingState` messages and the `Dock` action |
| `dockbot_description` | Xacro model of `dockbot` with a mast-mounted camera, RViz config |
| `dockbot_gazebo` | Worlds, ArUco marker models (`DICT_4X4_50`, IDs 0 to 4), topic bridge, `sim.launch.py` |
| `dockbot_perception` | `aruco_detector` node: camera image → marker poses in `base_link` |
| `dockbot_docking` | `docking_controller` node: marker poses and a `/dock` goal → `/cmd_vel` |
| `dockbot_bringup` | `dock.launch.py`, test scenarios, end-to-end tests |

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# fast tests (no simulator)
colcon test --packages-select dockbot_perception dockbot_docking dockbot_gazebo dockbot_description
colcon test-result --verbose
```

Required packages are listed in [docs/setup.md](docs/setup.md). Use the system OpenCV (4.6 on Ubuntu 24.04); do not install another copy.

## Simulation

```bash
# robot in an empty world; add headless:=true for no GUI, gpu:=nvidia|software to pick the renderer
ros2 launch dockbot_gazebo sim.launch.py
```

The full demo (`dock.launch.py` and the `/dock` action) arrives with the integration milestone:

```bash
ros2 launch dockbot_bringup dock.launch.py world:=docking_world rviz:=true
ros2 action send_goal /dock dockbot_interfaces/action/Dock "{marker_id: 2}" --feedback
```

## Docker

```bash
docker compose -f docker/compose.yaml run --rm test     # fast tests inside the container
docker compose -f docker/compose.yaml run --rm test-e2e # Gazebo end-to-end tests, headless (needs M5)
docker compose -f docker/compose.yaml run --rm dev      # development shell with the source mounted
./docker/run.sh                                         # GUI demo; picks the NVIDIA service when available (needs M5)
```

`WORLD` and `MARKER_ID` environment variables select the world and the marker to dock at. See [docs/setup.md](docs/setup.md#docker) for graphics options and the NVIDIA Container Toolkit.

## Key design points

- The 10 cm docking distance is measured from the robot's front face to the marker plane, not from `base_link` or the camera.
- The marker frame's z axis points out of the printed face; that is the docking normal.
- Marker poses come only from the camera. Ground truth is bridged for test scoring and never used for control.
- All nodes run with `use_sim_time:=true`.

## Documentation

Start with [docs/README.md](docs/README.md): architecture, interfaces, world and robot model, setup, test plan, and the milestone plan.

## Licence

Apache-2.0, see [LICENSE](LICENSE).
