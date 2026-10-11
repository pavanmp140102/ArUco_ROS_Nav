# M1: Robot

**Goal:** `dockbot` spawns in an empty Gazebo world, drives from `cmd_vel`, and publishes camera images with a correct TF tree.

**Depends on:** M0.

Geometry and camera values are in [../world_and_robot.md](../world_and_robot.md).

## Sub-tasks

- [x] `urdf/dockbot.urdf.xacro`: chassis, two drive wheels, two casters, camera mast, `camera_link`, `camera_optical_frame`, `base_footprint`. Inertias from Xacro macros for box, cylinder, and sphere.
- [x] Gazebo extensions in the Xacro: `gz-sim-diff-drive-system` (wheel separation 0.26, radius 0.04, topic `cmd_vel`, odometry frame `odom`), `gz-sim-joint-state-publisher-system`, and the camera sensor (1280x720, 1.0856 rad, 30 Hz, `gz_frame_id` = `camera_optical_frame`). Width, height, field of view, and rate are Xacro arguments with those defaults; `camera_rate` is also a launch argument.
- [x] Caster friction set to zero; wheel friction high enough to avoid slip at `a_max`.
- [x] `config/bridge.yaml`: `clock`, `joint_states`, `odom`, `tf`, `cmd_vel` (`TwistStamped`, ROS to Gazebo), `camera/camera_info`. Modelled on the TurtleBot3 file at `/opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_waffle_pi_bridge.yaml`.
- [x] `worlds/empty.sdf` with ground plane, light, and the Physics, SceneBroadcaster, UserCommands, and Sensors (`ogre2`) systems.
- [x] `launch/sim.launch.py`: Gazebo server, optional GUI, `robot_state_publisher` from the processed Xacro, `ros_gz_sim create` reading `robot_description`, `parameter_bridge`, `image_bridge`. Arguments `world`, `x`, `y`, `yaw`, `headless`, `gpu`. Appends the package's `models/` directory to `GZ_SIM_RESOURCE_PATH`.
- [x] `gpu` argument (`intel`, `nvidia` or `software`): `nvidia` sets `__NV_PRIME_RENDER_OFFLOAD=1` and `__GLX_VENDOR_LIBRARY_NAME=nvidia` for the Gazebo processes. See [../setup.md](../setup.md).
- [x] `rviz/dockbot.rviz`: robot model, TF, camera image, odometry.
- [x] Static test running `xacro` and `check_urdf`.
- [x] Headless check: launch with `headless:=true`, confirm `/camera/image_raw` publishes non-black frames. Record the result in the risk table of [README.md](README.md).
- [ ] GPU comparison: with `gpu:=intel` and `gpu:=nvidia`, with and without `headless`, record the real-time factor and the camera rate, and confirm with `nvtop` which GPU did the rendering. Make the better one the default and note the numbers in [../setup.md](../setup.md).

## Files created

```
src/dockbot_description/urdf/dockbot.urdf.xacro
src/dockbot_description/rviz/dockbot.rviz
src/dockbot_description/test/test_urdf.py
src/dockbot_gazebo/config/bridge.yaml
src/dockbot_gazebo/worlds/empty.sdf
src/dockbot_gazebo/launch/sim.launch.py
```

## Gate

- S3 for the robot model.
- MAN3: teleoperation (`teleop_twist_keyboard` with stamped output) drives the robot straight and turns it in place.
- `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` shows translation (-0.05, 0, 0.16) and the optical rotation.
- `ros2 topic hz /camera/image_raw` reports about 30 Hz, and `/camera/camera_info` reports 1280x720 with `fx` near 1061.
- Headless check passes, or the fallback is documented.

## Notes

- `base_link` is at axle height (0.04 m above ground), so a camera 0.20 m above ground is 0.16 m above `base_link`.
- Commanding 0.2 m/s for 5 s should move the robot 1.0 m by ground truth. A large mismatch means wheel radius or separation in the plugin does not match the model.
