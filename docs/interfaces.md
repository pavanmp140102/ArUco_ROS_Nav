# Node interfaces

Planned topics, actions, messages, and parameters. Defaults here are starting values; tuning during M4 and M5 may change them, and this file must be updated when they do.

## Topics

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `ros_gz_image` bridge | `aruco_detector` | 1280x720, 30 Hz, sensor QoS |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `ros_gz_bridge` | `aruco_detector` | Read once for intrinsics |
| `/aruco/markers` | `dockbot_interfaces/ArucoMarkerArray` | `aruco_detector` | `docking_controller` | One message per image, empty when nothing is seen. Poses in `base_link` |
| `/aruco/debug_image` | `sensor_msgs/Image` | `aruco_detector` | RViz | Marker outlines, IDs, axes. Optional |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | `docking_controller` | `ros_gz_bridge` to DiffDrive | 20 Hz while a goal is active |
| `/docking/state` | `dockbot_interfaces/DockingState` | `docking_controller` | RViz, tests | 20 Hz |
| `/odom` | `nav_msgs/Odometry` | `ros_gz_bridge` | RViz only | Not used for control |
| `/joint_states` | `sensor_msgs/JointState` | `ros_gz_bridge` | `robot_state_publisher` | |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher`, bridge, `aruco_detector` | all | Detector adds `base_link -> aruco_<id>` |
| `/clock` | `rosgraph_msgs/Clock` | `ros_gz_bridge` | all | Simulation time |
| `/ground_truth/odom` | `nav_msgs/Odometry` | `ros_gz_bridge` | **tests only** | True robot pose in the world frame (Gazebo `OdometryPublisher`, `base_footprint`) |

## Action

### `/dock` (`dockbot_interfaces/action/Dock`)

Server: `docking_controller`. One goal at a time; a new goal while one is active is rejected.

```
# Goal
int32 marker_id            # ArUco ID of the docking station to dock at
float32 dock_distance      # metres, front face to marker plane. 0.0 means use the node parameter
float32 timeout            # seconds. 0.0 means use the node parameter
---
# Result
uint8 OUTCOME_DOCKED=0
uint8 OUTCOME_TARGET_NOT_FOUND=1
uint8 OUTCOME_TARGET_LOST=2
uint8 OUTCOME_RETRIES_EXCEEDED=3
uint8 OUTCOME_TIMEOUT=4
uint8 OUTCOME_CANCELED=5
bool success
uint8 outcome
string message
float32 final_distance         # front face to marker plane, as measured by the camera
float32 final_lateral_error
float32 final_heading_error
---
# Feedback
DockingState state
```

Example:

```bash
ros2 action send_goal /dock dockbot_interfaces/action/Dock "{marker_id: 2}" --feedback
```

## Messages

### `ArucoMarker.msg`

```
int32 id
geometry_msgs/Pose pose        # marker frame expressed in the array's header frame
float32 range                  # ground-plane distance to marker centre, metres
float32 bearing                # angle to marker centre, radians, positive left
float32 reprojection_error     # pixels, RMS over the four corners
```

### `ArucoMarkerArray.msg`

```
std_msgs/Header header         # stamp of the source image; frame_id = base_frame
ArucoMarker[] markers
```

### `DockingState.msg`

```
uint8 IDLE=0
uint8 SEARCH=1
uint8 APPROACH=2
uint8 FINAL=3
uint8 RECOVER=4
uint8 DOCKED=5
uint8 FAILED=6

std_msgs/Header header
uint8 state
int32 target_id                # -1 when idle
bool target_visible
float32 distance_to_dock       # d - front_offset - dock_distance; zero when docked
float32 lateral_error          # e_y, metres
float32 heading_error          # psi, radians
uint8 retries
```

## Parameters

Both nodes declare every parameter with a `ParameterDescriptor` (description and, where it applies, a range), so `ros2 param describe` documents them and out-of-range values are rejected.

### `aruco_detector`

| Parameter | Default | Meaning |
|---|---|---|
| `image_topic` | `/camera/image_raw` | Input image |
| `camera_info_topic` | `/camera/camera_info` | Input intrinsics |
| `dictionary` | `DICT_4X4_50` | OpenCV ArUco dictionary name |
| `marker_size` | `0.15` | Side of the black square, metres |
| `base_frame` | `base_link` | Frame the poses are published in |
| `max_reprojection_error` | `2.0` | Pixels. Detections above this are dropped |
| `publish_tf` | `true` | Broadcast `base_frame -> <tf_prefix><id>` |
| `tf_prefix` | `aruco_` | Prefix for marker frame names |
| `publish_debug_image` | `true` | Publish `/aruco/debug_image` |

### `docking_controller`

| Parameter | Default | Meaning |
|---|---|---|
| `control_rate` | `20.0` | Control loop frequency, Hz |
| `dock_distance` | `0.10` | Target gap between front face and marker plane, metres |
| `front_offset` | `0.15` | Distance from `base_link` to the front face, metres |
| `final_range` | `0.50` | `d` below which the final approach may start, metres |
| `aim_gain` | `0.8` | Fraction of `d` at which the aim point sits in front of the marker |
| `max_bearing_offset` | `0.44` | Largest allowed marker bearing during approach, radians (25 degrees) |
| `yaw_trust_range` | `2.0` | Beyond this range the marker's orientation is ignored, metres |
| `v_max` | `0.30` | Linear speed limit, m/s |
| `v_final` | `0.05` | Speed during the final approach, m/s |
| `w_max` | `1.0` | Angular speed limit, rad/s |
| `a_max` | `0.5` | Linear acceleration limit, m/s^2 |
| `alpha_max` | `2.0` | Angular acceleration limit, rad/s^2 |
| `search_angular_speed` | `0.4` | Rotation speed while searching, rad/s |
| `search_timeout` | `40.0` | Give up searching after this long, seconds (about two full turns) |
| `acquire_frames` | `3` | Consecutive detections needed to leave SEARCH |
| `gate_lateral` | `0.03` | Largest `e_y` allowed when entering FINAL, metres |
| `gate_heading` | `0.087` | Largest `psi` allowed when entering FINAL, radians (5 degrees) |
| `lost_hold_time` | `0.3` | Keep driving on the last command this long after a dropout, seconds |
| `lost_timeout` | `1.0` | Return to SEARCH after this long without the target, seconds |
| `pose_stale_time` | `0.5` | Poses older than this are ignored, seconds |
| `pose_filter_alpha` | `0.5` | Smoothing factor for marker position, 1.0 disables it |
| `recover_range` | `2.0` | Distance to back off to before retrying, metres |
| `max_retries` | `3` | Recovery attempts before failing |
| `dock_timeout` | `120.0` | Overall time limit for one goal, seconds |
| `turn_gain` | `1.5` | Turn rate per radian of desired heading during approach, 1/s |
| `speed_gain` | `0.5` | Approach speed added per metre of range beyond `final_range`, 1/s |
| `final_heading_gain` | `1.5` | FINAL turn rate per radian of `psi`, 1/s |
| `final_lateral_gain` | `3.0` | FINAL turn rate per metre of `e_y`, rad/s/m |
| `cmd_frame` | `base_link` | Frame ID stamped on `/cmd_vel` and `/docking/state` |
| `autostart_marker_id` | `-1` | If zero or greater, start docking to this ID at launch without an action goal |

## Configuration files

| File | Package | Purpose |
|---|---|---|
| `config/bridge.yaml` | `dockbot_gazebo` | Gazebo to ROS topic bridges |
| `config/markers.yaml` | `dockbot_gazebo` | Marker ID to world pose. The single source of truth for marker placement; tests check the world file against it |
| `config/detector.yaml` | `dockbot_perception` | `aruco_detector` parameters |
| `config/controller.yaml` | `dockbot_docking` | `docking_controller` parameters |
| `config/scenarios/*.yaml` | `dockbot_bringup` | Per test scenario: world, spawn pose, goal ID, expected outcome |
