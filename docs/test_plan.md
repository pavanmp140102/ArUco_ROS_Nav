# Test plan

Every test has an ID. Milestone plan files refer to these IDs as their gates.

## Levels

| Level | Prefix | Needs | Run with | Typical duration |
|---|---|---|---|---|
| Static checks | S | Nothing running | `colcon build`, `colcon test` | Seconds |
| Perception unit | P | OpenCV, no ROS | `gtest` via `colcon test` | Seconds |
| Perception node | PN | ROS, no simulator | `gtest` spinning the node in-process | Under a minute |
| Controller unit | C | Eigen, no ROS | `gtest` via `colcon test` | Seconds |
| Controller closed loop | CL | A kinematic model, no ROS | `gtest` via `colcon test` | Under a minute |
| End to end | E | Gazebo, headless | `launch_testing` (Python) in `dockbot_bringup` | Minutes |
| Docker | DK | Docker and Compose | `docker compose` services | Minutes |
| Manual | MAN | Gazebo GUI + RViz | By hand | - |

The design keeps the detection maths and the control law in ROS-free C++ libraries precisely so that levels P, C, and CL carry most of the coverage and run without a simulator.

The nodes and their unit tests are C++. The end-to-end harness is Python because `launch_testing` is; it only launches the system, sends goals, and scores the outcome. Randomised tests use a fixed seed so that a failure can be reproduced.

## Definition of a successful dock

Measured from Gazebo ground truth (robot pose) and `config/markers.yaml` (marker pose), after the robot has been stationary for one second:

| Quantity | Requirement |
|---|---|
| Front face to marker plane | 0.10 m, plus or minus 0.02 m |
| Lateral offset from the marker normal | At most 0.03 m |
| Heading error against the marker normal | At most 5 degrees |
| Contact | The robot never touches the marker, a wall, or an obstacle |

## Static checks (S)

| ID | Check | Pass criterion |
|---|---|---|
| S1 | `colcon build --symlink-install` | Succeeds, no warnings from `dockbot_*` packages |
| S2 | `ament_cmake_cpplint`, `ament_cmake_uncrustify`, `ament_cmake_cppcheck`, and `ament_cmake_xmllint` on all packages | Clean. No installed linter checks that every function has a describing comment, so that project rule is checked in review |
| S3 | `xacro` then `check_urdf` on the robot; `gz sdf -k` on every world and model | All parse without errors |
| S4 | Compare marker poses in `docking_world.sdf` with `config/markers.yaml` | Every ID present in both, poses equal within 1 mm and 0.01 rad |

## Perception unit (P)

Inputs are synthetic images: a marker image warped into a blank frame (`cv::warpPerspective`) by projecting its corners with known intrinsics (the planned camera's: 1280x720, focal length 1061 px) and a known pose. The renderer is a C++ test helper. No ROS involved.

| ID | Case | Pass criterion |
|---|---|---|
| P1 | Marker at known poses over a grid: range 0.30, 0.5, 1, 2, 3, 4, 6 m; yaw -60 to +60 degrees | Correct ID. Range error at most 2% or 1 cm, whichever is larger. Yaw error at most 3 degrees for ranges under 2 m |
| P2 | Frame with no marker | Empty result, no exception |
| P3 | Three markers with different IDs in one frame | All three IDs returned, each pose within P1 bounds |
| P4 | Marker from a different dictionary; a mirrored marker | Neither is reported |
| P5 | Marker cut off by the image edge; marker with one corner covered | Not reported. No pose with wild values |
| P6 | Optical frame to `base_link` conversion with the planned camera mount | A marker 1 m straight ahead of the camera, facing it, gives x = 0.95 m in `base_link` (the camera sits 0.05 m behind it), y = 0, bearing = 0, and a normal pointing back at the robot |
| P7 | Detection wrapper on the installed OpenCV | Returns corners and IDs in the documented shape. The version branch is chosen at compile time, so the branch for 4.7 and later is verified by building and running the same tests against a newer OpenCV when one is available |
| P8 | Ambiguity handling: marker nearly face-on at 3 m | The solution with lower reprojection error is chosen; the normal points toward the camera, not away |

## Perception node (PN)

| ID | Case | Pass criterion |
|---|---|---|
| PN1 | Publish a synthetic image, `CameraInfo`, and a static transform | One `ArucoMarkerArray` in `base_link`, stamped with the image time, pose within P1 bounds |
| PN2 | Images arrive before any `CameraInfo` | No output, a throttled warning, node stays alive |
| PN3 | 1280x720 images at 30 Hz for 10 s | Output rate at least 20 Hz, one message per image including empty ones |
| PN4 | `publish_tf` on | `base_link -> aruco_<id>` appears on TF and matches the message pose |

## Controller unit (C)

Functions in the `control_law` and `state_machine` libraries.

| ID | Case | Pass criterion |
|---|---|---|
| C1 | Error terms from hand-built poses: dead ahead and square; offset left; offset right; rotated | `d`, `e_y`, `psi`, `bearing` match hand-computed values. This test fixes the sign conventions |
| C2 | Stop condition | Triggers exactly when `d - front_offset <= dock_distance`, not before |
| C3 | 10,000 seeded random inputs across the whole plausible range | Commands never exceed `v_max`, `w_max`, or the acceleration limits |
| C4 | Field-of-view clamp over a grid of marker poses | The commanded heading never asks for a marker bearing beyond `max_bearing_offset` |
| C5 | Every edge of the state machine, driven by scripted inputs and a fake clock | Each transition fires under its condition and only then. Covers acquire count, lost-target timing, retry counting, cancel, and timeout |
| C6 | Stream of detections that never contains the goal ID | State stays SEARCH, command is the search rotation only |
| C7 | Stale pose, NaN pose | Treated as no detection; command goes to zero within `lost_hold_time` |
| C8 | Beyond `yaw_trust_range` with a deliberately wrong marker orientation | Command depends on bearing and range only |
| C9 | Goal with `dock_distance` set, and with 0.0 | Explicit value is used; 0.0 falls back to the parameter |

## Controller closed loop (CL)

A unicycle model integrates the commanded velocities. A camera model returns the marker pose only when the marker is inside the field of view and facing the camera, optionally with noise and dropped frames. Both are C++ test helpers with no ROS, so thousands of runs take seconds.

| ID | Case | Pass criterion |
|---|---|---|
| CL1 | 500 random starts: range 1 to 4 m, offset from the normal up to 50 degrees either side, initial heading up to 30 degrees off the marker | At least 98% end docked within tolerance. No run crosses the marker plane |
| CL2 | CL1 with pose noise equivalent to half a pixel of corner error, and 10% of frames dropped | At least 95% docked. No run crosses the marker plane |
| CL3 | Marker never visible | Ends FAILED with outcome `TARGET_NOT_FOUND`, zero velocity |
| CL4 | Start close and badly offset, so the gates fail at `final_range` | RECOVER runs, and the retry docks |
| CL5 | Marker disappears permanently mid-approach | Robot stops, searches, and ends FAILED. It does not keep driving blind |
| CL6 | Across all CL1 runs, log the marker bearing | The marker stays inside the field of view for at least 99% of approach time |

## End to end in Gazebo (E)

Run headless through `launch_testing`. Each scenario is a YAML file in `dockbot_bringup/config/scenarios/` naming the world, spawn pose, goal ID, and expected outcome. Scoring uses ground truth; the nodes under test never see it.

| ID | Scenario | Pass criterion |
|---|---|---|
| E1 | `single_dock`, marker straight ahead at 2 m | Docked within tolerance in under 60 s of simulation time |
| E2 | `single_dock`, robot 1 m to the side of the normal and yawed 30 degrees | Docked within tolerance |
| E3 | `single_dock`, robot facing away from the marker | SEARCH finds it, then docked within tolerance |
| E4 | `docking_world`, default spawn, goal set to each of IDs 0 to 4 in separate runs | Docked at the requested marker every time. Final pose is never in front of a different marker |
| E5 | `docking_world`, spawn facing IDs 0 and 1 with both in view, goal = 1 | Docked at ID 1 |
| E6 | `docking_world`, goal ID 7, which is not in the world | Action result `TARGET_NOT_FOUND`, robot stationary at the end |
| E7 | `docking_world`, spawn chosen so a pillar interrupts the view of the goal marker during approach | Either docked within tolerance, or FAILED with the robot stopped. No collision in either case |
| E8 | Cancel the action two seconds into APPROACH | Robot stationary within 0.5 s, result `CANCELED` |
| E9 | E2 repeated ten times | All ten docked; standard deviation of the final front-face distance at most 1 cm |
| E10 | `single_dock`, robot placed at 0.3, 1, 2, and 3.8 m from the marker, stationary | Detector pose against ground truth within P1 bounds |
| E11 | E2 with camera noise enabled | Docked within tolerance |
| E12 | Two goals in a row in `docking_world`: ID 4, then ID 0 | Both docked. The second goal starts from the first dock |

## Docker (DK)

These prove the project runs on a machine that has only Docker. Details of the image and services are in [plan/M8_docker.md](plan/M8_docker.md).

| ID | Case | Pass criterion |
|---|---|---|
| DK1 | `docker build --no-cache` of the `runtime` stage from a fresh clone | Succeeds. Nothing from the host's ROS installation is used |
| DK2 | `test` service with `DISPLAY` unset and no GPU passed in | All S, P, PN, C, and CL tests pass and the exit code is 0. With one test deliberately broken, the exit code is non-zero |
| DK3 | `test-e2e` service under software rendering | E1 to E12 pass |
| DK4 | `sim` service on a machine with a display | Gazebo and RViz windows appear on the host, and a `/dock` goal sent with `docker compose exec` ends docked |
| DK5 | `dev` service: build the workspace in the mounted source tree | Build succeeds, and the new `build/`, `install/`, and `log/` directories on the host belong to the host user |
| DK6 | Edit one `.cpp` file and rebuild the image | The dependency layer is reused from cache; `rosdep` does not run again |
| DK7 | Inside the image, `rosdep check --from-paths src --ignore-src` | Reports all dependencies satisfied, and the Dockerfile contains no manual `apt install` of ROS packages |
| DK8 | Clone and run DK2 and DK4 on a second machine, ideally one with a different GPU or CPU architecture | Both pass with no steps beyond those in the README. Run when such a machine is available |
| DK9 | `sim-nvidia` service on a machine with an NVIDIA GPU | Same result as DK4, and `nvtop` or `nvidia-smi` on the host shows the Gazebo process on the NVIDIA GPU |

## Manual checks (MAN)

| ID | Check |
|---|---|
| MAN1 | RViz shows the robot model, the debug image with marker outlines and axes, the `aruco_<id>` frames, and the docking state |
| MAN2 | The README quick start works on a fresh clone: build, one launch command, one action goal, robot docks |
| MAN3 | Teleoperation drives the robot and the odometry arrow in RViz follows it |

## Commands

```bash
# everything except Gazebo tests
colcon test --packages-select dockbot_perception dockbot_docking dockbot_gazebo dockbot_description
colcon test-result --verbose

# end to end
colcon test --packages-select dockbot_bringup
colcon test-result --verbose

# the same, inside Docker
docker compose -f docker/compose.yaml run --rm test
docker compose -f docker/compose.yaml run --rm test-e2e

# one test binary, with its output shown
colcon test --packages-select dockbot_docking --ctest-args -R test_control_law --event-handlers console_direct+

# one test case inside a binary
./build/dockbot_docking/test_control_law --gtest_filter='ControlLaw.StopCondition'
```
