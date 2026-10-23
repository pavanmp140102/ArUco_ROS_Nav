# M4: Docking controller

**Goal:** the `docking_controller` node (C++, `rclcpp`) drives the robot from anywhere the goal marker is visible to a stop 10 cm in front of it, aligned, using only `/aruco/markers`.

**Depends on:** M0. Everything in this milestone is tested without Gazebo.

Design is in section 7 of [../architecture.md](../architecture.md); parameters are in [../interfaces.md](../interfaces.md).

## Sub-tasks

### Library `control_law` (Eigen only, no ROS headers)

- [x] `struct ControlParams`, `struct DockErrors`, `struct VelocityCommand`.
- [x] `compute_errors(marker_pose)`: returns `d`, `e_y`, `psi`, `bearing`, `range`.
- [x] `approach_command(errors, params)`: aim point on the normal, field-of-view clamp, speed scaling. Uses bearing only beyond `yaw_trust_range`.
- [x] `final_command(errors, params)`: slow forward creep with small corrections.
- [x] `recover_command(errors, params)`: reverse while keeping the marker centred.
- [x] `search_command(last_bearing, params)`: rotation direction and speed.
- [x] `is_docked(errors, params)` and `gates_pass(errors, params)`.
- [x] `limit_command(cmd, previous, dt, params)`: velocity and acceleration limits.
- [x] `class PoseFilter`: exponential smoothing with reset.

### Library `state_machine` (no ROS headers)

- [x] `class DockingStateMachine` with `step(detection, now_seconds)` returning the state and a command. Time is an argument so tests can drive a fake clock.
- [x] Transitions exactly as in the state diagram, including lost-target hold and timeout, retries, cancel, and overall timeout.
- [x] Goal-ID filtering happens here, at the single place where detections enter.

### Test simulator (`test/kinematic_sim.hpp`)

- [x] Unicycle integration of commanded velocities.
- [x] Camera model: returns the marker pose in the robot frame when the marker is within the field of view, within range, and facing the camera; otherwise nothing.
- [x] Options for pose noise, dropped frames, and a marker that disappears at a set time, all driven by a seeded `std::mt19937`.

### Node `DockingController` (`rclcpp::Node`, registered as a component)

- [x] Parameters with descriptors, and an `on_set_parameters` callback so gains can be tuned while running.
- [x] `/aruco/markers` subscription.
- [x] `/dock` action server (`rclcpp_action`): accept one goal at a time, publish feedback, support cancel, fill the result.
- [x] Control timer at `control_rate`, publishing `TwistStamped` and `DockingState`.
- [x] Zero command on shutdown, cancel, failure, and success.
- [x] `autostart_marker_id` handling.
- [x] `src/docking_controller_main.cpp` for the standalone executable; component registration.
- [x] `config/controller.yaml` and `launch/controller.launch.py`.

### Tests (`ament_cmake_gtest`)

- [x] `test_control_law.cpp`: C1 to C4, C8.
- [x] `test_state_machine.cpp`: C5 to C7, C9.
- [x] `test_closed_loop.cpp`: CL1 to CL6. Optionally writes trajectories to CSV when an environment variable is set, for plotting.

## Files created

```
src/dockbot_docking/include/dockbot_docking/{control_law.hpp,state_machine.hpp,docking_controller.hpp}
src/dockbot_docking/src/{control_law.cpp,state_machine.cpp,docking_controller.cpp,docking_controller_main.cpp}
src/dockbot_docking/config/controller.yaml
src/dockbot_docking/launch/controller.launch.py
src/dockbot_docking/test/{kinematic_sim.hpp,test_control_law.cpp,test_state_machine.cpp,test_closed_loop.cpp}
```

## Gate

- [x] C1 to C9 and CL1 to CL6 pass.
- [x] A plot of ten CL1 trajectories (from the CSV output) shows smooth convergence onto the normal line with no oscillation. Save it under `docs/images/` and link it from [../architecture.md](../architecture.md).

## Notes

- Write C1 first. It pins the sign conventions for `e_y`, `psi`, and `bearing`, and everything else depends on them.
- Tune in the kinematic simulator before touching Gazebo. A change there is tested in seconds.
- Starting points for tuning: turn-rate gain about 1.5, `aim_gain` 0.5. If CL6 shows the marker leaving view, lower `max_bearing_offset` or `aim_gain`. A *higher* `aim_gain` puts the aim point nearer the robot's own distance and joins the normal line earlier (the original note had this the wrong way round).
- Tuned values, from 500-run CL1 batches: `aim_gain` 0.8, `final_range` 0.5, `recover_range` 2.0 (1.0 did not work: reversing 0.3 m barely changes the angle off the normal). With these, CL1 docks 500/500, CL6 is 99.98%, and RECOVER is used in about half the runs. That is a consequence of the 25 degree field-of-view clamp, see architecture section 7.3, not a defect. Re-check these in M5 against Gazebo latency before trusting them.
- The kinematic camera is ideal (no latency), so the margins in CL6 are optimistic for Gazebo.
- The command is always computed from the latest filtered pose. Never integrate a pose forward from past commands; that would be dead reckoning, which the design excludes.
- The action callbacks and the control timer share state. Run the node on a single-threaded executor and have the action callbacks only record requests for the timer to act on; that avoids locks. Do not block inside the action's execute callback.
- The node's timeouts use `this->now()`, so they follow simulation time.
