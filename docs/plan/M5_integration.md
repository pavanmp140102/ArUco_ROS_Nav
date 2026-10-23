# M5: Integration and validation

**Goal:** the whole pipeline runs in Gazebo from one launch command, and an automated suite proves that the robot docks precisely at the requested marker.

**Depends on:** M2, M3, M4.

Scenarios and pass criteria are the E tests in [../test_plan.md](../test_plan.md).

## Sub-tasks

### Launch

- [x] `launch/dock.launch.py` in `dockbot_bringup`: includes `sim.launch.py`, starts `aruco_detector` and `docking_controller` with their YAML files and `use_sim_time`, optional RViz. Arguments: `world`, `x`, `y`, `yaw`, `marker_id`, `headless`, `gpu`, `rviz`.
- [x] Extend the RViz configuration with the debug image and marker frames (marker frames come from the detector's TF, which the existing TF display already shows).

### Ground truth for scoring

- [x] Add a pose publisher for the robot model in the world frame and bridge it to `/ground_truth/odom` (`nav_msgs/Odometry`; done during M3 for E10).
- [x] `test/scoring.py`: given the robot's true pose and a marker pose from `markers.yaml`, compute front-face distance, lateral offset, and heading error. This is a Python helper for the test harness, not a node.
- [x] Unit test for `scoring.py` with hand-computed cases.
- [ ] Contact check: subscribe to the robot's contact sensor, or verify from the trajectory that the front face never came closer than 0.05 m to the marker plane.

### Scenarios and tests

- [ ] One YAML file per scenario E1 to E12 under `config/scenarios/`: world, spawn pose, goal ID, expected outcome, time limit.
- [ ] Register the tests in `CMakeLists.txt` with `add_launch_test` from `launch_testing_ament_cmake`, with a generous timeout per scenario.
- [ ] `test/test_e2e.py`: a parametrised `launch_testing` test that launches a scenario headless, sends the goal, waits for the result, waits one second, then scores against ground truth.
- [ ] Support for stationary scenarios (E10) and scripted actions such as cancel (E8) and consecutive goals (E12).
- [ ] Camera noise switch in the Xacro for E11.

### Tuning

- [ ] Run E1 to E5 and adjust `controller.yaml` until all pass. Re-run the CL tests after every change.
- [ ] Run the full suite three times in a row.
- [ ] Record final parameter values in [../interfaces.md](../interfaces.md) and results in a short table at the end of [../test_plan.md](../test_plan.md).

## Files created

```
src/dockbot_bringup/launch/dock.launch.py
src/dockbot_bringup/config/scenarios/e01_straight.yaml ... e12_two_goals.yaml
src/dockbot_bringup/test/{scoring.py,test_scoring.py,test_e2e.py}
```

## Gate

- E1 to E12 pass in three consecutive full runs.
- All earlier gates still pass.

## Notes

- Differences between the kinematic simulator and Gazebo to expect: wheel acceleration lag, a one or two frame delay from camera to command, and slight pitch when braking, which shifts the apparent marker position. If Gazebo shows overshoot the kinematic tests did not, add the delay to the kinematic camera model so the cheap tests cover it.
- Start each scenario from a fresh simulator. Reusing one across scenarios makes failures depend on test order.
- The suite is slow. Mark it so `colcon test` on the other packages stays fast.
