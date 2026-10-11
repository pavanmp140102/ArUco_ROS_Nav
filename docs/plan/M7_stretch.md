# M7: Stretch goals

**Goal:** optional extensions, each independent of the others. None is needed to meet the brief.

**Depends on:** M5.

## 1. Lidar safety stop

The core robot cannot sense obstacles. A 2D lidar plus a small node that scales down or zeroes forward velocity when something is close ahead makes it safe in cluttered worlds. This is collision prevention, not localization, so it stays within the brief.

- [ ] Add a `gpu_lidar` sensor to the Xacro and bridge `/scan`.
- [ ] `safety_stop` node between the controller and the bridge: passes commands through, limiting forward speed by the nearest return in a forward cone.
- [ ] Exempt the docking station itself during FINAL, when the robot is meant to get close.
- [ ] Tests: an obstacle placed in the corridor makes the robot stop short and the goal fail by timeout, without contact.

## 2. Odometry bridging of detection gaps

Hold the last marker estimate in the `odom` frame and keep steering on it while the marker is briefly out of view. This allows wider approach angles and survives longer occlusions. Wheel odometry is onboard, so it is not external localization, but it does go beyond "relative marker pose only", which is why it is not in the core.

- [ ] Store the filtered marker pose in `odom` on each detection.
- [ ] When detections stop, transform the stored pose back to `base_link` for up to a configurable time.
- [ ] Tests: repeat CL2 with 40% dropped frames; repeat E7 and require a successful dock.

## 3. Search beyond line of sight

If a full rotation does not find the goal marker, the core robot gives up. With the lidar from item 1, it could wander to a new vantage point and search again.

- [ ] Simple frontier-free exploration: drive toward the longest free lidar direction, rotate, repeat, up to a limit.
- [ ] Test: a world where the goal marker is hidden from the spawn point.

## 4. Multi-dock missions

- [ ] A small mission node that takes a list of marker IDs and sends `/dock` goals in order, backing away from each dock before searching for the next.
- [ ] Test: visit IDs 4, 0, 2 in sequence.

## 5. Real camera readiness

- [ ] Run the detector on recorded footage from a real camera with a calibration file.
- [ ] Add lens distortion and motion blur to the simulated camera and re-run the E suite.
