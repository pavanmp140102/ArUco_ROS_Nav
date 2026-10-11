# M3: Perception

**Goal:** the `aruco_detector` node (C++, `rclcpp`) publishes the pose of every visible marker relative to the robot, accurately enough to dock on.

**Depends on:** M0 for all unit and node tests. M2 for the in-simulation accuracy check (E10).

Design is in section 6 of [../architecture.md](../architecture.md); the interface is in [../interfaces.md](../interfaces.md).

## Sub-tasks

### Library `pose_estimation` (OpenCV and Eigen only, no ROS headers)

- [ ] `get_dictionary(name)`: dictionary lookup by name, throwing a clear error for unknown names.
- [ ] `detect_markers(gray, dictionary)`: the wrapper over the two OpenCV API generations, selected at compile time on `CV_VERSION`, returning corners and IDs in one struct.
- [ ] `estimate_marker_pose(corners, marker_size, K, dist)`: `cv::solvePnPGeneric` with `cv::SOLVEPNP_IPPE_SQUARE`; returns the better solution as an `Eigen::Isometry3d` with its reprojection error.
- [ ] `transform_pose(T_optical_marker, T_base_optical)`: optical frame to base frame.
- [ ] `range_and_bearing(position)`: ground-plane range and bearing.
- [ ] `draw_debug(image, detections, K, dist)`: outlines, IDs, axes.

### Test support

- [ ] `test/synthetic_marker.hpp`: renders a marker into an image at a given pose and intrinsics, for the P tests and for PN1.

### Node `ArucoDetector` (`rclcpp::Node`, registered as a component)

- [ ] Parameters as listed in [../interfaces.md](../interfaces.md), each with a descriptor.
- [ ] `CameraInfo` subscription that stores intrinsics once.
- [ ] Image subscription with sensor QoS; `cv_bridge::toCvShare` to avoid a copy.
- [ ] TF lookup from the image's frame to `base_frame`, retried until available, then cached.
- [ ] Publish `ArucoMarkerArray` for every image, including empty arrays.
- [ ] Optional TF broadcast and debug image.
- [ ] `src/aruco_detector_main.cpp` for the standalone executable; `rclcpp_components_register_node` for the component.
- [ ] `config/detector.yaml` and `launch/detector.launch.py`.

### Tests (`ament_cmake_gtest`)

- [ ] `test_pose_estimation.cpp`: P1 to P8.
- [ ] `test_detector_node.cpp`: PN1 to PN4, creating the node and test publishers in one process and spinning an executor.
- [ ] E10 scenario in `dockbot_bringup` once M2 exists.

## Files created

```
src/dockbot_perception/include/dockbot_perception/{pose_estimation.hpp,aruco_detector.hpp}
src/dockbot_perception/src/{pose_estimation.cpp,aruco_detector.cpp,aruco_detector_main.cpp}
src/dockbot_perception/config/detector.yaml
src/dockbot_perception/launch/detector.launch.py
src/dockbot_perception/test/{synthetic_marker.hpp,test_pose_estimation.cpp,test_detector_node.cpp}
```

## Gate

- P1 to P8 and PN1 to PN4 pass.
- E10 passes (after M2).
- In RViz, the `aruco_<id>` frame sits on the marker in the camera image and its z axis points out of the marker toward the robot.

## Notes

- Corner order matters for `IPPE_SQUARE`: object points must be top-left, top-right, bottom-right, bottom-left, with the marker's y axis up. Getting this wrong flips the normal; P6 and P8 catch it.
- Simulated cameras have zero distortion, but pass the distortion vector through anyway so the node works with a real camera.
- Do not filter in the detector. It reports what it sees; smoothing belongs to the controller.
- The image callback does all the work on the executor thread. At 30 Hz (a 33 ms budget) and roughly 10 ms per 1280x720 frame that is fine; no extra threads are needed.
- Convert between OpenCV, Eigen, and ROS types only at the node boundary. The library takes and returns OpenCV and Eigen types.
