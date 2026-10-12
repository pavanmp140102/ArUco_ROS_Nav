# M3: Perception

**Goal:** the `aruco_detector` node (C++, `rclcpp`) publishes the pose of every visible marker relative to the robot, accurately enough to dock on.

**Depends on:** M0 for all unit and node tests. M2 for the in-simulation accuracy check (E10).

Design is in section 6 of [../architecture.md](../architecture.md); the interface is in [../interfaces.md](../interfaces.md).

## Sub-tasks

### Library `pose_estimation` (OpenCV and Eigen only, no ROS headers)

- [x] `get_dictionary(name)`: dictionary lookup by name, throwing a clear error for unknown names.
- [x] `detect_markers(gray, dictionary)`: the wrapper over the two OpenCV API generations, selected at compile time on `CV_VERSION`, returning corners and IDs in one struct.
- [x] `estimate_marker_pose(corners, marker_size, K, dist)`: `cv::solvePnPGeneric` with `cv::SOLVEPNP_IPPE_SQUARE`; returns the better solution as an `Eigen::Isometry3d` with its reprojection error.
- [x] `transform_pose(T_optical_marker, T_base_optical)`: optical frame to base frame.
- [x] `range_and_bearing(position)`: ground-plane range and bearing.
- [x] `draw_debug(image, detections, K, dist)`: outlines, IDs, axes.

### Test support

- [x] `test/synthetic_marker.hpp`: renders a marker into an image at a given pose and intrinsics, for the P tests and for PN1.

### Node `ArucoDetector` (`rclcpp::Node`, registered as a component)

- [x] Parameters as listed in [../interfaces.md](../interfaces.md), each with a descriptor.
- [x] `CameraInfo` subscription that stores intrinsics once.
- [x] Image subscription with sensor QoS; `cv_bridge::toCvShare` to avoid a copy.
- [x] TF lookup from the image's frame to `base_frame`, retried until available, then cached.
- [x] Publish `ArucoMarkerArray` for every image, including empty arrays.
- [x] Optional TF broadcast and debug image.
- [x] `src/aruco_detector_main.cpp` for the standalone executable; `rclcpp_components_register_node` for the component.
- [x] `config/detector.yaml` and `launch/detector.launch.py`.

### Tests (`ament_cmake_gtest`)

- [x] `test_pose_estimation.cpp`: P1 to P8.
- [x] `test_detector_node.cpp`: PN1 to PN4, creating the node and test publishers in one process and spinning an executor.
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

- Corner order matters: object points must be top-left, top-right, bottom-right, bottom-left. Getting this wrong flips the normal; P6 and P8 catch it.
- The library solves with `SOLVEPNP_IPPE` on corners in a y-down frame, not `IPPE_SQUARE` on y-up corners (OpenCV's usual marker frame). A marker seen face-on is then the identity rotation instead of exactly pi, where OpenCV 4.6's rotation vector comes back as zero and the pose is wrong, which would hit the robot exactly when it is docked. The result is flipped to the documented frame (y up, z out of the face) before it is returned. `IPPE_SQUARE` rejects y-down corners.
- Sub-pixel corner refinement is done in the library, with a window scaled to the marker's apparent cell size. OpenCV's fixed 5 px window is wider than a cell beyond about 5 m and biased the range by 5 to 7% at 6 m; the scaled window keeps it near 1%.
- Solutions whose printed face points away from the camera, or that lie behind it, are rejected as physically impossible before the lower reprojection error is chosen.
- Simulated cameras have zero distortion, but pass the distortion vector through anyway so the node works with a real camera.
- Do not filter in the detector. It reports what it sees; smoothing belongs to the controller.
- The image callback does all the work on the executor thread. At 30 Hz (a 33 ms budget) and roughly 10 ms per 1280x720 frame that is fine; no extra threads are needed.
- Convert between OpenCV, Eigen, and ROS types only at the node boundary. The library takes and returns OpenCV and Eigen types.
