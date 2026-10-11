# M0: Workspace and interfaces

**Goal:** a colcon workspace that builds, with every package present as a skeleton and the shared messages defined. Later milestones only add files to packages that already exist.

**Depends on:** nothing.

## Sub-tasks

- [x] Create `src/` and a `.gitignore` covering `build/`, `install/`, `log/`.
- [x] `dockbot_interfaces`: `msg/ArucoMarker.msg`, `msg/ArucoMarkerArray.msg`, `msg/DockingState.msg`, `action/Dock.action`, as specified in [../interfaces.md](../interfaces.md).
- [x] `dockbot_description`: manifest, `CMakeLists.txt` installing `urdf/` and `rviz/`.
- [x] `dockbot_gazebo`: manifest, `CMakeLists.txt` installing `worlds/`, `models/`, `config/`, `launch/`, `scripts/`.
- [x] `dockbot_perception`: manifest, `CMakeLists.txt` with C++17, an empty `pose_estimation` library target, `include/dockbot_perception/`, `src/`, `test/`, and install rules for `config/` and `launch/`.
- [x] `dockbot_docking`: same shape, with `control_law` and `state_machine` library targets.
- [x] `dockbot_bringup`: manifest, `CMakeLists.txt` installing `launch/` and `config/`, with `launch_testing_ament_cmake` for the tests.
- [x] Declare dependencies in each `package.xml`. Node packages: `rclcpp`, `rclcpp_action`, `rclcpp_components`, `cv_bridge`, `image_transport`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `sensor_msgs`, `geometry_msgs`, `dockbot_interfaces`, plus `libopencv-dev` and `eigen`. Simulation packages: `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, `xacro`, `robot_state_publisher`.
- [x] Shared compile options in each C++ package: `-Wall -Wextra -Wpedantic`, C++17.
- [x] Linting through `ament_lint_auto` with `ament_cmake_cpplint`, `ament_cmake_uncrustify`, `ament_cmake_cppcheck`, `ament_cmake_xmllint`. The project is licensed Apache-2.0, so the copyright check stays on: every source, CMake, and Python file starts with the Apache-2.0 header (see `LICENSE`).
- [x] Record the confirmed build commands in the contributor notes.

## Files created

```
.gitignore
LICENSE (Apache-2.0)
src/dockbot_interfaces/{package.xml,CMakeLists.txt,msg/*.msg,action/Dock.action}
src/dockbot_description/{package.xml,CMakeLists.txt}
src/dockbot_gazebo/{package.xml,CMakeLists.txt}
src/dockbot_perception/{package.xml,CMakeLists.txt,include/dockbot_perception/,src/,test/}
src/dockbot_docking/{package.xml,CMakeLists.txt,include/dockbot_docking/,src/,test/}
src/dockbot_bringup/{package.xml,CMakeLists.txt,launch/,config/scenarios/,test/}
```

## Gate

- S1: `colcon build --symlink-install` succeeds.
- S2: lint tests pass on the skeleton packages.
- `ros2 interface show dockbot_interfaces/action/Dock` prints the definition.

## Notes

- The linters do not check that functions are documented. The project rule, that each function has a comment saying what it does and how it serves docking, is checked in review. Use Doxygen-style `///` or `/** */` comments on declarations in the headers.
- Keep message field names exactly as in [../interfaces.md](../interfaces.md). M3 and M4 are developed against them in parallel.
- Use the system OpenCV through `find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)`. Do not vendor or build another copy; see [../setup.md](../setup.md).

## Result

Done. S1 passes (all six packages build), S2 passes (48 lint tests, 0 failures), and `ros2 interface show dockbot_interfaces/action/Dock` prints the definition. `cppcheck` skips in `dockbot_perception` and `dockbot_docking` because the library stubs hold no code yet; it runs once M3/M4 add sources.
