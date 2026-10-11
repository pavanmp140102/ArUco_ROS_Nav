# M2: World and markers

**Goal:** two Gazebo worlds containing ArUco markers at known poses, with obstacles, in which every marker can be read through the robot's camera.

**Depends on:** M1.

Layout and marker specification are in [../world_and_robot.md](../world_and_robot.md).

## Sub-tasks

- [ ] `scripts/generate_markers.py`: writes `DICT_4X4_50` IDs 0 to 4 as PNGs with the white quiet zone included (0.15 m code on a 0.20 m plate, so the code is 75% of the image side). Works on OpenCV 4.6 (`drawMarker`) and newer (`generateImageMarker`).
- [ ] Commit the generated PNGs.
- [ ] One model per ID under `models/aruco_marker_<id>/`: `model.config`, `model.sdf` (static thin box, PBR albedo map, matte), and the texture.
- [ ] `models/dock_stand/`: a simple post to carry a free-standing marker.
- [ ] `config/markers.yaml` as specified.
- [ ] `worlds/single_dock.sdf`: ground, light, one marker (ID 0) on a stand.
- [ ] `worlds/docking_world.sdf`: 7 m by 7 m walled room, five markers at the `markers.yaml` poses, boxes and pillars as obstacles, clear corridor in front of each marker.
- [ ] Place one pillar so that a documented spawn pose has its view of one marker interrupted during approach (for E7). Record that spawn pose in the E7 scenario notes.
- [ ] Static test S4 comparing the world file with `markers.yaml`.
- [ ] Static test running `gz sdf -k` on every world and model.
- [ ] Update the layout table in [../world_and_robot.md](../world_and_robot.md) with final obstacle positions.

## Files created

```
src/dockbot_gazebo/scripts/generate_markers.py
src/dockbot_gazebo/models/aruco_marker_{0..4}/{model.config,model.sdf,materials/textures/marker_<id>.png}
src/dockbot_gazebo/models/dock_stand/{model.config,model.sdf}
src/dockbot_gazebo/config/markers.yaml
src/dockbot_gazebo/worlds/{single_dock.sdf,docking_world.sdf}
src/dockbot_gazebo/test/{test_sdf_valid.py,test_marker_poses.py}
```

## Gate

- S3 and S4 pass.
- Readability check: for each ID, spawn the robot 1.5 m in front of the marker, save one camera frame, and run plain OpenCV detection on it. All five IDs are found with the right number. This needs no ROS node and catches mirrored or rotated textures.
- From the default spawn, rotating in place brings each of the five markers into view, and each is detected.

## Notes

- A mirrored ArUco pattern is not a valid code. If a marker is never detected, check texture mapping before anything else.
- Avoid shadows and specular highlights on marker faces; either can break detection at range.
- Marker yaw in `markers.yaml` is the direction the printed face points, which is the docking normal.
