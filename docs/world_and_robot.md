# World and robot model

Dimensions here are the design targets. The Xacro, the world file, and `config/markers.yaml` are authoritative once they exist; keep this document in step with them.

## Robot: `dockbot`

| Part | Value |
|---|---|
| Chassis | Box, 0.30 m long, 0.24 m wide, 0.08 m tall |
| Drive wheels | Two, radius 0.04 m, width 0.02 m, separation 0.26 m, on the centre axle |
| Casters | Two frictionless spheres, front and rear |
| `base_link` | At the axle centre. The front face is 0.15 m ahead of it (`front_offset`) |
| Mass | About 3 kg, with inertias computed from the primitive shapes |
| Drive | Gazebo `gz-sim-diff-drive-system`, subscribing to `cmd_vel`, publishing `odom` and its TF |
| Joint states | Gazebo `gz-sim-joint-state-publisher-system` |

![dockbot side view](images/robot_side_view.png)

### Camera

| Property | Value |
|---|---|
| Mount | On a mast on top of the chassis |
| Position | 0.05 m behind `base_link`, 0.20 m above the ground, looking straight ahead |
| Distance behind the front face | 0.20 m |
| Sensor modelled | Sony IMX219 (Raspberry Pi Camera v2, Jetson CSI modules), full sensor width scaled to 720p |
| Resolution | 1280 x 720 |
| Horizontal field of view | 1.0856 rad (62.2 degrees), so 31.1 degrees each side |
| Vertical field of view | 37.5 degrees |
| Frame rate | 30 Hz |
| Frame | `camera_optical_frame`, set as the sensor's `gz_frame_id` |
| Noise | None initially. Gaussian noise is added for robustness tests in M5 |
| Distortion | None. A real IMX219 lens distorts; see the real camera item in M7 |

Width, height, field of view, and rate are Xacro arguments with the values above as defaults. The launch files expose `camera_rate` so that slow renderers (software rendering in CI) can lower it. The detector takes its intrinsics from `CameraInfo`, so no node depends on these numbers.

The field of view is that of the full sensor width, not of the IMX219's centre-crop 720p mode (about 50 degrees wide and 30 tall). The crop is too short vertically to hold the marker at the dock.

### Why the camera sits where it does

Focal length in pixels is `640 / tan(0.5428) = 1061`. Two constraints follow.

**Far limit.** A 0.15 m marker at 4 m is `1061 * 0.15 / 4 = 40` pixels wide. A 4x4 marker has six cells across including its border, so that is about 6.6 pixels per cell. The practical floor for detection is near 3.7 pixels per cell, which is reached at about 7.2 m. Treat 6 m as the maximum reliable range.

**Near limit.** At distance `z` the image covers `0.679 * z` vertically and `1.206 * z` horizontally. The 0.15 m marker fits vertically only when `z >= 0.22 m`.

At the dock the front face is 0.10 m from the marker, so the camera is `0.10 + 0.20 = 0.30 m` away. There the image covers 0.204 m by 0.362 m:

- the marker fills 74% of the image height, leaving 27 mm of white plate above and below, just over one marker cell, and
- it stays fully visible with up to 0.106 m of sideways error.

A 720p image is wide and short, so the vertical fit is the tight one. With the camera 0.03 m further forward the white margin would shrink to 16 mm, less than one cell, with nothing to spare for the robot pitching as it brakes. That sets the 0.20 m setback.

Had the camera been at the front face, it would be 0.10 m from the marker at the dock and could not see the whole pattern. That is the reason for a setback at all, and the main reason for building a custom robot.

## Markers

| Property | Value |
|---|---|
| Dictionary | `DICT_4X4_50` |
| IDs used | 0 to 4 |
| Black square side | 0.15 m (`marker_size`) |
| Plate | 0.20 m square, white, giving a 0.025 m quiet zone on every side |
| Centre height | 0.20 m above the ground, level with the camera |
| Model | A thin box with the marker image as its albedo texture, matte, one SDF model per ID |

Marker images come from a generator script in `dockbot_gazebo` (using `cv2.aruco.drawMarker` on OpenCV 4.6, `generateImageMarker` on newer versions), and the PNGs are committed so that building does not depend on running it.

A marker texture that ends up mirrored is not a valid ArUco code and will never be detected. The M2 exit check, reading every ID through the simulated camera, exists to catch that.

## Worlds

Two worlds, both in `dockbot_gazebo/worlds/`.

![docking_world plan view](images/world_layout.png)

![docking_world in Gazebo](images/gazebo_world.png)

### `single_dock.sdf`

An open floor with one marker (ID 0) on a stand. Used for the basic approach tests, where the spawn pose is varied and nothing else is in view.

### `docking_world.sdf`

A walled room, 7 m by 7 m, centred on the origin. The robot spawns at the origin facing +x by default.

| ID | Position (x, y) | Faces | Mount | Distance from spawn |
|---|---|---|---|---|
| 0 | (3.45, 1.0) | -x (west) | East wall | 3.6 m |
| 1 | (3.45, 1.6) | -x (west) | East wall, 0.6 m from ID 0 | 3.8 m |
| 2 | (-1.5, 3.45) | -y (south) | North wall | 3.8 m |
| 3 | (-3.45, -1.0) | +x (east) | West wall | 3.6 m |
| 4 | (1.5, -2.5) | +y (north) | Free-standing dock stand | 2.9 m |

IDs 0 and 1 sit side by side on purpose: both are in view at once, which tests that the robot docks to the one it was asked for.

Obstacles are static boxes and cylindrical pillars placed between and beside the docking corridors. They make the scene realistic, block some lines of sight from some spawn positions, and give the detector non-marker clutter to reject. They are kept off the straight corridor in front of each marker.

Final obstacle layout in `docking_world.sdf` (centres in metres; the room is 7 m by 7 m with walls 1 m tall):

| Name | Shape | Position (x, y) | Size |
|---|---|---|---|
| `box_a` | Box | (2.6, -1.6) | 0.6 x 0.6 x 0.5 |
| `box_b` | Box | (-2.4, 1.8) | 0.5 x 0.8 x 0.5 |
| `box_c` | Box | (-0.5, -2.5) | 0.4 x 0.4 x 0.5 |
| `pillar_1` | Cylinder | (0.5, 2.5) | radius 0.2, height 0.8 |
| `pillar_occluder` | Cylinder | (0.25, 1.975) | radius 0.15, height 0.6 |

Markers sit on 0.05 m thick plates. Walls carry markers 0 to 3 directly. Marker 4 stands on a `dock_stand` post. `single_dock.sdf` has marker 0 on a stand at (3.0, 0.0) facing -x.

`pillar_occluder` lies on the line of sight from (2.0, 0.5) to marker 2. A robot placed at (2.0, 0.5) with yaw 2.44 rad does not see marker 2 (checked with plain OpenCV detection on a camera frame), while from the default spawn at the origin the marker is visible. That pose never sees marker 2, so it cannot test an interruption during approach. The E7 spawn is (2.0, 0.9) with yaw 2.51 rad: marker 2 is in view at the start, the robot approaches, the pillar then blocks the view, and the goal ends `TARGET_LOST` with the robot stopped and clear of every obstacle. A start at (2.2, 1.0) with yaw 2.45 rad showed the limit of a controller without obstacle sensing: it alternated between search and approach until the timeout and passed within about 1 cm of `pillar_1`.

The obstacle `box_a` was moved from (2.6, -1.2) to (2.6, -1.6) in M5. From the dock in front of marker 4 the line of sight to marker 0 passed through its corner, which made test E12 (marker 4, then marker 0) impossible. (`images/world_layout.png` still shows the old position.)

The default spawn sees every marker by turning in place, and each is detected by plain OpenCV on a camera frame. Marker plates are white, but the scene is lit with ambient light of 0.9, so they render light grey; detection is unaffected.

### `config/markers.yaml`

```yaml
# Marker ID -> pose of the marker face in the world frame.
# yaw is the direction the printed face points, in radians.
markers:
  0: {x: 3.45, y: 1.0, z: 0.20, yaw: 3.14159}
  1: {x: 3.45, y: 1.6, z: 0.20, yaw: 3.14159}
  2: {x: -1.5, y: 3.45, z: 0.20, yaw: -1.5708}
  3: {x: -3.45, y: -1.0, z: 0.20, yaw: 0.0}
  4: {x: 1.5, y: -2.5, z: 0.20, yaw: 1.5708}
```

This file is the source of truth for where markers are. Tests use it together with the ground-truth robot pose to score a dock, and a static test (S4) checks that the world file agrees with it. No runtime node reads it: the robot is not told where the markers are.

## Simulation settings

| Setting | Value |
|---|---|
| Physics step | 1 ms, real-time factor 1 |
| Render engine | `ogre2` |
| Systems | Physics, SceneBroadcaster, UserCommands, Sensors |
| Lighting | One directional light plus ambient, no shadows on the markers |
