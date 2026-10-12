# M8: Docker

**Goal:** anyone with Docker can build the project and run the tests and the demo with two commands, without installing ROS 2, Gazebo, or OpenCV on their own system.

**Depends on:** M0 for the first working image (it only needs a workspace that builds). The image is extended as later milestones land, and is finished after M5, when the end-to-end tests exist to run inside it.

Unlike M7, this milestone is not optional. It is numbered last only because it was added after the others; start it right after M0 so every later milestone can be checked in the container as well as on the host.

## Design

### One image definition, three stages

A single multi-stage `docker/Dockerfile`:

| Stage | Built from | Contains | Used for |
|---|---|---|---|
| `base` | `ros:jazzy-ros-base` | Every dependency declared in the `package.xml` files, installed by `rosdep`. Plus `xvfb` and Mesa utilities | Shared foundation |
| `dev` | `base` | Compiler cache, `gdb`, `clang-format`, `clang-tidy`, a non-root user whose user ID matches the host's | Day-to-day development with the source tree mounted from the host |
| `runtime` | `base` | The source copied in and built with `colcon build` in Release mode | Running the demo and the tests on another machine; continuous integration |

Reasons for these choices:

- **`ros:jazzy-ros-base` plus `rosdep`**, not a large desktop image. The official `ros` image is published for both x86-64 and ARM64. Installing through `rosdep` means the image contains exactly what the manifests declare, so a missing dependency in a `package.xml` shows up as a failed image build instead of surviving unnoticed because the host happened to have the package.
- **Dependencies before source.** The Dockerfile copies only the `package.xml` files, runs `rosdep`, and copies the rest of the source afterwards. Editing code then reuses the cached dependency layer, and a rebuild takes seconds.
- **Same OpenCV as the host.** Ubuntu 24.04 inside the image provides OpenCV 4.6 from apt, matching `cv_bridge`. Nothing is built from source or installed with pip. See [../setup.md](../setup.md).
- **One container runs everything** (simulator, bridges, both nodes, RViz). ROS discovery between containers is a common source of trouble and nothing here needs it.

### Compose services

`docker/compose.yaml` defines what a user actually runs:

| Service | Stage | What it does | Graphics |
|---|---|---|---|
| `test` | `runtime` | Runs the fast test suite, then prints results. Exit code reflects pass or fail | None needed |
| `test-e2e` | `runtime` | Runs the Gazebo end-to-end suite headless | Software rendering by default |
| `sim` | `runtime` | Launches the full demo with the Gazebo GUI and RViz | Host display, Intel or AMD GPU through `/dev/dri` |
| `sim-nvidia` | `runtime` | Same as `sim`, on an NVIDIA GPU | Host display, NVIDIA GPU. Needs the NVIDIA Container Toolkit on the host |
| `dev` | `dev` | Interactive shell with the repository mounted at `/ws` | Same as `sim` |

### Graphics inside a container

This is the part most likely to differ between machines, so every case gets an explicit path.

| Host | How the simulator renders | Notes |
|---|---|---|
| Linux, Intel or AMD GPU | Mount `/dev/dri`, share the X11 socket and `DISPLAY` | The default `sim` service |
| Linux, NVIDIA GPU | `--gpus all` with `NVIDIA_DRIVER_CAPABILITIES=all` | Host needs `nvidia-container-toolkit`. Verify with `docker run --rm --gpus all ubuntu:24.04 nvidia-smi` |
| Linux on Wayland | Same as above through XWayland | Most desktops provide XWayland by default |
| No GPU, or a CI runner | Mesa software rendering (`LIBGL_ALWAYS_SOFTWARE=1`), headless | Slow but correct. Camera rate drops, which is why controller timing uses simulation time |
| Windows 11 with WSL 2 | WSLg provides the display and GPU | Run the Linux commands inside WSL |
| macOS | Tests only | Docker on macOS has no GPU access and no native X server; the GUI demo is not supported there |

Showing windows on the host requires allowing local containers to use the display (`xhost +local:docker`). A helper script, `docker/run.sh`, does this, detects whether an NVIDIA runtime is available, and picks `sim` or `sim-nvidia` accordingly.

## Sub-tasks

### Image

- [x] `.dockerignore`: `build/`, `install/`, `log/`, `.git/`, `docs/images/`, editor folders.
- [x] `docker/Dockerfile`, stage `base`: `ros:jazzy-ros-base`, copy only `src/**/package.xml`, `rosdep install --from-paths src --ignore-src -y`, add `xvfb` and `mesa-utils`, clean the apt lists.
- [x] Stage `runtime`: copy `src/`, `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`, keep `build/` so tests can run.
- [x] Stage `dev`: development tools, non-root user created from `UID` and `GID` build arguments, `ccache` directory as a volume.
- [x] `docker/entrypoint.sh`: sources `/opt/ros/jazzy/setup.bash` and the workspace's `install/setup.bash` when present, then runs the given command.
- [ ] Pin the base image by digest once the image is known to work, and record the digest and date in [../setup.md](../setup.md).

### Compose and helper

- [x] `docker/compose.yaml` with the five services above. Shared settings in one anchor: `ROS_DOMAIN_ID`, `ipc: host`, the X11 socket, `DISPLAY`, `QT_X11_NO_MITSHM=1`.
- [x] `test` command: `colcon test` on the non-simulator packages, then `colcon test-result --verbose`.
- [x] `test-e2e` command: the `dockbot_bringup` tests with `headless:=true`, under software rendering, with `xvfb-run` as the fallback if headless EGL is unavailable.
- [x] `sim` and `sim-nvidia` commands: `ros2 launch dockbot_bringup dock.launch.py`, passing through `WORLD` and `MARKER_ID` environment variables.
- [x] `docker/run.sh`: display permission, GPU detection, service selection, and a clear message when the NVIDIA toolkit is missing.

### Making the code container-friendly

- [x] `sim.launch.py`: the `gpu` argument gains a third value, `software`, which sets `LIBGL_ALWAYS_SOFTWARE=1`.
- [x] Confirm that no launch file, test, or config uses an absolute host path or assumes a home directory. Everything is found through `ament_index`.
- [ ] End-to-end test time limits are in simulation time, so slow software rendering does not cause false failures.

### Documentation

- [x] Docker section in [../setup.md](../setup.md): prerequisites, the commands, the graphics table, how to install the NVIDIA Container Toolkit.
- [x] README quick start gains a "With Docker" variant next to the native one.
- [x] The README lists the Docker commands.

### Continuous integration (optional)

- [ ] GitHub Actions workflow: build the `runtime` stage, run the `test` service on every push; run `test-e2e` on pull requests to `main`.
- [ ] Cache image layers between runs.

## Files created

```
.dockerignore
docker/Dockerfile
docker/compose.yaml
docker/entrypoint.sh
docker/run.sh
.github/workflows/ci.yml        (optional)
```

## Gate

Tests DK1 to DK9 in [../test_plan.md](../test_plan.md). In short:

- The image builds from a fresh clone with no host ROS installation involved.
- The fast tests pass inside the container with no display and no GPU.
- The end-to-end tests pass inside the container under software rendering.
- The GUI demo runs and docks through the `sim` service, and through `sim-nvidia` with rendering on the NVIDIA GPU.
- Files created in the mounted source tree by the `dev` service belong to the host user.

## What a user on another machine will type

```bash
git clone <repo> && cd ArUco_ROS_Nav

# run the tests
docker compose -f docker/compose.yaml run --rm test

# run the demo with GUI (Linux)
./docker/run.sh
# in a second terminal
docker compose -f docker/compose.yaml exec sim \
  ros2 action send_goal /dock dockbot_interfaces/action/Dock "{marker_id: 2}" --feedback
```

## Notes

- Expect an image of roughly 3 to 4 GB: Gazebo, RViz, and their dependencies account for almost all of it.
- If `rosdep` cannot resolve a key, fix the `package.xml`; do not add a manual `apt install` line to the Dockerfile to work around it.
- Software rendering of a 1280x720 camera at 30 Hz is CPU heavy. If the end-to-end suite is too slow in CI, lower the camera rate for CI through a launch argument instead of loosening test tolerances.
- ARM64 (for example a Raspberry Pi 5 or an Apple-silicon Linux VM) should build from the same Dockerfile, since the base image and all apt packages exist for it, but it is untested. DK8 covers it if such a machine is available.
- Do not put the NVIDIA settings in the default services. A machine without the NVIDIA runtime refuses to start a container that requests it.
