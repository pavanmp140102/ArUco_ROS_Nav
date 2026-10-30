# Machine setup

What the project needs installed, what is worth adding, and what to leave alone. Written for Ubuntu 24.04 with ROS 2 Jazzy.

## Required

On a machine with ROS 2 Jazzy desktop:

```bash
sudo apt install \
  ros-dev-tools python3-colcon-common-extensions \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image ros-jazzy-ros-gz-interfaces \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros ros-jazzy-launch-testing-ament-cmake \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-lint-common \
  ros-jazzy-teleop-twist-keyboard \
  libopencv-dev libopencv-contrib-dev libeigen3-dev python3-opencv python3-yaml liburdfdom-tools
```

The other ROS packages the workspace declares (`rclcpp`, `rclcpp_action`, the message packages, `launch_ros`, the `ament` tools) come with `ros-jazzy-desktop`. The authoritative list is the `package.xml` files; to install exactly what they declare, run `rosdep install --from-paths src --ignore-src -y` from the repository root. The Docker image does this, which is how undeclared dependencies get caught.

## Worth adding

```bash
# faster rebuilds, editor formatting, static analysis
sudo apt install ccache clang-format clang-tidy

# see which GPU the simulator is using; fallback for headless tests
sudo apt install nvtop vulkan-tools xvfb

# frame debugging and controller tuning
sudo apt install ros-jazzy-rqt-tf-tree ros-jazzy-plotjuggler-ros
```

To use `ccache` with colcon, add `-DCMAKE_CXX_COMPILER_LAUNCHER=ccache` to `--cmake-args`.

## Environment this project was built on

The project was developed and tested with the versions below. They describe that setup, not requirements for yours; other versions may work but were not tried.

| Item | Used in development | Notes |
|---|---|---|
| OpenCV | Ubuntu 24.04 packages, 4.6.0 | `cv_bridge` on Jazzy links against the system OpenCV, so the detector uses the system copy too and mixing in a second OpenCV (source build, `pip install opencv-python`) can cause hard-to-diagnose crashes. The code selects the legacy or the newer ArUco API at compile time, so it also builds against newer OpenCV |
| CUDA, cuDNN, TensorRT, PyTorch | Not installed | Nothing in this project uses them. ArUco detection on a 1280x720 frame took roughly 10 ms on a CPU, well inside the 33 ms between frames |

## Graphics

On a laptop with hybrid graphics (on-demand mode), programs render on the integrated GPU by default. To run the simulator on the NVIDIA GPU:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch dockbot_bringup dock.launch.py
```

`sim.launch.py` takes a `gpu` argument (`auto`, `intel`, `nvidia` or `software`) that sets these variables, so the command above becomes `gpu:=nvidia`. The default, `auto`, uses the NVIDIA GPU when `nvidia-smi` finds one with a working driver on the host, and the integrated GPU otherwise (and always inside a container, where `nvidia-smi` can succeed without a usable OpenGL stack; the NVIDIA Docker service passes `gpu:=nvidia`), so the same command works unchanged in Docker and on machines without an NVIDIA GPU.

Measured on a laptop with an entry-level NVIDIA GPU (M5 and M8, October 2026):

| Run | Result |
|---|---|
| End-to-end suite E1 to E12, headless, hardware rendering, on the host | about 19 to 20 minutes |
| The same suite in Docker, software rendering (`LIBGL_ALWAYS_SOFTWARE=1`) | 27 minutes, every scenario passing |
| GUI demo through `sim-nvidia` | the NVIDIA GPU at about half load, 430 MiB of video memory |
| ArUco detection on a 1280x720 frame | roughly 10 ms on a CPU core |

Software rendering is therefore slower but still practical for CI.

Check what is in use:

```bash
nvidia-smi                 # driver loaded, processes on the NVIDIA GPU
glxinfo -B | grep renderer # default renderer
nvtop                      # live view of both GPUs
```

## Docker

Docker is the way to run the project on a machine without ROS 2 installed. The image design and the list of services are in [plan/M8_docker.md](plan/M8_docker.md); all five services (`test`, `test-e2e`, `sim`, `sim-nvidia`, `dev`) and `docker/run.sh` are defined.

What a machine needs:

| To do this | The host needs |
|---|---|
| Build the image, run the tests | Docker with the Compose plugin |
| Run the demo with windows | Linux with an X11 or XWayland display (or Windows 11 with WSL 2) |
| Use an NVIDIA GPU inside the container | The NVIDIA driver and the NVIDIA Container Toolkit |

On another machine with an NVIDIA GPU, add NVIDIA's apt repository for the toolkit (see their install guide), then:

```bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu:24.04 nvidia-smi    # should list the GPU
```

The toolkit is optional. Without it the container uses an Intel or AMD GPU, or software rendering.

Commands:

```bash
docker compose -f docker/compose.yaml run --rm test       # fast tests
docker compose -f docker/compose.yaml run --rm test-e2e   # Gazebo tests, headless
./docker/run.sh                                           # demo with GUI; picks the GPU service
docker compose -f docker/compose.yaml run --rm dev        # development shell, source mounted
WORLD=docking_world MARKER_ID=2 ./docker/run.sh           # choose the world and the marker to dock at
```

Graphics paths:

| Host | Service | Notes |
|---|---|---|
| Linux, Intel or AMD GPU | `sim` | Mounts `/dev/dri` and the X11 socket. `run.sh` runs `xhost +local:docker` and removes it on exit |
| Linux, NVIDIA GPU | `sim-nvidia` | Needs the NVIDIA Container Toolkit; `run.sh` picks it when the runtime is registered |
| Wayland | `sim` | Works through XWayland |
| No GPU, CI | `test-e2e` | Software rendering (`LIBGL_ALWAYS_SOFTWARE=1`), headless. Set `E2E_XVFB=1` to run under `xvfb-run` if headless EGL fails |
| Windows 11 with WSL 2 | `sim` | Run the Linux commands inside WSL; WSLg provides the display |
| macOS | `test` only | No GPU access or X server for the GUI |

With the demo running, send a goal from a second terminal:

```bash
docker compose -f docker/compose.yaml exec sim \
  ros2 action send_goal /dock dockbot_interfaces/action/Dock "{marker_id: 2}" --feedback
```

Expect the image to take 3 to 4 GB of disk.

The base image is pinned by digest in `docker/Dockerfile`: `ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617`, recorded on 2026-10-30, the image the tests below were run on. To move to a newer base, pull `ros:jazzy-ros-base`, read its digest with `docker buildx imagetools inspect ros:jazzy-ros-base`, run both test services, then replace the digest.
