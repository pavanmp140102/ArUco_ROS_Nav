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
  libopencv-dev libopencv-contrib-dev libeigen3-dev python3-opencv
```

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

Check what is in use:

```bash
nvidia-smi                 # driver loaded, processes on the NVIDIA GPU
glxinfo -B | grep renderer # default renderer
nvtop                      # live view of both GPUs
```

## Docker

Docker is the way to run the project on a machine without ROS 2 installed. The image design and the list of services are in [plan/M8_docker.md](plan/M8_docker.md); the image and the `test` and `dev` services exist; `sim`, `sim-nvidia` and `test-e2e` come with M1 and M5.

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

Commands (`test-e2e`, `sim` and `run.sh` arrive with later milestones):

```bash
docker compose -f docker/compose.yaml run --rm test       # fast tests
docker compose -f docker/compose.yaml run --rm test-e2e   # Gazebo tests, headless
./docker/run.sh                                           # demo with GUI; picks the GPU service
docker compose -f docker/compose.yaml run --rm dev        # development shell, source mounted
```

Expect the image to take 3 to 4 GB of disk.
