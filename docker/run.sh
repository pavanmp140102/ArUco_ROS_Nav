#!/bin/bash
# Start the GUI demo in Docker: allow the container to use the display, pick the NVIDIA service
# when the NVIDIA runtime is available (else the Intel/AMD one), and bring it up.
# Usage: ./docker/run.sh    WORLD and MARKER_ID may be set in the environment.
set -euo pipefail
cd "$(dirname "$0")/.."
COMPOSE=(docker compose -f docker/compose.yaml)

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set: the demo needs a graphical session. Use the 'test' service instead." >&2
  exit 1
fi
if ! command -v xhost >/dev/null; then
  echo "xhost not found (install x11-xserver-utils); the container may not reach the display." >&2
elif xhost 2>/dev/null | grep -qi 'LOCAL:'; then
  : # access was already granted (for example by another container); leave it as it is on exit
elif xhost +local:docker >/dev/null; then
  trap 'xhost -local:docker >/dev/null' EXIT
else
  echo "xhost could not allow local containers; the GUI may not open." >&2
fi

service=sim
docker_info=$(docker info 2>/dev/null || true)  # captured first: grep -q would SIGPIPE the pipe
if grep -qi 'runtimes:.*nvidia' <<<"$docker_info"; then
  service=sim-nvidia
elif command -v nvidia-smi >/dev/null; then
  echo "NVIDIA GPU found but the NVIDIA Container Toolkit is not set up; using the Intel/AMD service."
  echo "To render on the NVIDIA GPU, install nvidia-container-toolkit (see docs/setup.md)."
fi
echo "Starting service: ${service}"
"${COMPOSE[@]}" up --build "${service}"
