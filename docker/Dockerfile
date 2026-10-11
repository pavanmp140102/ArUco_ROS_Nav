# Multi-stage image for the ArUco docking project (see docs/plan/M8_docker.md).
#   base    : ROS 2 Jazzy plus every dependency declared in the package.xml files
#   dev     : base plus development tools and a non-root user matching the host
#   runtime : base plus the source tree built in Release mode
ARG ROS_IMAGE=ros:jazzy-ros-base

FROM ${ROS_IMAGE} AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /ws

# Dependencies come first and only from the manifests, so editing source reuses this layer
# and a key missing from a package.xml fails the build here instead of going unnoticed.
COPY src/ /tmp/src_manifests/
RUN find /tmp/src_manifests -type f ! -name package.xml -delete \
    && apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/src_manifests --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && apt-get install -y --no-install-recommends xvfb mesa-utils \
    && rm -rf /tmp/src_manifests /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM base AS dev
ARG UID=1000
ARG GID=1000
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        gdb ccache clang-format clang-tidy sudo \
    && rm -rf /var/lib/apt/lists/* \
    && (userdel -r ubuntu 2>/dev/null || true) \
    && groupadd --gid "${GID}" dev \
    && useradd --uid "${UID}" --gid "${GID}" -m -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev \
    && mkdir -p /home/dev/.ccache && chown -R dev:dev /home/dev /ws
ENV CCACHE_DIR=/home/dev/.ccache
VOLUME /home/dev/.ccache
USER dev

FROM base AS runtime
COPY src/ /ws/src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
