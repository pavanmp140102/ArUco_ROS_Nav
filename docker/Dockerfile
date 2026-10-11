# Multi-stage image for the ArUco docking project (see docs/plan/M8_docker.md).
#   base    : ROS 2 Jazzy plus every dependency declared in the package.xml files
#   dev     : base plus development tools and a non-root user matching the host
#   runtime : base plus the source tree built in Release mode
ARG ROS_IMAGE=ros:jazzy-ros-base

# Keeps only the package.xml files. The next stage copies from here, so its cache key depends
# on the manifests alone and editing source files does not rerun the dependency install.
FROM ${ROS_IMAGE} AS manifests
COPY src/ /src/
RUN find /src -type f ! -name package.xml -delete

FROM ${ROS_IMAGE} AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /ws

# Dependencies come first and only from the manifests, so a key missing from a package.xml
# fails the build here instead of going unnoticed. rosdep reads the <depend> tags; the scratch
# copy is deleted afterwards and the real source is added in the runtime stage.
COPY --from=manifests /src/ /tmp/src_manifests/
RUN echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/90norecommends \
    && apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/src_manifests --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && apt-get install -y xvfb mesa-utils \
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
# CMake reads these, so every colcon build in the dev container goes through ccache.
ENV CCACHE_DIR=/home/dev/.ccache \
    CMAKE_C_COMPILER_LAUNCHER=ccache \
    CMAKE_CXX_COMPILER_LAUNCHER=ccache
VOLUME /home/dev/.ccache
USER dev

FROM base AS runtime
COPY src/ /ws/src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
