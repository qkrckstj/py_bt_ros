# Ubuntu 24.04 / ROS 2 Jazzy 환경. 애플리케이션 소스는 복사하지 않고
# compose.yaml에서 사용자가 지정한 현재 체크아웃만 마운트합니다.
FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG WEBOTS_RELEASE=R2025a
ARG WEBOTS_DEB=webots_2025a_amd64.deb

RUN . /etc/os-release && test "$VERSION_ID" = "24.04"
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl build-essential git \
      python3-colcon-common-extensions python3-pytest python3-yaml \
      python3-numpy python3-opencv \
      ros-jazzy-webots-ros2 ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup \
      ros-jazzy-rviz2 ros-jazzy-tf2-geometry-msgs \
      xvfb xauth libgl1-mesa-dri mesa-utils \
    && curl --fail --location --retry 3 \
      "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_RELEASE}/${WEBOTS_DEB}" \
      --output /tmp/webots.deb \
    && apt-get install -y --no-install-recommends /tmp/webots.deb \
    && test "$(cat /usr/local/webots/resources/version.txt)" = "R2025a" \
    && rm -f /tmp/webots.deb \
    && rm -rf /var/lib/apt/lists/*

# Jazzy Webots importer가 ros2_supervisor 시작 중 Pillow를 직접 import합니다.
# 배포 패키지의 누락된 런타임 의존성을 명시적으로 설치합니다.
RUN apt-get update && apt-get install -y --no-install-recommends python3-pil \
    && rm -rf /var/lib/apt/lists/*

# Mesa 25.1 이상에서 Turing nouveau의 OpenGL은 NVK+Zink를 사용합니다.
# libgl1-mesa-dri만 있으면 Vulkan ICD가 없어 llvmpipe로 돌아갈 수 있습니다.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libvulkan1 mesa-vulkan-drivers vulkan-tools \
    && rm -rf /var/lib/apt/lists/*
LABEL io.py_bt_ros.lunar.graphics="mesa-vulkan-v1"

# 고정 버전 팀원 BT의 frontier 계산과 센서 기반 탐사 지표 출력.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-scipy python3-matplotlib fonts-noto-cjk \
    && rm -rf /var/lib/apt/lists/*
LABEL io.py_bt_ros.lunar.exploration="team-bt-v1"

ENV WEBOTS_HOME=/usr/local/webots \
    ROS2_WEBOTS_HOME=/usr/local/webots \
    USER=lunar \
    ROS_LOG_DIR=/tmp/lunar_ros_logs \
    ROS_HOME=/tmp/lunar_ros_home \
    XDG_CONFIG_HOME=/tmp/lunar_config \
    XDG_CACHE_HOME=/tmp/lunar_cache

WORKDIR /workspace/py_bt_ros
CMD ["bash"]
