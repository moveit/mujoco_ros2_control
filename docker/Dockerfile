FROM ros:jazzy-ros-base@sha256:9619dc0891d399c0357e62721af5f7e7eeb0303e878562ba03d75d52151749fe

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO=jazzy
ARG MUJOCO_VERSION=3.3.6
ARG CPU_ARCH=x86_64

ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV CPU_ARCH=${CPU_ARCH}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MUJOCO_DIR}/lib

RUN apt-get update && apt-get install -y \
    python3-pip \
    wget \
    git \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libosmesa6-dev \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${MUJOCO_DIR}
RUN wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz \
    && tar -xzf "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -C $(dirname "${MUJOCO_DIR}") \
    && rm "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz"
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MUJOCO_DIR}/lib

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS="/root/ros2_ws"
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

COPY . ./src

COPY mujoco_ros2_control_demos/scripts/requirements.txt .

RUN --mount=type=cache,target=/root/.cache/pip \
    pip install \
    --break-system-packages \
    --ignore-installed \
    -r requirements.txt

RUN . /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

RUN echo "alias balance_bot='source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build && source install/setup.bash && ros2 launch mujoco_ros2_control_demos balance_bot.launch.py'" >> ~/.bashrc