ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO
ARG MUJOCO_VERSION=3.0.1
ARG BRANCH=humble-devel

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip wget
ENV SHELL /bin/bash

RUN pip3 install xacro
# install mujoco
ENV MUJOCO_VERSION $MUJOCO_VERSION
RUN apt-get install -y libglfw3-dev
RUN wget https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz
RUN mkdir -p /home/mujoco
RUN tar -xzf mujoco-3.0.1-linux-x86_64.tar.gz -C "/home/mujoco"

RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src/ \
    && git clone -b $BRANCH https://github.com/sangteak601/mujoco_ros2_control.git \
    && rosdep fix-permissions && rosdep update \
    && rosdep install rosdep install --from-paths ./ --ignore-src -y

RUN echo 'export MUJOCO_DIR=/home/mujoco/mujoco-$MUJOCO_VERSION' >> ~/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

CMD ["/bin/bash"]