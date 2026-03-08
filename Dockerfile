# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    ros-humble-slam-toolbox \	
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bash
RUN apt-get update && apt-get install -y bash

SHELL ["/bin/bash", "-c"]

RUN echo "Now using bash"
RUN source /etc/profile

COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY start_stack.sh /start_stack.sh
RUN chmod 755 /ros_entrypoint.sh
RUN chmod 755 /start_stack.sh

#Add Users
ARG USERNAME=hailey
ARG UID=1000
ARG GID=1000

RUN groupadd --gid $GID $USERNAME \
 && useradd --uid $UID --gid $GID -m $USERNAME

RUN usermod -aG dialout,video,plugdev $USERNAME

USER $USERNAME

ENV ROS_WS=/ssd/ros2_ws

WORKDIR /ssd/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/start_stack.sh"]
#CMD ["ros2", "launch", "lidar_launch", "lidar_with_tf.launch.py", "serial_port:=/dev/lidar", "frame_id:=laser"]
