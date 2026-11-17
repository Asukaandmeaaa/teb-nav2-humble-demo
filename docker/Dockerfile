FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-cv-bridge ros-humble-tf2-tools \
    ros-humble-libg2o libeigen3-dev libsuitesparse-dev libboost-all-dev \
    python3-colcon-common-extensions python3-numpy python3-yaml python3-matplotlib python3-pil \
    liburdfdom-tools mesa-utils && rm -rf /var/lib/apt/lists/*
ENV ROS_DOMAIN_ID=87
ENV ROS_LOCALHOST_ONLY=1
ENV GAZEBO_MODEL_DATABASE_URI=""
WORKDIR /workspace
ARG SIM_UID=1000
ARG SIM_GID=1000
RUN groupadd --gid ${SIM_GID} sim && useradd --uid ${SIM_UID} --gid ${SIM_GID} --create-home sim
ENV USER=sim LOGNAME=sim
