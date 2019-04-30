# Dockerfile based on
# - https://github.com/gandrein/docker_ros_kinetic_gazebo8/Dockerfile
# and Gazebo instructions at
# http://gazebosim.org/tutorials/?tut=ros_wrapper_versions

FROM nvidia/cuda:10.0-devel-ubuntu16.04

# ------------------------------------------ Install required (&useful) packages --------------------------------------
RUN apt-get update && apt-get install -y \
software-properties-common python-software-properties \
lsb-release \
mesa-utils \
wget \
curl \
sudo vim nano byobu \
python-rosdep python-rosinstall \
python3-pip python-pip \
build-essential socat supervisor x11vnc xvfb xterm \
net-tools iputils-ping fluxbox git novnc \
&& apt-get clean


# ---------------------------------- ROS-Kinetic Desktop Full Image -----------------------------
# Based on
# https://github.com/osrf/docker_images/blob/5399f380af0a7735405a4b6a07c6c40b867563bd/ros/kinetic/ubuntu/xenial/desktop-full/Dockerfile

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full \
 	&& rm -rf /var/lib/apt/lists/*

RUN pip install catkin_tools

# Configure ROS
RUN sudo rosdep init && sudo rosdep fix-permissions && rosdep update
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc


# ---------------------------------- Gazebo 9  -----------------------------

# Remove Gazebo installed with ROS-Kinetic full
RUN sudo apt-get purge gazebo* -y
# Setup osrfoundation repository keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# Add osrfoundation repository to sources.list
RUN . /etc/os-release \
    && . /etc/lsb-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/gazebo-latest.list



RUN sudo apt-get update -y && sudo apt-get upgrade -y && sudo apt-get install -y \
	gazebo9 \
	ros-kinetic-gazebo9-ros-pkgs \
	ros-kinetic-gazebo9-ros-control \
	&& apt-get clean

RUN apt-get install -y ros-kinetic-moveit
   

RUN sudo apt install -y libjansson-dev nodejs npm nodejs-legacy libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential

RUN cd /root && \
    git clone git://github.com/c9/core.git c9sdk && \
    cd c9sdk && \
    scripts/install-sdk.sh && \
    sed -i -e 's_127.0.0.1_0.0.0.0_g' /root/c9sdk/configs/standalone.js
# Setup demo environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1400 \
    DISPLAY_HEIGHT=768 \
    RUN_XTERM=yes \
    RUN_FLUXBOX=yes
RUN apt-get update && apt-get install -q -y \
    libboost-all-dev \
    libgts-dev \
    pkg-config \
    psmisc\
    && rm -rf /var/lib/apt/lists/*
    
# Setup environment

# Expose port
EXPOSE 11345 7000 7681 8181 11311
COPY . /app
ARG NB_USER="jovyan"
ARG NB_UID="1001"
ARG NB_GID="100"
ENV SHELL=/bin/bash \
    NB_USER=$NB_USER \
    NB_UID=$NB_UID \
    NB_GID=$NB_GID 
USER root
RUN useradd -m -s /bin/bash -N -u $NB_UID $NB_USER && usermod -aG sudo jovyan && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN chgrp -R 0 /app && \
    chmod -R g=u /app
CMD ["sudo","bash","/app/entrypoint.sh"]
RUN chmod g=u /etc/passwd
ENTRYPOINT [ "/app/uid_entrypoint" ]
USER 1001

