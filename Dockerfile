# Dockerfile based on
# - https://github.com/gandrein/docker_ros_kinetic_cuda9/Dockerfile
# and Gazebo instructions at
# http://gazebosim.org/tutorials/?tut=ros_wrapper_versions

FROM nvidia/cuda:9.0-devel-ubuntu16.04

MAINTAINER Andrei Gherghescu <gandrein@gmail.com>

LABEL Description="ROS-Kinetic-Full-Desktop & Gazebo 8 with CUDA 9 support for Ubuntu 16.04" Version="1.0"

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


# ---------------------------------- Gazebo 8  -----------------------------
# Setup osrfoundation repository keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# Add osrfoundation repository to sources.list
RUN . /etc/os-release \
    && . /etc/lsb-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/gazebo-latest.list

# Remove Gazebo installed with ROS-Kinetic full
RUN sudo apt-get purge gazebo* -y
RUN sudo apt-get update -y && sudo apt-get upgrade -y && sudo apt-get install -y \
	gazebo8 \
	ros-kinetic-gazebo8-ros-pkgs \
	ros-kinetic-gazebo8-ros-control \
	&& apt-get clean

RUN apt-get install -y ros-kinetic-moveit



RUN nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log & && DISPLAY=:1.0 && export DISPLAY

RUN sudo apt install -y libjansson-dev nodejs npm nodejs-legacy libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
RUN cd ~; hg clone https://bitbucket.org/osrf/gzweb && cd ~/gzweb && hg up gzweb_1.4.0 && npm run deploy --- -m



# Setup environment
# Expose port
EXPOSE 11345 8080 7000
COPY . /app

COPY ./entrypoint_setup.sh /

ENTRYPOINT ["app/entrypoint.sh"]
CMD [/bin/bash"]
