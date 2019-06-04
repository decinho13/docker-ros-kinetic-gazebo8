# Dockerfile based on
# - https://github.com/gandrein/docker_ros_kinetic_gazebo8/Dockerfile
# and Gazebo instructions at
# http://gazebosim.org/tutorials?tut=install_ubuntu#Alternativeinstallation:step-by-step

FROM nvidia/cuda:10.0-devel-ubuntu16.04

# ------------------------------------------ Install required (&useful) packages --------------------------------------
RUN apt-get update && apt-get install -y \
software-properties-common python-software-properties \
lsb-release \
mesa-utils \
wget \
curl gdb\
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


# Install Gazebo 9 Simulator with ros interfaces
RUN sudo apt-get update -y && sudo apt-get upgrade -y && sudo apt-get install -y \
	gazebo9 \
	ros-kinetic-gazebo9-ros-pkgs \
	ros-kinetic-gazebo9-ros-control \
	&& apt-get clean
	
# Install state of the art ros tools and other neccesary tools
RUN apt-get install -y ros-kinetic-moveit \
	ros-kinetic-teleop-twist-keyboard \
	ros-kinetic-map-server \
	ros-kinetic-fake-localization \
	ros-kinetic-ros-control ros-kinetic-ros-controllers \
    	python-rospkg ros-kinetic-teb-local-planner ros-kinetic-ackermann-msgs \
    	ros-kinetic-effort-controllers ros-kinetic-joy \
    	ros-kinetic-tf2-sensor-msgs python-rosinstall ros-kinetic-geometry-tutorials ros-kinetic-rosbash \
	ros-kinetic-rqt-tf-tree \
	ros-kinetic-rosserial-arduino ros-kinetic-rosserial-embeddedlinux ros-kinetic-rosserial-windows \
	ros-kinetic-rosserial-server ros-kinetic-rosserial-python \
	ros-kinetic-openni-camera ros-kinetic-joystick-drivers ros-kinetic-navigation ros-kinetic-industrial-core
	
#Install dependencies for Cloud9
RUN sudo apt install -y libjansson-dev nodejs npm nodejs-legacy libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
 
# Setup demo environment variables for display with noVNC
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
    psmisc gdb\
    && rm -rf /var/lib/apt/lists/*   

# Expose port
EXPOSE 11345 7000 7681 8181 11311

# Install Cloud9
USER root
ENV APP_ROOT=/opt/app-root
ENV PATH=${APP_ROOT}/bin:${PATH} HOME=${APP_ROOT}
COPY . ${APP_ROOT}/bin/
RUN cd ${APP_ROOT}/bin/ && \
    git clone git://github.com/c9/core.git c9sdk && \
    cd c9sdk && \
    scripts/install-sdk.sh && \
    sed -i -e 's_127.0.0.1_0.0.0.0_g' ${APP_ROOT}/bin//c9sdk/configs/standalone.js
    
# Change User permissions to user 10001, so it is deployable in openshift as in 
# https://docs.openshift.com/container-platform/3.3/creating_images/guidelines.html#openshift-container-platform-specific-guidelines
RUN chmod -R u+x ${APP_ROOT}/bin && \
    chgrp -R 0 ${APP_ROOT} && \
    chmod -R g=u ${APP_ROOT} /etc/passwd
RUN cd ${APP_ROOT}/bin/ && mkdir share
RUN chown -R 10001 ${APP_ROOT}/bin/share
USER 10001
WORKDIR ${APP_ROOT}
#Entrypoint for Openshift Version
ENTRYPOINT [ "uid_entrypoint" ]
CMD ["bash","entrypoint.sh"]

