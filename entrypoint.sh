#!/bin/bash
set -ex

nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log &
DISPLAY=:1.0
export DISPLAY

# Source ROS setuo
source  /opt/ros/kinetic/setup.bash

# Source enviroment variables for Gazebo
source /usr/share/gazebo/setup.sh
RUN_FLUXBOX=${RUN_FLUXBOX:-yes}
RUN_XTERM=${RUN_XTERM:-yes}

case $RUN_FLUXBOX in
  false|no|n|0)
    rm -f /app/conf.d/fluxbox.conf
    ;;
esac

case $RUN_XTERM in
  false|no|n|0)
    rm -f /app/conf.d/xterm.conf
    ;;
esac

exec supervisord -c /app/supervisord.conf

roslaunch smart_grasping_sandbox smart_grasping_sandbox.launch gui:=false gzweb:=true verbose:=true &

sleep 5

cd ~/gzweb
./start_gzweb.sh &
