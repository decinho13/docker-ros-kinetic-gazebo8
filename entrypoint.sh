#!/bin/bash
set -ex

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

exec supervisord -c /app/supervisord.conf &

roscore &

gzserver --verbose &

sleep 5

cd ~/c9sdk
node server.js --listen 0.0.0.0 --port 8181 -w /workspace/src &

cd ~/gzweb
npm start
