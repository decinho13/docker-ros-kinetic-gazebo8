#!/bin/bash
set -ex

# Source ROS setuo
source  /opt/ros/kinetic/setup.bash


exec supervisord -c /opt/app-root/bin/supervisord.conf &

cd /opt/app-root/bin/c9sdk
npm install pty.js
node server.js --listen 0.0.0.0 --port 8181 -w /opt/app-root/bin
