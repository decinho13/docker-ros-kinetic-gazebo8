#!/bin/bash
set -ex

# Source ROS setuo
source  /opt/ros/kinetic/setup.bash


exec supervisord -c /app/supervisord.conf &

cd ~/c9sdk
node server.js --listen 0.0.0.0 --port 8181 -w /workspace/src &

./root/gzweb/start_gzweb.sh && gzserver






