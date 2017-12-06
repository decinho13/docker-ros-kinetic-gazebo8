#!/usr/bin/env bash

IMAGE_NAME=ros_kinetic_gazebo8
# Check args
if [ "$#" -ne 1 ]; then
  echo "usage: ./build.sh GIVEN_IMAGE_NAME"
  echo "Using default name $IMAGE_NAME"
else
	IMAGE_NAME=$1
fi

# Build the docker image
docker build -t $IMAGE_NAME .
