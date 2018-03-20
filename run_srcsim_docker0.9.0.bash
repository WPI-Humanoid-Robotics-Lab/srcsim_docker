#!/usr/bin/env bash

if [[ "$(docker images -q srcsim:0.9.0 2> /dev/null)" == "" ]]; then
  git clone -b 0.9.0 https://github.com/WPI-Humanoid-Robotics-Lab/srcsim_docker.git
  cd srcsim_docker
  docker build -t srcsim:0.9.0 .
  cd .. && rm -rf srcsim_docker
fi

if [[ "$(docker network ls | grep srcsim 2> /dev/null)" == "" ]]; then
  echo "creating network bridge for docker image"
  docker network create --subnet 201.1.1.0/16 --driver bridge srcsim
fi

echo "running srcsim 0.9.0 docker container"

# XAUTH=/tmp/.docker.xauth
# xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
if [ ! -f /tmp/.docker.xauth ]
then
  export XAUTH=/tmp/.docker.xauth
  xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# Use lspci to check for the presence of an nvidia graphics card
has_nvidia=`lspci | grep -i nvidia | wc -l`

# Set docker gpu parameters
if [ ${has_nvidia} -gt 0 ]
then
  # check if nvidia-modprobe is installed
  if ! which nvidia-modprobe > /dev/null
  then
    echo nvidia-docker-plugin requires nvidia-modprobe
    echo please install nvidia-modprobe
    exit -1
  fi
  # check if nvidia-docker-plugin is installed
  if curl -s http://localhost:3476/docker/cli > /dev/null
  then
    DOCKER_GPU_PARAMS=" $(curl -s http://localhost:3476/docker/cli)"
  else
    echo nvidia-docker-plugin not responding on http://localhost:3476/docker/cli
    echo please install nvidia-docker-plugin
    echo https://github.com/NVIDIA/nvidia-docker/wiki/Installation
    exit -1
  fi
else
  DOCKER_GPU_PARAMS=""
fi

DISPLAY="${DISPLAY:-:0}"

docker run --rm --name srcsim \
  -e DISPLAY=unix$DISPLAY \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e ROS_MASTER_URI=http://201.1.1.10:11311 \
  -e ROS_IP=201.1.1.10 \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "/tmp/.docker.xauth:/tmp/.docker.xauth" \
  -v /dev/log:/dev/log \
  --ulimit rtprio=99 \
  --net=srcsim \
  --ip=201.1.1.10 \
  ${DOCKER_GPU_PARAMS} \
  srcsim:0.9.0
