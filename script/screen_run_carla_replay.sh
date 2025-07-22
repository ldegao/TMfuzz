#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

xhost +si:localuser:$USER

VOLUMES="--volume=$XSOCK:$XSOCK:rw \
         --volume=$XAUTH:$XAUTH:rw \
         --volume=/home/$USER/carla_data_replay:/home/carla/.config/Epic/CarlaUE4/Saved:rw"

docker run --name="carla-replay-$USER" \
  -d --rm \
  -p 6000-6002:6000-6002 \
  $VOLUMES \
  --privileged \
  --runtime=nvidia \
  --gpus 1 \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$XAUTH \
  carlasim/carla:0.9.13 \
  /bin/bash -c './CarlaUE4.sh -ResX=1920 -ResY=1080 -windowed -carla-rpc-port=6001 -quality-level=Epic' 