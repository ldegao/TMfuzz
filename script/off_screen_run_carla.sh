#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

xhost +si:localuser:$USER

VOLUMES="--volume=$XSOCK:$XSOCK:rw \
         --volume=$XAUTH:$XAUTH:rw \
         --volume=/home/linshenghao/carla_data/:/home/carla/.config/Epic/CarlaUE4/Saved:rw"

docker run --name="carla-$USER" \
  -d --rm \
  -p 4000-4002:4000-4002 \
  $VOLUMES \
  --privileged \
  --runtime=nvidia \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$XAUTH \
  carlasim/carla:0.9.13 \
  /bin/bash -c './CarlaUE4.sh -RenderOffScreen -carla-rpc-port=4000 -quality-level=Epic'

