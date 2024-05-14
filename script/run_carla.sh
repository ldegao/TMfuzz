#!/bin/bash

# Find the most idle GPU
idle_gpu=0
port=4000
carla_cmd="./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=$port -quality-level=Epic && /bin/bash"

docker run --name="carla-$USER" \
  -d \
  --gpus "device=$idle_gpu" \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.13 \
  $carla_cmd
