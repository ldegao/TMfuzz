#!/bin/bash

# Find the most idle GPU
idle_gpu=0
port=2000
# Set the port based on the current user
case $USER in
    linshenghao)
        port=4000
        idle_gpu=1
        ;;
    chenpansong)
        port=5000
        idle_gpu=1
        ;;
    *)
        port=2000
        ;;
esac
carla_cmd="./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=$port -quality-level=Epic && /bin/bash"

docker run --name="carla-$USER" \
  -d \
  --gpus "device=$idle_gpu" \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.13 \
  $carla_cmd
