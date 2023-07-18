#!/bin/bash

# Find the most idle GPU
# idle_gpu=$(nvidia-smi --query-gpu=index,utilization.gpu --format=csv,noheader,nounits | awk -F',' '{print $1, $2}' | sort -k2n | head -n1 | awk '{print $1}')

docker run   --name="carla-$USER" \
  -d \
  --gpus "device=1" \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.13 \
  /bin/bash -c './CarlaUE4.sh -RenderOffScreen -carla-rpc-port=4000  -quality-level=Epic && /bin/bash'
#--gpus "device=$idle_gpu"
