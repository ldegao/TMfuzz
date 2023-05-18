#!/bin/bash

docker run --privileged --gpus all --name="carla-$USER" \
  -d \
  --privileged \
  --gpus all \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.13 \
  /bin/bash -c './CarlaUE4.sh -RenderOffScreen -quality-level=Epic && /bin/bash'

