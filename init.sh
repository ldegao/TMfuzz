#!/bin/bash

fuzzerdata_dir="/tmp/fuzzerdata/$USER"
docker_name="carla-$USER"

# Create fuzzerdata_dir if it doesn't exist
if [[ ! -d "$fuzzerdata_dir" ]]; then
  mkdir -p "$fuzzerdata_dir"
  echo "Created directory $fuzzerdata_dir"
fi

# Check if the Docker container is not in running state
if [[ "$(docker inspect -f '{{.State.Status}}' $docker_name 2>/dev/null)" != "running" ]]; then
  echo "Docker container $docker_name is not in running state. Running stop_carla.sh..."
  "./stop_carla.sh"
fi

# Check if the Docker container doesn't exist
if [[ ! "$(docker ps -a --filter name=$docker_name --format '{{.Names}}')" ]]; then
  echo "Docker container $docker_name doesn't exist. Running run_carla.sh..."
  "./run_carla.sh"
fi

# Remove files in fuzzerdata_dir
find "$fuzzerdata_dir" -type f -delete
echo "Deleted files in $fuzzerdata_dir"
# Save videos
./savefile.sh

# Remove directories
rm -rf out-artifact
rm -rf seed-artifact

# Remove Docker containers
containers=$(docker ps -a --filter ancestor=carla-autoware:improved-df --format="{{.ID}}")
if [ -n "$containers" ]; then
  docker rm -f $containers
fi

