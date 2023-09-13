#!/bin/bash

fuzzerdata_dir="/tmp/fuzzerdata/$USER/"
docker_name="carla-$USER"

# Create fuzzerdata_dir if it doesn't exist
if [[ ! -d "$fuzzerdata_dir" ]]; then
  mkdir -p "$fuzzerdata_dir"
  echo "Created directory $fuzzerdata_dir"
fi
# Check if the Docker container is in the exited state
if [[ "$(docker inspect -f '{{.ScenarioState.Status}}' "$docker_name" 2>/dev/null)" == "exited" ]]; then
  echo "Docker container $docker_name is in exited state. Running stop_carla.sh..."
  "./stop_carla.sh"
fi

# Check if the Docker container doesn't exist
# shellcheck disable=SC2143
if [[ ! "$(docker ps -a --format '{{.Names}}' | grep -w "$docker_name")" ]]; then
  echo "Docker container $docker_name doesn't exist. Running run_carla.sh..."
  "./run_carla.sh"
fi

source savefile.sh
if [ "$(ls -A "$fuzzerdata_dir")" ]; then
  find "$fuzzerdata_dir"* -type f -delete
  echo "Deleted files in $fuzzerdata_dir"
else
  echo "$fuzzerdata_dir does not have any files. Skipping..."
fi
rm -rf out-artifact
rm -rf seed-artifact
containers=$(docker ps -a --filter ancestor=carla-autoware:improved --format="{{.ID}}"); if [ -n "$containers" ]; then docker rm -f $containers; fi
