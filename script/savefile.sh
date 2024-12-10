#!/bin/bash

# Create a timestamp for the save directory
timestamp=$(date +"%Y%m%d%H%M%S")
save_dir="../data/save/$timestamp/"

# Define source and target directories
camera_dir="../data/output/camera/"
new_camera_dir="${save_dir}camera/"

errors_dir="../data/output/errors/"
new_errors_dir="${save_dir}errors/"

npc_dir="../data/output/time_record/"
new_npc_dir="${save_dir}time_record/"

queue_dir="../data/output/queue/"
new_queue_dir="${save_dir}queue/"

# Create the save directory
mkdir -p "$save_dir"

# Copy camera files
if [ -d "$camera_dir" ]; then
  if [ "$(ls -A $camera_dir)" ]; then
    mkdir -p "$new_camera_dir"
    cp "$camera_dir"* "$new_camera_dir"
    echo "Copied files from $camera_dir to $new_camera_dir"
  else
    echo "$camera_dir is empty. Skipping..."
  fi
else
  echo "$camera_dir does not exist. Skipping..."
fi

# Copy errors files
if [ -d "$errors_dir" ]; then
  if [ "$(ls -A $errors_dir)" ]; then
    mkdir -p "$new_errors_dir"
    cp "$errors_dir"* "$new_errors_dir"
    echo "Copied files from $errors_dir to $new_errors_dir"
  else
    echo "$errors_dir is empty. Skipping..."
  fi
else
  echo "$errors_dir does not exist. Skipping..."
fi

# Copy NPC files
if [ -d "$npc_dir" ]; then
  if [ "$(ls -A $npc_dir)" ]; then
    mkdir -p "$new_npc_dir"
    cp "$npc_dir"* "$new_npc_dir"
    echo "Copied files from $npc_dir to $new_npc_dir"
  else
    echo "$npc_dir is empty. Skipping..."
  fi
else
  echo "$npc_dir does not exist. Skipping..."
fi

# Copy queue files
if [ -d "$queue_dir" ]; then
  if [ "$(ls -A $queue_dir)" ]; then
    mkdir -p "$new_queue_dir"
    cp "$queue_dir"* "$new_queue_dir"
    echo "Copied files from $queue_dir to $new_queue_dir"
  else
    echo "$queue_dir is empty. Skipping..."
  fi
else
  echo "$queue_dir does not exist. Skipping..."
fi

# Check if save_dir has any files
if [ "$(ls -A $save_dir)" ]; then
  echo "Saving done"
else
  echo "There is nothing to save now"
  rm -rf "$save_dir"
fi

