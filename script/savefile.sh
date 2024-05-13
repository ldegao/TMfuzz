#!/bin/bash

timestamp=$(date +"%Y%m%d%H%M%S")
save_dir="../data/save/$timestamp/"

camera_dir="../data/output/camera/"
new_camera_dir="${save_dir}camera/"

errors_dir="../data/output/errors/"
new_errors_dir="${save_dir}errors/"

trace_dir="../data/output/trace/"
new_trace_dir="${save_dir}trace/"
mkdir -p "$save_dir"
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

if [ -d "$trace_dir" ]; then
  if [ "$(ls -A $trace_dir)" ]; then
    mkdir -p "$new_trace_dir"
    cp "$trace_dir"* "$new_trace_dir"
    echo "Copied files from $trace_dir to $new_trace_dir"
  else
    echo "$trace_dir is empty. Skipping..."
  fi
else
  echo "$trace_dir does not exist. Skipping..."
fi

if [ "$(ls -A $save_dir)" ]; then
  echo "Saving done"
else
  echo "There is nothing to save now"
  rm -rf $save_dir
fi

