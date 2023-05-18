#!/bin/bash

timestamp=$(date +"%Y%m%d%H%M%S")
save_dir="./save/$timestamp/"

camera_dir="./out-artifact/camera/"
new_camera_dir="${save_dir}camera/"

errors_dir="./out-artifact/errors/"
new_errors_dir="${save_dir}errors/"

seed_dir="./seed-artifact/"
new_seed_dir="${save_dir}seed/"
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

if [ -d "$seed_dir" ]; then
  if [ "$(ls -A $seed_dir)" ]; then
    mkdir -p "$new_seed_dir"
    cp "$seed_dir"* "$new_seed_dir"
    echo "Copied files from $seed_dir to $new_seed_dir"
  else
    echo "$seed_dir is empty. Skipping..."
  fi
else
  echo "$seed_dir does not exist. Skipping..."
fi

if [ "$(ls -A $save_dir)" ]; then
  echo "Saving done"
else
  echo "There is nothing to save now"
  rm -rf $save_dir
fi

