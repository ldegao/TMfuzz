#!/bin/bash

# Create a timestamp for the save directory
timestamp=$(date +"%Y%m%d%H%M%S")
save_dir="../data/save/$timestamp/"

declare -A dir_map=(
  ["../data/output/camera"]="camera"
  ["../data/output/errors"]="errors"
  ["../data/output/trace"]="trace"
  ["../data/output/time_record"]="time_record"
  ["../data/output/queue"]="queue"
  ["../data/output/replay_pic"]="replay_pic"
)

mkdir -p "$save_dir"

for src_dir in "${!dir_map[@]}"; do
  dest_subdir="${dir_map[$src_dir]}"
  dest_dir="$save_dir/$dest_subdir"

  if [[ -d "$src_dir" && $(ls -A "$src_dir") ]]; then
    mkdir -p "$dest_dir"
    cp -r "$src_dir"/* "$dest_dir"/
    echo "Copied files from $src_dir to $dest_dir"
  else
    echo "$src_dir does not exist or is empty. Skipping..."
  fi
done

log_files=( ../data/output/*.log )
if [[ -e "${log_files[0]}" ]]; then
  mkdir -p "$save_dir/logs"
  mv ../data/output/*.log "$save_dir/logs/"
  echo "Moved .log files to $save_dir/logs/"
else
  echo "No .log files to move."
fi


if [[ $(find "$save_dir" -type f | wc -l) -gt 0 ]]; then
  echo "Saving done in $save_dir"
else
  echo "There is nothing to save now. Removing $save_dir"
  rm -rf "$save_dir"
fi

