#!/bin/bash

# 自动从camera目录下视频文件名提取timestamp作为save_dir名
camera_dir="../data/output/camera"
first_video=$(ls $camera_dir/*_front.mp4 2>/dev/null | head -n 1)
if [[ -z "$first_video" ]]; then
  echo "No video found in $camera_dir, fallback to current time."
  timestamp=$(date +"%Y%m%d%H%M%S")
else
  # 提取形如2025-07-16-13-45-43的部分
  basename=$(basename "$first_video")
  timestamp=$(echo "$basename" | grep -oE '^[0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}')
  # 如果没提取到，fallback
  if [[ -z "$timestamp" ]]; then
    timestamp=$(date +"%Y%m%d%H%M%S")
  fi
fi
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

