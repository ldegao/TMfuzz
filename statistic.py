#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import pdb
import shutil
import sys
import time
from datetime import datetime, timedelta
import json
import cv2

original_stdout = sys.stdout
output_file_path = f'./save/output-{time.time()}.txt'

try:
    sys.stdout = open(output_file_path, 'w')
except Exception as e:
    print(f"An error occurred: {e}")


# Function to calculate video duration using OpenCV
def video_duration(video_path):
    try:
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            return 0  # Ignore unopenable video files
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        duration_seconds = frame_count / fps
        return duration_seconds
    except Exception as e:
        print(f"Failed to get video duration for {video_path}: {e}")
        return 0


# Function to clear a folder
def clear_folder(folder):
    if os.path.exists(folder):
        for filename in os.listdir(folder):
            file_path = os.path.join(folder, filename)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                print(f"Failed to delete {file_path}: {e}")


# List of event types
event_types = ["crash", "stuck", "lane_invasion", "red", "speeding", "other"]

# Create "correct" and "error" folders if they don't exist
correct_folder = './save/correct'
error_folder = './save/error'
os.makedirs(correct_folder, exist_ok=True)
os.makedirs(error_folder, exist_ok=True)

# Clear event folders before processing
event_folders_to_clear = event_types + ["correct", "error"]
for event_type in event_folders_to_clear:
    event_folder = os.path.join('./save', event_type)
    clear_folder(event_folder)

# Initialize variables for video duration statistics
video_durations = {event_type: [] for event_type in event_types}
error_video_durations = []
correct_video_durations = []

# List of scene folders
scene_folders = [folder for folder in os.listdir('./save') if os.path.isdir(os.path.join('./save', folder))]

# 1. Count the total number of scenes
num_scenes = len(scene_folders)
print(f"Total rounds: {num_scenes}")

# 2. Calculate the total runtime
start_time = float('inf')
end_time = float('-inf')
for folder in scene_folders:
    try:
        timestamp = float(folder)
    except ValueError:
        scene_folders.remove(folder)
        continue
    if timestamp < start_time:
        start_time = timestamp
    if timestamp > end_time:
        end_time = timestamp

# total_runtime = end_time - start_time
# print(f"Total runtime: {total_runtime} seconds")


# 4. Copy videos to event folders based on event type
for scene_folder in scene_folders:
    camera_folder = os.path.join('./save', scene_folder, 'camera')
    try:
        video_files = [f for f in os.listdir(camera_folder) if f.endswith('-top.mp4')]
        json_file_name = None
        for video_file in video_files:
            gid_sid = video_file.split('-')[0]  # Extract gid:sid from video file name
            json_file_name = f"{gid_sid}.json"  # Create JSON file name
            json_file_path = os.path.join('./save', scene_folder, 'errors', json_file_name)

        # Now, we have the JSON file path (json_file_path) that corresponds to these videos
        for video_file in video_files:
            source_video_path = os.path.join(camera_folder, video_file)
            destination_name = f"{scene_folder}-{video_file}"  # New video file name

            # Check if the corresponding JSON file has event information
            if os.path.exists(json_file_path):
                with open(json_file_path, 'r') as json_file:
                    data = json.load(json_file)
                    events = data.get("events", {})
                    for event_type, event_value in events.items():
                        if event_value:
                            event_folder = os.path.join('./save', event_type)
                            os.makedirs(event_folder, exist_ok=True)
                            destination = os.path.join(event_folder, destination_name)
                            video_dur = video_duration(source_video_path)
                            if video_dur >= 1:
                                # Copy the video file to the event folder
                                # Comment out the following line to disable video copying
                                shutil.copy2(source_video_path, destination)
                                video_durations[event_type].append(video_dur)
                            else:
                                error_video_durations.append(video_dur)
                            break  # Move to the next video
            else:
                # If no event information, classify the video as "correct"
                os.makedirs(correct_folder, exist_ok=True)
                destination = os.path.join(correct_folder, destination_name)
                video_dur = video_duration(source_video_path)
                if video_dur >= 1:
                    # Copy the video file to the "correct" folder
                    # Comment out the following line to disable video copying
                    shutil.copy2(source_video_path, destination)
                    correct_video_durations.append(video_dur)
                else:
                    error_video_durations.append(video_dur)

    except FileNotFoundError:
        continue

# 5. Calculate and display statistics for video durations
for event_type in event_types:
    event_video_durations = video_durations[event_type]
    if event_video_durations:
        max_duration = max(event_video_durations)
        avg_duration = sum(event_video_durations) / len(event_video_durations)
        total_duration = sum(event_video_durations)
        print(f"Total videos in {event_type}: {len(event_video_durations)}")
        print(f"Max duration in {event_type}: {max_duration:.2f} seconds")
        print(f"Average duration in {event_type}: {avg_duration:.2f} seconds")
        print(f"Total duration in {event_type}: {total_duration:.2f} seconds")
    else:
        print(f"Event folder {event_type} does not exist.")

# Statistics for videos without corresponding JSON files in "correct" folder
correct_video_durations = [d for d in correct_video_durations if d >= 1]
if correct_video_durations:
    max_duration_correct = max(correct_video_durations)
    avg_duration_correct = sum(correct_video_durations) / len(correct_video_durations)
    total_duration_correct = sum(correct_video_durations)
    print(f"Total videos in correct: {len(correct_video_durations)}")
    print(f"Max duration in correct: {max_duration_correct:.2f} seconds")
    print(f"Average duration in correct: {avg_duration_correct:.2f} seconds")
    print(f"Total duration in correct: {total_duration_correct:.2f} seconds")

# Statistics for videos with less than 1-second duration in "error" folder
error_video_durations = [d for d in error_video_durations if d < 1]
if error_video_durations:
    max_duration_error = max(error_video_durations)
    avg_duration_error = sum(error_video_durations) / len(error_video_durations)
    total_duration_error = sum(error_video_durations)
    print(f"Total videos in error (duration < 1 second): {len(error_video_durations)}")
    print(f"Max duration in error: {max_duration_error:.2f} seconds")
    print(f"Average duration in error: {avg_duration_error:.2f} seconds")
    print(f"Total duration in error: {total_duration_error:.2f} seconds")

sys.stdout = original_stdout
print("Script execution completed.")
