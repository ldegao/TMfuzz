#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import json

# Specify the path for data saving
data_save_path = '../data/save'

# Create a target folder if it does not exist
def create_folder_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path, exist_ok=True)

# Copy and rename the video to the specified folder to avoid filename conflicts
def copy_and_rename_video(source_video_path, destination_folder, scene_folder):
    video_name = os.path.basename(source_video_path)
    new_video_name = f"{scene_folder}-{video_name}"
    destination_video_path = os.path.join(destination_folder, new_video_name)
    shutil.copy2(source_video_path, destination_video_path)

# Main function
def main():
    event_count = {event: 0 for event in ["crash", "stuck", "lane_invasion", "red", "speeding", "other", "correct", "error"]}
    with open('../data/save/output.txt', 'w') as output_file:
        # Iterate through all scene folders
        for scene_folder in os.listdir(data_save_path):
            scene_folder_path = os.path.join(data_save_path, scene_folder)
            if os.path.isdir(scene_folder_path):
                camera_folder = os.path.join(scene_folder_path, 'camera')
                errors_folder = os.path.join(scene_folder_path, 'errors')

                # Iterate through all video files
                for video_file in os.listdir(camera_folder):
                    if video_file.endswith('-top.mp4') or video_file.endswith('-front.mp4'):
                        video_path = os.path.join(camera_folder, video_file)
                        gid_sid = video_file.split('-')[0]  # Extract gid:sid
                        json_file_name = f"{gid_sid}.json"
                        json_file_path = os.path.join(errors_folder, json_file_name)

                        # Check if the corresponding JSON file exists
                        if os.path.exists(json_file_path):
                            with open(json_file_path, 'r') as json_file:
                                data = json.load(json_file)
                                events = data.get("events", {})

                                # Copy the video to the respective event folder based on event type
                                for event_type, event_value in events.items():
                                    if event_value:
                                        event_folder = os.path.join(data_save_path, event_type)
                                        create_folder_if_not_exists(event_folder)
                                        copy_and_rename_video(video_path, event_folder, scene_folder)
                                        event_count[event_type] += 0.5
                                        break  # Process the next video file
                        else:
                            # Classify as an error if JSON file is missing
                            correct_folder = os.path.join(data_save_path, "correct")
                            create_folder_if_not_exists(correct_folder)
                            copy_and_rename_video(video_path, correct_folder, scene_folder)
                            event_count["correct"] += 0.5

        # Display the count statistics
        for event, count in event_count.items():
            output_file.write(f"Total videos in {event}: {count}\n")
            print(f"Total videos in {event}: {count}")

if __name__ == "__main__":
    main()
