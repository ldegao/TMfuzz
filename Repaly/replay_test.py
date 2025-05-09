#!/usr/bin/env python

import glob
import os
import re
import sys
import time
import shutil
import argparse
import random

try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def main():
    parser = argparse.ArgumentParser(description="Replay CARLA log and track ego vehicle in real time")
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', default=4000, type=int)
    parser.add_argument('--recorder-filename', default="2025-04-22-19-58-17.log")
    parser.add_argument('--start', default=0.0, type=float)
    parser.add_argument('--duration', default=0.0, type=float)
    parser.add_argument('--time-factor', default=1.0, type=float)
    parser.add_argument('--interval', default=10, type=int)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)

    log_name = args.recorder_filename
    src_log_path = os.path.abspath("../data/" + log_name)
    dest_dir = os.path.expanduser("~/carla_data")
    dest_log_path = os.path.join(dest_dir, log_name)

    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
    if os.path.exists(dest_log_path):
        os.remove(dest_log_path)
    shutil.copy(src_log_path, dest_log_path)
    print(f"[INFO] Copied log file to {dest_log_path}")

    client.set_replayer_time_factor(args.time_factor)
    recorder_info = client.show_recorder_file_info(log_name, True)

    # ?? hero_id
    hero_id = None
    for line in recorder_info.splitlines():
        if "vehicle." in line:
            parts = line.split()
            hero_id = int(re.findall(r'\d+', parts[1])[0])
            break
    if hero_id is None:
        print("[WARN] No hero vehicle found, using follow_id = 0.")
        hero_id = 0

    # ????
    print(f"[INFO] Starting replay of {log_name}")
    client.replay_file(log_name, args.start, args.duration, hero_id)

    world = client.get_world()
    spectator = world.get_spectator()

    # ?? ego actor ??
    ego_actor = None
    print("[INFO] Waiting for hero actor to appear...")
    for _ in range(100):
        ego_actor = world.get_actor(hero_id)
        if ego_actor:
            break
        time.sleep(0.1)

    if not ego_actor:
        print("[ERROR] Ego actor not found in simulation.")
        return

    print(f"[INFO] Tracking hero vehicle (ID={hero_id}) every {args.interval} frames...")

    frame_count = 0

    def on_tick(snapshot):
        nonlocal frame_count
        frame_count += 1
        if frame_count % args.interval != 0:
            return

        transform = ego_actor.get_transform()
        location = transform.location
        print(f"[Frame {frame_count}] Real Ego Location: x = {location.x:.2f}, y = {location.y:.2f}")

        spectator.set_transform(
            carla.Transform(location + carla.Location(z=30), carla.Rotation(pitch=-90))
        )

    world.on_tick(on_tick)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")

    print("Done.")


if __name__ == '__main__':
    main()
