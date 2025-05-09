#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import re
import sys
import shutil

try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=4000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--start',
        metavar='S',
        default=0.0,
        type=float,
        help='starting time (default: 0.0)')
    argparser.add_argument(
        '-d', '--duration',
        metavar='D',
        default=0.0,
        type=float,
        help='duration (default: 0.0)')
    argparser.add_argument(
        '-f', '--recorder-filename',
        metavar='F',
        default="2025-04-22-19-58-17.log",
        help='recorder filename')
    argparser.add_argument(
        '-c', '--camera',
        metavar='C',
        default=0,
        type=int,
        help='camera follows an actor (ex: 82)')
    argparser.add_argument(
        '-x', '--time-factor',
        metavar='X',
        default=1.0,
        type=float,
        help='time factor (default 1.0)')
    argparser.add_argument(
        '-i', '--ignore-hero',
        action='store_true',
        help='ignore hero vehicles')
    args = argparser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        log_name = args.recorder_filename
        src_log_path = os.path.abspath("../data/"+log_name)
        dest_dir = os.path.expanduser("~/carla_data")
        dest_log_path = os.path.join(dest_dir, log_name)

        if not os.path.exists(dest_dir):
            os.makedirs(dest_dir)

        if os.path.exists(dest_log_path):
            os.remove(dest_log_path)
        shutil.copy(src_log_path, dest_log_path)
        print(f"[INFO] Copyed log file to {dest_log_path}")

        client.set_replayer_time_factor(args.time_factor)

        actors_info = client.show_recorder_file_info(log_name, True)
        hero_id = None


        for line in actors_info.split("\n"):
            if "vehicle." in line:
                parts = line.split()
                print(parts)
                hero_id = int(re.findall(r'\d+', parts[1])[0])
                break

        if hero_id is None:
            hero_id = 0
            print("[WARN] No hero vehicle found, using follow_id = 0.")

        print(client.replay_file(log_name, args.start, args.duration, hero_id))

    finally:
        pass



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
