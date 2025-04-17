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
import time
import argparse


try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import FollowWaypointsAgent  # Custom module containing the FollowWaypointsAgent class
import carla

def main():
    # Parse command-line arguments
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=5000, type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-s', '--start', metavar='S', default=5.0, type=float,
                           help='starting time (default: 5.0)')
    argparser.add_argument('-d', '--duration', metavar='D', default=5.0, type=float,
                           help='duration (default: 5.0)')
    argparser.add_argument('-f', '--recorder-filename', metavar='F', default="test1.log",
                           help='recorder filename (test1.log)')
    argparser.add_argument('-c', '--camera', metavar='C', default=0, type=int,
                           help='camera follows an actor (ex: 82)')
    argparser.add_argument('-x', '--time-factor', metavar='X', default=1.0, type=float,
                           help='time factor (default 1.0)')
    argparser.add_argument('-i', '--ignore-hero', action='store_true',
                           help='ignore hero vehicles')
    args = argparser.parse_args()

    try:
        # Connect to the CARLA server
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)

        # Set the time factor for the replayer
        client.set_replayer_time_factor(args.time_factor)

        # Retrieve actor information from the recorder file
        actors_info = client.show_recorder_file_info(args.recorder_filename, True)
        hero_id = None
        jump = 0
        check = 0
        for line in actors_info.split("\n"):
            if "vehicle." in line:
                if jump < check:
                    jump += 1
                    continue
                parts = line.split()
                print(parts)
                hero_id = int(re.findall(r'\d+', parts[1])[0])
                break

        # Start replay using the recorder file and designate the hero actor by id
        client.replay_file(args.recorder_filename, args.start, args.duration, hero_id)
        print("Replay started with hero actor id:", hero_id)

        # Get the world and wait until the hero vehicle appears
        world = client.get_world()
        hero_vehicle = None
        timeout = 10.0  # seconds to wait for the hero actor
        start_time = time.time()
        while time.time() - start_time < timeout:
            hero_vehicle = world.get_actor(hero_id)
            if hero_vehicle is not None:
                break
            time.sleep(0.1)
        if hero_vehicle is None:
            print("Hero vehicle not found.")
            return

        # Define a series of waypoints (list of carla.Transform)
        # For demonstration, create waypoints relative to the hero's current position
        hero_transform = hero_vehicle.get_transform()
        waypoints = []
        # Add the current position as the first waypoint
        waypoints.append(hero_transform)
        # Generate additional waypoints along the X axis (modify as needed)
        for i in range(1, 4):
            new_location = carla.Location(
                hero_transform.location.x + i * 10.0,
                hero_transform.location.y,
                hero_transform.location.z
            )
            new_transform = carla.Transform(new_location, hero_transform.rotation)
            waypoints.append(new_transform)

        # Instantiate the FollowWaypointsAgent using the hero vehicle and waypoints
        follow_agent = FollowWaypointsAgent.FollowWaypointsAgent(hero_vehicle, waypoints)

        # Main loop to update hero vehicle control to follow waypoints
        while True:
            destination = follow_agent.update_destination()
            if destination is None:
                print("All waypoints reached.")
                break
            print("Heading to waypoint at location:", destination.location)
            reached = False
            while not reached:
                control = follow_agent.run_step()
                hero_vehicle.apply_control(control)
                world.tick()  # Advance the simulation
                time.sleep(0.05)
                # Check if the vehicle has reached the current waypoint
                current_location = hero_vehicle.get_location()
                target_location = destination.location
                distance = current_location.distance(target_location)
                if distance < 2.0:
                    print("Reached waypoint, distance: {:.2f}".format(distance))
                    reached = True

    finally:
        print("Script finished.")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
