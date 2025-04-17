#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import logging

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=5000, type=int, help='TCP port to listen to (default: 5000)')
    argparser.add_argument('-n', '--number-of-vehicles', metavar='N', default=10, type=int, help='number of vehicles (default: 10)')
    argparser.add_argument('-d', '--delay', metavar='D', default=2.0, type=float, help='delay in seconds between spawns (default: 2.0)')
    argparser.add_argument('--safe', action='store_true', help='avoid spawning vehicles prone to accidents')
    argparser.add_argument('-f', '--recorder_filename', metavar='F', default="test1.log", help='recorder filename (default: test1.log)')
    argparser.add_argument('-t', '--recorder_time', metavar='T', default=0, type=int, help='recorder duration (auto-stop)')
    args = argparser.parse_args()

    actor_list = []
    actor_ids = []
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)
        client.reload_world()

        # Wait until the world is ready
        world = None
        for _ in range(100):
            try:
                world = client.get_world()
                if world.get_map():
                    break
            except RuntimeError:
                pass
            time.sleep(0.1)

        if world is None or not world.get_map():
            raise RuntimeError("World failed to reload.")

        tm = client.get_trafficmanager(8000)
        map = world.get_map()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        spawn_points = map.get_spawn_points()
        random.shuffle(spawn_points)

        print('Found %d spawn points.' % len(spawn_points))
        count = args.number_of_vehicles

        print("Recording on file: %s" % client.start_recorder(args.recorder_filename, True))

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

        # Choose the first point as hero spawn
        hero_transform = spawn_points[0]
        hero_bp = random.choice(blueprints)
        if hero_bp.has_attribute('color'):
            color = random.choice(hero_bp.get_attribute('color').recommended_values)
            hero_bp.set_attribute('color', color)
        hero_bp.set_attribute('role_name', 'hero')

        hero_vehicle = world.try_spawn_actor(hero_bp, hero_transform)
        if hero_vehicle is None:
            raise RuntimeError("Failed to spawn hero vehicle")
        hero_vehicle.set_autopilot(True, tm.get_port())
        actor_list.append(hero_vehicle.id)
        print('Spawned hero vehicle: ID=%d' % hero_vehicle.id)

        # Collect nearby spawn points from connected roads until we have enough
        hero_wp = map.get_waypoint(hero_transform.location)
        visited_road_ids = set()
        road_ids_to_check = [hero_wp.road_id]
        extended_road_ids = set()

        while len(extended_road_ids) < 15 and road_ids_to_check:
            current_road_id = road_ids_to_check.pop(0)
            if current_road_id in visited_road_ids:
                continue
            visited_road_ids.add(current_road_id)
            extended_road_ids.add(current_road_id)

            wps_on_road = [wp for wp in map.generate_waypoints(2.0) if wp.road_id == current_road_id]
            for wp in wps_on_road:
                for next_wp in wp.next(5.0):
                    if next_wp.road_id not in visited_road_ids:
                        road_ids_to_check.append(next_wp.road_id)
                for prev_wp in wp.previous(5.0):
                    if prev_wp.road_id not in visited_road_ids:
                        road_ids_to_check.append(prev_wp.road_id)

        nearby_points = [pt for pt in spawn_points[1:] if map.get_waypoint(pt.location).road_id in extended_road_ids]

        # Spawn remaining vehicles
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        remaining = min(count - 1, len(nearby_points))
        for transform in nearby_points[:remaining]:
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, tm.get_port())))

        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                actor_ids.append(response.actor_id)

        print('Spawned %d autopilot vehicles.' % len(actor_ids))

        # Spectator follows hero vehicle from behind and above
        spectator = world.get_spectator()
        def follow_hero():
            if hero_vehicle and hero_vehicle.is_alive:
                transform = hero_vehicle.get_transform()
                if transform.location.z > 0:
                    loc = transform.location + carla.Location(x=-10, z=6)
                    rot = carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
                    spectator.set_transform(carla.Transform(loc, rot))

        if args.recorder_time > 0:
            start_time = time.time()
            while time.time() - start_time < args.recorder_time:
                world.tick()
                follow_hero()
        else:
            while True:
                world.tick()
                follow_hero()

    finally:
        print('\nDestroying %d actors' % len(actor_list + actor_ids))
        all_ids = actor_list + actor_ids
        client.apply_batch_sync([carla.command.DestroyActor(x) for x in all_ids])

        print("Stop recording")
        client.stop_recorder()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone.')
