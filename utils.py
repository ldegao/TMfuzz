import os
import json
import math
import pdb
import re
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import shapely
from pygame.draw_py import Point

import constants
import constants as c
import config

config.set_carla_api_path()

try:
    import carla
except ModuleNotFoundError as e:
    print("Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)
try:
    proj_root = config.get_proj_root()
    sys.path.append(os.path.join(proj_root, "carla", "PythonAPI", "carla"))
except IndexError:
    pass
from agents.navigation.behavior_agent import BehaviorAgent


def monitor_docker_container(image_name, check_interval=10):
    def monitor():
        while True:
            result = subprocess.run(["docker", "ps", "-a"], stdout=subprocess.PIPE)
            output = result.stdout.decode('utf-8')

            match = re.search(r'(\w+)\s+(' + re.escape(image_name) + r')\s+.*\s+(\w+ \w+ ago)\s+(\w+)', output)
            if match and match.group(4) != 'Up':
                exit(-1)

            time.sleep(check_interval)

    monitoring_thread = threading.Thread(target=monitor)
    monitoring_thread.daemon = True
    monitoring_thread.start()
    return monitoring_thread


def timeout_handler(signum, frame):
    raise TimeoutError


def set_traffic_lights_state(world, state):
    traffic_lights = world.get_actors().filter("*traffic_light*")
    for traffic_light in traffic_lights:
        traffic_light.set_state(state)


def get_carla_transform(loc_rot_tuples):
    """
    Convert loc_rot_tuples = ((x, y, z), (roll, pitch, yaw)) to
    carla.Transform object
    """

    if loc_rot_tuples is None:
        return None

    loc = loc_rot_tuples[0]
    rot = loc_rot_tuples[1]

    t = carla.Transform(
        carla.Location(loc[0], loc[1], loc[2]),
        carla.Rotation(roll=rot[0], pitch=rot[1], yaw=rot[2])
    )

    return t


def get_valid_xy_range(town):
    try:
        with open(os.path.join("town_info", town + ".json")) as fp:
            town_data = json.load(fp)
    except:
        return -999, 999, -999, 999

    x_list = []
    y_list = []
    for coord in town_data:
        x_list.append(town_data[coord][0])
        y_list.append(town_data[coord][1])

    return min(x_list), max(x_list), min(y_list), max(y_list)


def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    # Copied from
    # https://github.com/davheld/tf/blob/master/src/tf/transformations.py#L1100

    _AXES2TUPLE = {
        'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
        'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
        'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
        'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
        'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
        'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
        'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
        'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

    _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

    _NEXT_AXIS = [1, 2, 0, 1]

    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    quaternion = np.empty((4,), dtype=np.float64)
    if repetition:
        quaternion[i] = cj * (cs + sc)
        quaternion[j] = sj * (cc + ss)
        quaternion[k] = sj * (cs - sc)
        quaternion[3] = cj * (cc - ss)
    else:
        quaternion[i] = cj * sc - sj * cs
        quaternion[j] = cj * ss + sj * cc
        quaternion[k] = cj * cs - sj * sc
        quaternion[3] = cj * cc + sj * ss
    if parity:
        quaternion[j] *= -1

    return quaternion


def connect(conf):
    client = carla.Client(conf.sim_host, conf.sim_port)
    print("Connecting to %s:%d" % (conf.sim_host, conf.sim_port))
    client.set_timeout(10.0)
    try:
        client.get_server_version()
    except Exception:
        print("[-] Error: Check client connection.")
        sys.exit(-1)
    if conf.debug:
        print("[debug] Connected to:", client)

    return client


def switch_map(conf, town, client):
    """
    Switch map in the simulator and retrieve legitimate waypoints (a list of
    carla.Transform objects) in advance.
    """

    # assert (g.client is not None)

    try:

        world = client.get_world()
        # if world.get_map().name != town: # force load every time
        if conf.debug:
            print("[debug] Switching town to {} (slow)".format(town))
        client.set_timeout(20)  # Handle sluggish loading bug
        client.load_world(str(town))  # e.g., "/Game/Carla/Maps/Town01"
        if conf.debug:
            print("[debug] Switched")
        client.set_timeout(10.0)

        town_map = world.get_map()

    except Exception as e:
        print("[-] Error:", e)
        sys.exit(-1)


def get_distance_to_target(vehicle_location, target_location):
    dx = target_location.x - vehicle_location.x
    dy = target_location.y - vehicle_location.y
    return math.sqrt(dx ** 2 + dy ** 2)


def get_angle_between_vectors(vector1, vector2):
    dot_product = vector1.x * vector2.x + vector1.y * vector2.y
    magnitudes_product = math.sqrt(vector1.x ** 2 + vector1.y ** 2) * math.sqrt(vector2.x ** 2 + vector2.y ** 2)
    if magnitudes_product == 0:
        return 0
    else:
        cos_angle = dot_product / magnitudes_product
        return math.degrees(math.acos(cos_angle))


def set_autopilot(vehicle, nav_type=c.BEHAVIOR_AGENT, sp_location=None, wp_location=None, world=None):
    if nav_type == constants.BEHAVIOR_AGENT:
        world.tick()  # sync once with simulator
        vehicle.set_simulate_physics(True)
        agent = BehaviorAgent(
            vehicle,
            behavior="cautious"
        )
        agent.set_destination(
            start_location=sp_location,
            end_location=wp_location,
        )
        return agent


def get_relative_position(x1, y1, x2, y2, v_x, v_y):
    # get relative vector
    relative_x = x2 - x1
    relative_y = y2 - y1

    # get direction vector
    direction_x = v_x
    direction_y = v_y

    # get cross product
    cross_product = relative_x * direction_y - relative_y * direction_x

    if cross_product >= 0:
        position = constants.LEFT
    else:
        position = constants.RIGHT

    # get dot product
    dot_product = relative_x * direction_x + relative_y * direction_y
    if dot_product > 0:
        direction = constants.FRONT
    else:
        direction = constants.BACK
    return direction, position


def normalize_vector(vector):
    length = math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)
    if length != 0.0:
        return carla.Vector3D(vector.x / length, vector.y / length, vector.z / length)
    else:
        return carla.Vector3D(0.0, 0.0, 0.0)


def draw_arrow(world, start, end, color=carla.Color(255, 0, 0), arrow_size=0.2):
    direction = end - start
    direction = normalize_vector(direction)
    perpendicular = carla.Vector3D(-direction.y, direction.x, 0.0)
    arrow_start = end - arrow_size * direction
    arrow_end = arrow_start + arrow_size * 0.5 * perpendicular
    arrow_start_location = carla.Location(arrow_start.x, arrow_start.y, arrow_start.z)
    world.debug.draw_box(
        box=carla.BoundingBox(
            arrow_start_location,
            carla.Vector3D(0.05, 0.05, 0.05)
        ),
        rotation=carla.Rotation(0, 0, 0),
        life_time=0.5,
        thickness=0.2,
        color=color
    )
    world.debug.draw_line(arrow_start, arrow_end, life_time=0.5, color=color)


def mark_npc(npc, frame=0):
    npc.death_time = frame


def delete_npc(npc, npc_vehicles, sensors, agents_now=None, npc_now=None):
    for agent_tuple in agents_now:
        if agent_tuple[1] == npc.instance:
            agents_now.remove(agent_tuple)
            break
    npc_vehicles.remove(npc.instance)
    npc_now.remove(npc)
    npc.instance.destroy()
    npc.instance = None
    npc.stuck_duration = 0
    if npc.sensor_collision:
        sensors.remove(npc.sensor_collision)
        npc.sensor_collision.stop()
        npc.sensor_collision.destroy()
        npc.sensor_collision = None
    if npc.sensor_lane_invasion:
        sensors.remove(npc.sensor_lane_invasion)
        npc.sensor_lane_invasion.stop()
        npc.sensor_lane_invasion.destroy()
        npc.sensor_lane_invasion = None


def _on_collision(event, state):
    if state.end:
        # ignore collision happened AFTER simulation ends
        # (can happen because of sluggish garbage collection of Carla)
        return
    if event.other_actor.type_id != "static.road":
        if not state.crashed:
            print("COLLISION:", event.other_actor.type_id)
            # do not count collision while spawning ego vehicle (hard drop)
            state.crashed = True
            state.collision_to = event.other_actor
            state.collision_to_actor = event.other_actor
            state.min_dist = 0
            state.min_dist_frame = state.num_frames


def _on_invasion(event, state):
    crossed_lanes = event.crossed_lane_markings
    for crossed_lane in crossed_lanes:
        if crossed_lane.lane_change == carla.LaneChange.NONE:
            print("LANE INVASION:", event)
            state.laneinvaded = True
            state.laneinvasion_event.append(event)

    # print(crossed_lane.color, crossed_lane.lane_change, crossed_lane.type)
    # print(type(crossed_lane.color), type(crossed_lane.lane_change),
    # type(crossed_lane.type))


def carla_location_to_ros_point(carla_location):
    """
      Convert a carla location to a ROS point

      Considers the conversion from left-handed system (unreal) to a right-handed
      system (ROS)

      :param carla_location: The carla location
      :type carla_location: carla.Location
      :return: a ROS point
      :rtype: geometry_msgs.msg.Point
    """
    ros_point = Point()
    ros_point.x = carla_location.x
    ros_point.y = -carla_location.y
    ros_point.z = carla_location.z

    return ros_point


def carla_rotation_to_RPY(carla_rotation):
    """
      Convert a carla rotation to a roll, pitch, yaw tuple

      Considers the conversion from left-handed system (unreal) to a right-handed
      system (ROS).
      Consider the conversion from degrees (carla) to radians (ROS).

      :param carla_rotation: The carla rotation
      :type carla_rotation: carla.Rotation
      :return: a tuple with three elements (roll, pitch, yaw)
      :rtype: tuple
      """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return roll, pitch, yaw


def check_autoware_status(world, timeout):
    left = 2 * 60
    try:
        left = signal.alarm(timeout)
        print("left time:", left)
        i = 0
        while True:
            time.sleep(1)
            output = subprocess.check_output("rosnode list | wc -l", shell=True)
            print("[*] Waiting for Autoware nodes " + "." * i + "\r", end="")
            i += 1
            world.tick()
            if output == b"":
                continue
            output = int(output.strip())
            # print("[*] Autoware nodes found: {}".format(output))
            # print("[*] WAIT_AUTOWARE_NUM_NODES:", c.WAIT_AUTOWARE_NUM_NODES)
            if output >= c.WAIT_AUTOWARE_NUM_NODES:
                print("Autoware nodes are ready.")
                break
    except TimeoutError:
        print("Autoware nodes did not Ready within timeout.")
        raise KeyboardInterrupt
    finally:
        signal.alarm(left)


def serialize_vehicle(actor, n=-1, target_location=None):
    """
    Serializes the vehicle actor's important properties including type_id, velocity, control, transform,
    target_location, and wheel data, and returns it as a dictionary.

    :param actor: The carla.Actor object representing the vehicle
    :param target_location: The target location of the vehicle, if applicable
    :param n: Number of decimal places for rounding (use -1 for full precision)
    :return: Dictionary containing serialized vehicle information
    """

    def round_value(value):
        """
        Helper function to round values to 'n' decimal places, or return as is if n < 0.
        """
        return round(value, n) if n >= 0 else value

    # 1. Type ID
    vehicle_type = actor.type_id

    # 2. Velocity (x, y, z)
    velocity = actor.get_velocity()
    velocity_data = {
        'x': round_value(velocity.x),
        'y': round_value(velocity.y),
        'z': round_value(velocity.z)
    }

    # 3. Control (throttle, steer, brake, hand_brake, reverse, gear)
    control = actor.get_control()
    control_data = {
        'throttle': round_value(control.throttle),
        'steer': round_value(control.steer),
        'brake': round_value(control.brake),
        'hand_brake': control.hand_brake,
        'reverse': control.reverse,
        'gear': control.gear
    }

    # 4. Transform (location and rotation)
    transform = actor.get_transform()
    transform_data = {
        'location': {
            'x': round_value(transform.location.x),
            'y': round_value(transform.location.y),
            'z': round_value(transform.location.z)
        },
        'rotation': {
            'pitch': round_value(transform.rotation.pitch),
            'yaw': round_value(transform.rotation.yaw),
            'roll': round_value(transform.rotation.roll)
        }
    }

    # # 5. Target Location (if applicable)
    #
    # if target_location:
    #     target_data = {
    #         'x': round_value(target_location.x),
    #         'y': round_value(target_location.y),
    #         'z': round_value(target_location.z)
    #     }

    # Combine all data into a dictionary
    vehicle_data = {
        'type_id': vehicle_type,
        'velocity': velocity_data,
        'control': control_data,
        'transform': transform_data,
    }
    # if target_data:
    #     vehicle_data['target_location'] = target_data

    return vehicle_data


def update_vehicle_file(state, closest_cars_list, player, npc_list, json_cache):
    # Initialize data structure for the current frame
    frame_data = {
        # "min_dist_frame": state.min_dist_frame,
        str(state.num_frames): {
            "NPC": [],
            "player": serialize_vehicle(player)
        }
    }
    # Process closest_cars_list to find NPCs close to the player
    for closest_cars in closest_cars_list:
        for npc in npc_list:
            if npc.instance is not None and npc.instance.id == closest_cars.id:
                frame_data[str(state.num_frames)]["NPC"].append(serialize_vehicle(closest_cars, 2))

    # Update json_cache with data for the current frame
    if state.scenario_id not in json_cache:
        json_cache[state.scenario_id] = {}
    json_cache[state.scenario_id].update(frame_data)
    return json_cache


def write_json_cache_to_file(conf, state, json_cache):
    # Define the file path for JSON output
    time_record_file = "{}/gid:{}_sid:{}.json".format(conf.time_record_dir, state.generation_id, state.scenario_id)

    # Write the entire json_cache data to the file
    with open(time_record_file, "w") as f:
        json.dump(json_cache.get(state.scenario_id, {}), f, indent=4)

    # Return the JSON data and clear the cache for this scenario
    output_data = json_cache.pop(state.scenario_id, None)
    return output_data


def record_closest_cars(npc_vehicles, player_loc, state):
    # record min_dist
    min_dist = state.min_dist
    for npc_vehicle in npc_vehicles:
        distance = npc_vehicle.get_location().distance(player_loc)
        if distance < min_dist:
            min_dist = distance
    if min_dist < state.min_dist:
        state.min_dist = min_dist
        state.min_dist_frame = state.num_frames

    camera_tf = carla.Transform(
        carla.Location(x=player_loc.x, y=player_loc.y, z=50.0),
        carla.Rotation(pitch=-90.0)
    )
    return filter_vehicles_in_frustum(npc_vehicles, camera_tf, 105, 800, 600, 50)


def filter_vehicles_in_frustum(vehicle_list, camera_transform, vertical_fov, image_width, image_height, camera_height):
    # Get frustum vertices from the camera view
    coord1, coord2, coord3, coord4 = calculate_view_frustum(
        camera_transform, vertical_fov, image_width, image_height, camera_height
    )

    # print("Frustum vertices:")
    # print(f"Coord1: {coord1}, Coord2: {coord2}, Coord3: {coord3}, Coord4: {coord4}")

    # Create a polygon from frustum vertices
    area_polygon = shapely.geometry.Polygon([coord1, coord2, coord3, coord4])

    filtered_vehicles = []
    for vehicle in vehicle_list:
        vehicle_position = vehicle.get_transform().location
        vehicle_point = shapely.geometry.Point(vehicle_position.x, vehicle_position.y)
        # Check if the vehicle is within the frustum polygon
        if area_polygon.contains(vehicle_point):
            filtered_vehicles.append(vehicle)

    return filtered_vehicles


def calculate_view_frustum(camera_transform, vertical_fov, image_width, image_height, camera_height):
    # Step 1: Calculate aspect ratio and horizontal FOV
    aspect_ratio = image_width / image_height
    vertical_fov_rad = math.radians(vertical_fov)
    horizontal_fov_rad = 2 * math.atan(aspect_ratio * math.tan(vertical_fov_rad / 2))

    # Step 2: Correct calculation for ground projection depth (d_ground) based on camera height
    d_ground = camera_height * math.tan(vertical_fov_rad / 2)

    # Step 3: Calculate the ground projection width (w_ground)
    w_ground = d_ground * aspect_ratio

    # Step 4: Calculate camera yaw
    yaw = math.radians(camera_transform.rotation.yaw)

    # Step 5: Calculate frustum in camera coordinates projected onto z=0 (ground level)
    frustum_camera = np.array([
        [-w_ground / 2, d_ground],  # Left far corner
        [w_ground / 2, d_ground],  # Right far corner
        [-w_ground / 2, 0],  # Left near corner
        [w_ground / 2, 0]  # Right near corner
    ])

    # Step 6: Rotate and translate frustum to world coordinates
    rotation_matrix = np.array([
        [math.cos(yaw), -math.sin(yaw)],
        [math.sin(yaw), math.cos(yaw)]
    ])
    frustum_world = frustum_camera @ rotation_matrix.T

    # Translate to camera's (x, y) position
    cam_x, cam_y = camera_transform.location.x, camera_transform.location.y
    frustum_world[:, 0] += cam_x
    frustum_world[:, 1] += cam_y

    return frustum_world[0], frustum_world[1], frustum_world[2], frustum_world[3]
