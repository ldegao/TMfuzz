import os
import json
import math
import pdb
import sys

import constants
import globals as g
import config
from driving_quality import *

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)


# def dry_run(conf, client, tm, town, sp, wp, weather):
# """
# Dry-runs the base scenario to infer the oracle state
# params: None
# return: packed object of feedbacks (tbd)
# """
# return # debug

# dry_run_states = []

# for i in range(conf.num_dry_runs):
# print("performing {}-th dry run".format(i+1))
# simutale.simulate(client, town, tm, sp, wp, weather, [], [])
# state = {
# "num_frames": states.NUM_FRAMES,
# "elapsed_time": states.ELAPSED_TIME,
# "crash": states.COLLISION_EVENT,
# "lane_invasions": states.LANEINVASION_EVENT,
# "isstuck": states.STUCK,
# # "avg_iae_lon": sum(states.IAE_LON) / len(states.IAE_LON),
# # "avg_iae_lat": sum(states.IAE_LAT) / len(states.IAE_LAT)
# }
# dry_run_states.append(state)

# # get oracle states out of dry_run_states, and return
# # now just consider raw states as an oracle
# return dry_run_states

def set_traffic_lights_state(world, state):
    traffic_lights = world.get_actors().filter("*traffic_light*")
    print(len(traffic_lights))
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
        return (-999, 999, -999, 999)

    x_list = []
    y_list = []
    for coord in town_data:
        x_list.append(town_data[coord][0])
        y_list.append(town_data[coord][1])

    return (min(x_list), max(x_list), min(y_list), max(y_list))


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
    # global client, tm
    g.client = carla.Client(conf.sim_host, conf.sim_port)
    print("Connecting to %s:%d" % (conf.sim_host, conf.sim_port))
    g.client.set_timeout(10.0)
    try:
        g.client.get_server_version()
    except Exception as e:
        print("[-] Error: Check client connection.")
        sys.exit(-1)
    if conf.debug:
        print("Connected to:", g.client)

    g.tm = g.client.get_trafficmanager(conf.sim_tm_port)
    g.tm.set_synchronous_mode(True)
    g.tm.set_random_device_seed(0)
    if conf.debug:
        print("Traffic Manager Server:", g.tm)

    return g.client, g.tm


def switch_map(conf, town):
    """
    Switch map in the simulator and retrieve legitimate waypoints (a list of
    carla.Transform objects) in advance.
    """

    assert (g.client is not None)

    try:
        world = g.client.get_world()
        # if world.get_map().name != town: # force load every time
        if conf.debug:
            print("[*] Switching town to {} (slow)".format(town))
        g.client.set_timeout(20)  # Handle sluggish loading bug
        g.client.load_world(str(town))  # e.g., "/Game/Carla/Maps/Town01"
        if conf.debug:
            print("[+] Switched")
        g.client.set_timeout(10.0)

        town_map = world.get_map()
        g.town_map = town_map

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


def set_autopilot(vehicle):
    #
    vehicle.set_autopilot(True, g.tm.get_port())
    g.tm.ignore_lights_percentage(vehicle, 0)
    g.tm.ignore_signs_percentage(vehicle, 0)
    g.tm.ignore_vehicles_percentage(vehicle, 0)
    g.tm.ignore_walkers_percentage(vehicle, 0)

    g.tm.auto_lane_change(vehicle, True)

    g.tm.vehicle_percentage_speed_difference(vehicle, 10)

    g.tm.set_route(vehicle, ["Straight"])
    vehicle.set_simulate_physics(True)


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
