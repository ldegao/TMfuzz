#!/usr/bin/env python3
import logging
import os
import pdb
import sys
import time
import random
import argparse
import copyreg
import json
# from collections import deque
import concurrent.futures
import math
from typing import List
from subprocess import Popen, PIPE

import docker
import numpy as np
from deap import base, tools, algorithms
import signal
import traceback
import networkx as nx
from shapely.geometry import LineString
import config
import constants as c
from actor import Actor
from scenario import Scenario
import states
import utils
from utils import check_autoware_status
import cluster

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("[-] Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("    Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)

client, world, G, blueprint_library, town_map = None, None, None, None, None
model = cluster.create_cnn_model()
pca = cluster.create_pca()
accumulated_trace_graphs = []
autoware_container = None
exec_state = states.ExecState()
# monitor carla
monitoring_thread = utils.monitor_docker_container('carlasim/carla:0.9.13')
# vehicle_bp_library = blueprint_library.filter("vehicle.*")
# vehicle_bp.set_attribute("color", "255,0,0")
# walker_bp = blueprint_library.find("walker.pedestrian.0001")  # 0001~0014
# walker_controller_bp = blueprint_library.find('controller.ai.walker')
# player_bp = blueprint_library.filter('nissan')[0]

def carla_ActorBlueprint_pickle(actor_blueprint):
    return actor_blueprint.id


def carla_ActorBlueprint_unpickle(blueprint_id):
    return blueprint_library.find(blueprint_id)


def carla_ActorBlueprint_unpickle(blueprint_id):
    return blueprint_library.find(blueprint_id)


def carla_location_pickle(location):
    data = {
        'location_x': location.x,
        'location_y': location.y,
        'location_z': location.z,
    }
    json_string = json.dumps(data)
    return json_string


def carla_location_unpickle(json_string):
    data = json.loads(json_string)
    x, y, z = data
    return carla.Location(x, y, z)


def carla_rotation_pickle(rotation):
    data = {
        'rotation_pitch': rotation.pitch,
        'rotation_yaw': rotation.yaw,
        'rotation_roll': rotation.roll
    }
    json_string = json.dumps(data)
    return json_string


def carla_rotation_unpickle(json_string):
    data = json.loads(json_string)
    pitch, yaw, roll = data
    return carla.Rotation(pitch, yaw, roll)


def carla_transform_pickle(transform):
    location = transform.location
    rotation = transform.rotation
    data = {
        'location_x': location.x,
        'location_y': location.y,
        'location_z': location.z,
        'rotation_pitch': rotation.pitch,
        'rotation_yaw': rotation.yaw,
        'rotation_roll': rotation.roll
    }
    json_string = json.dumps(data)
    return json_string


def carla_transform_unpickle(json_string):
    data = json.loads(json_string)
    x = data['location_x']
    y = data['location_y']
    z = data['location_z']
    pitch = data['rotation_pitch']
    yaw = data['rotation_yaw']
    roll = data['rotation_roll']
    return carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch, yaw, roll))


def create_test_scenario(conf, seed_dict):
    return Scenario(conf, seed_dict)


def handler(signum, frame):
    raise Exception("HANG")


def ini_hyperparameters(conf, args):
    conf.cur_time = time.time()
    if args.determ_seed:
        conf.determ_seed = args.determ_seed
    else:
        conf.determ_seed = conf.cur_time
    random.seed(conf.determ_seed)
    print("[info] determ seed set to:", conf.determ_seed)
    conf.out_dir = args.out_dir
    try:
        os.mkdir(conf.out_dir)
    except Exception:
        estr = f"Output directory {conf.out_dir} already exists. Remove with " \
               "caution; it might contain data from previous runs."
        print(estr)
        sys.exit(-1)

    conf.seed_dir = args.seed_dir
    if not os.path.exists(conf.seed_dir):
        os.mkdir(conf.seed_dir)
    else:
        print(f"Using seed dir {conf.seed_dir}")
    conf.set_paths()

    with open(conf.meta_file, "w") as f:
        f.write(" ".join(sys.argv) + "\n")
        f.write("start: " + str(int(conf.cur_time)) + "\n")

    try:
        os.mkdir(conf.queue_dir)
        os.mkdir(conf.error_dir)
        os.mkdir(conf.rosbag_dir)
        os.mkdir(conf.cam_dir)
        os.mkdir(conf.trace_dir)
    except Exception as e:
        print(e)
        sys.exit(-1)
    if args.no_lane_check:
        conf.check_dict["lane"] = False
    conf.sim_host = args.sim_host
    conf.sim_port = args.sim_port
    conf.max_mutations = args.max_mutations
    conf.timeout = args.timeout
    conf.function = args.function

    if args.target.lower() == "behavior":
        conf.agent_type = c.BEHAVIOR
    elif args.target.lower() == "autoware":
        conf.agent_type = c.AUTOWARE
    else:
        print("[-] Unknown target: {}".format(args.target))
        sys.exit(-1)

    conf.town = args.town
    conf.num_mutation_car = args.num_mutation_car
    conf.density = args.density
    conf.no_traffic_lights = args.no_traffic_lights
    conf.debug = args.debug


def mutate_weather(test_scenario):
    test_scenario.weather["cloud"] = random.randint(0, 100)
    test_scenario.weather["rain"] = random.randint(0, 100)
    test_scenario.weather["wind"] = random.randint(0, 100)
    test_scenario.weather["fog"] = random.randint(0, 100)
    test_scenario.weather["wetness"] = random.randint(0, 100)
    test_scenario.weather["angle"] = random.randint(0, 360)
    test_scenario.weather["altitude"] = random.randint(-90, 90)


def mutate_weather_fixed(test_scenario):
    test_scenario.weather["cloud"] = 0
    test_scenario.weather["rain"] = 0
    test_scenario.weather["wind"] = 0
    test_scenario.weather["fog"] = 0
    test_scenario.weather["wetness"] = 0
    test_scenario.weather["angle"] = 0
    test_scenario.weather["altitude"] = 60


def set_args():
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("--debug", action="store_true", default=False)
    argument_parser.add_argument("-o", "--out-dir", default="./data/output", type=str,
                                 help="Directory to save fuzzing logs")
    argument_parser.add_argument("-m", "--max-mutations", default=5, type=int,
                                 help="Size of the mutated population per cycle")
    argument_parser.add_argument("-d", "--determ-seed", type=float,
                                 help="Set seed num for deterministic mutation (e.g., for replaying)")
    argument_parser.add_argument("-u", "--sim-host", default="localhost", type=str,
                                 help="Hostname of Carla simulation server")
    argument_parser.add_argument("-p", "--sim-port", default=2000, type=int,
                                 help="RPC port of Carla simulation server")
    argument_parser.add_argument("-s", "--seed-dir", default="./data/seed", type=str,
                           help="Seed directory")
    argument_parser.add_argument("-t", "--target", default="behavior", type=str,
                                 help="Target autonomous driving system (behavior/Autoware)")
    argument_parser.add_argument("-f", "--function", default="general", type=str,
                                 choices=["general", "collision", "traction", "eval-os", "eval-us",
                                          "figure", "sens1", "sens2", "lat", "rear"],
                                 help="Functionality to test (general / collision / traction)")
    argument_parser.add_argument("-k", "--num_mutation_car", default=3, type=int,
                                 help="Number of max weight vehicles to mutation per cycle, default=1,negative means "
                                      "random")
    argument_parser.add_argument("--density", default=1, type=float,
                                 help="density of vehicles,1.0 means add 1 bg vehicle per 1 sec")
    argument_parser.add_argument("--town", default=3, type=int,
                                 help="Test on a specific town (e.g., '--town 3' forces Town03)")
    argument_parser.add_argument("--timeout", default="60", type=int,
                                 help="Seconds to timeout if vehicle is not moving")
    argument_parser.add_argument("--no-speed-check", action="store_true")
    argument_parser.add_argument("--no-lane-check", action="store_true")
    argument_parser.add_argument("--no-crash-check", action="store_true")
    argument_parser.add_argument("--no-stuck-check", action="store_true")
    argument_parser.add_argument("--no-red-check", action="store_true")
    argument_parser.add_argument("--no-other-check", action="store_true")
    argument_parser.add_argument("--no-traffic-lights", action="store_true")
    return argument_parser


def evaluation(ind: Scenario):
    global autoware_container
    min_dist = 99999
    distance = 0
    nova = 0
    g_name = f'Generation_{ind.generation_id:05}'
    s_name = f'Scenario_{ind.scenario_id:05}'
    # todo: run test here
    # for test
    mutate_weather_fixed(ind)
    signal.alarm(15 * 60)  # timeout after 15 min
    print("timeout after 15 min")
    try:
        # profiler = cProfile.Profile()
        # profiler.enable()  #
        ret = ind.run_test(exec_state)
        if ret == -1:
            print("[-] Fatal error occurred during test")
            exit(0)
        min_dist = ind.state.min_dist
        trace_graph_important = ind.state.trace_graph_important
        accumulated_trace_graphs.append(trace_graph_important)
        if not ind.state.stuck:
            distance_list = cluster.calculate_distance(model, pca, accumulated_trace_graphs)
            distance = distance_list[-1]
        else:
            distance = 0
        for i in range(1, len(ind.state.speed)):
            acc = abs(ind.state.speed[i] - ind.state.speed[i - 1])
            nova += acc
        nova = nova / len(ind.state.speed)
        # reload scenario state
        ind.state = states.ScenarioState()
        # profiler.disable()  #
        # profiler.print_stats(sort="cumulative")
        # pdb.set_trace()
    except Exception as e:
        if e == TimeoutError:
            print("[-] simulation hanging. abort.")
            ret = 1
        else:
            print("[-] run_test error:")
            traceback.print_exc()
            exit(0)
    signal.alarm(0)
    if ret is None:
        pass
    elif ret == -1:
        print("[-] Fatal error occurred during test")
        exit(0)
    elif ret == 1:
        print("fuzzer - found an error")
    elif ret == 128:
        print("Exit by user request")

    # mutation loop ends
    if ind.found_error:
        print("[-]error detected. start a new cycle with a new seed")
        # todo: get violation here
    return min_dist, nova, distance


# MUTATION OPERATOR


def mut_actor_list(ind: List[Actor]):
    if len(ind) <= 1:
        return ind
    mut_pb = random.random()
    random_index = random.randint(0, len(ind) - 1)
    # remove a random 1
    if mut_pb < 0.1:
        ind.pop(random_index)
        return ind
    # add a random 1
    if mut_pb < 0.4:
        template_actor = ind[random_index]
        new_ad = Actor.get_actor_by_one(template_actor, town_map, len(ind) - 1)
        ind.append(new_ad)
        return ind
    # mutate a random agent
    template_actor = ind[random_index]
    new_ad = Actor.get_actor_by_one(template_actor, town_map, len(ind) - 1)
    ind.append(new_ad)
    ind.pop(random_index)
    return ind


def mut_scenario(ind: Scenario):
    mut_pb = random.random()
    if mut_pb < 1:
        ind.actor_list = mut_actor_list(ind.actor_list)
    # else:
    #     ind.tc_section = mut_tc_section(ind.tc_section)
    return ind,


# CROSSOVER OPERATOR

def cx_actor(ind1: List[Actor], ind2: List[Actor]):
    # todo: swap entire ad section
    cx_pb = random.random()
    if cx_pb < 0.05:
        return ind2, ind1

    for adc1 in ind1:
        for adc2 in ind2:
            Actor.actor_cross(adc1, adc2)

    # # if len(ind1.adcs) < MAX_ADC_COUNT:
    # #     for adc in ind2.adcs:
    # #         if ind1.has_conflict(adc) and ind1.add_agent(deepcopy(adc)):
    # #             # add an agent from parent 2 to parent 1 if there exists a conflict
    # #             ind1.adjust_time()
    # #             return ind1, ind2
    #
    # # if none of the above happened, no common adc, no conflict in either
    # # combine to make a new populations
    # available_adcs = ind1.adcs + ind2.adcs
    # random.shuffle(available_adcs)
    # split_index = random.randint(2, min(len(available_adcs), MAX_ADC_COUNT))
    #
    # result1 = ADSection([])
    # for x in available_adcs[:split_index]:
    #     result1.add_agent(copy.deepcopy(x))
    #
    # # make sure offspring adc count is valid
    #
    # while len(result1.adcs) > MAX_ADC_COUNT:
    #     result1.adcs.pop()
    #
    # while len(result1.adcs) < 2:
    #     new_ad = ADAgent.get_one()
    #     if result1.has_conflict(new_ad) and result1.add_agent(new_ad):
    #         break
    # result1.adjust_time()
    return ind1, ind2


def cx_scenario(ind1: Scenario, ind2: Scenario):
    cx_pb = random.random()
    if cx_pb < 1:
        ind1.actor_list, ind2.actor_list = cx_actor(
            ind1.actor_list, ind2.actor_list
        )
    # else:
    #     ind1.tc_section, ind2.tc_section = cx_tc_section(
    #         ind1.tc_section, ind2.tc_section
    #     )
    return ind1, ind2





def seed_initialize(town, town_map):
    spawn_points = town.get_spawn_points()
    sp = random.choice(spawn_points)
    sp_x = sp.location.x
    sp_y = sp.location.y
    sp_z = sp.location.z
    pitch = sp.rotation.pitch
    yaw = sp.rotation.yaw
    roll = sp.rotation.roll
    # restrict destination to be within 200 meters
    destination_flag = True
    wp, wp_x, wp_y, wp_z, wp_yaw = None, None, None, None, None
    while destination_flag:
        wp = random.choice(spawn_points)
        wp_x = wp.location.x
        wp_y = wp.location.y
        wp_z = wp.location.z
        wp_yaw = wp.rotation.yaw
        if math.sqrt((sp_x - wp_x) ** 2 + (sp_y - wp_y) ** 2) > c.MIN_DIST:
            destination_flag = False
        if math.sqrt((sp_x - wp_x) ** 2 + (sp_y - wp_y) ** 2) > c.MAX_DIST:
            destination_flag = True
    seed_dict = {
        "map": town_map,
        "sp_x": sp_x,
        "sp_y": sp_y,
        "sp_z": sp_z,
        "pitch": pitch,
        "yaw": yaw,
        "roll": roll,
        "wp_x": wp_x,
        "wp_y": wp_y,
        "wp_z": wp_z,
        "wp_yaw": wp_yaw
    }
    return seed_dict


def init_env():
    conf = config.Config()
    argument_parser = set_args()
    args = argument_parser.parse_args()
    ini_hyperparameters(conf, args)
    if conf.town is not None:
        town_map = "Town0{}".format(conf.town)
    else:
        town_map = "Town0{}".format(random.randint(1, 5))
    if conf.no_traffic_lights:
        conf.check_dict["red"] = False
    signal.signal(signal.SIGALRM, handler)
    client = utils.connect(conf)
    client.set_timeout(20)
    client.load_world(town_map)
    world = client.get_world()
    town = world.get_map()
    map_topology = town.get_topology()
    G = nx.DiGraph()
    lane_list = {}
    for edge in map_topology:
        # 1.add_edge for every lane that is connected
        G.add_edge((edge[0].road_id, edge[0].lane_id), (edge[1].road_id, edge[1].lane_id))
        if (edge[0].road_id, edge[0].lane_id) not in lane_list:
            edge_end = edge[0].next_until_lane_end(500)[-1]
            lane_list[(edge[0].road_id, edge[0].lane_id)] = (edge[0], edge_end)
    added_edges = []
    for lane_A in lane_list:
        for lane_B in lane_list:
            # 2.add_edge for every lane that is cross in junction
            if lane_A != lane_B:
                point_a = lane_list[lane_A][0].transform.location.x, lane_list[lane_A][0].transform.location.y
                point_b = lane_list[lane_A][1].transform.location.x, lane_list[lane_A][1].transform.location.y
                point_c = lane_list[lane_B][0].transform.location.x, lane_list[lane_B][0].transform.location.y
                point_d = lane_list[lane_B][1].transform.location.x, lane_list[lane_B][1].transform.location.y
                line_ab = LineString([point_a, point_b])
                line_cd = LineString([point_c, point_d])
                if line_ab.crosses(line_cd):
                    if (lane_B, lane_A) not in added_edges:
                        G.add_edge(lane_A, lane_B)
                        G.add_edge(lane_B, lane_A)
                        # added_edges.append((lane_A, lane_B))
    for lane in lane_list:
        # 3.add_edge for evert lane that could change to
        lane_change_left = lane_list[lane][0].lane_change == carla.LaneChange.Left or \
                           lane_list[lane][0].lane_change == carla.LaneChange.Both or \
                           lane_list[lane][1].lane_change == carla.LaneChange.Left or \
                           lane_list[lane][1].lane_change == carla.LaneChange.Both
        lane_change_right = lane_list[lane][0].lane_change == carla.LaneChange.Right or \
                            lane_list[lane][0].lane_change == carla.LaneChange.Both or \
                            lane_list[lane][1].lane_change == carla.LaneChange.Right or \
                            lane_list[lane][1].lane_change == carla.LaneChange.Both
        if lane_change_left:
            if (lane[0], lane[1] + 1) in lane_list:
                G.add_edge(lane, (lane[0], lane[1] + 1))
        if lane_change_right:
            if (lane[0], lane[1] - 1) in lane_list:
                G.add_edge(lane, (lane[0], lane[1] - 1))
    utils.switch_map(conf, town_map, client)
    return conf, town, town_map, client, world, G


def print_all_attr(obj):
    attributes = dir(obj)
    for attr_name in attributes:
        if not callable(getattr(obj, attr_name)):
            attr_value = getattr(obj, attr_name)
            attr_type = type(attr_value)
            print(f"Attribute: {attr_name}, Value: {attr_value}, Type: {attr_type}")


def main():
    # STEP 0: init env
    global client, world, G, blueprint_library, town_map
    logging.basicConfig(filename='./data/record.log', filemode='a', level=logging.INFO, format='%(asctime)s - %(message)s')
    copyreg.pickle(carla.libcarla.Location, carla_location_pickle, carla_location_unpickle)
    copyreg.pickle(carla.libcarla.Rotation, carla_rotation_pickle, carla_rotation_unpickle)
    copyreg.pickle(carla.libcarla.Transform, carla_transform_pickle, carla_transform_unpickle)
    copyreg.pickle(carla.libcarla.ActorBlueprint, carla_ActorBlueprint_pickle, carla_ActorBlueprint_unpickle)

    conf, town, town_map, exec_state.client, exec_state.world, exec_state.G = init_env()
    world = exec_state.world
    blueprint_library = world.get_blueprint_library()
    # if conf.agent_type == c.AUTOWARE:
    #     autoware_launch(exec_state.world, conf, town)
    population = []
    # GA Hyperparameters
    POP_SIZE = 5  # amount of population
    OFF_SIZE = 5  # number of offspring to produce
    MAX_GEN = 5  #
    CXPB = 0.8  # crossover probability
    MUTPB = 0.2  # mutation probability
    toolbox = base.Toolbox()
    toolbox.register("evaluate", evaluation)
    toolbox.register("mate", cx_scenario)
    toolbox.register("mutate", mut_scenario)
    toolbox.register("select", tools.selNSGA2)
    hof = tools.ParetoFront()
    # Evaluate Initial Population
    print(f' ====== Analyzing Initial Population ====== ')
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    stats = tools.Statistics(key=lambda ind: ind.fitness.values)
    stats.register("avg", np.mean, axis=0)
    stats.register("max", np.max, axis=0)
    stats.register("min", np.min, axis=0)
    logbook = tools.Logbook()
    logbook.header = 'gen', 'avg', 'max', 'min'
    # begin a generational process
    curr_gen = 0
    # init some seed if seed pool is empty
    for i in range(POP_SIZE):
        seed_dict = seed_initialize(town, town_map)
        # Creates and initializes a Scenario instance based on the metadata
        with concurrent.futures.ThreadPoolExecutor() as my_simulate:
            future = my_simulate.submit(create_test_scenario, conf, seed_dict)
            test_scenario = future.result(timeout=15)
        population.append(test_scenario)
        test_scenario.scenario_id = len(population)
    while True:
        # Main loop
        curr_gen += 1
        if curr_gen > MAX_GEN:
            break
        print(f' ====== GA Generation {curr_gen} ====== ')
        # Vary the population
        offspring = algorithms.varOr(
            population, toolbox, OFF_SIZE, CXPB, MUTPB)
        # update chromosome generation_id and scenario_id
        for index, d in enumerate(offspring):
            d.generation_id = curr_gen
            d.scenario_id = index
        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
        hof.update(offspring)
        # Select the next generation population
        population[:] = toolbox.select(population + offspring, POP_SIZE)
        record = stats.compile(population)
        logbook.record(gen=curr_gen, **record)
        print(logbook.stream)
        # Save directory for trace graphs


if __name__ == "__main__":
    main()
