#!/usr/bin/env python3
import cProfile
import os
import pdb
import sys
import time
import random
import argparse
import json
# from collections import deque
import concurrent.futures
import math
from typing import List
from subprocess import Popen, PIPE
import datetime
from copy import deepcopy

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
from simulate import autoware_goal_publish

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("[-] Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("    Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)

client, world, G = None, None, None
autoware_container = None
autoware_universe_container = None
exec_state = states.ExecState()


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
        os.mkdir(conf.cov_dir)
        os.mkdir(conf.rosbag_dir)
        os.mkdir(conf.cam_dir)
        os.mkdir(conf.score_dir)
    except Exception as e:
        print(e)
        sys.exit(-1)

    conf.sim_host = args.sim_host
    conf.sim_port = args.sim_port
    conf.max_mutations = args.max_mutations
    conf.timeout = args.timeout
    conf.function = args.function

    if args.target.lower() == "behavior":
        conf.agent_type = c.BEHAVIOR
    elif args.target.lower() == "autoware":
        conf.agent_type = c.AUTOWARE
    elif args.target.lower() == "autoware-universe":
        conf.agent_type = c.AUTOWARE_UNIVERSE
    else:
        print("[-] Unknown target: {}".format(args.target))
        print("[-] Available target: behavior, autoware, autoware-universe")
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
    argument_parser.add_argument("-o", "--out-dir", default="out-artifact", type=str,
                                 help="Directory to save fuzzing logs")
    argument_parser.add_argument("-s", "--seed-dir", default="seed-artifact", type=str,
                                 help="Seed directory")
    argument_parser.add_argument("-m", "--max-mutations", default=5, type=int,
                                 help="Size of the mutated population per cycle")
    argument_parser.add_argument("-d", "--determ-seed", type=float,
                                 help="Set seed num for deterministic mutation (e.g., for replaying)")
    argument_parser.add_argument("-u", "--sim-host", default="localhost", type=str,
                                 help="Hostname of Carla simulation server")
    argument_parser.add_argument("-p", "--sim-port", default=2000, type=int,
                                 help="RPC port of Carla simulation server")
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
    g_name = f'Generation_{ind.generation_id:05}'
    s_name = f'Scenario_{ind.scenario_id:05}'
    # todo: run test here
    ret = None
    # for test
    mutate_weather_fixed(ind)
    signal.alarm(12 * 60)  # timeout after 12 min
    try:
        # profiler = cProfile.Profile()
        # profiler.enable()  #
        ret = ind.run_test(exec_state)
        # profiler.disable()  #
        # profiler.print_stats(sort="cumulative")
        # pdb.set_trace()
    except Exception as e:
        if e.args[0] == "HANG":
            print("[-] simulation hanging. abort.")
            ret = -1
        else:
            print("[-] run_test error:")
            traceback.print_exc()
    signal.alarm(0)
    if ret is None:
        pass
    elif ret == -1:
        print("Spawn / simulation failure - don't add round cnt")
        # round_cnt -= 1
    elif ret == 1:
        print("fuzzer - found an error")
    elif ret == 128:
        print("Exit by user request")
        exit(0)
    else:
        if ret == -1:
            print("[-] Fatal error occurred during test")
            exit(-1)
    # mutation loop ends
    if ind.found_error:
        print("[-]error detected. start a new cycle with a new seed")
    # todo: get violation here

    return 1, 1, 1, 1


# MUTATION OPERATOR


def mut_actor_list(ind: List[Actor]):
    mut_pb = random.random()

    # todo:remove a random 1

    # if mut_pb < 0.1 and len(ind.adcs) > 2:
    #     random.shuffle(ind.adcs)
    #     ind.adcs.pop()
    #     ind.adjust_time()
    #     return ind

    # todo:add a random 1

    # if mut_pb < 0.4
    #     while True:
    #         new_ad = ADAgent.get_one()
    #         if ind.has_conflict(new_ad) and ind.add_agent(new_ad):
    #             break
    #     ind.adjust_time()
    #     return ind

    # todo:mutate a random agent

    # index = random.randint(0, len(ind.adcs) - 1)
    # routing = ind.adcs[index].routing
    # original_adc = ind.adcs.pop(index)
    # mut_counter = 0
    # while True:
    #     if ind.add_agent(ADAgent.get_one_for_routing(routing)):
    #         break
    #     mut_counter += 1
    #     if mut_counter == 5:
    #         # mutation kept failing, dont mutate
    #         ind.add_agent(original_adc)
    #         pass
    # ind.adjust_time()

    return ind


# def mut_pd_section(ind: PDSection):
#     if len(ind.pds) == 0:
#         ind.add_agent(PDAgent.get_one())
#         return ind
#
#     mut_pb = random()
#     # remove a random
#     if mut_pb < 0.2 and len(ind.pds) > 0:
#         random.shuffle(ind.pds)
#         ind.pds.pop()
#         return ind
#
#     # add a random
#     if mut_pb < 0.4 and len(ind.pds) <= MAX_PD_COUNT:
#         ind.pds.append(PDAgent.get_one())
#         return ind
#
#     # mutate a random
#     index = random.randint(0, len(ind.pds) - 1)
#     ind.pds[index] = PDAgent.get_one_for_cw(ind.pds[index].cw_id)
#     return ind
#
#
# def mut_tc_section(ind: TCSection):
#     mut_pb = random()
#
#     if mut_pb < 0.3:
#         ind.initial = TCSection.generate_config()
#         return ind
#     elif mut_pb < 0.6:
#         ind.final = TCSection.generate_config()
#     elif mut_pb < 0.9:
#         ind.duration_g = TCSection.get_random_duration_g()
#
#     return TCSection.get_one()


def mut_scenario(ind: Scenario):
    mut_pb = random.random()
    # if mut_pb < 1 / 3:
    #     ind.actor_list = mut_actor_list(ind.actor_list)
    # elif mut_pb < 2 / 3:
    #     ind.pd_section = mut_pd_section(ind.pd_section)
    # else:
    #     ind.tc_section = mut_tc_section(ind.tc_section)
    return ind,


# CROSSOVER OPERATOR

def cx_actor(ind1: List[Actor], ind2: List[Actor]):
    # todo: swap entire ad section
    # cx_pb = random.random()
    # if cx_pb < 0.05:
    #     return ind2, ind1
    #
    # cxed = False
    #
    # for adc1 in ind1.adcs:
    #     for adc2 in ind2.adcs:
    #         if adc1.routing_str == adc2.routing_str:
    #             # same routing in both parents
    #             # swaps start_s and start_t
    #             if random.random() < 0.5:
    #                 adc1.start_s = adc2.start_s
    #             else:
    #                 adc1.start_t = adc2.start_t
    #             mutated = True
    # if cxed:
    #     ind1.adjust_time()
    #     return ind1, ind2
    #
    # if len(ind1.adcs) < MAX_ADC_COUNT:
    #     for adc in ind2.adcs:
    #         if ind1.has_conflict(adc) and ind1.add_agent(deepcopy(adc)):
    #             # add an agent from parent 2 to parent 1 if there exists a conflict
    #             ind1.adjust_time()
    #             return ind1, ind2
    #
    # # if none of the above happened, no common adc, no conflict in either
    # # combine to make a new populations
    # available_adcs = ind1.adcs + ind2.adcs
    # random.shuffle(available_adcs)
    # split_index = random.randint(2, min(len(available_adcs), MAX_ADC_COUNT))
    #
    # result1 = ADSection([])
    # for x in available_adcs[:split_index]:
    #     result1.add_agent(deepcopy(x))
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
    # return result1, ind2
    return Actor, Actor


# def cx_pd_section(ind1: PDSection, ind2: PDSection):
#     cx_pb = random()
#     if cx_pb < 0.1:
#         return ind2, ind1
#
#     available_pds = ind1.pds + ind2.pds
#
#     result1 = PDSection(
#         sample(available_pds, k=random.randint(0, min(MAX_PD_COUNT, len(available_pds)))))
#     result2 = PDSection(
#         sample(available_pds, k=random.randint(0, min(MAX_PD_COUNT, len(available_pds)))))
#     return result1, result2
#
#
# def cx_tc_section(ind1: TCSection, ind2: TCSection):
#     cx_pb = random.random()
#     if cx_pb < 0.1:
#         return ind2, ind1
#     elif cx_pb < 0.4:
#         ind1.initial, ind2.initial = ind2.initial, ind1.initial
#     elif cx_pb < 0.7:
#         ind1.final, ind2.final = ind2.final, ind1.final
#     else:
#         ind1.duration_g, ind2.duration_g = ind2.duration_g, ind1.duration_g
#     return ind1, ind2


def cx_scenario(ind1: Scenario, ind2: Scenario):
    cx_pb = random.random()
    # if cx_pb < 0.6:
    #     ind1.actor_list, ind2.actor_list = cx_actor(
    #         ind1.actor_list, ind2.actor_list
    #     )
    # elif cx_pb < 0.6 + 0.2:
    #     ind1.pd_section, ind2.pd_section = cx_pd_section(
    #         ind1.pd_section, ind2.pd_section
    #     )
    # else:
    #     ind1.tc_section, ind2.tc_section = cx_tc_section(
    #         ind1.tc_section, ind2.tc_section
    #     )
    return ind1, ind2


def autoware_launch(carla_error, world, conf, town_map):
    username = os.getenv("USER")
    global autoware_container
    # print("before launching autoware", time.time())
    num_walker_topics = 0
    # clock = pygame.time.Clock()
    docker_client = docker.from_env()
    proj_root = config.get_proj_root()
    xauth = os.path.join(os.getenv("HOME"), ".Xauthority")
    vol_dict = {
        "{}/carla-autoware/autoware-contents".format(proj_root): {
            "bind": "/home/autoware/autoware-contents",
            "mode": "ro"
        },
        "/tmp/.X11-unix": {
            "bind": "/tmp/.X11-unix",
            "mode": "rw"
        },
        f"/home/{username}/.Xauthority": {
            "bind": xauth,
            "mode": "rw"
        },
        "/tmp/fuzzerdata/{}".format(username): {
            "bind": "/tmp/fuzzerdata",
            "mode": "rw"
        }
    }
    env_dict = {
        "DISPLAY": os.getenv("DISPLAY"),
        "XAUTHORITY": xauth,
        "QT_X11_NO_MITSHM": 1
    }
    list_spawn_points = town_map.get_spawn_points()
    sp = list_spawn_points[11]
    loc = sp.location
    rot = sp.rotation
    sp_str = "{},{},{},{},{},{}".format(loc.x, loc.y, loc.z, rot.roll,
                                        rot.pitch, rot.yaw * -1)
    autoware_cla = "{} \'{}\' \'{}\'".format(town_map.name.split("/")[-1], sp_str, conf.sim_port)
    print(autoware_cla)
    exec_state.autoware_cmd = autoware_cla
    while autoware_container is None:
        try:
            autoware_container = docker_client.containers.run(
                "carla-autoware:improved2",
                command=autoware_cla,
                detach=True,
                auto_remove=True,
                name="autoware-{}".format(os.getenv("USER")),
                volumes=vol_dict,
                privileged=True,
                network_mode="host",
                # runtime="nvidia",
                device_requests=[
                    docker.types.DeviceRequest(device_ids=["all"], capabilities=[['gpu']])],
                environment=env_dict,
                stdout=True,
                stderr=True
            )

        except docker.errors.APIError as e:
            print("[-] Could not launch docker:", e)
            if "Conflict" in str(e):
                os.system("docker rm -f autoware-{}".format(
                    os.getenv("USER")))
                killed = True
            time.sleep(1)
        except:
            # https://github.com/docker/for-mac/issues/4957
            print("[-] Fatal error. Check dmesg")
            exit(-1)
    while True:
        running_container_list = docker_client.containers.list()
        if autoware_container in running_container_list:
            break
        print("[*] Waiting for Autoware container to be launched")
        time.sleep(1)

    # wait for autoware bridge to spawn player vehicle
    autoware_agent_found = False
    i = 0
    while True:
        print("[*] Waiting for Autoware agent " + "." * i + "\r", end="")
        vehicles = world.get_actors().filter("*vehicle.*")
        for vehicle in vehicles:
            if vehicle.attributes["role_name"] == "ego_vehicle":
                autoware_agent_found = True
                break
        if autoware_agent_found:
            break
        if i > 60:
            print("\n something is wrong")
            exit(-1)
        i += 1
        time.sleep(0.5)

    i = 0
    time.sleep(3)
    while True:
        proc1 = Popen(["rostopic", "list"], stdout=PIPE)
        proc2 = Popen(["wc", "-l"], stdin=proc1.stdout, stdout=PIPE)
        print("[*] Waiting for Autoware nodes " + "." * i + "\r", end="")
        output = proc2.communicate()[0]
        print(int(output))
        if int(output) >= c.WAIT_AUTOWARE_NUM_TOPICS:
            # FIXME: hardcoding the num of topics :/
            # on top of that, each vehicle adds one topic, and any walker
            # contribute to two pedestrian topics.
            print("")
            break
        i += 1
        if i == 60:
            carla_error = True
            print("    [-] something went wrong while launching Autoware.")
            raise KeyboardInterrupt
        time.sleep(1)
        world.tick()
    proc1.stdout.close()
    proc2.stdout.close()
    # exec a detached process that monitors the output of Autoware's
    # decision-maker state, with which we can get an idea of when Autoware
    # thinks it has reached the goal
    exec_state.proc_state = Popen(["rostopic echo /decision_maker/state"],
                                  shell=True, stdout=PIPE, stderr=PIPE)
    # set_camera(conf, player, spectator)
    # Wait for Autoware (esp, for Town04)
    # i = 0
    # while True:
    #     output_state = state.proc_state.stdout.readline()
    #     if b"---" in output_state:
    #         output_state = state.proc_state.stdout.readline()
    #     if b"VehicleReady" in output_state:
    #         break
    #     i += 1
    #     if i == 45:
    #         carla_error = True
    #         print("    [-] something went wrong while launching Autoware.")
    #         raise KeyboardInterrupt
    #     time.sleep(1)
    time.sleep(3)
    return carla_error

def autoware_universe_launch(carla_error, world, conf, town_map):
    username = os.getenv("USER")
    global autoware_universe_container
    num_walker_topics = 0
    docker_client = docker.from_env()
    proj_root = config.get_proj_root()
    xauth = os.path.join(os.getenv("HOME"), ".Xauthority")
    vol_dict = {
        "/home/chenpansong/shared/op_carla": {
            "bind": "/root/op_carla",
            "mode": "rw"
        },
        "/home/chenpansong/shared/carla-0.9.13": {
            "bind": "/root/carla-0.9.13",
            "mode": "rw"
        },
        "/home/chenpansong/shared/autoware_map": {
            "bind": "/root/autoware_map",
            "mode": "ro"
        },
        f"/home/{username}/.Xauthority": {
            "bind": xauth,
            "mode": "rw"
        },
        "/tmp/.X11-unix": {
            "bind": "/tmp/.X11-unix",
            "mode": "rw"
        },
    }
    env_dict = {
        "DISPLAY": os.getenv("DISPLAY"),
        "XAUTHORITY": xauth,
        "QT_X11_NO_MITSHM": 1
    }
    list_spawn_points = town_map.get_spawn_points()
    sp = list_spawn_points[11]
    loc = sp.location
    rot = sp.rotation
    sp_str = "'{},{},{},{},{},{}'".format(loc.x, loc.y, loc.z, rot.roll,
                                        rot.pitch, rot.yaw )
    map_name = town_map.name.split("/")[-1]
    docker_network_host = "172.17.0.1"
    load_cmd = "/root/op_carla/op_bridge/op_scripts/run_ros2.sh "
    load_cmd = load_cmd + f"-m {map_name} -p {sp_str} -H {docker_network_host} "
    load_cmd = load_cmd + f"-P {conf.sim_port} -a ego_vehicle "
    docker_cmd = f"bash -c \"{load_cmd}\" "
    print(docker_cmd)
    exec_state.autoware_universe_cmd = docker_cmd
    while autoware_universe_container is None:
        try:
            autoware_universe_container = docker_client.containers.run(
                "carla-autoware-universe:improve_v_1_0",
                command=docker_cmd,
                detach=True,
                auto_remove=True,
                name="carla-autoware-universe-{}".format(os.getenv("USER")),
                volumes=vol_dict,
                privileged=False,
                network_mode="bridge",
                # runtime="nvidia",
                device_requests=[
                    docker.types.DeviceRequest(device_ids=["all"], capabilities=[['gpu']])],
                environment=env_dict,
                stdout=True,
                stderr=True
            )

        except docker.errors.APIError as e:
            print("[-] Could not launch docker:", e)
            if "Conflict" in str(e):
                os.system("docker rm -f carla-autoware-universe-{}".format(
                    os.getenv("USER")))
                killed = True
            time.sleep(1)
        except:
            # https://github.com/docker/for-mac/issues/4957
            print("[-] Fatal error. Check dmesg")
            exit(-1)
    while True:
        running_container_list = docker_client.containers.list()
        if autoware_universe_container in running_container_list:
            break
        print("[*] Waiting for Autoware container to be launched")
        time.sleep(1)
    au_containner_network_settings = autoware_universe_container.attrs['NetworkSettings']
    au_containner_host = au_containner_network_settings['IPAddress']
    pdb.set_trace()

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
        if math.sqrt((sp_x - wp_x) ** 2 + (sp_y - wp_y) ** 2) > 100:
            destination_flag = False
        if math.sqrt((sp_x - wp_x) ** 2 + (sp_y - wp_y) ** 2) > 200:
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


def main():
    # STEP 0: init env
    global client, world, G

    carla_error = False
    conf, town, town_map, exec_state.client, exec_state.world, exec_state.G = init_env()
    if conf.agent_type == c.AUTOWARE:
        carla_error = autoware_launch(carla_error, exec_state.world, conf, town)
    elif conf.agent_type == c.AUTOWARE_UNIVERSE:
        carla_error = autoware_universe_launch(carla_error, exec_state.world, conf, town)
    population = []
    # GA Hyperparameters
    POP_SIZE = 3  # amount of population
    OFF_SIZE = 10  # number of offspring to produce
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
        # STEP 1: choice a seed in seed pool
        curr_gen += 1
        print(f' ====== GA Generation {curr_gen} ====== ')
        # Vary the population
        offspring = algorithms.varOr(
            population, toolbox, OFF_SIZE, CXPB, MUTPB)
        # update chromosome gid and cid
        for index, d in enumerate(offspring):
            d.gid = curr_gen
            d.cid = index
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

        # vt.save_to_file()
        # with open('./data/log.bin', 'wb') as fp:
        #     pickle.dump(logbook, fp)
        # with open('./data/hof.bin', 'wb') as fp:
        #     pickle.dump(hof, fp)

        # curr_time = datetime.now()
        # tdelta = (curr_time - start_time).total_seconds()
        # if tdelta / 3600 > RUN_FOR_HOUR:
        #     break


if __name__ == "__main__":
    main()
