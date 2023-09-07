#!/usr/bin/env python3
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
import datetime
from deap import base, tools, algorithms
import signal
import traceback
import networkx as nx
from shapely.geometry import LineString
import config
import constants as c
import states

import scenario
import utils

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("[-] Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("    Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)


def create_test_scenario(conf, seed_dict):
    return scenario.Scenario(conf, seed_dict)


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

#
# def eval_scenario(ind: Scenario):
#     g_name = f'Generation_{ind.gid:05}'
#     s_name = f'Scenario_{ind.cid:05}'
#     srunner = ScenarioRunner.get_instance()
#     srunner.set_scenario(ind)
#     srunner.init_scenario()
#     runners = srunner.run_scenario(g_name, s_name, True)
#
#     obs_routing_map = dict()
#     for a, r in runners:
#         obs_routing_map[a.nid] = r.routing_str
#
#     unique_violation = 0
#     duplicate_violation = 0
#     min_distance = list()
#     decisions = set()
#     for a, r in runners:
#         min_distance.append(a.get_min_distance())
#         decisions.update(a.get_decisions())
#         c_name = a.container.container_name
#         r_name = f"{c_name}.{s_name}.00000"
#         record_path = os.path.join(RECORDS_DIR, g_name, s_name, r_name)
#         ra = RecordAnalyzer(record_path)
#         ra.analyze()
#         for v in ra.get_results():
#             main_type = v[0]
#             sub_type = v[1]
#             if main_type == 'collision':
#                 if sub_type < 100:
#                     # pedestrian collisoin
#                     related_data = frozenset(
#                         [r.routing_str, ind.pd_section.pds[sub_type].cw_id])
#                     sub_type = 'A&P'
#                 else:
#                     # adc to adc collision
#                     related_data = frozenset(
#                         [r.routing_str, obs_routing_map[sub_type]]
#                     )
#                     sub_type = 'A&A'
#             else:
#                 related_data = r.routing_str
#             if ViolationTracker.get_instance().add_violation(
#                     gname=g_name,
#                     sname=s_name,
#                     record_file=record_path,
#                     mt=main_type,
#                     st=sub_type,
#                     data=related_data
#             ):
#                 unique_violation += 1
#
#     ma = MapParser.get_instance()
#     conflict = ind.has_ad_conflict()
#
#     if unique_violation == 0:
#         # no unique violation, remove records
#         remove_record_files(g_name, s_name)
#         pass
#
#     return min(min_distance), len(decisions), conflict, unique_violation
#
#
# # MUTATION OPERATOR
#
#
# def mut_ad_section(ind: ADSection):
#     mut_pb = random()
#
#     # remove a random 1
#     if mut_pb < 0.1 and len(ind.adcs) > 2:
#         shuffle(ind.adcs)
#         ind.adcs.pop()
#         ind.adjust_time()
#         return ind
#
#     # add a random 1
#     if mut_pb < 0.4 and len(ind.adcs) < MAX_ADC_COUNT:
#         while True:
#             new_ad = ADAgent.get_one()
#             if ind.has_conflict(new_ad) and ind.add_agent(new_ad):
#                 break
#         ind.adjust_time()
#         return ind
#
#     # mutate a random agent
#     index = randint(0, len(ind.adcs) - 1)
#     routing = ind.adcs[index].routing
#     original_adc = ind.adcs.pop(index)
#     mut_counter = 0
#     while True:
#         if ind.add_agent(ADAgent.get_one_for_routing(routing)):
#             break
#         mut_counter += 1
#         if mut_counter == 5:
#             # mutation kept failing, dont mutate
#             ind.add_agent(original_adc)
#             pass
#     ind.adjust_time()
#     return ind
#
#
# def mut_pd_section(ind: PDSection):
#     if len(ind.pds) == 0:
#         ind.add_agent(PDAgent.get_one())
#         return ind
#
#     mut_pb = random()
#     # remove a random
#     if mut_pb < 0.2 and len(ind.pds) > 0:
#         shuffle(ind.pds)
#         ind.pds.pop()
#         return ind
#
#     # add a random
#     if mut_pb < 0.4 and len(ind.pds) <= MAX_PD_COUNT:
#         ind.pds.append(PDAgent.get_one())
#         return ind
#
#     # mutate a random
#     index = randint(0, len(ind.pds) - 1)
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
#
#
# def mut_scenario(ind: Scenario):
#     mut_pb = random()
#     if mut_pb < 1 / 3:
#         ind.ad_section = mut_ad_section(ind.ad_section)
#     elif mut_pb < 2 / 3:
#         ind.pd_section = mut_pd_section(ind.pd_section)
#     else:
#         ind.tc_section = mut_tc_section(ind.tc_section)
#     return ind,
#
#
# # CROSSOVER OPERATOR
#
# def cx_ad_section(ind1: ADSection, ind2: ADSection):
#     # swap entire ad section
#     cx_pb = random()
#     if cx_pb < 0.05:
#         return ind2, ind1
#
#     cxed = False
#
#     for adc1 in ind1.adcs:
#         for adc2 in ind2.adcs:
#             if adc1.routing_str == adc2.routing_str:
#                 # same routing in both parents
#                 # swap start_s and start_t
#                 if random() < 0.5:
#                     adc1.start_s = adc2.start_s
#                 else:
#                     adc1.start_t = adc2.start_t
#                 mutated = True
#     if cxed:
#         ind1.adjust_time()
#         return ind1, ind2
#
#     if len(ind1.adcs) < MAX_ADC_COUNT:
#         for adc in ind2.adcs:
#             if ind1.has_conflict(adc) and ind1.add_agent(deepcopy(adc)):
#                 # add an agent from parent 2 to parent 1 if there exists a conflict
#                 ind1.adjust_time()
#                 return ind1, ind2
#
#     # if none of the above happened, no common adc, no conflict in either
#     # combine to make a new populations
#     available_adcs = ind1.adcs + ind2.adcs
#     shuffle(available_adcs)
#     split_index = randint(2, min(len(available_adcs), MAX_ADC_COUNT))
#
#     result1 = ADSection([])
#     for x in available_adcs[:split_index]:
#         result1.add_agent(deepcopy(x))
#
#     # make sure offspring adc count is valid
#
#     while len(result1.adcs) > MAX_ADC_COUNT:
#         result1.adcs.pop()
#
#     while len(result1.adcs) < 2:
#         new_ad = ADAgent.get_one()
#         if result1.has_conflict(new_ad) and result1.add_agent(new_ad):
#             break
#     result1.adjust_time()
#     return result1, ind2
#
#
# def cx_pd_section(ind1: PDSection, ind2: PDSection):
#     cx_pb = random()
#     if cx_pb < 0.1:
#         return ind2, ind1
#
#     available_pds = ind1.pds + ind2.pds
#
#     result1 = PDSection(
#         sample(available_pds, k=randint(0, min(MAX_PD_COUNT, len(available_pds)))))
#     result2 = PDSection(
#         sample(available_pds, k=randint(0, min(MAX_PD_COUNT, len(available_pds)))))
#     return result1, result2
#
#
# def cx_tc_section(ind1: TCSection, ind2: TCSection):
#     cx_pb = random()
#     if cx_pb < 0.1:
#         return ind2, ind1
#     elif cx_pb < 0.4:
#         ind1.initial, ind2.initial = ind2.initial, ind1.initial
#     elif cx_pb < 0.7:
#         ind1.final, ind2.final = ind2.final, ind1.final
#     else:
#         ind1.duration_g, ind2.duration_g = ind2.duration_g, ind1.duration_g
#     return ind1, ind2
#
#
# def cx_scenario(ind1: Scenario, ind2: Scenario):
#     cx_pb = random()
#     if cx_pb < 0.6:
#         ind1.ad_section, ind2.ad_section = cx_ad_section(
#             ind1.ad_section, ind2.ad_section
#         )
#     elif cx_pb < 0.6 + 0.2:
#         ind1.pd_section, ind2.pd_section = cx_pd_section(
#             ind1.pd_section, ind2.pd_section
#         )
#     else:
#         ind1.tc_section, ind2.tc_section = cx_tc_section(
#             ind1.tc_section, ind2.tc_section
#         )
#     return ind1, ind2
#

def main():
    # STEP 0: init env
    conf, town, town_map, client, world, G = init_env()
    # GA Hyperparameters
    POP_SIZE = 10  # amount of population
    OFF_SIZE = 10  # number of offspring to produce
    CXPB = 0.8  # crossover probablitiy
    MUTPB = 0.2  # mutation probability

    toolbox = base.Toolbox()
    # toolbox.register("evaluate", eval_scenario)
    # toolbox.register("mate", cx_scenario)
    # toolbox.register("mutate", mut_scenario)
    toolbox.register("select", tools.selNSGA2)
    while True:
        # Main loop
        # STEP 1: choice a seed in seed pool
        # init a seed if seed pool is empty
        seed_dict = seed_initialize(town, town_map)
        # wait for a moment
        time.sleep(3)
        # STEP 2: TEST CASE INITIALIZATION
        # Creates and initializes a Scenario instance based on the metadata
        try:
            with concurrent.futures.ThreadPoolExecutor() as my_simulate:
                future = my_simulate.submit(create_test_scenario, conf, seed_dict)
                test_scenario = future.result(timeout=15)
        except concurrent.futures.TimeoutError:
            print("Test scenario creation timed out after 15 seconds.")
            continue
        if conf.debug:
            print("[debug] USING SEED FILE:", scenario)
        while True:
            # STEP 3: EXECUTE SIMULATION
            ret = None
            state = states.State()
            state.client = client
            state.world = world
            state.G = G
            # for test
            mutate_weather_fixed(test_scenario)
            signal.alarm(12 * 60 * 60)  # timeout after 12 hours
            try:
                ret = test_scenario.run_test(state)
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
                break
            elif ret == 128:
                print("Exit by user request")
                exit(0)
            else:
                if ret == -1:
                    print("[-] Fatal error occurred during test")
                    exit(-1)
            # mutation loop ends
        if test_scenario.found_error:
            print("[-]error detected. start a new cycle with a new seed")
            continue


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
    return conf, town, town_map, client, world, G


if __name__ == "__main__":
    main()
