#!/usr/bin/env python3
import os
import pdb
import sys
import time
import random
import argparse
import json
from collections import deque
import concurrent.futures
import math
import datetime
import signal
import traceback
import networkx as nx
from shapely.geometry import LineString
import globals as g
import config
import constants as c
import states

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("[-] Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("    Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)
import scenario
import utils


def create_test_scenario(conf):
    return scenario.Scenario(conf)


def handler(signum, frame):
    raise Exception("HANG")


def init(conf, args):
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
    except Exception as e:
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
    conf.sim_tm_port = args.sim_tm_port

    conf.max_mutations = args.max_mutations
    conf.timeout = args.timeout
    conf.function = args.function

    if args.target.lower() == "basic":
        conf.agent_type = c.BASIC
    elif args.target.lower() == "behavior":
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
    test_scenario.weather["puddle"] = random.randint(0, 100)
    test_scenario.weather["wind"] = random.randint(0, 100)
    test_scenario.weather["fog"] = random.randint(0, 100)
    test_scenario.weather["wetness"] = random.randint(0, 100)
    test_scenario.weather["angle"] = random.randint(0, 360)
    test_scenario.weather["altitude"] = random.randint(-90, 90)


def mutate_weather_fixed(test_scenario):
    test_scenario.weather["cloud"] = 0
    test_scenario.weather["rain"] = 0
    test_scenario.weather["puddle"] = 0
    test_scenario.weather["wind"] = 0
    test_scenario.weather["fog"] = 0
    test_scenario.weather["wetness"] = 0
    test_scenario.weather["angle"] = 0
    test_scenario.weather["altitude"] = 60


def set_args():
    argparser = argparse.ArgumentParser()
    argparser.add_argument("--debug", action="store_true", default=False)
    argparser.add_argument("-o", "--out-dir", default="out-artifact", type=str,
                           help="Directory to save fuzzing logs")
    argparser.add_argument("-s", "--seed-dir", default="seed-artifact", type=str,
                           help="Seed directory")
    argparser.add_argument("-m", "--max-mutations", default=1, type=int,
                           help="Size of the mutated population per cycle")
    argparser.add_argument("-d", "--determ-seed", type=float,
                           help="Set seed num for deterministic mutation (e.g., for replaying)")
    argparser.add_argument("-u", "--sim-host", default="localhost", type=str,
                           help="Hostname of Carla simulation server")
    argparser.add_argument("-p", "--sim-port", default=2000, type=int,
                           help="RPC port of Carla simulation server")
    argparser.add_argument("--sim-tm-port", default=8000, type=int,
                           help="RPC port of Carla traffic manager server")
    argparser.add_argument("-t", "--target", default="behavior", type=str,
                           help="Target autonomous driving system (behavior/Autoware)")
    argparser.add_argument("-f", "--function", default="general", type=str,
                           choices=["general", "collision", "traction", "eval-os", "eval-us",
                                    "figure", "sens1", "sens2", "lat", "rear"],
                           help="Functionality to test (general / collision / traction)")
    argparser.add_argument("-k", "--num_mutation_car", default=3, type=int,
                           help="Number of max weight vehicles to mutation per cycle, default=1,negative means random")
    argparser.add_argument("--density", default=1, type=float,
                           help="density of vehicles,1.0 means add 1 bg vehicle per 1 sec")
    argparser.add_argument("--town", default=3, type=int,
                           help="Test on a specific town (e.g., '--town 3' forces Town03)")
    argparser.add_argument("--timeout", default="30", type=int,
                           help="Seconds to timeout if vehicle is not moving")
    argparser.add_argument("--no-speed-check", action="store_true")
    argparser.add_argument("--no-lane-check", action="store_true")
    argparser.add_argument("--no-crash-check", action="store_true")
    argparser.add_argument("--no-stuck-check", action="store_true")
    argparser.add_argument("--no-red-check", action="store_true")
    argparser.add_argument("--no-other-check", action="store_true")
    argparser.add_argument("--no-traffic-lights", action="store_true")
    return argparser


def main():
    conf = config.Config()
    argparser = set_args()
    args = argparser.parse_args()
    init(conf, args)
    queue = deque(conf.enqueue_seed_scenarios())
    scene_id = 0
    campaign_cnt = 0
    signal.signal(signal.SIGALRM, handler)
    while True:
        # STEP 0: Restart Carla simulator at the beginning of each cycle
        # (Carla hangs after a while due to a memory leak)
        # UPDATE: can't do this due to a bug in Carla. TimeoutException will
        # be raised when we stop the container.
        # print("[*] Stopping existing Carla simulator")
        # os.system(f"docker rm -f carla-{os.getenv('USER')}")
        # for i in range(10):
        # print(".", end="", flush=True)
        # time.sleep(1)
        # print()

        # print("[*] Starting Carla simulator")
        # os.system("../run_carla.sh")
        # for i in range(30):
        # print(".", end="", flush=True)
        # time.sleep(1)
        # print()

        # STEP 1: SEED
        # Pop a seed from the seed queue. If seed queue is empty, ramdonly
        # generate a seed, enqueue it, and come back here.

        # print("before seed gen", time.time())
        try:
            scenario = queue.popleft()
            conf.cur_scenario = scenario
            scene_id += 1
            campaign_cnt += 1
            print("\n\033[1m\033[92m" + "=" * 10 + " NEW CAMPAIGN " + "=" * 10 + "\033[0m\n")

        except IndexError:
            print("[-] Seed queue is empty. Continue with random seed.")

            if conf.town is not None:
                town_map = "Town0{}".format(conf.town)
            else:
                town_map = "Town0{}".format(random.randint(1, 5))
            (client, tm) = utils.connect(conf)
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
            g.topography = G
            # pos = nx.kamada_kawai_layout(G)
            # nx.draw(G, pos, with_labels=False, node_size=200, node_color="gray",
            #         edge_color="skyblue")
            # plt.show()
            # pdb.set_trace()
            spawn_points = town.get_spawn_points()
            sp = random.choice(spawn_points)
            sp_x = sp.location.x
            sp_y = sp.location.y
            sp_z = sp.location.z
            pitch = sp.rotation.pitch
            yaw = sp.rotation.yaw
            roll = sp.rotation.roll
            # restrict destinition to be within 200 meters
            destinition_flag = True
            while destinition_flag:
                wp = random.choice(spawn_points)
                wp_x = wp.location.x
                wp_y = wp.location.y
                wp_z = wp.location.z
                wp_yaw = wp.rotation.yaw
                if math.sqrt((sp_x - wp_x) ** 2 + (sp_y - wp_y) ** 2) > 100:
                    destinition_flag = False
                if math.sqrt((sp_x - wp_x) ** 2 + (sp_y - wp_y) ** 2) > 200:
                    destinition_flag = True
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
            scene_name = "scene-created{}.json".format(scene_id)
            with open(os.path.join(conf.seed_dir, scene_name), "w") as fp:
                json.dump(seed_dict, fp)
            queue.append(scene_name)
            continue
        # wait for a moment
        time.sleep(3)
        # STEP 2: TEST CASE INITIALIZATION
        # Create and initialize a Scenario instance based on the metadata
        try:
            with concurrent.futures.ThreadPoolExecutor() as my_simulate:
                future = my_simulate.submit(create_test_scenario, conf)
                test_scenario = future.result(timeout=15)
        except concurrent.futures.TimeoutError:
            print("Test scenario creation timed out after 15 seconds.")
            continue
        if conf.debug:
            print("[debug] USING SEED FILE:", scenario)
        # STEP 3: SCENE MUTATION
        round_cnt = 0
        while round_cnt < conf.max_mutations:  # mutation rounds
            round_cnt += 1
            print("\n\033[1m\033[92mCampaign #{}  Mutation #{}/{}".format(
                campaign_cnt, round_cnt,
                conf.max_mutations), "\033[0m", datetime.datetime.now())
            # STEP 3-1: EXECUTE SIMULATION
            ret = None
            state = states.State()
            state.campaign_cnt = campaign_cnt
            state.mutation = round_cnt
            # for test
            mutate_weather_fixed(test_scenario)
            signal.alarm(15 * 60)  # timeout after 15 mins
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
            # STEP 3-2: mutation due to weight
            # STEP 3-4: DECIDE WHAT TO DO BASED ON THE RESULT OF EXECUTION
            print("ret", ret)
            if ret is None:
                # failure
                # Find the k vehicles with the highest weight
                k = conf.num_mutation_car
                max_actors = []
                for actor in test_scenario.actor_list:
                    max_actors.append(actor)
                if k < 0:
                    random.shuffle(max_actors)
                else:
                    max_actors.sort(key=lambda x: x.weight, reverse=True)
                # choose some good cars
                max_actors = max_actors[:abs(k * 2)]
                random.shuffle(max_actors)
                max_actors = max_actors[:abs(k)]
                # randomly mutate the max weight actor
                for actor in max_actors:
                    # todo: change the logic of mutation
                    mutation_type = random.randint(2, 3)
                    if mutation_type == 0:
                        # change the actor's location
                        # prove to be useless
                        pass
                    elif mutation_type == 1:
                        # change the actor's velocity
                        # prove to be useless
                        velocity_change = random.randint(-5, 10)
                        actor.speed = actor.speed + velocity_change
                        print("speed change:", actor.actor_id, "velocity_change:", velocity_change)
                    elif mutation_type == 2:
                        """
                        Randomly change the actor's behavior according to the state of actor
                        
                        The vehicle is in front of the ego: brake
                        The vehicle is in a different lane to the left of the ego, 
                        and the lane change is allowed on the left of the ego: change lanes to the right
                        The vehicle is in a different lane on the right side of the ego, 
                        and the right side of the ego is allowed to change lanes: left lane change
                        The vehicle is behind the ego: throttle
                        """
                        behavior_list = []
                        if actor.max_weight_loc == c.FRONT:
                            behavior_list.append(c.BRAKE)
                        if actor.max_weight_lane == c.LEFT:
                            if actor.player_lane_change == carla.LaneChange.Left or actor.player_lane_change == carla.LaneChange.Both:
                                behavior_list.append(c.MOVE_TO_THE_LEFT)
                            if actor.max_weight_loc == c.BACK:
                                behavior_list.append(c.THROTTLE)
                        elif actor.max_weight_lane == c.RIGHT:
                            if actor.player_lane_change == carla.LaneChange.Right or actor.player_lane_change == carla.LaneChange.Both:
                                behavior_list.append(c.MOVE_TO_THE_RIGHT)
                            if actor.max_weight_loc == c.BACK:
                                behavior_list.append(c.THROTTLE)
                        # # Randomly take a behavior from behavior_list
                        # behavior_id = random.choice(behavior_list)
                        # actor.add_event(actor.max_weight_frame, behavior_id)
                        # print("behavior change:", actor.actor_id, "id:", behavior_id)
                    elif mutation_type == 3:
                        # split the actor
                        new_actor = actor.splitting(g.town_map, len(test_scenario.actor_list))
                        test_scenario.actor_list.append(new_actor)
                        g.test_split_1 = actor
                        g.test_split_2 = new_actor
                        new_actor.is_split = True
                # change all weights to 0
                for actor in test_scenario.actor_list:
                    actor.weight = 0
                pass
            elif ret == -1:
                print("Spawn / simulation failure - don't add round cnt")
                round_cnt -= 1
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


if __name__ == "__main__":
    main()
