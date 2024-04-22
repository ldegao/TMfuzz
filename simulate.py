#!/usr/bin/env python3
import fcntl
import glob
import logging
# Python packages
import os
import pdb
import random
import select
import shutil
import sys
import threading
import signal
import time
import math
import traceback
from subprocess import Popen, PIPE

import docker
import networkx as nx
import numpy as np
import pygame
from npc import NPC
import config
import constants as c
from utils import quaternion_from_euler, set_traffic_lights_state, get_angle_between_vectors, \
    set_autopilot, delete_npc, check_autoware_status, mark_npc, timeout_handler

config.set_carla_api_path()
try:
    import carla

    print(carla.__file__)
except ModuleNotFoundError as e:
    print("Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)
# to import carla agents
try:
    proj_root = config.get_proj_root()
    sys.path.append(os.path.join(proj_root, "carla", "PythonAPI", "carla"))
except IndexError:
    pass
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error

username = os.getenv("USER")
docker_client = docker.from_env()


def record_min_distance(npc_vehicles, player_loc, state):
    min_dist = state.min_dist
    closest_car = None
    for npc_vehicle in npc_vehicles:
        distance = npc_vehicle.get_location().distance(player_loc)
        if distance < min_dist:
            min_dist = distance
            closest_car = npc_vehicle
    if min_dist < state.min_dist:
        state.min_dist = min_dist
        state.closest_car = closest_car


def simulate(conf, state, exec_state, sp, wp, weather_dict, npc_list):
    # simulate() is always called by Scenario instance,
    # so we won't need to switch a map unless a new instance is created.
    # switch_map(conf, client, town)

    # always reuse the existing client instance
    assert (exec_state.client is not None)

    retval = 0
    wait_until_end = 0
    max_wheels_for_non_motorized = 2
    carla_error = False
    state.min_dist = 99999
    player_loc = None
    time_start = time.time()
    npc_now = []
    agents_now = []
    sensors = []
    npc_vehicles = []
    npc_walkers = []
    trace_graph = []
    trace_graph_important = []
    nearby_dict = []

    # for autoware
    frame_gap = 0
    autoware_last_frames = 0
    autoware_stuck = 0
    client = exec_state.client
    world = exec_state.world
    goal_loc = wp.location
    goal_rot = wp.rotation

    trace_dict = {}
    valid_frames = 0
    all_frame = 0
    try:

        # initialize the simulation and the ego vehicle

        add_car_frame, blueprint_library, clock, player_bp, town_map, vehicle_bp_library = simulate_initialize(client,
                                                                                                               conf,
                                                                                                               weather_dict,
                                                                                                               world)
        autoware_container, ego, player, max_steer_angle = ego_initialize(
            agents_now, exec_state,
            blueprint_library, conf,
            player_bp, sensors,
            sp, state,
            world, wp)
        trace_dict[player.id] = []
        if conf.agent_type == c.AUTOWARE:
            autoware_goal_publish(goal_loc, goal_rot, state, world)

        # SIMULATION LOOP FOR AUTOWARE and BasicAgent
        signal.signal(signal.SIGALRM, timeout_handler)
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGSEGV, state.sig_handler)
        signal.signal(signal.SIGABRT, state.sig_handler)

        try:

            found_frame = -999
            snapshot0 = world.get_snapshot()
            first_frame_id = snapshot0.frame
            first_sim_time = snapshot0.timestamp.elapsed_seconds
            last_frame_id = first_frame_id
            state.first_frame_id = first_frame_id
            state.sim_start_time = snapshot0.timestamp.platform_timestamp
            state.num_frames = 0
            state.elapsed_time = 0
            frame_speed_lim_changed = 0
            s_started = False
            # actual monitoring of the driving simulation begins here
            print("START DRIVING: {} {}".format(first_frame_id, first_sim_time))
            # mark goal position
            if conf.debug:
                world.debug.draw_box(
                    box=carla.BoundingBox(
                        goal_loc,
                        carla.Vector3D(0.2, 0.2, 1.0)
                    ),
                    rotation=goal_rot,
                    life_time=0,
                    thickness=1.0,
                    color=carla.Color(r=0, g=255, b=0)
                )
            # simulate start here
            state.end = False
            time_start = time.time()
            while True:
                # world tick
                if conf.agent_type == c.BEHAVIOR:
                    world.tick()
                # Use sampling frequency of FPS  for precision
                clock.tick(c.FRAME_RATE)
                # Get frame info
                snapshot = world.get_snapshot()
                cur_frame_id = snapshot.frame
                cur_sim_time = snapshot.timestamp.elapsed_seconds
                if cur_frame_id <= last_frame_id:
                    # skip if we got the same frame data as last
                    continue
                last_frame_id = cur_frame_id  # update last
                state.num_frames = cur_frame_id - first_frame_id
                state.elapsed_time = cur_sim_time - first_sim_time

                frame_speed_lim_changed, player_lane_id, player_loc, player_road_id, player_rot, speed, speed_limit, vel = get_player_info(
                    cur_frame_id, goal_loc, player, sp, state, town_map, conf)

                # drive-fuzz's thing, not sure if we need it yaw = sp.rotation.yaw calculate_control(npc_vehicles,
                # npc_walkers, max_steer_angle, player, player_loc, player_rot, state, vel, yaw)

                # Check destination
                break_flag, retval, autoware_stuck, s_started = check_destination(npc_vehicles, npc_now,
                                                                                  agents_now, autoware_stuck, conf,
                                                                                  goal_loc, goal_rot, player_loc,
                                                                                  exec_state.proc_state, retval,
                                                                                  s_started, sensors, speed, state,
                                                                                  world)
                # record the min distance between every two npcs
                record_min_distance(npc_vehicles, player_loc, state)
                # mark useless vehicles for any frame
                mark_useless_npc(npc_now, conf, player_lane_id, player_loc, player_rot, player_road_id, exec_state.G, town_map)

                # add old vehicles for any frame
                found_frame = add_old_npc(npc_list, npc_vehicles, npc_now, agents_now, conf,
                                          found_frame, max_wheels_for_non_motorized, player_loc,
                                          sensors, state, vel, world, wp)
                # add a new npc per 1s here
                # because of autoware's strange frame, we should use a interesting method
                found_frame, autoware_last_frames, frame_gap = add_new_car(npc_list, npc_vehicles, npc_now,
                                                                           add_car_frame,
                                                                           agents_now,
                                                                           autoware_last_frames, conf, ego, found_frame,
                                                                           frame_gap,
                                                                           goal_loc, goal_rot,
                                                                           max_wheels_for_non_motorized, player_lane_id,
                                                                           player_loc, player_road_id,
                                                                           sensors, state, town_map, vehicle_bp_library,
                                                                           world, wp, exec_state.G)
                control_npc(agents_now, speed_limit)
                # delete vehicles which life is end
                for npc in npc_list:
                    if npc.instance is not None:
                        if npc.death_time == 0:
                            delete_npc(npc, npc_vehicles, sensors, agents_now, npc_now)
                        elif npc.death_time > 0:
                            npc.death_time -= 1

                # record track of every npc_vehicle
                if break_flag:
                    break
                if wait_until_end == 0:
                    valid_frames = nearby_record(state, npc_vehicles, player, trace_dict, player_loc, town_map,
                                                 valid_frames,
                                                 exec_state.G)
                    all_frame += 1
                    retval, wait_until_end = check_violation(conf, cur_frame_id, frame_speed_lim_changed, retval, speed,
                                                             speed_limit, state, wait_until_end)
                else:
                    wait_until_end += 1
                if wait_until_end > 6:
                    break
            # find the biggest weight of npc-list
            nearby_dict, trace_graph_important = record_trace(npc_vehicles, exec_state, player, player_loc, state,
                                                              town_map, trace_dict,
                                                              trace_graph, trace_graph_important)
            state.trace_graph_important = trace_graph_important
            state.nearby_dict = nearby_dict
        except KeyboardInterrupt:
            print("quitting")
            retval = 128
        # jump to finally
        return
    except Exception:
        # update states
        # state.num_frames = frame_id - frame_0
        # state.elapsed_time = time.time() - start_time
        print("[-] Runtime error:")
        traceback.print_exc()
        exc_type, exc_obj, exc_tb = sys.exc_info()
        print("   (line #{0}) {1}".format(exc_tb.tb_lineno, exc_type))
        retval = -1
    finally:
        signal.alarm(0)
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        all_time = time.time() - time_start
        FPS = all_frame / all_time
        valid_time = valid_frames / FPS
        logging.info("crashed:%s", state.crashed)
        logging.info("nearby_car:%s", len(nearby_dict))
        logging.info("valid_time/time: %s/%s", valid_time, all_time)
        logging.info("distance:%s", state.distance)
        logging.info("FPS:%s", FPS)
        state.end = True

        if exec_state.proc_state:
            exec_state.proc_state.terminate()
            exec_state.proc_state.wait()
            exec_state.proc_state.stdout.close()
            exec_state.proc_state.stderr.close()

        # save video in output_dir
        if conf.agent_type == c.BEHAVIOR:
            save_behavior_video(carla_error, state)
        elif conf.agent_type == c.AUTOWARE:
            save_behavior_video(carla_error, state)
            os.system("rosnode kill /recorder_video_front")
            time.sleep(1)
            os.system("rosnode kill /recorder_video_top")
            time.sleep(1)
            # os.system("rosnode kill /recorder_bag")
            # while os.path.exists(f"/tmp/fuzzerdata/{c.USERNAME}/bagfile.lz4.bag.active"):
            #     print("waiting for rosbag to dump data")
            #     time.sleep(1)
        try:
            autoware_container.kill()
        except docker.errors.APIError as e:
            print("[-] Couldn't kill Autoware container:", e)
        except UnboundLocalError:
            print("[-] Autoware container was not launched")
        except:
            print("[-] Autoware container was not killed for an unknown reason")
            print("    Trying manually")
            os.system("docker rm -f autoware-{}".format(os.getenv("USER")))
        # Finalize simulation
        if not is_carla_running():
            retval = -1
        if retval == -1:
            print("[debug] exit because of Runtime error")
            return retval, npc_list, state
        # remove ego
        # if conf.agent_type == c.BEHAVIOR:
        #     delete_npc(ego, npc_vehicles, sensors, agents_now, npc_now)
        # remove npcs and sensors to reload the world
        # if not world_reload(npc_list, npc_vehicles, npc_walkers, npc_now, sensors, world):
        #     retval = 128
        #     if conf.debug:
        #         print("[debug] world reload fail")
        #     return retval, npc_list, state
        if conf.debug:
            print("[debug] reload")
        for npc in npc_now:
            npc.instance = None
        for npc in npc_list:
            npc.fresh = True
        client.reload_world()
        if retval == 128:
            print("[debug] exit by user requests")
            return retval, npc_list, state
        return retval, npc_list, state


def record_trace(npc_vehicles, exec_state, player, player_loc, state, town_map, trace_dict, trace_graph,
                 trace_graph_important):
    nearby_dict = {}
    for vehicle_id in trace_dict:
        if player.id == vehicle_id:
            continue
        nearby_dict[vehicle_id] = len(trace_dict[vehicle_id])
    # record nearby cars when test is end
    if town_map:
        player_waypoint = town_map.get_waypoint(player_loc, project_to_road=True,
                                                lane_type=carla.libcarla.LaneType.Driving)
    else:
        return [], []
    for npc_vehicle in npc_vehicles:
        if state.crashed:
            if state.collision_to == npc_vehicle.id:
                continue
        waypoint = town_map.get_waypoint(npc_vehicle.get_location(), project_to_road=True,
                                         lane_type=carla.libcarla.LaneType.Driving)
        if check_topo(player_waypoint, waypoint, exec_state.G):
            # trim and thin the trace,return trace at last 5 seconds
            try:
                trace = trace_dict[npc_vehicle.id]
            except KeyError:
                continue
            # trace = trace_thin(trace, 5, 25)
            trace_graph.append(trace)
    trace_graph_important.append(trace_dict[player.id])
    if state.crashed:
        # if collied to a car
        if trace_dict.keys().__contains__(state.collision_to):
            trace_graph_important.append(trace_dict[state.collision_to])
    else:
        # todo:better
        # if state.closest_car:
        #     trace_graph_important.append(trace_dict[state.closest_car])
        for trace in trace_dict:
            trace_graph_important.append(trace_dict[trace])
            break

    # Do not change the speed
    ego_start_loc = (trace_graph_important[0][0][0], trace_graph_important[0][0][1], 0)
    for i in range(len(trace_graph_important)):
        # save at most 250 points
        if len(trace_graph_important[i]) > 250:
            trace_graph_important[i] = trace_graph_important[i][-250:]
        normalize_points(trace_graph_important[i], ego_start_loc)
    # change trace_graph_important from list to ndarray
    trace_graph_important = np.array(trace_graph_important)
    return nearby_dict, trace_graph_important


def normalize_points(points, start_point):
    origin_point = np.array(start_point)
    points = np.array(points)
    for i in range(len(points)):
        points[i] = points[i] - origin_point


def nearby_record(state, npc_vehicles, player, trace_dict, player_loc, town_map, valid_frames, G):
    vel = player.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    player_waypoint = town_map.get_waypoint(player_loc, project_to_road=True,
                                            lane_type=carla.libcarla.LaneType.Driving)
    trace_dict[player.id].append((player_loc.x, player_loc.y, speed))
    has_nearby = False
    for npc_vehicle in npc_vehicles:
        if npc_vehicle.id not in trace_dict:
            trace_dict[npc_vehicle.id] = []
        waypoint = town_map.get_waypoint(npc_vehicle.get_location(), project_to_road=True,
                                         lane_type=carla.libcarla.LaneType.Driving)
        is_nearby = check_topo(player_waypoint, waypoint, G)
        npc_vehicle_speed = 3.6 * math.sqrt(
            npc_vehicle.get_velocity().x ** 2 + npc_vehicle.get_velocity().y ** 2)
        not_stuck = (npc_vehicle_speed > 0.5) or (npc_vehicle_speed > 0.5)
        if is_nearby and not_stuck:
            trace_dict[npc_vehicle.id].append(
                (npc_vehicle.get_location().x, npc_vehicle.get_location().y, npc_vehicle_speed))
            has_nearby = True
    if has_nearby and state.stuck_duration < 15 * c.FRAME_RATE:
        valid_frames += 1
    return valid_frames


def is_carla_running():
    client = docker.from_env()
    user = os.getenv("USER")
    container_name = f"carla-{user}"

    try:
        container = client.containers.get(container_name)
        return container.status != "exited"
    except docker.errors.NotFound:
        print(f"Container {container_name} not found.")
        return False
    except Exception as e:
        print(f"An error occurred: {e}")
        return False


def control_npc(agents_now, speed_limit):
    for agent_tuple in agents_now:
        # todo:rewrite here
        agent = agent_tuple[0]
        agent_vehicle = agent_tuple[1]
        agent_npc = agent_tuple[2]
        agent._update_information()
        agent.get_local_planner().set_speed(speed_limit)
        lp = agent.get_local_planner()
        if len(lp._waypoints_queue) != 0:
            control = agent.run_step()
            # that guy who has the agent
            agent_vehicle.apply_control(control)
        vel = agent_vehicle.get_velocity()
        speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        # Check inactivity
        if speed < 1:  # km/h
            agent_npc.stuck_duration += 1
        else:
            agent_npc.stuck_duration = 0


def add_new_car(npc_list, npc_vehicles, npc_now, add_car_frame, agents_now, autoware_last_frames, conf, ego,
                found_frame, frame_gap, goal_loc, goal_rot, max_wheels_for_non_motorized, player_lane_id, player_loc,
                player_road_id, sensors, state, town_map, vehicle_bp_library, world, wp, G):
    if conf.agent_type == c.AUTOWARE:
        frame_gap = frame_gap + state.num_frames - autoware_last_frames
        autoware_last_frames = state.num_frames
        add_flag = frame_gap > add_car_frame - 1
        if add_flag:
            frame_gap = frame_gap - add_car_frame
    else:
        add_flag = state.num_frames % add_car_frame == add_car_frame - 1
    if add_flag:
        # try to spawn a test linear car to see if the simulation is still running
        # a choose npc from npc_list first
        # delete background vehicle which is too far
        if abs(state.num_frames - found_frame) > add_car_frame:
            add_type = random.randint(1, 100)  # this value controls the type of npc
            npc_vehicle = None
            new_npc = None
            repeat_times = 0
            while npc_vehicle is None:
                repeat_times += 1
                # stuck too long
                if repeat_times > 100 or state.stuck_duration > 100:
                    # add a fake npc
                    new_npc = NPC(npc_type=None,
                                    spawn_point=None, speed=None,
                                    npc_id=len(npc_list),
                                    ego_loc=player_loc)
                    new_npc.instance = None
                    npc_list.append(new_npc)
                    new_npc.fresh = False
                    break
                x = random.uniform(-50, 50)
                y = random.uniform(-50, 50)
                # 1.don't add vehicles that are physically too far away
                if x ** 2 + y ** 2 > 50 ** 2:
                    repeat_times -= 1
                    continue
                # 2.don't add vehicles that are topologically too far away
                location = carla.Location(x=player_loc.x + x, y=player_loc.y + y, z=player_loc.z)
                waypoint = town_map.get_waypoint(location, project_to_road=True,
                                                 lane_type=carla.libcarla.LaneType.Driving)
                neighbors_A = nx.single_source_shortest_path_length(G,
                                                                    source=(player_road_id, player_lane_id),
                                                                    cutoff=conf.topo_k)
                neighbors_A[(player_road_id, player_lane_id)] = 0
                neighbors_B = nx.single_source_shortest_path_length(G, source=(
                    waypoint.road_id, waypoint.lane_id), cutoff=conf.topo_k)
                neighbors_B[(waypoint.road_id, waypoint.lane_id)] = 0
                if not any(node in neighbors_A and node in neighbors_B for node in G.nodes()):
                    repeat_times -= 1
                    continue
                # 3.we don't want to add bg car in junction or near it
                # because it may cause a red light problem
                if waypoint.is_junction:
                    continue
                # if waypoint.is_junction or waypoint.next(30 * 3 / 3.6)[-1].is_junction:
                #     continue
                temp_flag = False
                # 4. don't add a vehicle in the same lane
                for other_npc in npc_list:
                    if other_npc.instance is not None:
                        other_npc_waypoint = other_npc.get_waypoint(town_map)
                        if (other_npc_waypoint.lane_id == waypoint.lane_id) & (
                                other_npc_waypoint.road_id == waypoint.road_id):
                            temp_flag = True
                            break
                if player_loc.distance(waypoint.transform.location) < 20:
                    if (player_lane_id == waypoint.lane_id) & (player_road_id == waypoint.road_id):
                        temp_flag = True
                if temp_flag:
                    continue
                # we don't want to add an immobile bg car at lane that can't change lane
                if waypoint.lane_change == carla.LaneChange.NONE:
                    if add_type <= conf.immobile_percentage:
                        repeat_times -= 1
                        continue
                road_direction = waypoint.transform.rotation.get_forward_vector()
                road_direction_x = road_direction.x
                road_direction_y = road_direction.y
                roll = math.atan2(road_direction_y, road_direction_x)
                roll_degrees = math.degrees(roll)
                npc_spawn_point = carla.Transform(
                    carla.Location(x=waypoint.transform.location.x, y=waypoint.transform.location.y,
                                   z=waypoint.transform.location.z + 0.1),
                    carla.Rotation(pitch=0, yaw=roll_degrees, roll=0)
                )
                # random choose a car bp from vehicle_bp_library
                npc_bp = random.choice(vehicle_bp_library)

                if add_type <= conf.immobile_percentage:
                    # add a immobile car
                    bg_speed = 0
                    new_npc = NPC(npc_type=c.VEHICLE,
                                    spawn_point=npc_spawn_point, speed=bg_speed,
                                    npc_id=len(npc_list),
                                    ego_loc=player_loc,
                                    npc_bp=npc_bp, spawn_stuck_frame=state.stuck_duration)
                else:
                    bg_speed = random.uniform(0 / 3.6, 20 / 3.6)
                    new_npc = NPC(npc_type=c.VEHICLE,
                                    spawn_point=npc_spawn_point, speed=bg_speed,
                                    npc_id=len(npc_list),
                                    ego_loc=player_loc,
                                    npc_bp=npc_bp, spawn_stuck_frame=state.stuck_duration)
                # do safe check
                flag = True
                for npc in npc_now:
                    if not new_npc.safe_check(npc):
                        flag = False
                        break
                if not new_npc.safe_check(ego):
                    flag = False
                if flag:
                    npc_vehicle = world.try_spawn_actor(new_npc.npc_bp, npc_spawn_point)
                else:
                    continue
            if npc_vehicle is not None:
                spawn_npc(new_npc, npc_vehicle, npc_vehicles, npc_now, agents_now, conf,
                          max_wheels_for_non_motorized, road_direction, sensors, state, world, wp)
                npc_list.append(new_npc)
        found_frame = False
    return found_frame, autoware_last_frames, frame_gap


def add_old_npc(npc_list, npc_vehicles, npc_now, agents_now, conf, found_frame, max_wheels_for_non_motorized,
                player_loc, sensors, state, vel, world, wp):
    for npc in npc_list:
        if npc.fresh & (npc.ego_loc.distance(player_loc) < 1.5):
            found_frame = state.num_frames
            # check if this npc is good to spawn
            v1 = carla.Vector2D(npc.ego_loc.x - player_loc.x, npc.ego_loc.y - player_loc.y)
            v2 = vel
            if state.stuck_duration != 0:
                # if ego is stuck,check stuck duration
                if npc.spawn_stuck_frame != state.stuck_duration:
                    continue
            angle = get_angle_between_vectors(v1, v2)
            if angle < 90 and angle != 0:
                # the better time will come later
                continue
            # check if this npc is not exist
            if npc.npc_type is None:
                npc.fresh = False
                break
            npc_vehicle = world.try_spawn_actor(npc.npc_bp, npc.spawn_point)
            if npc_vehicle is not None:
                npc_spawn_rotation = npc.spawn_point.rotation
                roll_degrees = npc_spawn_rotation.yaw
                roll = math.radians(roll_degrees)
                road_direction_x = math.cos(roll)
                road_direction_y = math.sin(roll)
                road_direction = carla.Vector3D(road_direction_x, road_direction_y, 0.0)
                spawn_npc(npc, npc_vehicle, npc_vehicles, npc_now, agents_now, conf,
                          max_wheels_for_non_motorized, road_direction, sensors, state, world, wp)
                continue
    return found_frame


def is_in_front(yaw_degrees, car1_position, car2_position):
    yaw_radians = math.radians(yaw_degrees)
    direction_vector = (math.cos(yaw_radians), math.sin(yaw_radians))

    vector_12 = (car2_position.x - car1_position.x, car2_position.y - car1_position.y)
    angle_12 = math.atan2(vector_12[1], vector_12[0])

    angle_diff = math.degrees(angle_12 - yaw_radians)
    angle_diff = (angle_diff + 180) % 360 - 180  # Normalize to [-180, 180]

    return -30 <= angle_diff <= 30


def mark_useless_npc(npc_now, conf, player_lane_id, player_loc, player_rot, player_road_id, G, town_map):
    for npc in npc_now:
        if npc.death_time != -1:
            # (npc.death_time == -1) means it is alive
            continue
        # if is_in_front(player_rot.yaw, player_loc, npc.instance.get_location()):
        #     print("npc in front")
        #     continue
        vehicle = npc.instance
        vehicle_waypoint = town_map.get_waypoint(vehicle.get_location(), project_to_road=True,
                                                 lane_type=carla.libcarla.LaneType.Driving)
        vehicle_lane_id = vehicle_waypoint.lane_id
        vehicle_road_id = vehicle_waypoint.road_id
        # 1. Delete vehicles that are physically too far away
        if vehicle.get_location().distance(player_loc) > 100 * math.sqrt(2):
            if is_in_front(player_rot.yaw, player_loc, npc.instance.get_location()):
                continue
            mark_npc(npc, 0)
            # delete_npc(npc, npc_vehicles, sensors, agents_now, npc_now)
            continue
        # 2. Delete vehicles that are topologically too far away
        neighbors_A = nx.single_source_shortest_path_length(G,
                                                            source=(player_road_id, player_lane_id),
                                                            cutoff=conf.topo_k + 1)
        neighbors_A[(player_road_id, player_lane_id)] = 0
        neighbors_B = nx.single_source_shortest_path_length(G,
                                                            source=(vehicle_road_id, vehicle_lane_id),
                                                            cutoff=conf.topo_k + 1)
        neighbors_B[(vehicle_road_id, vehicle_lane_id)] = 0
        if not any(node in neighbors_A and node in neighbors_B for node in G.nodes()):
            if is_in_front(player_rot.yaw, player_loc, npc.instance.get_location()):
                continue
            mark_npc(npc, 1 * c.FRAME_RATE)
        # 3. Delete vehicles that stuck too long
        if npc.stuck_duration > (conf.timeout * c.FRAME_RATE / 10):
            # let stuck car go away
            npc_rot = npc.instance.get_transform().rotation
            yaw_radians = math.radians(npc_rot.yaw)
            direction_vector = (math.cos(yaw_radians), math.sin(yaw_radians))
            velocity_magnitude = 5
            velocity = carla.Vector3D(velocity_magnitude * direction_vector[0],
                                      velocity_magnitude * direction_vector[1], 0)
            npc.instance.set_target_velocity(velocity)
            mark_npc(npc, 5 * c.FRAME_RATE)


def get_player_info(cur_frame_id, goal_loc, player, sp, state, town_map, conf=None):
    # Get player info
    frame_speed_lim_changed = 0
    player_transform = player.get_transform()
    player_loc = player_transform.location
    player_rot = player_transform.rotation
    player_waypoint = town_map.get_waypoint(player_loc, project_to_road=True,
                                            lane_type=carla.libcarla.LaneType.Driving)
    player_lane_id = player_waypoint.lane_id
    player_road_id = player_waypoint.road_id
    vel = player.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    speed_limit = player.get_speed_limit()
    try:
        last_speed_limit = state.speed_lim[-1]
    except:
        last_speed_limit = 0
    if speed_limit != last_speed_limit:
        frame_speed_lim_changed = cur_frame_id
    state.speed.append(speed)
    state.speed_lim.append(speed_limit)
    state.distance += speed / 3.6 / c.FRAME_RATE
    if conf.debug:
        print("[debug] (%.2f,%.2f)>(%.2f,%.2f)>(%.2f,%.2f) %.2f m left, %.2f/%d km/h   \r" % (
            sp.location.x, sp.location.y, player_loc.x,
            player_loc.y, goal_loc.x, goal_loc.y,
            player_loc.distance(goal_loc),
            speed, speed_limit), end="")
    if player.is_at_traffic_light():
        traffic_light = player.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            # within red light triggerbox
            if state.on_red:
                state.on_red_speed.append(speed)
            else:
                state.on_red = True
                state.on_red_speed = list()
    else:
        # not at traffic light
        if state.on_red:
            # out of red light triggerbox
            state.on_red = False
            stopped_at_red = False
            for i, ors in enumerate(state.on_red_speed):
                if ors < 0.1:
                    stopped_at_red = True
            if not stopped_at_red:
                state.red_violation = True
    return frame_speed_lim_changed, player_lane_id, player_loc, player_road_id, player_rot, speed, speed_limit, vel


def check_destination(npc_vehicles, npc_now, agents_now, autoware_stuck, conf, goal_loc, goal_rot, player_loc,
                      proc_state, retval, s_started, sensors, speed, state, world):
    dist_to_goal = player_loc.distance(goal_loc)
    break_flag = False
    # mark goal position
    if conf.agent_type == c.AUTOWARE:
        # # Check Autoware-defined destination
        # VehicleReady\nDriving\nMoving\nLaneArea\nCruise\nStraight\nDrive\nGo\n
        # VehicleReady\nWaitOrder\nStopping\nWaitDriveReady\n
        if speed < 3.6:
            if dist_to_goal < 3:
                print("\n[*] (Autoware) Reached the destination dist_to_goal=",
                      dist_to_goal)
                retval = 0
                break_flag = True
        if not conf.function.startswith("eval"):
            output_state = non_blocking_read(proc_state.stdout)
            # output_state = proc_state.stdout.readline()
            # if b"---" in output_state:
            #     output_state = non_blocking_read(proc_state.stdout)
            if "Go" in output_state:
                s_started = True
                autoware_stuck = 0
            elif "WaitDriveReady" in output_state and s_started:
                autoware_stuck += 1
                if autoware_stuck >= 300 and speed < 3.6:
                    if dist_to_goal > 2:
                        print("\n[*] (Autoware) don't Reached the destination")
                        print("      dist to goal:", dist_to_goal)
                        state.other_error = "goal"
                        state.other_error_val = dist_to_goal
                        retval = 1
                        break_flag = True
    elif conf.agent_type == c.BEHAVIOR:
        delete_indices = []
        for i in range(len(agents_now)):
            lp = agents_now[i][0].get_local_planner()
            if len(lp._waypoints_queue) == 0:
                if i == 0:
                    if speed < 0.1:
                        if dist_to_goal < 2:
                            print("\n[*] (BehaviorAgent) Reached the destination dist_to_goal=",
                                  dist_to_goal)
                            retval = 0
                            break_flag = True
                            break
                        else:
                            print("\n[*] (BehaviorAgent) dont Reached the destination dist_to_goal=",
                                  dist_to_goal)
                            state.other_error = "goal"
                            state.other_error_val = dist_to_goal
                            retval = 1
                            break_flag = True
                            break
                else:
                    delete_indices.append(i)
        for index in delete_indices:
            delete_npc(agents_now[index][2], npc_vehicles, sensors, agents_now, npc_now)
    return break_flag, retval, autoware_stuck, s_started


def world_reload(npc_list, npc_vehicles, npc_walkers, npc_now, sensors, world):
    try:
        # if conf.agent_type == c.AUTOWARE:
        #     cmd = f'/tmp/reload_autoware.sh {world.get_map().name.split("/")[-1]} > output.log 2>&1 &'
        #     t = threading.Thread(target=run_cmd_in_container, args=(autoware_container, cmd))
        #     t.daemon = True
        #     t.start()
        #     time.sleep(1)
        #     check_autoware_status(world, 60)
        for npc in npc_now:
            npc.instance = None
        for npc in npc_list:
            npc.fresh = True
        for s in sensors:
            s.stop()
            s.destroy()
        for w in npc_walkers:
            w.destroy()
        for v in npc_vehicles:
            v.destroy()

        return True
    except RuntimeError:
        return False


def check_and_remove_excess_images(pattern, max_frames):
    images = sorted(glob.glob(pattern))
    while len(images) > max_frames:
        os.remove(images.pop(0))


def save_jpg_for_gpt(interval):
    # save the last 15 s frame
    src_dir = f"/tmp/fuzzerdata/{username}/front-*.jpg"
    dest_dir = "data/output/gpt"
    os.makedirs(dest_dir, exist_ok=True)
    max_frames = c.FRAME_RATE * c.VIDEO_TIME
    check_and_remove_excess_images(f"/tmp/fuzzerdata/{username}/front-*.jpg", max_frames)
    files = sorted(glob.glob(src_dir))
    for i, file in enumerate(files):
        if i % interval == 0:
            dest_file = os.path.join(dest_dir, f"top_{i // interval}.jpg")
            shutil.copy(file, dest_file)
            print(f"Copied {file} to {dest_file}")
    print("done")


def save_behavior_video(carla_error, state):
    # # remove jpg files
    max_frames = c.FRAME_RATE * c.VIDEO_TIME
    if state.crashed and not state.laneinvaded:
        print(f"Saving front camera video for last {c.VIDEO_TIME} second", end=" ")
        check_and_remove_excess_images(f"/tmp/fuzzerdata/{username}/front-*.jpg", max_frames)
    else:
        print(f"Saving the whole front camera video", end=" ")
    vid_filename = f"/tmp/fuzzerdata/{username}/front.mp4"
    if os.path.exists(vid_filename):
        os.remove(vid_filename)
    cmd_cat = f"cat /tmp/fuzzerdata/{username}/front-*.jpg"
    cmd_ffmpeg = " ".join([
        "ffmpeg",
        "-f image2pipe",
        f"-r {c.FRAME_RATE}",
        "-vcodec mjpeg",
        "-i -",
        "-vcodec libx264",
        "-crf 5",
        vid_filename
    ])
    cmd = f"{cmd_cat} | {cmd_ffmpeg} {c.DEVNULL}"
    if not carla_error:
        os.system(cmd)
    else:
        print("error:dont save any video")
    cmd = f"rm -f /tmp/fuzzerdata/{username}/front-*.jpg"
    os.system(cmd)
    if state.crashed and not state.laneinvaded:
        print(f"Saving top camera video for last {c.VIDEO_TIME}", end=" ")
        check_and_remove_excess_images(f"/tmp/fuzzerdata/{username}/top-*.jpg", max_frames)
    else:
        print(f"Saving the whole top camera video", end=" ")
    vid_filename = f"/tmp/fuzzerdata/{username}/top.mp4"
    if os.path.exists(vid_filename):
        os.remove(vid_filename)
    cmd_cat = f"cat /tmp/fuzzerdata/{username}/top-*.jpg"
    cmd_ffmpeg = " ".join([
        "ffmpeg",
        "-f image2pipe",
        f"-r {c.FRAME_RATE}",
        "-vcodec mjpeg",
        "-i -",
        "-vcodec libx264",
        "-crf 15",
        vid_filename
    ])
    cmd = f"{cmd_cat} | {cmd_ffmpeg} {c.DEVNULL}"
    if not carla_error:
        os.system(cmd)
    else:
        print("error:dont save any video")
    cmd = f"rm -f /tmp/fuzzerdata/{username}/top-*.jpg"
    os.system(cmd)


def autoware_goal_publish(goal_loc, goal_rot, state, world):
    goal_quaternion = quaternion_from_euler(0.0, 0.0, goal_rot.yaw)
    goal_ox = goal_quaternion[0]
    goal_oy = goal_quaternion[1]
    goal_oz = goal_quaternion[2]
    goal_ow = goal_quaternion[3]
    pub_topic = "/move_base_simple/goal"
    msg_type = "geometry_msgs/PoseStamped"
    goal_hdr = "header: {stamp: now, frame_id: \'map\'}"
    goal_pose = "pose: {position: {x: %.6f, y: %.6f, z: 0}, orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}}" % (
        goal_loc.x, (-1) * float(goal_loc.y), goal_ox, goal_oy, goal_oz, goal_ow)
    # goal_pose = "pose: {position: {x: %.6f, y: %.6f, z: 0}, orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}}" % (
    #     goal_loc.x, float(goal_loc.y), goal_ox, goal_oy, goal_oz, goal_ow)
    goal_msg = "'{" + goal_hdr + ", " + goal_pose + "}'"
    pub_cmd = "rostopic pub --once {} {} {} > /dev/null".format(pub_topic, msg_type, goal_msg)
    os.system(pub_cmd)
    print("[carla] Goal published")
    time.sleep(3)  # give some time (Autoware initialization is slow)
    state.autoware_goal = pub_cmd
    world.tick()
    return


def autoware_launch(exec_state, conf, town_map, sp):
    world = exec_state.world
    username = os.getenv("USER")
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
    loc = sp.location
    rot = sp.rotation
    sp_str = "{},{},{},{},{},{}".format(loc.x, -1 * loc.y, loc.z, 0,
                                        0, -1 * rot.yaw)
    # sp_str = "{},{},{},{},{},{}".format(loc.x, loc.y, loc.z, rot.roll,
    #                                     rot.pitch, rot.yaw * -1)
    autoware_cla = "{} \'{}\' \'{}\'".format(town_map.name.split("/")[-1], sp_str, conf.sim_port)
    print(autoware_cla)
    temp_player = None
    autoware_container = None
    while autoware_container is None:
        try:
            autoware_container = docker_client.containers.run(
                "carla-autoware:improved-record",
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
            # print("[-] Could not launch docker:", e)
            if "Conflict" in str(e):
                os.system("docker rm -f autoware-{}".format(
                    os.getenv("USER")))
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
    print("[*] Waiting for ROS to be launched")
    time.sleep(5)
    if temp_player:
        print("player loc after:", temp_player.get_transform())
    # exec a detached process that monitors the output of Autoware's
    # decision-maker state, with which we can get an idea of when Autoware
    # thinks it has reached the goal
    exec_state.proc_state = Popen(["rostopic echo /decision_maker/state"],
                                  shell=True, stdout=PIPE, stderr=PIPE)
    # wait for autoware bridge to spawn player vehicle
    check_autoware_status(world, 120)

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
    return autoware_container


def ego_initialize(agents_now, exec_state, blueprint_library, conf, player_bp, sensors, sp, state,
                   world, wp):
    # for autoware
    autoware_container = None
    town = world.get_map()
    player = None
    if conf.agent_type == c.BEHAVIOR:
        player = world.try_spawn_actor(player_bp, sp)
        ego = NPC(npc_type=c.VEHICLE, spawn_point=sp,
                  npc_id=-1)
        ego.set_instance(player)
        world.tick()  # sync once with simulator
        player.set_simulate_physics(True)
        agent = BehaviorAgent(
            player,
            behavior="cautious"
        )
        agent.set_destination(
            start_location=sp.location,
            end_location=wp.location,
        )
        agents_now.append((agent, player, ego))
        print("[+] spawned cautious BehaviorAgent")
    else:
        autoware_container = autoware_launch(exec_state, conf, town, sp)
        i = 0
        autoware_agent_found = False
        while True:
            print("[*] Waiting for Autoware agent " + "." * i + "\r", end="")
            vehicles = world.get_actors().filter("*vehicle.*")
            for vehicle in vehicles:
                if vehicle.attributes["role_name"] == "ego_vehicle":
                    if vehicle.get_location().distance(sp.location) < 1:
                        player = vehicle
                        autoware_agent_found = True
                        break
            if autoware_agent_found:
                break
            if i > 60:
                print("\n Autoware agent launch fail")
                raise TimeoutError
            i += 1
            time.sleep(1)
        ego = NPC(npc_type=c.VEHICLE, spawn_point=sp, npc_id=-1)
        ego.set_instance(player)
        # loc = sp.location
        # rot = sp.rotation
        # cmd = f'bash -c "source /opt/ros/melodic/setup.bash && python /tmp/pub_initialpose.py {loc.x} {-1 * loc.y} {loc.z + 2} {0} {0} {-1 * rot.yaw / 180 * math.pi}"'
        # t = threading.Thread(target=run_cmd_in_container, args=(autoware_container, cmd))
        # t.daemon = True
        # t.start()
        # check_autoware_status(world, 60)
        # autoware_container.exec_run(cmd, stdout=True, stderr=True, user='root')
        # print(result.output)
        time.sleep(5)
        print("\n    [*] found [{}] at {}".format(player.id,
                                                  player.get_location()))
        timeout = 120
        try:
            left = signal.alarm(timeout)
            print("left time:", left)
            check_vehicle(exec_state.proc_state)
        except TimeoutError:
            print("Vehicle did not Ready within timeout.")
            raise KeyboardInterrupt
        finally:
            signal.alarm(left)
        world.tick()  # sync with simulator
        time.sleep(3)
        # x2 = 0
        # while True:
        #     world.tick()  # spin until the player is moved to the sp
        #     location_1 = player.get_location()
        #     location_1.z = 0
        #     location_2 = sp.location
        #     location_2.z = 0
        #     if location_1.distance(location_2) < 5:
        #         break
        #     else:
        #         x1 = location_1.distance(location_2)
        #         if x1 == x2:
        #             print(x1)
        #             raise KeyboardInterrupt
        #         x2 = x1
        #         time.sleep(1)
    # Attach RGB camera (front)
    rgb_camera_bp = blueprint_library.find("sensor.camera.rgb")
    rgb_camera_bp.set_attribute("image_size_x", "800")
    rgb_camera_bp.set_attribute("image_size_y", "600")
    rgb_camera_bp.set_attribute("fov", "105")

    camera_tf = carla.Transform(carla.Location(z=1.8))
    camera_front = world.spawn_actor(
        rgb_camera_bp,
        camera_tf,
        attach_to=player,
        attachment_type=carla.AttachmentType.Rigid
    )

    camera_front.listen(lambda image: _on_front_camera_capture(image, state))
    sensors.append(camera_front)
    camera_tf2 = carla.Transform(
        carla.Location(z=50.0),
        carla.Rotation(pitch=-90.0)
    )
    camera_top = world.spawn_actor(
        rgb_camera_bp,
        camera_tf2,
        attach_to=player,
        attachment_type=carla.AttachmentType.Rigid
    )

    camera_top.listen(lambda image: _on_top_camera_capture(image, state))
    sensors.append(camera_top)

    ego.attach_collision(world, sensors, state)
    if conf.check_dict["lane"]:
        ego.attach_lane_invasion(world, sensors, state)
    # get vehicle's maximum steering angle
    physics_control = player.get_physics_control()
    max_steer_angle = 0
    for wheel in physics_control.wheels:
        if wheel.max_steer_angle > max_steer_angle:
            max_steer_angle = wheel.max_steer_angle
    world.tick()  # sync with simulator
    return autoware_container, ego, player, max_steer_angle


# def calculate_control(npc_vehicles, npc_walkers, max_steer_angle, player, player_loc, player_rot, state, vel, yaw):
#     # record ego information
#     control = player.get_control()
#     state.cont_throttle.append(control.throttle)
#     state.cont_brake.append(control.brake)
#     state.cont_steer.append(control.steer)
#     steer_angle = control.steer * max_steer_angle
#     state.steer_angle_list.append(steer_angle)
#     current_yaw = player_rot.yaw
#     state.yaw_list.append(current_yaw)
#     yaw_diff = current_yaw - yaw
#     # Yaw range is -180 ~ 180. When vehicle's yaw is oscillating
#     # b/w -179 and 179, yaw_diff can be messed up even if the
#     # diff is very small. Assuming that it's unrealistic that
#     # a vehicle turns more than 180 degrees in under 1/20 seconds,
#     # just round the diff around 360.
#     if yaw_diff > 180:
#         yaw_diff = 360 - yaw_diff
#     elif yaw_diff < -180:
#         yaw_diff = 360 + yaw_diff
#     yaw_rate = yaw_diff * c.FRAME_RATE
#     state.yaw_rate_list.append(yaw_rate)
#     yaw = current_yaw
#     # uncomment below to follow along the player
#     # set_camera(conf, player, spectator)
#     # Get the lateral speed
#     player_right_vec = player_rot.get_right_vector()
#     lat_speed = abs(vel.x * player_right_vec.x + vel.y * player_right_vec.y)
#     lat_speed *= 3.6  # m/s to km/h
#     state.lat_speed_list.append(lat_speed)
#     player_fwd_vec = player_rot.get_forward_vector()
#     lon_speed = abs(vel.x * player_fwd_vec.x + vel.y * player_fwd_vec.y)
#     lon_speed *= 3.6
#     state.lon_speed_list.append(lon_speed)


def simulate_initialize(client, conf, weather_dict, world):
    client.set_timeout(10.0)
    if conf.no_traffic_lights:
        # set all traffic lights to green
        set_traffic_lights_state(world, carla.TrafficLightState.Green)
        world.freeze_all_traffic_lights(True)

    if conf.debug:
        print("[debug] world:", world)
    else:
        world.reset_all_traffic_lights()
    town_map = world.get_map()
    if conf.debug:
        print("[debug] map:", town_map)
    blueprint_library = world.get_blueprint_library()
    vehicle_bp_library = blueprint_library.filter("vehicle.*")
    # vehicle_bp.set_attribute("color", "255,0,0")
    walker_bp = blueprint_library.find("walker.pedestrian.0001")  # 0001~0014
    walker_controller_bp = blueprint_library.find('controller.ai.walker')
    player_bp = blueprint_library.filter('nissan')[0]
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / c.FRAME_RATE  # FPS
    settings.no_rendering_mode = False
    world.apply_settings(settings)
    frame_id = world.tick()
    clock = pygame.time.Clock()
    if conf.density != 0:
        add_car_frame = c.FRAME_RATE // conf.density
    else:
        add_car_frame = 9999999
    # set weather
    weather = world.get_weather()
    weather.cloudiness = weather_dict["cloud"]
    weather.precipitation = weather_dict["rain"]
    # weather.precipitation_deposits = weather_dict["puddle"]
    weather.wetness = weather_dict["wetness"]
    weather.wind_intensity = weather_dict["wind"]
    weather.fog_density = weather_dict["fog"]
    weather.sun_azimuth_angle = weather_dict["angle"]
    weather.sun_altitude_angle = weather_dict["altitude"]
    world.set_weather(weather)
    world.tick()  # sync with simulator
    return add_car_frame, blueprint_library, clock, player_bp, town_map, vehicle_bp_library


def spawn_npc(npc, npc_vehicle, npc_vehicles, npcs_now, agents_now, conf, max_wheels_for_non_motorized,
              road_direction, sensors, state, world, wp):
    npc_vehicles.append(npc_vehicle)
    npc_vehicle.set_transform(npc.spawn_point)
    x_offset = random.uniform(5, 10)
    y_offset = random.uniform(5, 10)
    wp_new_location = wp.location + carla.Location(x=x_offset * random.choice([-1, 1]),
                                                   y=y_offset * random.choice([-1, 1]))
    new_agent = set_autopilot(npc_vehicle, c.BEHAVIOR_AGENT, npc.spawn_point.location,
                              wp_new_location, world)
    agents_now.append((new_agent, npc_vehicle, npc))
    npc_vehicle.set_target_velocity(
        npc.speed * road_direction)
    npc.set_instance(npc_vehicle)
    # # just add it for behavior
    # if conf.agent_type == c.BEHAVIOR:
    #     # don't add sensors for non_motorized vehicles
    #     if npc.npc_bp.get_attribute(
    #             "number_of_wheels").as_int() > max_wheels_for_non_motorized:
    #         npc.attach_collision(world, sensors, state)
    npcs_now.append(npc)
    npc.fresh = False


def _on_front_camera_capture(image, state):
    if not state.end:
        image.save_to_disk(f"/tmp/fuzzerdata/{username}/front-{image.frame:05d}.jpg")


def _on_top_camera_capture(image, state):
    if not state.end:
        image.save_to_disk(f"/tmp/fuzzerdata/{username}/top-{image.frame:05d}.jpg")


def _set_camera(conf, player, spectator):
    if conf.view == c.BIRDSEYE:
        _cam_over_player(player, spectator)
    elif conf.view == c.ONROOF:
        _cam_chase_player(player, spectator)
    else:  # fallthrough default
        _cam_chase_player(player, spectator)


def _cam_chase_player(player, spectator):
    location = player.get_location()
    rotation = player.get_transform().rotation
    fwd_vec = rotation.get_forward_vector()

    # chase from behind
    constant = 4
    location.x -= constant * fwd_vec.x
    location.y -= constant * fwd_vec.y
    # and above
    location.z += 3
    rotation.pitch -= 5
    spectator.set_transform(
        carla.Transform(location, rotation)
    )


def _cam_over_player(player, spectator):
    location = player.get_location()
    location.z += 100
    # rotation = player.get_transform().rotation
    rotation = carla.Rotation()  # fix rotation for better sim performance
    rotation.pitch -= 90
    spectator.set_transform(
        carla.Transform(location, rotation)
    )


def non_blocking_read(stdout):
    fd = stdout.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    output_state = b""
    rlist, _, _ = select.select([stdout], [], [], 0.01)
    if stdout in rlist:
        data = os.read(fd, 4096)
        output_state = data
    output_state = output_state.decode("utf-8")
    return output_state


def check_violation(conf, cur_frame_id, frame_speed_lim_changed, retval, speed, speed_limit, state, wait_until_end):
    # Check speeding
    if conf.check_dict["speed"]:
        # allow T seconds to slow down if speed limit suddenly
        # decreases
        T = 3  # 0 for strict checking
        if (speed > speed_limit + 2 and
                cur_frame_id > frame_speed_lim_changed + T * c.FRAME_RATE):
            print("\n[*] Speed violation: {} km/h on a {} km/h road".format(
                speed, speed_limit))
            state.speeding = True
            retval = 1
            wait_until_end = 1
    # Check crash
    if conf.check_dict["crash"]:
        if state.crashed:
            print("\n[*] Collision detected: %.2f" % (
                state.elapsed_time))
            retval = 1
            wait_until_end = 1
    # Check lane violation
    if conf.check_dict["lane"]:
        if state.laneinvaded:
            # print("\n[*] Lane invasion detected: %.2f" % (
            #     state.elapsed_time))
            retval = 1
            # wait_until_end = 1
    # Check traffic light violation
    if conf.check_dict["red"]:
        if state.red_violation:
            print("\n[*] Red light violation detected: %.2f" % (
                state.elapsed_time))
            retval = 1
            wait_until_end = 1
    # Check inactivity
    if speed < 1:  # km/h
        state.stuck_duration += 1
    else:
        state.stuck_duration = 0
    if conf.check_dict["stuck"]:
        if state.stuck_duration > (conf.timeout * c.FRAME_RATE):
            state.stuck = True
            print("\n[*] Stuck for too long: %d" % state.stuck_duration)
            retval = 1
            wait_until_end = 1
    if conf.check_dict["other"]:
        if state.num_frames > 60 * c.FRAME_RATE * 15:  # over 15 minutes
            print("\n[*] Simulation taking too long")
            state.other_error = "timeout"
            state.other_error_val = state.num_frames
            retval = 1
            wait_until_end = 1
        if state.other_error:
            print("\n[*] Other error: %d" % state.signal)
            retval = 1
            wait_until_end = 1
    return retval, wait_until_end


def get_docker(container_name):
    all_containers = docker_client.containers.list()
    target_container = None
    for container in all_containers:
        if container.name == container_name:
            target_container = container
            break
    # if target_container:
    #     print(f"Found container '{container_name}':")
    #     print(f"Container ID: {target_container.id}")
    #     print(f"Container Status: {target_container.status}")
    # else:
    #     print(f"Container '{container_name}' not found.")
    return target_container


def run_cmd_in_container(container, cmd):
    container.exec_run(cmd, stdout=True, stderr=True, user='root')


def check_topo(player_waypoint=None, waypoint=None, G=None):
    player_lane_id = player_waypoint.lane_id
    player_road_id = player_waypoint.road_id
    neighbors_A = nx.single_source_shortest_path_length(G,
                                                        source=(player_road_id, player_lane_id),
                                                        cutoff=2)
    neighbors_A[(player_road_id, player_lane_id)] = 0
    neighbors_B = nx.single_source_shortest_path_length(G, source=(
        waypoint.road_id, waypoint.lane_id), cutoff=2)
    neighbors_B[(waypoint.road_id, waypoint.lane_id)] = 0
    if not any(node in neighbors_A and node in neighbors_B for node in G.nodes()):
        return False
    else:
        return True


def check_vehicle(proc):
    while True:
        output_state = non_blocking_read(proc.stdout)
        if "VehicleReady" in output_state:
            break
        time.sleep(1)
        print("[*] Waiting for Autoware vehicle Ready" + "\r", end="")
