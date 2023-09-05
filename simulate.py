#!/usr/bin/env python3

# Python packages
import os
import pdb
import random
import sys
from subprocess import Popen, PIPE
import signal
import time
import math
import traceback
import docker
import networkx as nx
import pygame
from actor import Actor
import config
import constants as c
from utils import quaternion_from_euler, set_traffic_lights_state, get_angle_between_vectors, \
    set_autopilot, delete_actor

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




def simulate(conf, state, sp, wp, weather_dict, actor_list):
    # simulate() is always called by Scenario instance,
    # so we won't need to switch a map unless a new instance is created.
    # switch_map(conf, client, town)

    # always reuse the existing client instance
    assert (state.client is not None)

    player = None
    ego = None
    retval = 0
    wait_until_end = 0
    max_wheels_for_non_motorized = 2
    carla_error = False
    username = os.getenv("USER")

    actors_now = []
    agents_now = []
    sensors = []
    actor_vehicles = []
    actor_walkers = []
    actor_frictions = []

    # for autoware
    frame_gap = 0
    autoware_last_frames = 0
    autoware_stuck = 0

    client = state.client
    world = state.world

    try:
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
        add_car_frame = c.FRAME_RATE // conf.density

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

        world.tick()  # sync once with simulator
        goal_loc = wp.location
        goal_rot = wp.rotation

        if conf.agent_type == c.BEHAVIOR:
            player = world.try_spawn_actor(player_bp, sp)
            ego = Actor(actor_type=c.VEHICLE, nav_type=c.EGO, spawn_point=sp,
                        dest_point=wp, actor_id=-1)
            ego.set_instance(player)
            ego.attach_collision(world, sensors, state)
            ego.attach_lane_invasion(world, sensors, state)
            if player is None:
                print("[-] Failed spawning player")
                state.spawn_failed = True
                state.spawn_failed_object = 0  # player
                retval = -1
                return  # trap to finally
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
        elif conf.agent_type == c.AUTOWARE:
            loc = sp.location
            rot = sp.rotation
            if conf.function == "collision":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = 1.0
                goal_ow = 0.0
            elif conf.function == "traction":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = -0.96
                goal_ow = 0.26
            elif conf.function == "eval-us":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = -0.01
                goal_ow = 0.9998
            elif conf.function == "eval-os":
                goal_ox = 0.0
                goal_oy = 0.0
                goal_oz = 0.679
                goal_ow = 0.733
            else:
                goal_quaternion = quaternion_from_euler(0.0, 0.0, goal_rot.yaw)
                goal_ox = goal_quaternion[0]
                goal_oy = goal_quaternion[1]
                goal_oz = goal_quaternion[2]
                goal_ow = goal_quaternion[3]
            sp_str = "{},{},{},{},{},{}".format(loc.x, loc.y, loc.z, rot.roll,
                                                rot.pitch, rot.yaw * -1)
            goal_str = "{},{},{},{},{},{},{}".format(goal_loc.x, goal_loc.y,
                                                     goal_loc.z, goal_ox, goal_oy, goal_oz, goal_ow)
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

            autoware_cla = "{} \'{}\' \'{}\'".format(town_map.name.split("/")[-1], sp_str, conf.sim_port)
            print(autoware_cla)
            state.autoware_cmd = autoware_cla

            autoware_container = None
            while autoware_container is None:
                try:
                    autoware_container = docker_client.containers.run(
                        "carla-autoware:improved",
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
                        player = vehicle
                        ego = Actor(actor_type=c.VEHICLE, nav_type=c.EGO, spawn_point=sp,
                                    dest_point=wp, actor_id=-1)
                        ego.set_instance(player)
                        ego.attach_collision(world, sensors, state)
                        ego.attach_lane_invasion(world, sensors, state)
                        print("\n    [*] found [{}] at {}".format(player.id,
                                                                  player.get_location()))
                        break
                if autoware_agent_found:
                    break
                if i > 60:
                    print("\n something is wrong")
                    exit(-1)
                i += 1
                time.sleep(1)
            world.tick()  # sync with simulator
            player.set_transform(sp)
            while True:
                world.tick()  # spin until the player is moved to the sp
                if player.get_location().distance(sp.location) < 1:
                    break
        if conf.agent_type == c.BEHAVIOR:
            # Attach RGB camera (front)
            rgb_camera_bp = blueprint_library.find("sensor.camera.rgb")

            rgb_camera_bp.set_attribute("image_size_x", "800")
            rgb_camera_bp.set_attribute("image_size_y", "600")
            rgb_camera_bp.set_attribute("fov", "105")

            # position relative to the parent actor (player)
            camera_tf = carla.Transform(carla.Location(z=1.8))

            # time in seconds between sensor captures - should sync w/ fps?
            # rgb_camera_bp.set_attribute("sensor_tick", "1.0")

            camera_front = world.spawn_actor(
                rgb_camera_bp,
                camera_tf,
                attach_to=player,
                attachment_type=carla.AttachmentType.Rigid
            )

            camera_front.listen(lambda image: _on_front_camera_capture(image,username))

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

            camera_top.listen(lambda image: _on_top_camera_capture(image,username))
            # bug was here!!!!2023.5.16
            sensors.append(camera_top)

        world.tick()  # sync with simulator

        # get vehicle's maximum steering angle
        physics_control = player.get_physics_control()
        max_steer_angle = 0
        for wheel in physics_control.wheels:
            if wheel.max_steer_angle > max_steer_angle:
                max_steer_angle = wheel.max_steer_angle

        if conf.agent_type == c.AUTOWARE:
            # print("before launching autoware", time.time())
            num_vehicle_topics = len(actor_vehicles)
            num_walker_topics = 0
            if len(actor_walkers) > 0:
                num_walker_topics = 2
            # clock = pygame.time.Clock()
            i = 0
            while True:
                print("[*] Waiting for Autoware nodes " + "." * i + "\r", end="")
                proc1 = Popen(["rostopic", "list"], stdout=PIPE)
                proc2 = Popen(["wc", "-l"], stdin=proc1.stdout, stdout=PIPE)
                proc1.stdout.close()
                output = proc2.communicate()[0]

                num_topics = c.WAIT_AUTOWARE_NUM_TOPICS + num_vehicle_topics + num_walker_topics
                if int(output) >= num_topics:
                    # FIXME: hardcoding the num of topics :/
                    # on top of that, each vehicle adds one topic, and any walker
                    # contribute to two pedestrian topics.
                    print("")
                    break
                i += 1
                if i == 30:
                    carla_error = True
                    print("    [-] something went wrong while launching Autoware.")
                    raise KeyboardInterrupt
                time.sleep(1)
            world.tick()

            # exec a detached process that monitors the output of Autoware's
            # decision-maker state, with which we can get an idea of when Autoware
            # thinks it has reached the goal
            proc_state = Popen(["rostopic echo /decision_maker/state"],
                               shell=True, stdout=PIPE, stderr=PIPE)
            # set_camera(conf, player, spectator)

            # Wait for Autoware (esp, for Town04)
            i = 0
            while True:
                output_state = proc_state.stdout.readline()
                if b"---" in output_state:
                    output_state = proc_state.stdout.readline()
                if b"VehicleReady" in output_state:
                    break
                i += 1
                if i == 45:
                    carla_error = True
                    print("    [-] something went wrong while launching Autoware.")
                    raise KeyboardInterrupt
                time.sleep(1)

            pub_topic = "/move_base_simple/goal"
            msg_type = "geometry_msgs/PoseStamped"
            goal_hdr = "header: {stamp: now, frame_id: \'map\'}"
            goal_pose = "pose: {position: {x: %.6f, y: %.6f, z: 0}, orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}}" % (
                goal_loc.x, (-1) * float(goal_loc.y), goal_ox, goal_oy, goal_oz, goal_ow)
            goal_msg = "'{" + goal_hdr + ", " + goal_pose + "}'"
            pub_cmd = "rostopic pub --once {} {} {} > /dev/null".format(pub_topic, msg_type, goal_msg)
            os.system(pub_cmd)
            if conf.debug:
                print(goal_msg)
            print("[carla] Goal published")
            time.sleep(3)  # give some time (Autoware initialization is slow)
            state.autoware_goal = pub_cmd
            world.tick()
        start_time = time.time()
        yaw = sp.rotation.yaw
        player_loc = player.get_transform().location
        # SIMULATION LOOP FOR AUTOWARE and BasicAgent
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGSEGV, state.sig_handler)
        signal.signal(signal.SIGABRT, state.sig_handler)
        try:
            # actual monitoring of the driving simulation begins here
            snapshot0 = world.get_snapshot()
            first_frame_id = snapshot0.frame
            first_sim_time = snapshot0.timestamp.elapsed_seconds
            last_frame_id = first_frame_id
            state.first_frame_id = first_frame_id
            state.sim_start_time = snapshot0.timestamp.platform_timestamp
            state.num_frames = 0
            state.elapsed_time = 0
            s_started = False
            s_stopped_frames = 0
            if conf.debug:
                print("[debug] START DRIVING: {} {}".format(first_frame_id,
                                                            first_sim_time))
            last_time = start_time
            found_frame = -999
            while True:
                # Use sampling frequency of FPS*2 for precision!
                clock.tick(c.FRAME_RATE * 2)
                # Carla agents are running in synchronous mode,
                # so we need to send ticks. Not needed for Autoware
                if conf.agent_type == c.BEHAVIOR:
                    world.tick()
                snapshot = world.get_snapshot()
                cur_frame_id = snapshot.frame
                cur_sim_time = snapshot.timestamp.elapsed_seconds
                if cur_frame_id <= last_frame_id:
                    # skip if we got the same frame data as last
                    continue
                last_frame_id = cur_frame_id  # update last
                state.num_frames = cur_frame_id - first_frame_id
                state.elapsed_time = cur_sim_time - first_sim_time

                player_transform = player.get_transform()
                player_loc = player_transform.location
                player_rot = player_transform.rotation
                player_waypoint = town_map.get_waypoint(player_loc, project_to_road=True,
                                                        lane_type=carla.libcarla.LaneType.Driving)
                player_lane_id = player_waypoint.lane_id
                player_road_id = player_waypoint.road_id
                # Get speed
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
                print("(%.2f,%.2f)>(%.2f,%.2f)>(%.2f,%.2f) %.2f m left, %.2f/%d km/h   \r" % (
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
                # Delete useless vehicles for any frame
                for actor in actors_now:
                    vehicle = actor.instance
                    vehicle_waypoint = town_map.get_waypoint(vehicle.get_location(), project_to_road=True,
                                                             lane_type=carla.libcarla.LaneType.Driving)
                    vehicle_lane_id = vehicle_waypoint.lane_id
                    vehicle_road_id = vehicle_waypoint.road_id
                    # 1. Delete vehicles that are physically too far away
                    if vehicle.get_location().distance(player_loc) > 50 * math.sqrt(2):
                        delete_actor(actor, actor_vehicles, sensors, agents_now, actors_now)
                        continue
                    # 2. Delete vehicles that are topologically too far away
                    neighbors_A = nx.single_source_shortest_path_length(state.G,
                                                                        source=(player_road_id, player_lane_id),
                                                                        cutoff=conf.topo_k)
                    neighbors_A[(player_road_id, player_lane_id)] = 0
                    neighbors_B = nx.single_source_shortest_path_length(state.G,
                                                                        source=(vehicle_road_id, vehicle_lane_id),
                                                                        cutoff=conf.topo_k)
                    neighbors_B[(vehicle_road_id, vehicle_lane_id)] = 0
                    if not any(node in neighbors_A and node in neighbors_B for node in state.G.nodes()):
                        delete_actor(actor, actor_vehicles, sensors, agents_now, actors_now)
                # add old vehicles for any frame
                for actor in actor_list:
                    if actor.fresh & (actor.ego_loc.distance(player_loc) < 1.5):
                        found_frame = state.num_frames
                        # check if this actor is good to spawn
                        v1 = carla.Vector2D(actor.ego_loc.x - player_loc.x, actor.ego_loc.y - player_loc.y)
                        v2 = vel
                        if state.stuck_duration != 0:
                            # if ego is stuck,check stuck duration
                            if actor.spawn_stuck_frame != state.stuck_duration:
                                continue
                        angle = get_angle_between_vectors(v1, v2)
                        if angle < 90 and angle != 0:
                            # the better time will come later
                            continue
                        # check if this actor is not exist
                        if actor.actor_type is None:
                            actor.fresh = False
                            break
                        actor_vehicle = world.try_spawn_actor(actor.actor_bp, actor.spawn_point)
                        if actor_vehicle is not None:
                            actor_vehicles.append(actor_vehicle)
                            actor_spawn_rotation = actor.spawn_point.rotation
                            roll_degrees = actor_spawn_rotation.yaw
                            roll = math.radians(roll_degrees)
                            road_direction_x = math.cos(roll)
                            road_direction_y = math.sin(roll)
                            road_direction = carla.Vector3D(road_direction_x, road_direction_y, 0.0)
                            actor_vehicle.set_transform(actor.spawn_point)
                            x_offset = random.uniform(5, 10)
                            y_offset = random.uniform(5, 10)
                            wp_new_location = wp.location + carla.Location(x=x_offset * random.choice([-1, 1]),
                                                                           y=y_offset * random.choice([-1, 1]))
                            new_agent = set_autopilot(actor_vehicle, c.BEHAVIOR_AGENT, actor.spawn_point.location,
                                                      wp_new_location, world)
                            agents_now.append((new_agent, actor_vehicle, actor))
                            actor_vehicle.set_target_velocity(
                                actor.speed * road_direction)
                            actor.set_instance(actor_vehicle)
                            # just add it for behavior
                            if conf.agent_type == c.BEHAVIOR:
                                # don't add sensors for non_motorized vehicles
                                if actor.actor_bp.get_attribute(
                                        "number_of_wheels").as_int() > max_wheels_for_non_motorized:
                                    actor.attach_collision(world, sensors, state)
                                    actor.attach_lane_invasion(world, sensors, state)
                            actors_now.append(actor)
                            actor.fresh = False
                            if conf.debug:
                                print("[debug] spawn old car:", actor.actor_id, "at", state.num_frames)
                            continue
                # add a new actor per 1s here
                # because of autoware's strange frame, we should use a interesting method
                if conf.agent_type == c.AUTOWARE:
                    frame_gap = frame_gap + state.num_frames - autoware_last_frames
                    autoware_last_frames = state.num_frames
                    add_flag = frame_gap > add_car_frame - 1
                    if add_flag:
                        frame_gap = frame_gap - add_car_frame
                else:
                    add_flag = state.num_frames % add_car_frame == add_car_frame - 1
                if add_flag:
                    # mark goal position
                    world.debug.draw_box(
                        box=carla.BoundingBox(
                            goal_loc,
                            carla.Vector3D(0.2, 0.2, 1.0)
                        ),
                        rotation=goal_rot,
                        life_time=2,
                        thickness=1.0,
                        color=carla.Color(r=0, g=255, b=0)
                    )
                    # try to spawn a test linear car to see if the simulation is still running
                    # a choose actor from actor_list first
                    # delete backgound car which is too far
                    if abs(state.num_frames - found_frame) > add_car_frame:
                        add_type = random.randint(1, 100)  # this value controls the type of actor
                        actor_vehicle = None
                        new_actor = None
                        repeat_times = 0
                        while actor_vehicle is None:
                            repeat_times += 1
                            # stuck too long
                            if repeat_times > 100 or state.stuck_duration > 100:
                                # add a fake actor
                                new_actor = Actor(actor_type=None, nav_type=None,
                                                  spawn_point=None,
                                                  dest_point=None, speed=None,
                                                  actor_id=len(actor_list),
                                                  ego_loc=player_loc, ego_vel=vel)
                                if conf.debug:
                                    print("[debug] dont spawn car", new_actor.actor_id, "at", state.num_frames)
                                new_actor.instance = None
                                actor_list.append(new_actor)
                                new_actor.fresh = False
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
                            neighbors_A = nx.single_source_shortest_path_length(state.G,
                                                                                source=(player_road_id, player_lane_id),
                                                                                cutoff=conf.topo_k)
                            neighbors_A[(player_road_id, player_lane_id)] = 0
                            neighbors_B = nx.single_source_shortest_path_length(state.G, source=(
                                waypoint.road_id, waypoint.lane_id), cutoff=conf.topo_k)
                            neighbors_B[(waypoint.road_id, waypoint.lane_id)] = 0
                            if not any(node in neighbors_A and node in neighbors_B for node in state.G.nodes()):
                                repeat_times -= 1
                                continue
                            # we don't want to add bg car in junction or near it
                            # because it may cause a red light problem
                            if waypoint.is_junction or waypoint.next(30 * 3 / 3.6)[-1].is_junction:
                                continue
                            temp_flag = False
                            # don't let sensor distance too close in the same lane
                            for other_actor in actor_list:
                                if other_actor.instance is not None:
                                    other_actor_waypoint = other_actor.get_waypoint(town_map)
                                    if other_actor.instance.get_location().distance(waypoint.transform.location) < 10:
                                        if (other_actor_waypoint.lane_id == waypoint.lane_id) & (
                                                other_actor_waypoint.road_id == waypoint.road_id):
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
                            actor_spawn_point = carla.Transform(
                                carla.Location(x=waypoint.transform.location.x, y=waypoint.transform.location.y,
                                               z=waypoint.transform.location.z + 0.1),
                                carla.Rotation(pitch=0, yaw=roll_degrees, roll=0)
                            )
                            # random choose a car bp from vehicle_bp_library
                            actor_bp = random.choice(vehicle_bp_library)
                            if add_type <= conf.immobile_percentage:
                                # add a immobile car
                                bg_speed = 0
                                new_actor = Actor(actor_type=c.VEHICLE, nav_type=c.IMMOBILE,
                                                  spawn_point=actor_spawn_point,
                                                  dest_point=None, speed=bg_speed,
                                                  actor_id=len(actor_list),
                                                  ego_loc=player_loc, ego_vel=vel, spawn_frame=state.num_frames,
                                                  actor_bp=actor_bp, spawn_stuck_frame=state.stuck_duration)
                            else:
                                bg_speed = random.uniform(0 / 3.6, 20 / 3.6)
                                new_actor = Actor(actor_type=c.VEHICLE, nav_type=c.BEHAVIOR_AGENT,
                                                  spawn_point=actor_spawn_point,
                                                  dest_point=None, speed=bg_speed,
                                                  actor_id=len(actor_list),
                                                  ego_loc=player_loc, ego_vel=vel, spawn_frame=state.num_frames,
                                                  actor_bp=actor_bp, spawn_stuck_frame=state.stuck_duration)
                            # do safe check
                            flag = True
                            for actor in actors_now:
                                if not new_actor.safe_check(actor):
                                    flag = False
                                    break
                            if not new_actor.safe_check(ego):
                                flag = False
                            if flag:
                                actor_vehicle = world.try_spawn_actor(new_actor.actor_bp, actor_spawn_point)
                            else:
                                continue
                        if actor_vehicle is not None:
                            actor_vehicles.append(actor_vehicle)
                            actor_vehicle.set_transform(actor_spawn_point)
                            x_offset = random.uniform(5, 10)
                            y_offset = random.uniform(5, 10)
                            wp_new_location = wp.location + carla.Location(x=x_offset * random.choice([-1, 1]),
                                                                           y=y_offset * random.choice([-1, 1]))
                            new_agent = set_autopilot(actor_vehicle, c.BEHAVIOR_AGENT, actor_spawn_point.location,
                                                      wp_new_location, world)
                            agents_now.append((new_agent, actor_vehicle, new_actor))
                            actor_vehicle.set_target_velocity(
                                new_actor.speed * road_direction)
                            new_actor.set_instance(actor_vehicle)
                            # just add it for BEHAVIOR
                            if conf.agent_type == c.BEHAVIOR:
                                # don't add sensors for non_motorized vehicles
                                if new_actor.actor_bp.get_attribute(
                                        "number_of_wheels").as_int() > max_wheels_for_non_motorized:
                                    new_actor.attach_collision(world, sensors, state)
                                    new_actor.attach_lane_invasion(world, sensors, state)
                            new_actor.fresh = False
                            actors_now.append(new_actor)
                            actor_list.append(new_actor)
                            if conf.debug:
                                print("[debug] spawn new car", new_actor.actor_id, "at", state.num_frames)
                    found_frame = False
                for agent_tuple in agents_now:
                    agent = agent_tuple[0]
                    agent_vehicle = agent_tuple[1]
                    agent._update_information()
                    agent.get_local_planner().set_speed(speed_limit)
                    lp = agent.get_local_planner()
                    if len(lp._waypoints_queue) != 0:
                        control = agent.run_step()
                        # that guy who has the agent
                        agent_vehicle.apply_control(control)

                control = player.get_control()
                state.cont_throttle.append(control.throttle)
                state.cont_brake.append(control.brake)
                state.cont_steer.append(control.steer)
                steer_angle = control.steer * max_steer_angle
                state.steer_angle_list.append(steer_angle)

                current_yaw = player_rot.yaw
                state.yaw_list.append(current_yaw)
                yaw_diff = current_yaw - yaw
                # Yaw range is -180 ~ 180. When vehicle's yaw is oscillating
                # b/w -179 and 179, yaw_diff can be messed up even if the
                # diff is very small. Assuming that it's unrealistic that
                # a vehicle turns more than 180 degrees in under 1/20 seconds,
                # just round the diff around 360.
                if yaw_diff > 180:
                    yaw_diff = 360 - yaw_diff
                elif yaw_diff < -180:
                    yaw_diff = 360 + yaw_diff

                yaw_rate = yaw_diff * c.FRAME_RATE
                state.yaw_rate_list.append(yaw_rate)
                yaw = current_yaw

                # uncomment below to follow along the player
                # set_camera(conf, player, spectator)

                for v in actor_vehicles:
                    dist = player_loc.distance(v.get_location())
                    if dist < state.min_dist:
                        state.min_dist = dist

                for w in actor_walkers:
                    dist = player_loc.distance(w.get_location())
                    if dist < state.min_dist:
                        state.min_dist = dist

                # Get the lateral speed
                player_right_vec = player_rot.get_right_vector()

                lat_speed = abs(vel.x * player_right_vec.x + vel.y * player_right_vec.y)
                lat_speed *= 3.6  # m/s to km/h
                state.lat_speed_list.append(lat_speed)

                player_fwd_vec = player_rot.get_forward_vector()
                lon_speed = abs(vel.x * player_fwd_vec.x + vel.y * player_fwd_vec.y)
                lon_speed *= 3.6
                state.lon_speed_list.append(lon_speed)

                # # Check Autoware-defined destination
                # VehicleReady\nDriving\nMoving\nLaneArea\nCruise\nStraight\nDrive\nGo\n
                # VehicleReady\nWaitOrder\nStopping\nWaitDriveReady\n

                # Check destination
                dist_to_goal = player_loc.distance(goal_loc)
                if conf.agent_type == c.AUTOWARE:
                    if not conf.function.startswith("eval"):
                        output_state = proc_state.stdout.readline()
                        if b"---" in output_state:
                            output_state = proc_state.stdout.readline()
                        if b"Go" in output_state:
                            s_started = True
                            autoware_stuck = 0
                        elif b"nWaitDriveReady" in output_state and s_started:
                            os.system(pub_cmd)
                            if conf.debug:
                                print(goal_msg)
                            print("[carla] Goal republished")
                            autoware_stuck += 1
                            if autoware_stuck == 100:
                                print("\n[*] (Autoware) Reached the destination")
                                print("      dist to goal:", dist_to_goal)
                                if dist_to_goal > 2 and state.num_frames > 300:
                                    state.other_error = "goal"
                                    state.other_error_val = dist_to_goal
                                retval = 0
                                break
                elif conf.agent_type == c.BEHAVIOR:
                    break_flag = False
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
                        delete_actor(agents_now[index][2], actor_vehicles, sensors, agents_now, actors_now)
                    if break_flag:
                        break

                if wait_until_end == 0:
                    retval, wait_until_end = check_violation(conf, cur_frame_id, frame_speed_lim_changed, retval, speed,
                                                             speed_limit, state, wait_until_end)
                else:
                    wait_until_end += 1
                if wait_until_end > 24:
                    break
        except KeyboardInterrupt:
            print("quitting")
            retval = 128

        # jump to finally
        return

    except Exception as e:
        # update states
        # state.num_frames = frame_id - frame_0
        # state.elapsed_time = time.time() - start_time

        print("[-] Runtime error:")
        traceback.print_exc()
        # exc_type, exc_obj, exc_tb = sys.exc_info()
        # print("   (line #{0}) {1}".format(exc_tb.tb_lineno, exc_type))

        retval = 1

    finally:
        # Finalize simulation
        # rospy.signal_shutdown("fin")
        # find biggest weight of actor-list
        state.end = True
        if conf.agent_type == c.BEHAVIOR:
            # remove jpg files
            print("Saving front camera video", end=" ")
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
                print("(done)")
            else:
                print("error:dont save any video")

            cmd = f"rm -f /tmp/fuzzerdata/{username}/front-*.jpg"
            os.system(cmd)

            print("Saving top camera video", end=" ")
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
                print("(done)")
            else:
                print("error:dont save any video")

            cmd = f"rm -f /tmp/fuzzerdata/{username}/top-*.jpg"
            os.system(cmd)

        elif conf.agent_type == c.AUTOWARE:
            os.system("rosnode kill /recorder_video_front")
            os.system("rosnode kill /recorder_video_rear")
            os.system("rosnode kill /recorder_video_top")
            os.system("rosnode kill /recorder_bag")
            while os.path.exists(f"/tmp/fuzzerdata/{username}/bagfile.lz4.bag.active"):
                print("waiting for rosbag to dump data")
                time.sleep(3)
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
                # still doesn't fix docker hanging.

        # if retval == 0:
        # # update states
        # state.num_frames = frame_id - frame_0
        # state.elapsed_time = time.time() - start_time

        # Stop sync mode to prevent the simulator from being blocked
        # waiting for another tick.
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        for actor in actor_list:
            actor.fresh = True
        for s in sensors:
            s.stop()
            s.destroy()
        for w in actor_walkers:
            w.destroy()
        for f in actor_frictions:
            f.destroy()
        for v in actor_vehicles:
            try:
                ret = v.destroy()
                print("destroyed {}: {}".format(v, ret))
            except Exception as e:
                print("Failed to destroy {}: {}".format(v, e))
        for actor in actors_now:
            actor.instance = None
        # Don't reload and exit if user requests so
        if retval == 128:
            return retval
        else:
            if conf.debug:
                print("[debug] reload")
            client.reload_world()
            if conf.debug:
                print('[debug] done.')
            return retval


def _on_front_camera_capture(image,username):
    image.save_to_disk(f"/tmp/fuzzerdata/{username}/front-{image.frame:05d}.jpg")


def _on_top_camera_capture(image,username):
    image.save_to_disk(f"/tmp/fuzzerdata/{username}/top-{image.frame:05d}.jpg")


def _set_camera(conf, player, spectator):
    if conf.view == c.BIRDSEYE:
        _cam_over_player(player, spectator)
    elif conf.view == c.ONROOF:
        _cam_chase_player(player, spectator)
    else:  # fallthru default
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


# def _is_player_on_puddle(player_loc, actor_frictions):
#     for friction in actor_frictions:
#         len_x = float(friction.attributes["extent_x"])
#         len_y = float(friction.attributes["extent_y"])
#         loc_x = friction.get_location().x
#         loc_y = friction.get_location().y
#         p1 = loc_x - len_x / 100
#         p2 = loc_x + len_x / 100
#         p3 = loc_y - len_y / 100
#         p4 = loc_y + len_y / 100
#         p_x = player_loc.x
#         p_y = player_loc.y
#         if p1 <= p_x <= p2 and p3 <= p_y <= p4:
#             return True
#         else:
#             return False


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
            print("\n[*] Lane invasion detected: %.2f" % (
                state.elapsed_time))
            retval = 1
            wait_until_end = 1
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
            if not state.on_red:
                state.stuck = True
                print("\n[*] Stuck for too long: %d" % (state.stuck_duration))
                retval = 1
                wait_until_end = 1
    if conf.check_dict["other"]:
        if state.num_frames > 6000:  # over 5 minutes
            print("\n[*] Simulation taking too long")
            state.other_error = "timeout"
            state.other_error_val = state.num_frames
            retval = 1
            wait_until_end = 1
        if state.other_error:
            print("\n[*] Other error: %d" % (state.signal))
            retval = 1
            wait_until_end = 1
    return retval, wait_until_end
