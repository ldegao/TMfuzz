import json
import os
import random
import shutil
import time

import carla
import numpy as np

from simulate import simulate
import constants as c
import utils
import globals as g
from driving_quality import get_vx_light, get_ay_list, get_ay_diff_list, get_ay_heavy, get_swa_diff_list, get_swa_heavy, \
    get_ay_gain, get_ay_peak, get_frac_drop, get_abs_yr, check_hard_acc, check_hard_braking, check_hard_turn, \
    get_oversteer_level, get_understeer_level
from utils import get_carla_transform
from actor import Actor


class Scenario:
    seed_data = {}
    town = None
    weather = {}
    actor_now = []
    actor_list = []
    puddles = []
    driving_quality_score = None
    found_error = False

    def __init__(self, conf):
        """
        When initializing, perform dry run and get the oracle state
        """
        self.conf = conf

        # First, connect to the client once.
        # This is important because the same TM instance has to be reused
        # over multiple simulations for autopilot to be enabled.
        (client, tm) = utils.connect(self.conf)

        self.weather["cloud"] = 0
        self.weather["rain"] = 0
        self.weather["puddle"] = 0
        self.weather["wind"] = 0
        self.weather["fog"] = 0
        self.weather["wetness"] = 0
        self.weather["angle"] = 0
        self.weather["altitude"] = 90

        self.actor_now = []
        self.actor_list = []
        self.puddles = []
        self.driving_quality_score = 0
        self.found_error = False
        seedfile = os.path.join(conf.seed_dir, conf.cur_scenario)
        with open(seedfile, "r") as fp:
            seed = json.load(fp)
            self.seed_data = seed
        # why do author delete this?
        # because carla.Transform can not be
        # self.sp = carla.Transform(
        #     carla.Location(seed["sp_x"], seed["sp_y"], seed["sp_z"]+2),
        #     carla.Rotation(seed["roll"], seed["yaw"], seed["pitch"])
        # )
        self.sp = {
            "Location": (seed["sp_x"], seed["sp_y"], seed["sp_z"] + 2),
            "Rotation": (seed["roll"], seed["yaw"], seed["pitch"])
        }
        # self.wp = carla.Transform(
        #     carla.Location(seed["wp_x"], seed["wp_y"], seed["wp_z"]),
        #     carla.Rotation(0.0, seed["wp_yaw"], 0.0)
        # )

        self.town = seed["map"]
        utils.switch_map(conf, self.town)

        # self.oracle_state = dry_run(self.conf, self.client, self.tm,
        # self.town, self.sp, self.wp, self.weather)
        # print("oracle:", self.oracle_state)

        # simutale.get_waypoints()

        # self.quota = self.conf.initial_quota

        print("[+] test case initialized")

    def get_seed_sp_transform(self, seed):
        sp = carla.Transform(
            carla.Location(seed["sp_x"], seed["sp_y"], seed["sp_z"] + 2),
            carla.Rotation(seed["roll"], seed["yaw"], seed["pitch"])
        )

        return sp

    def get_seed_wp_transform(self, seed):
        wp = carla.Transform(
            carla.Location(seed["wp_x"], seed["wp_y"], seed["wp_z"]),
            carla.Rotation(0.0, seed["wp_yaw"], 0.0)
        )

        return wp

    def get_distance_from_player(self, location):
        sp = self.get_seed_sp_transform(self.seed_data)
        return location.distance(sp.location)

    def add_actor(self, actor_type, nav_type, location, rotation, speed,
                  sp_idx, dp_idx):
        """
        Mutator calls `ret = add_actor` with the mutated parameters
        until ret == 0.

        actor_type: int
        location: (float x, float y, float z)
        rotation: (float yaw, float roll, float pitch)
        speed: float

        1) check if the location is within the map
        2) check if the location is not preoccupied by other actor_now
        3) check if the actor's path collides with player's path
        return 0 iif 1), 2), 3) are satisfied.
        """

        maneuvers = None
        # do validity checks
        if nav_type == c.LINEAR or nav_type == c.IMMOBILE:
            spawn_point = (location, rotation)  # carla.Transform(location, rotation)
            dest_point = None

        elif nav_type == c.MANEUVER:
            spawn_point = (location, rotation)  # carla.Transform(location, rotation)
            dest_point = None

            # [direction (-1: L, 0: Fwd, 1: R),
            #  velocity (m/s) if fwd / apex degree if lane change,
            #  frame_maneuver_performed]
            maneuvers = [
                [0, 0, 0],
                [0, 8, 0],
                [0, 8, 0],
                [0, 8, 0],
                [0, 8, 0],
            ]

        elif nav_type == c.AUTOPILOT:
            assert (g.list_spawn_points)
            if sp_idx == dp_idx:
                return -1
            sp = g.list_spawn_points[sp_idx]

            # prevent autopilot vehicles from being
            # spawned beneath the player vehicle
            sp.location.z = 1.5

            spawn_point = (
                (sp.location.x, sp.location.y, sp.location.z),
                (sp.rotation.roll, sp.rotation.pitch, sp.rotation.yaw)
            )

            dp = g.list_spawn_points[dp_idx]

            dest_point = (
                (dp.location.x, dp.location.y, dp.location.z),
                (dp.rotation.roll, dp.rotation.pitch, dp.rotation.yaw)
            )

        dist = self.get_distance_from_player(
            get_carla_transform(spawn_point).location
        )

        if dist > c.MAX_DIST_FROM_PLAYER:
            # print("[-] too far from player: {}m".format(int(dist)))
            return -1

        elif (dist < c.MIN_DIST_FROM_PLAYER) and nav_type != c.MANEUVER:
            # print("[-] too close to the player: {}m".format(int(dist)))
            return -1

        # new_actor = {
        #     "type": actor_type,
        #     "nav_type": nav_type,
        #     "spawn_point": spawn_point,
        #     "dest_point": dest_point,
        #     "speed": speed,
        #     "maneuvers": maneuvers
        # }
        # TODO: deal with maneuvers
        new_actor = Actor(actor_type=actor_type, nav_type=nav_type, spawn_point=spawn_point, dest_point=dest_point,
                          speed=speed)
        self.actor_now.append(new_actor)
        return 0

    def add_puddle(self, level, location, size):
        """
        Mutator calls `ret = add_friction` with the mutated parameters
        until ret == 0.

        level: float [0.0:1.0]
        location: (float x, float y, float z)
        size: (float xlen, float ylen, float zlen)

        1) check if the location is within the map
        2) check if the friction box lies on the player's path
        return 0 iff 1, and 2) are satisfied.
        """

        if self.conf.function != "eval-os" and self.conf.function != "eval-us":
            dist = self.get_distance_from_player(
                carla.Location(location[0], location[1], location[2])
            )
            if (dist > c.MAX_DIST_FROM_PLAYER):
                # print("[-] too far from player: {}m".format(int(dist)))
                return -1

        rotation = (0, 0, 0)
        spawn_point = (location, rotation)  # carla.Transform(location, carla.Rotation())

        new_puddle = {
            "level": level,
            "size": size,
            "spawn_point": spawn_point
        }

        self.puddles.append(new_puddle)

        return 0

    def dump_states(self, state, log_type):
        if self.conf.debug:
            print("[*] dumping {} data".format(log_type))

        state_dict = {}

        state_dict["fuzzing_start_time"] = self.conf.cur_time
        state_dict["determ_seed"] = self.conf.determ_seed
        state_dict["seed"] = self.seed_data
        state_dict["weather"] = self.weather
        state_dict["autoware_cmd"] = state.autoware_cmd
        state_dict["autoware_goal"] = state.autoware_goal

        # actor_list = []
        # for actor in self.actor_now:
        #     # re-convert from carla.transform to xyz
        #     # TODO: after actor was chaged to new style, this part should be changed
        #     actor_dict = {
        #         "type": actor["type"],
        #         "nav_type": actor["nav_type"],
        #         "speed": actor["speed"],
        #     }
        #     if actor["spawn_point"] is not None:
        #         actor_dict["sp_x"] = actor["spawn_point"][0][0]
        #         actor_dict["sp_y"] = actor["spawn_point"][0][1]
        #         actor_dict["sp_z"] = actor["spawn_point"][0][2]
        #         actor_dict["sp_roll"] = actor["spawn_point"][1][0]
        #         actor_dict["sp_pitch"] = actor["spawn_point"][1][1]
        #         actor_dict["sp_yaw"] = actor["spawn_point"][1][2]
        #
        #     if actor["dest_point"] is not None:
        #         actor_dict["dp_x"] = actor["dest_point"][0][0]
        #         actor_dict["dp_y"] = actor["dest_point"][0][1]
        #         actor_dict["dp_z"] = actor["dest_point"][0][2]
        #     actor_list.append(actor_dict)
        # state_dict["actor_now"] = actor_list

        puddle_list = []
        for puddle in self.puddles:
            puddle_dict = {"level": puddle["level"], "sp_x": puddle["spawn_point"][0][0],
                           "sp_y": puddle["spawn_point"][0][1], "sp_z": puddle["spawn_point"][0][2],
                           "size_x": puddle["size"][0], "size_y": puddle["size"][1], "size_z": puddle["size"][2]}
            puddle_list.append(puddle_dict)
        state_dict["puddles"] = puddle_list

        state_dict["first_frame_id"] = state.first_frame_id
        state_dict["first_sim_elapsed_time"] = state.first_sim_elapsed_time
        state_dict["sim_start_time"] = state.sim_start_time
        state_dict["num_frames"] = state.num_frames
        state_dict["elapsed_time"] = state.elapsed_time

        state_dict["deductions"] = state.deductions

        vehicle_state_dict = {
            "speed": state.speed,
            "steer_wheel_angle": state.steer_angle_list,
            "yaw": state.yaw_list,
            "yaw_rate": state.yaw_rate_list,
            "lat_speed": state.lat_speed_list,
            "lon_speed": state.lon_speed_list,
            "min_dist": state.min_dist
        }
        state_dict["vehicle_states"] = vehicle_state_dict

        control_dict = {
            "throttle": state.cont_throttle,
            "brake": state.cont_brake,
            "steer": state.cont_steer
        }
        state_dict["control_cmds"] = control_dict

        event_dict = {
            "crash": state.crashed,
            "stuck": state.stuck,
            "lane_invasion": state.laneinvaded,
            "red": state.red_violation,
            "speeding": state.speeding,
            "other": state.other_error,
            "other_error_val": state.other_error_val
        }
        state_dict["events"] = event_dict

        config_dict = {
            "fps": c.FRAME_RATE,
            "max_dist_from_player": c.MAX_DIST_FROM_PLAYER,
            "min_dist_from_player": c.MIN_DIST_FROM_PLAYER,
            "abort_seconds": self.conf.timeout,
            "wait_autoware_num_topics": c.WAIT_AUTOWARE_NUM_TOPICS
        }
        state_dict["config"] = config_dict

        filename = "{}_{}_{}.json".format(state.campaign_cnt, state.mutation, time.time())

        if log_type == "queue":
            out_dir = self.conf.queue_dir

        # elif log_type == "error":
        # out_dir = self.conf.error_dir
        # elif log_type == "cov":
        # out_dir = self.conf.cov_dir

        with open(os.path.join(out_dir, filename), "w") as fp:
            json.dump(state_dict, fp)

        if self.conf.debug:
            print("[*] dumped")

        return filename

    def run_test(self, state):
        if self.conf.debug:
            print("[*] call simutale.simulate()")
            # print("Weather:", self.weather)
        # print("before sim", time.time())
        state.end = False
        sp = self.get_seed_sp_transform(self.seed_data)
        wp = self.get_seed_wp_transform(self.seed_data)
        ret = simulate(
            conf=self.conf,
            state=state,
            sp=sp,
            wp=wp,
            weather_dict=self.weather,
            frictions_list=self.puddles,
            actor_list=self.actor_list
        )
        print("actor_list", len(self.actor_list))
        # print("after sim", time.time())
        # print("before logging", time.time())
        log_filename = self.dump_states(state, log_type="queue")
        self.log_filename = log_filename
        # print("after logging", time.time())

        if state.spawn_failed:
            obj = state.spawn_failed_object
            if self.conf.debug:
                print("failed object:", obj)

            # don't try to spawn an infeasible actor in the next run
            # XXX: and we need a map of coordinates that represent
            #      spawn-feasibility
            if "level" in obj:
                self.puddles.remove(obj)
            else:
                self.actor_now.remove(obj)
            return -1

        # print("before error checking", time.time())
        if self.conf.debug:
            print("----- Check for errors -----")
        error = False
        if self.conf.check_dict["crash"] and state.crashed:
            if self.conf.debug:
                print("Crashed:", state.collision_event)
                oa = state.collision_event.other_actor
                print(f"  - against {oa.type_id}")
            error = True
        if self.conf.check_dict["stuck"] and state.stuck:
            if self.conf.debug:
                print("Vehicle stuck:", state.stuck_duration)
            error = True
        if self.conf.check_dict["lane"] and state.laneinvaded:
            if self.conf.debug:
                le_list = state.laneinvasion_event
                le = le_list[0]  # only consider the very first invasion
                print("Lane invasion:", le)
                lm_list = le.crossed_lane_markings
                for lm in lm_list:
                    print("  - crossed {} lane (allows {} change)".format(
                        lm.color, lm.lane_change))
            error = True
        if self.conf.check_dict["red"] and state.red_violation:
            error = True
        if self.conf.check_dict["speed"] and state.speeding:
            if self.conf.debug:
                print("Speeding: {} km/h".format(state.speed[-1]))
            error = True
        if self.conf.check_dict["other"] and state.other_error:
            if state.other_error == "timeout":
                if self.conf.debug:
                    print("Simulation took too long")
            elif state.other_error == "goal":
                if self.conf.debug:
                    print("Goal is too far:", state.other_error_val, "m")
            error = True

        # print("before file ops", time.time())
        if self.conf.agent_type == c.AUTOWARE:
            if error:
                # print("copying bag & video files")
                shutil.copyfile(
                    os.path.join(self.conf.queue_dir, log_filename),
                    os.path.join(self.conf.error_dir, log_filename)
                )
                shutil.copyfile(
                    f"/tmp/fuzzerdata/{g.username}/bagfile.lz4.bag",
                    os.path.join(self.conf.rosbag_dir, log_filename.replace(".json", ".bag"))
                )

            shutil.copyfile(
                f"/tmp/fuzzerdata/{g.username}/front.mp4",
                os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-front.mp4"))
            )
            shutil.copyfile(
                f"/tmp/fuzzerdata/{g.username}/rear.mp4",
                os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-rear.mp4"))
            )

        elif self.conf.agent_type == c.BASIC or self.conf.agent_type == c.BEHAVIOR:
            if error:
                shutil.copyfile(
                    os.path.join(self.conf.queue_dir, log_filename),
                    os.path.join(self.conf.error_dir, log_filename)
                )
            shutil.copyfile(
                f"/tmp/fuzzerdata/{g.username}/front.mp4",
                os.path.join(
                    self.conf.cam_dir,
                    log_filename.replace(".json", "-front.mp4")
                )
            )

            shutil.copyfile(
                f"/tmp/fuzzerdata/{g.username}/top.mp4",
                os.path.join(
                    self.conf.cam_dir,
                    log_filename.replace(".json", "-top.mp4")
                )
            )

        # print("after file ops", time.time())

        if not self.conf.function.startswith("eval"):
            if ret == 128:
                return 128
        if error:
            self.found_error = True
            return 1

        if state.num_frames <= c.FRAME_RATE:
            # Trap for unlikely situation where test target didn't load
            # but we somehow got here.
            print("[-] Not enough data for scoring ({} frames)".format(
                state.num_frames))
            return 1

        # print("before scoring", time.time())
        if self.conf.debug:
            print("----- Scoring -----")
            # print("[debug] # frames:", state.num_frames)
            # print("[debug] elapsed time:", state.elapsed_time)
            # print("[debug] dist:", state.min_dist)
        np.set_printoptions(precision=3, suppress=True)

        # Attributes
        speed_list = np.array(state.speed)
        acc_list = np.diff(speed_list)

        Vx_list = np.array(state.lon_speed_list)
        Vy_list = np.array(state.lat_speed_list)
        SWA_list = np.array(state.steer_angle_list)

        # filter & process attributes
        Vx_light = get_vx_light(Vx_list)
        Ay_list = get_ay_list(Vy_list)
        Ay_diff_list = get_ay_diff_list(Ay_list)
        Ay_heavy = get_ay_heavy(Ay_list)
        SWA_diff_list = get_swa_diff_list(Vy_list)
        SWA_heavy_list = get_swa_heavy(SWA_list)
        Ay_gain = get_ay_gain(SWA_heavy_list, Ay_heavy)
        Ay_peak = get_ay_peak(Ay_gain)
        frac_drop = get_frac_drop(Ay_gain, Ay_peak)
        abs_yr = get_abs_yr(state.yaw_rate_list)

        deductions = 0

        # avoid infinitesimal md
        if int(state.min_dist) > 100:
            md = 0
        else:
            md = (1 / int(state.min_dist))

        ha = int(check_hard_acc(acc_list))
        hb = int(check_hard_braking(acc_list))
        ht = int(check_hard_turn(Vy_list, SWA_list))

        deductions += ha + hb + ht + md

        # check oversteer and understeer
        os_thres = 4
        us_thres = 4
        num_oversteer = 0
        num_understeer = 0
        for fid in range(len(Vy_list) - 2):
            SWA_diff = SWA_diff_list[fid]
            Ay_diff = Ay_diff_list[fid]
            yr = abs_yr[fid]

            Vx = Vx_light[fid]
            SWA2 = SWA_heavy_list[fid]
            fd = frac_drop[fid]
            os_level = get_oversteer_level(SWA_diff, Ay_diff, yr)
            us_level = get_understeer_level(fd)

            if os_level >= os_thres:
                if Vx > 5 and Ay_diff > 0.1:
                    num_oversteer += 1
                    # print("OS @%d %.2f (SWA %.4f Ay %.4f AVz %.4f Vx %.4f)" %(
                    # fid, os_level, SWA_diff, Ay_diff, yr, Vx))
            if us_level >= us_thres:
                if Vx > 5 and SWA2 > 10:
                    num_understeer += 1
                    # print("US @%d %.2f (SA %.4f FD %.4f Vx %.4f)" %(
                    # fid, us_level, sa2, fd, Vx))
        ovs = int(num_oversteer)
        uds = int(num_understeer)
        deductions += ovs + uds
        state.deductions = {
            "ha": ha, "hb": hb, "ht": ht, "os": ovs, "us": uds, "md": md
        }

        self.driving_quality_score = -deductions

        print("[*] driving quality score: {}".format(
            self.driving_quality_score))

        # print("after scoring", time.time())

        with open(os.path.join(self.conf.score_dir, log_filename), "w") as fp:
            json.dump(state.deductions, fp)
