import cProfile
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


class Scenario:
    seed_data = {}
    town = None
    weather = {}
    actors_now = []
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

        self.actors_now = []
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
            print("[debug] dumping {} data".format(log_type))

        state_dict = {}
        state_dict["fuzzing_start_time"] = self.conf.cur_time
        state_dict["determ_seed"] = self.conf.determ_seed
        state_dict["seed"] = self.seed_data
        state_dict["weather"] = self.weather
        state_dict["autoware_cmd"] = state.autoware_cmd
        state_dict["autoware_goal"] = state.autoware_goal
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
        # state_dict["vehicle_states"] = vehicle_state_dict

        control_dict = {
            "throttle": state.cont_throttle,
            "brake": state.cont_brake,
            "steer": state.cont_steer
        }
        # state_dict["control_cmds"] = control_dict
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
            print("[debug] dumped")
        return filename

    def run_test(self, state):
        if self.conf.debug:
            print("[debug] call simutale.simulate()")
            # print("Weather:", self.weather)
        # print("before sim", time.time())
        state.end = False
        sp = self.get_seed_sp_transform(self.seed_data)
        wp = self.get_seed_wp_transform(self.seed_data)

        # profiler = cProfile.Profile()
        # profiler.enable()
        ret = simulate(
            conf=self.conf,
            state=state,
            sp=sp,
            wp=wp,
            weather_dict=self.weather,
            frictions_list=self.puddles,
            actor_list=self.actor_list
        )
        # profiler.disable()
        # profiler.print_stats(sort='time')

        print("actor_list", len(self.actor_list))
        log_filename = self.dump_states(state, log_type="queue")
        self.log_filename = log_filename
        # print("after logging", time.time())

        if state.spawn_failed:
            obj = state.spawn_failed_object
            if self.conf.debug:
                print("[debug] failed object:", obj)

            # don't try to spawn an infeasible actor in the next run
            # XXX: and we need a map of coordinates that represent
            #      spawn-feasibility
            if "level" in obj:
                self.puddles.remove(obj)
            else:
                self.actors_now.remove(obj)
            return -1
        if self.conf.debug:
            print("----- Check for errors -----")
        error = False
        if self.conf.check_dict["crash"] and state.crashed:
            if self.conf.debug:
                print("[debug] Crashed:", state.collision_event)
                oa = state.collision_event.other_actor
                print(f"  - against {oa.type_id}")
            error = True
        if self.conf.check_dict["stuck"] and state.stuck:
            if self.conf.debug:
                print("[debug] Vehicle stuck:", state.stuck_duration)
            error = True
        if self.conf.check_dict["lane"] and state.laneinvaded:
            if self.conf.debug:
                le_list = state.laneinvasion_event
                le = le_list[0]  # only consider the very first invasion
                print("[debug] Lane invasion:", le)
                lm_list = le.crossed_lane_markings
                for lm in lm_list:
                    print("  - crossed {} lane (allows {} change)".format(
                        lm.color, lm.lane_change))
            error = True
        if self.conf.check_dict["red"] and state.red_violation:
            error = True
        if self.conf.check_dict["speed"] and state.speeding:
            if self.conf.debug:
                print("[debug] Speeding: {} km/h".format(state.speed[-1]))
            error = True
        if self.conf.check_dict["other"] and state.other_error:
            if state.other_error == "timeout":
                if self.conf.debug:
                    print("[debug] Simulation took too long")
            elif state.other_error == "goal":
                if self.conf.debug:
                    print("[debug] Goal is too far:", state.other_error_val, "m")
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
            shutil.copyfile(
                f"/tmp/fuzzerdata/{g.username}/top.mp4",
                os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-top.mp4"))
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

        with open(os.path.join(self.conf.score_dir, log_filename), "w") as fp:
            json.dump(state.deductions, fp)
