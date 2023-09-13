import cProfile
import json
import os
import random
import shutil
import time
from typing import List

import carla
import numpy as np
import deap.base

from actor import Actor
from simulate import simulate
import constants as c
import utils
from states import ScenarioState


def get_seed_sp_transform(seed):
    sp = carla.Transform(
        carla.Location(seed["sp_x"], seed["sp_y"], seed["sp_z"] + 2),
        carla.Rotation(seed["roll"], seed["yaw"], seed["pitch"])
    )

    return sp


def get_seed_wp_transform(seed):
    wp = carla.Transform(
        carla.Location(seed["wp_x"], seed["wp_y"], seed["wp_z"]),
        carla.Rotation(0.0, seed["wp_yaw"], 0.0)
    )

    return wp


class ScenarioFitness(deap.base.Fitness):
    """
    Class to represent weight of each fitness function
    """
    # minimize closest distance between pair of ADC
    # maximize number of unique decisions being made
    # maximize pairs of conflict trajectory
    # maximize unique violation
    weights = (-1.0, 1.0, 1.0, 1.0)
    """
    :note: minimize closest distance, maximize number of decisions,
      maximize pairs having conflicting trajectory,
      maximize unique violation. Refer to our paper for more
      detailed explanation.
    """


class Scenario:
    generation_id: int = -1
    scenario_id: int = -1
    fitness: deap.base.Fitness = ScenarioFitness()
    seed_data = {}
    town = None
    weather = {}
    actors_now = []
    actor_list: List[Actor]
    driving_quality_score = None
    found_error = False
    username = os.getenv("USER")

    def __init__(self, conf, seed_data):
        """
        When initializing, perform dry run and get the oracle state
        """
        self.log_filename = None
        self.conf = conf
        self.seed_data = seed_data
        self.state = ScenarioState()

        self.weather["cloud"] = 0
        self.weather["rain"] = 0
        self.weather["wind"] = 0
        self.weather["fog"] = 0
        self.weather["wetness"] = 0
        self.weather["angle"] = 0
        self.weather["altitude"] = 90

        self.actors_now = []
        self.actor_list = []
        self.driving_quality_score = 0
        self.found_error = False

        self.sp = {
            "Location": (self.seed_data["sp_x"], self.seed_data["sp_y"], self.seed_data["sp_z"] + 2),
            "Rotation": (self.seed_data["roll"], self.seed_data["yaw"], self.seed_data["pitch"])
        }
        self.town = self.seed_data["map"]
        # utils.switch_map(conf, self.town, client)

        print("[+] test case initialized")

    def get_distance_from_player(self, location):
        sp = get_seed_sp_transform(self.seed_data)
        return location.distance(sp.location)

    def dump_states(self, state, log_type):
        if self.conf.debug:
            print("[debug] dumping {} data".format(log_type))
        event_dict = {
            "crash": state.crashed,
            "stuck": state.stuck,
            "lane_invasion": state.laneinvaded,
            "red": state.red_violation,
            "speeding": state.speeding,
            "other": state.other_error,
            "other_error_val": state.other_error_val
        }
        config_dict = {
            "fps": c.FRAME_RATE,
            "max_dist_from_player": c.MAX_DIST_FROM_PLAYER,
            "min_dist_from_player": c.MIN_DIST_FROM_PLAYER,
            "abort_seconds": self.conf.timeout,
            "wait_autoware_num_topics": c.WAIT_AUTOWARE_NUM_TOPICS
        }
        state_dict = {"fuzzing_start_time": self.conf.cur_time, "determ_seed": self.conf.determ_seed,
                      "seed": self.seed_data, "weather": self.weather, "autoware_cmd": state.autoware_cmd,
                      "autoware_goal": state.autoware_goal, "first_frame_id": state.first_frame_id,
                      "first_sim_elapsed_time": state.first_sim_elapsed_time, "sim_start_time": state.sim_start_time,
                      "num_frames": state.num_frames, "elapsed_time": state.elapsed_time, "events": event_dict,
                      "config": config_dict}

        filename = "{}_{}.json".format(state.mutation, time.time())
        if log_type == "queue":
            out_dir = self.conf.queue_dir
        with open(os.path.join(out_dir, filename), "w") as fp:
            json.dump(state_dict, fp)
        if self.conf.debug:
            print("[debug] dumped")
        return filename

    def run_test(self, exec_state):
        if self.conf.debug:
            print("[debug] use scenario:id=", self.scenario_id)
            # print("Weather:", self.weather)
        # print("before sim", time.time())
        self.state.end = False
        sp = get_seed_sp_transform(self.seed_data)
        wp = get_seed_wp_transform(self.seed_data)

        ret, self.actor_list = simulate(
            conf=self.conf,
            state=self.state,
            exec_state=exec_state,
            sp=sp,
            wp=wp,
            weather_dict=self.weather,
            # frictions_list=self.puddles,
            actor_list=self.actor_list
        )

        print("actor_list", len(self.actor_list))
        log_filename = self.dump_states(self.state, log_type="queue")
        self.log_filename = log_filename

        error = self.check_error(self.state)

        self.save_video(error, log_filename)

        if not self.conf.function.startswith("eval"):
            if ret == 128:
                return 128
        if error:
            self.found_error = True
            return 1
        # if state.num_frames <= c.FRAME_RATE:
        #     # Trap for an unlikely situation where test target didn't load
        #     # but we somehow got here.
        #     print("[-] Not enough data for scoring ({} frames)".format(
        #         state.num_frames))
        #     return 1

        # with open(os.path.join(self.conf.score_dir, log_filename), "w") as fp:
        #     json.dump(state.deductions, fp)

    def save_video(self, error, log_filename):
        if self.conf.agent_type == c.AUTOWARE:
            if error:
                # print("copying bag & video files")
                shutil.copyfile(
                    os.path.join(self.conf.queue_dir, log_filename),
                    os.path.join(self.conf.error_dir, log_filename)
                )
                shutil.copyfile(
                    f"/tmp/fuzzerdata/{self.username}/bagfile.lz4.bag",
                    os.path.join(self.conf.rosbag_dir, log_filename.replace(".json", ".bag"))
                )

            shutil.copyfile(
                f"/tmp/fuzzerdata/{self.username}/front.mp4",
                os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-front.mp4"))
            )
            shutil.copyfile(
                f"/tmp/fuzzerdata/{self.username}/rear.mp4",
                os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-rear.mp4"))
            )
            shutil.copyfile(
                f"/tmp/fuzzerdata/{self.username}/top.mp4",
                os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-top.mp4"))
            )

        elif self.conf.agent_type == c.BEHAVIOR:
            if error:
                shutil.copyfile(
                    os.path.join(self.conf.queue_dir, log_filename),
                    os.path.join(self.conf.error_dir, log_filename)
                )
            shutil.copyfile(
                f"/tmp/fuzzerdata/{self.username}/front.mp4",
                os.path.join(
                    self.conf.cam_dir,
                    log_filename.replace(".json", "-front.mp4")
                )
            )

            shutil.copyfile(
                f"/tmp/fuzzerdata/{self.username}/top.mp4",
                os.path.join(
                    self.conf.cam_dir,
                    log_filename.replace(".json", "-top.mp4")
                )
            )

    def check_error(self, state):
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
        return error
