import cProfile
import json
import os
import pdb
import shutil
from typing import List

import carla
import cv2
import numpy as np
import deap.base

from npc import NPC
from cluster import draw_picture, shift_scale_points_group
from simulate import simulate
import constants as c
from states import ScenarioState

colors = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (0, 255, 255), (255, 0, 255), (192, 192, 192), (128, 0, 0),
    (128, 128, 0), (0, 128, 0)
]


def get_seed_sp_transform(seed):
    sp = carla.Transform(
        carla.Location(seed["sp_x"], seed["sp_y"], seed["sp_z"]),
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
    # minimize the closest distance between a pair of ADC
    # for test
    weights = (-1.0, -1.0, -1.0)
    """
    Todo: note: 
    """


class Scenario:
    generation_id: int = -1
    scenario_id: int = -1
    fitness: deap.base.Fitness = ScenarioFitness()
    seed_data = {}
    town = None
    weather = {}
    npc_now = []
    npc_list: List[NPC]
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

        self.npc_now = []
        self.npc_list = []
        self.driving_quality_score = 0
        self.found_error = False

        self.sp = {
            "Location": (self.seed_data["sp_x"], self.seed_data["sp_y"], self.seed_data["sp_z"]),
            "Rotation": (self.seed_data["roll"], self.seed_data["yaw"], self.seed_data["pitch"])
        }
        self.town = self.seed_data["map"]
        # utils.switch_map(conf, self.town, client)

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
            "wait_autoware_num_topics": c.WAIT_AUTOWARE_NUM_NODES
        }
        # state_dict = {"fuzzing_start_time": self.conf.cur_time, "determ_seed": self.conf.determ_seed,
        #               "seed": self.seed_data, "weather": self.weather, "autoware_cmd": state.autoware_cmd,
        #               "autoware_goal": state.autoware_goal, "first_frame_id": state.first_frame_id,
        #               "first_sim_elapsed_time": state.first_sim_elapsed_time, "sim_start_time": state.sim_start_time,
        #               "num_frames": state.num_frames, "elapsed_time": state.elapsed_time, "events": event_dict,
        #               "config": config_dict}
        state_dict = {"events": event_dict, "config": config_dict}
        filename = "gid:{}_sid:{}.json".format(self.generation_id, self.scenario_id)
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
        self.reload_state()
        sp = get_seed_sp_transform(self.seed_data)
        wp = get_seed_wp_transform(self.seed_data)
        ret, self.npc_list, self.state = simulate(
            conf=self.conf,
            state=self.state,
            exec_state=exec_state,
            sp=sp,
            wp=wp,
            weather_dict=self.weather,
            npc_list=self.npc_list
        )
        if ret == -1:
            return -1
        if not self.conf.function.startswith("eval"):
            if ret == 128:
                return 128
        log_filename = self.dump_states(self.state, log_type="queue")
        self.log_filename = log_filename
        error = self.check_error(self.state)
        # # reload scenario state
        # self.state = ScenarioState()
        self.save_video(error, log_filename)
        if self.state.trace_graph_important != []:
            self.save_trace(self.state.trace_graph_important, log_filename)
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

    def reload_state(self):
        self.state.crashed = False
        self.state.collision_to = None
        self.state.stuck = False
        self.state.stuck_duration = 0
        self.state.laneinvaded = False
        self.state.laneinvasion_event = []
        self.state.speeding = False
        self.state.speed = []
        self.state.speed_lim = []
        self.state.on_red = False
        self.state.on_red_speed = []
        self.state.red_violation = False
        self.state.other_error = False
        self.state.other_error_val = 0

    def save_video(self, error, log_filename):
        try:
            # if self.conf.agent_type == c.AUTOWARE:
            #     if error:
            #         # print("copying bag & video files")
            #         shutil.copyfile(
            #             os.path.join(self.conf.queue_dir, log_filename),
            #             os.path.join(self.conf.error_dir, log_filename)
            #         )
            #         # shutil.copyfile(
            #         #     f"/tmp/fuzzerdata/{c.USERNAME}/bagfile.lz4.bag",
            #         #     os.path.join(self.conf.rosbag_dir, log_filename.replace(".json", ".bag"))
            #         # )
            #
            #     shutil.copyfile(
            #         f"/tmp/fuzzerdata/{c.USERNAME}/front.mp4",
            #         os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-front.mp4"))
            #     )
            #     shutil.copyfile(
            #         f"/tmp/fuzzerdata/{c.USERNAME}/top.mp4",
            #         os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-top.mp4"))
            #     )
            # elif self.conf.agent_type == c.BEHAVIOR:
            if True:
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
            print("save video done")
        except FileNotFoundError:
            print("FileNotFoundError")
            os._exit(0)

    def save_trace(self, trace_graph, log_filename):
        new_trace_graph = np.array([np.array([point[:2] for point in trace]) for trace in trace_graph])
        trace_graph_points = shift_scale_points_group(np.array(new_trace_graph), (1024, 1024))
        img = np.full((1024, 1024, 3), 255, dtype=np.uint8)
        for j, trace in enumerate(trace_graph_points):
            color = colors[j % len(colors)]
            img = draw_picture(trace, color=color, base_image=img)
            cv2.imwrite(os.path.join(
                self.conf.trace_dir,
                log_filename.replace(".json", ".png")
            ), img)
        print("save trace done")

    def check_error(self, state):
        if self.conf.debug:
            print("----- Check for errors -----")
        error = False
        if self.conf.check_dict["crash"] and state.crashed:
            if self.conf.debug:
                print("[debug] Crashed with:", state.collision_to)
                oa = state.collision_to
                print(f"  - against {oa}")
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
