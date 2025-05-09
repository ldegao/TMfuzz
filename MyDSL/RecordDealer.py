import glob
import math
import os
import random
import re
import sys
import time
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import numpy as np

from APFPlanner import APFPlanner, local_to_global
from Obstacle import Obstacle

try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


class HeroPlanner:
    def __init__(self, client, recorder_path, frame_id):
        """
        Initializes the HeroPlanner with CARLA client, recorder path, hero vehicle ID, and frame ID.
        """
        self.model_size_map = {}
        self.client = client
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

        self.recorder_info = self.client.show_recorder_file_info(recorder_path, True)
        self.frame_id = frame_id
        self.car_data = parse_car_data(self.recorder_info)
        self.hero_id = next((vid for vid, vdata in self.car_data.items() if vdata.get('role_name') == 'hero'), None)
        self.ego_data, self.surrounding_vehicles = self.parse_frame_data(self.recorder_info, frame_id)
        self.load_map_from_recorder_info()

    def load_map_from_recorder_info(self):
        """
        Extracts map name from recorder_info and loads the corresponding CARLA world.
        """
        map_name = None

        match = re.search(r"Map\s*:\s*(\S+)", self.recorder_info)
        if match:
            map_name = match.group(1)

        if map_name:
            print(f"[INFO] Loading map: {map_name}")
            self.world = self.client.load_world(map_name)
        else:
            print("[WARN] Map name not found in recorder_info.")

    def plan(self):
        ego_location = carla.Location(x=self.ego_data.x, y=self.ego_data.y, z=0.5)
        ego_waypoint = self.carla_map.get_waypoint(ego_location, project_to_road=True, lane_type=carla.LaneType.Driving)

        lane_waypoints = []
        current_waypoint = ego_waypoint
        while current_waypoint:
            lane_waypoints.append(current_waypoint)
            current_waypoint = current_waypoint.next(ego_waypoint.lane_width)[0]
            if current_waypoint.road_id != ego_waypoint.road_id or current_waypoint.lane_id != ego_waypoint.lane_id:
                break

        goal_waypoint = lane_waypoints[-1]
        goal = (goal_waypoint.transform.location.x, goal_waypoint.transform.location.y)

        road_waypoints = self.carla_map.generate_waypoints(1.0)
        current_road_id = ego_waypoint.road_id
        lanes_in_road = [wp for wp in road_waypoints if wp.road_id == current_road_id]
        num_lanes = len(set(wp.lane_id for wp in lanes_in_road))
        road_width = ego_waypoint.lane_width * num_lanes

        lane_line_types = []
        for i in range(1, num_lanes):
            left_lane = ego_waypoint.get_left_lane()
            if left_lane and left_lane.lane_change in [carla.LaneChange.Left, carla.LaneChange.Both]:
                lane_line_types.append(1)
            else:
                lane_line_types.append(0)
            ego_waypoint = left_lane if left_lane else ego_waypoint

        intended_lane_index = 0
        for i, wp in enumerate(lane_waypoints):
            if wp.road_id == ego_waypoint.road_id and wp.lane_id == ego_waypoint.lane_id:
                intended_lane_index = i
                break

        lane_angle = math.radians(ego_waypoint.transform.rotation.yaw)
        road_origin = (ego_waypoint.transform.location.x, ego_waypoint.transform.location.y)

        start = (self.ego_data.x, self.ego_data.y)
        ego_speed = math.sqrt(self.ego_data.vx ** 2 + self.ego_data.vy ** 2)
        apf_planner = APFPlanner(start, goal, self.surrounding_vehicles, ego_speed)

        apf_planner.set_lane_info(
            road_width=road_width,
            num_lanes=num_lanes,
            lane_line_types=lane_line_types,
            intended_lane_index=intended_lane_index,
            lane_angle=lane_angle,
            road_origin=road_origin
        )

        trajectory, trajectory_velocity = apf_planner.plan()
        return apf_planner, trajectory, trajectory_velocity

    def parse_frame_data(self, recorder_info, frame_id):
        frame_pattern = rf"Positions:.*?Frame {frame_id}.*?Dynamic actors.*?Frame"
        frame_match = re.search(frame_pattern, recorder_info, re.DOTALL)
        frame_str = frame_match.group(0) if frame_match else ''

        static_pattern = r"Id:\s*(\d+)\s+Location:\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)\s*Rotation\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)"
        static_matches = re.findall(static_pattern, frame_str)
        static_data = {}
        for match in static_matches:
            vid = int(match[0])
            static_data[vid] = {
                'id': vid,
                'location': tuple(map(float, match[1:4])),
                'rotation': tuple(map(float, match[4:7]))
            }

        dynamic_pattern = r"Id:\s*(\d+)\s*linear_velocity:\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)\s*angular_velocity:\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)"
        dynamic_matches = re.findall(dynamic_pattern, frame_str)
        for match in dynamic_matches:
            vid = int(match[0])
            if vid in static_data:
                static_data[vid].update({
                    'linear_velocity': tuple(map(float, match[1:4])),
                    'angular_velocity': tuple(map(float, match[4:7]))
                })

        destroyed_ids = set(map(int, re.findall(r"\bDestroy\s+(\d+)", recorder_info)))
        print(f"[INFO] Destroyed IDs: {destroyed_ids}")

        ego_data = None
        surrounding_vehicles = []

        for vid, vdata in static_data.items():
            if vid in destroyed_ids:
                print(f"[INFO] Vehicle {vid} was destroyed, skipping.")
                continue
            yaw_rad = math.radians(vdata['rotation'][2])
            hx, hy = math.cos(yaw_rad), math.sin(yaw_rad)

            vehicle_model = self.car_data[vid]['vehicle_model'] if vid in self.car_data else None

            if vehicle_model in self.model_size_map:
                length, width = self.model_size_map[vehicle_model]
                print(
                    f"[INFO] Vehicle model {vehicle_model} found in cache: length = {length:.2f}, width = {width:.2f}")
            else:
                length, width = 4.5, 2.0
                if vehicle_model:
                    MAX_ATTEMPTS = 10
                    SPAWN_SLEEP = 0.1

                    bp = self.world.get_blueprint_library().find(vehicle_model)
                    spawn_points = self.world.get_map().get_spawn_points()

                    temp_actor = None
                    for attempt in range(MAX_ATTEMPTS):
                        spawn_point = random.choice(spawn_points)
                        time.sleep(SPAWN_SLEEP)
                        temp_actor = self.world.try_spawn_actor(bp, spawn_point)
                        if temp_actor:
                            print(f"[INFO] Successfully spawned {vehicle_model} at attempt {attempt + 1}")
                            break
                        else:
                            print(
                                f"[RETRY] Failed to spawn {vehicle_model}, retrying... ({attempt + 1}/{MAX_ATTEMPTS})")

                    if temp_actor:
                        time.sleep(0.2)
                        extent = temp_actor.bounding_box.extent
                        length = extent.x * 2
                        width = extent.y * 2
                        self.model_size_map[vehicle_model] = (length, width)
                        print(
                            f"[INFO] Measured vehicle model {vehicle_model}: length = {length:.2f}, width = {width:.2f}")

                        time.sleep(0.1)
                        temp_actor.destroy()
                        time.sleep(0.1)
                    else:
                        print(
                            f"[WARN] Failed to spawn actor for model {vehicle_model} after {MAX_ATTEMPTS} attempts, using default size.")
            obs = Obstacle(
                x=vdata['location'][0] / 100,
                y=vdata['location'][1] / 100,
                vx=vdata['linear_velocity'][0] if 'linear_velocity' in vdata else 0.0,
                vy=vdata['linear_velocity'][1] if 'linear_velocity' in vdata else 0.0,
                hx=hx,
                hy=hy,
                length=length,
                width=width
            )

            if vid == self.hero_id:
                ego_data = obs
            else:
                surrounding_vehicles.append(obs)

        return ego_data, surrounding_vehicles


def parse_car_data(recorder_info):
    car_data = {}

    # Split the recorder info into blocks starting with 'Create'
    blocks = recorder_info.split("Create ")

    for block in blocks[1:]:  # Skip the first empty block
        lines = block.strip().splitlines()
        header = lines[0] if lines else ""

        # Extract vehicle ID and model from header
        header_match = re.match(r"(\d+): ([^\s]+)", header)
        if not header_match:
            continue

        vid = int(header_match.group(1))
        vehicle_model = header_match.group(2)

        # Try to find color and role_name within the block
        color_match = re.search(r"color = ([\d,]+)", block)
        role_match = re.search(r"role_name = ([^\n]+)", block)

        if not (color_match and role_match):
            continue  # Skip if any info is missing

        try:
            color = tuple(map(int, color_match.group(1).split(",")))
        except ValueError:
            continue  # Skip if color is malformed

        car_data[vid] = {
            'vehicle_model': vehicle_model,
            'color': color,
            'role_name': role_match.group(1).strip()
        }

    return car_data

def draw_lane_edges_continuous(carla_map, center_point, radius=50.0, resolution=1.0, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))

    center_x, center_y = center_point
    waypoints = carla_map.generate_waypoints(resolution)

    from collections import defaultdict
    lane_edge_left = defaultdict(list)
    lane_edge_right = defaultdict(list)
    lane_change_map = {}

    for wp in waypoints:
        x = wp.transform.location.x
        y = wp.transform.location.y
        if (x - center_x) ** 2 + (y - center_y) ** 2 > radius ** 2:
            continue

        yaw = np.deg2rad(wp.transform.rotation.yaw)
        dx = np.cos(yaw)
        dy = np.sin(yaw)
        half_w = wp.lane_width / 2.0

        left_x = x - dy * half_w
        left_y = y + dx * half_w
        right_x = x + dy * half_w
        right_y = y - dx * half_w

        lane_id = (wp.road_id, wp.lane_id)
        lane_edge_left[lane_id].append((left_x, left_y))
        lane_edge_right[lane_id].append((right_x, right_y))

        if lane_id not in lane_change_map:
            lane_change_map[lane_id] = wp.lane_change

    def get_color_for_lane_change(lane_change):
        if lane_change == carla.LaneChange.NONE:
            return 'black'
        elif lane_change == carla.LaneChange.Left:
            return 'blue'
        elif lane_change == carla.LaneChange.Right:
            return 'green'
        elif lane_change == carla.LaneChange.Both:
            return 'orange'
        else:
            return 'gray'

    for lane_id, points in lane_edge_left.items():
        if len(points) < 2:
            continue
        points = sorted(points, key=lambda p: p[1])
        xs, ys = zip(*points)
        color = get_color_for_lane_change(lane_change_map.get(lane_id, carla.LaneChange.NONE))
        ax.plot(xs, ys, color=color, linewidth=1)

    for lane_id, points in lane_edge_right.items():
        if len(points) < 2:
            continue
        points = sorted(points, key=lambda p: p[1])
        xs, ys = zip(*points)
        color = get_color_for_lane_change(lane_change_map.get(lane_id, carla.LaneChange.NONE))
        ax.plot(xs, ys, color=color, linewidth=1)

    return ax


if __name__ == '__main__':
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)

    recorder_path = "2025-04-22-19-58-17.log"
    frame_id = 560

    planner = HeroPlanner(client, recorder_path, frame_id)
    apf, trajectory, trajectory_velocity = planner.plan()
    trajectory = np.array(trajectory)

    fig, ax = plt.subplots(figsize=(10, 10))

    center_point = tuple(trajectory[len(trajectory) // 2])
    draw_lane_edges_continuous(planner.carla_map, center_point, radius=30, ax=ax)

    ax.plot(trajectory[:, 0], trajectory[:, 1], '-o', label='Ego Trajectory', markersize=1)

    for obs in apf.obstacles:
        angle_deg = np.degrees(np.arctan2(obs.hy, obs.hx))
        rect = patches.Rectangle(
            (-obs.length / 2, -obs.width / 2),
            obs.length, obs.width,
            linewidth=1, edgecolor='r', facecolor='r', alpha=0.5
        )
        transform = Affine2D().rotate_deg(angle_deg).translate(obs.x, obs.y) + ax.transData
        rect.set_transform(transform)
        ax.add_patch(rect)

    goal = apf.goal
    ax.plot(goal[0], goal[1], marker='*', color='red', markersize=5, label='Goal')

    ax.set_title("Ego Trajectory in Scenario")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.axis('equal')
    ax.legend()
    ax.grid(True)

    plt.show()
