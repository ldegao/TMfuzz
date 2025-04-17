import glob
import math
import os
import re
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
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
        self.client = client
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

        self.recorder_info = self.client.show_recorder_file_info(recorder_path, True)
        self.frame_id = frame_id
        self.car_data = parse_car_data(self.recorder_info)
        self.hero_id = next((vid for vid, vdata in self.car_data.items() if vdata.get('role_name') == 'hero'), None)
        self.ego_data, self.surrounding_vehicles = self.parse_frame_data(self.recorder_info, frame_id)

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

    def parse_frame_data(self, recorder_info, frame_id, default_length=4.5, default_width=2.0):
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

        ego_data = None
        surrounding_vehicles = []

        for vid, vdata in static_data.items():
            yaw_rad = math.radians(vdata['rotation'][2])
            hx, hy = math.cos(yaw_rad), math.sin(yaw_rad)

            obs = Obstacle(
                x=vdata['location'][0] / 100,
                y=vdata['location'][1] / 100,
                vx=vdata['linear_velocity'][0] if 'linear_velocity' in vdata else 0.0,
                vy=vdata['linear_velocity'][1] if 'linear_velocity' in vdata else 0.0,
                hx=hx,
                hy=hy,
                length=default_length,
                width=default_width
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


def draw_lane_lines(planner, road_length=80):
    for i in range(planner.num_lanes - 1):
        line_type = planner.lane_line_types[i]
        y_offset = (-planner.lane_width * planner.num_lanes / 2) + (i + 1) * planner.lane_width

        local_start = np.array([0, y_offset])
        local_end = np.array([road_length, y_offset])

        global_start = local_to_global(local_start, planner.road_origin, planner.lane_angle)
        global_end = local_to_global(local_end, planner.road_origin, planner.lane_angle)

        linestyle = '-' if line_type == 0 else '--'
        plt.plot(
            [global_start[0], global_end[0]],
            [global_start[1], global_end[1]],
            color='black', linestyle=linestyle, linewidth=1.5
        )

    # Draw left and right road boundaries (always solid)
    for side in [0, planner.num_lanes]:
        y_offset = (-planner.lane_width * planner.num_lanes / 2) + side * planner.lane_width
        local_start = np.array([0, y_offset])
        local_end = np.array([road_length, y_offset])
        global_start = local_to_global(local_start, planner.road_origin, planner.lane_angle)
        global_end = local_to_global(local_end, planner.road_origin, planner.lane_angle)
        plt.plot(
            [global_start[0], global_end[0]],
            [global_start[1], global_end[1]],
            color='black', linestyle='-', linewidth=2
        )


if __name__ == '__main__':

    client = carla.Client('localhost', 5000)
    client.set_timeout(10.0)

    recorder_path = "/home/carla/.config/Epic/CarlaUE4/Saved/test1.log"
    frame_id = 25

    planner = HeroPlanner(client, recorder_path, frame_id)
    apf, trajectory, trajectory_velocity = planner.plan()  # This returns the trajectory and plots internally

    # Plot trajectory
    trajectory = np.array(trajectory)
    plt.plot(trajectory[:, 0], trajectory[:, 1], '-o', label='Ego Trajectory', markersize=1)

    # Plot lane centers
    for idx, lane_center in enumerate(apf.lane_centers):
        plt.plot(lane_center[0], lane_center[1], 'x', markersize=8, label=f'Lane Center {idx}')

    # Plot obstacles
    for obs in apf.obstacles:
        angle_deg = np.degrees(np.arctan2(obs.hy, obs.hx))
        obs_rect = patches.Rectangle(
            (obs.x - obs.length / 2, obs.y - obs.width / 2),
            obs.length, obs.width,
            angle=float(angle_deg),
            color='r', alpha=0.5
        )
        plt.gca().add_patch(obs_rect)

    # Plot goal point
    goal = apf.goal
    plt.plot(goal[0], goal[1], marker='*', color='red', markersize=5, label='Goal')

    # Plot lane lines

    draw_lane_lines(apf)

    # Finalize plot
    plt.title("Ego Trajectory in Scenario")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()
