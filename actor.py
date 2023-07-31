import pdb
import random

import config
import constants as c
import math
from shapely.geometry import Point, LineString, Polygon

from utils import get_carla_transform

config.set_carla_api_path()
import carla


class Actor:
    actor_id = 0
    actor_type = 0
    nav_type = 0
    actor_bp = None
    spawn_point = None
    dest_point = None
    speed = 0
    weight = 0
    spawn_frame = 0
    max_weight_frame = 0
    spawn_stuck_frame = 0
    # Indicates the position relative to EGO when the vehicle weight is maximum
    max_weight_loc = -1
    max_weight_lane = -1
    player_lane_change = None
    instance = None
    is_player = False
    fresh = True
    ego_state = None
    is_spilt = False

    def __init__(self, actor_type, nav_type, spawn_point, dest_point=None, actor_id=0, speed=0, ego_loc=None,
                 ego_vel=None, spawn_frame=0, actor_bp=None, spawn_stuck_frame=0, agent=None):
        self.spawn_stuck_frame = spawn_stuck_frame
        self.event_list = []
        self.spawn_frame = spawn_frame
        self.actor_type = actor_type
        self.nav_type = nav_type
        self.spawn_point = spawn_point
        self.dest_point = dest_point
        self.speed = speed
        self.actor_id = actor_id
        self.ego_loc = ego_loc
        self.ego_vel = ego_vel
        self.actor_bp = actor_bp
        self.agent = agent

    def safe_check(self, another_actor, width=1.5, adjust=2):
        """
        :param another_actor: another actor
        :param width: the width of the vehicle
        :param adjust: to adjust of the HARD_ACC_THRES
        :return: True if safe, False if not safe

        check if two vehicles are safe to each other, if not, return False
        """
        self_is_ego = False
        another_is_ego = False
        if self.actor_id == -1:
            self_is_ego = True
        if another_actor.actor_id == -1:
            another_is_ego = True
        # calculate points of two vehicles safe rectangle
        points_list1 = calculate_safe_rectangle(self.get_position_now(), self.get_speed_now(),
                                                c.HARD_ACC_THRES / 3.6 / adjust,
                                                width, self_is_ego)
        points_list2 = calculate_safe_rectangle(another_actor.get_position_now(), another_actor.get_speed_now(),
                                                c.HARD_ACC_THRES / 3.6 / adjust, width, another_is_ego)
        self_rect = Polygon(points_list1)
        another_rect = Polygon(points_list2)
        if self_rect.intersects(another_rect):
            return False
        else:
            return True

    def get_position_now(self):
        if self.instance is None:
            position = self.spawn_point.location
        else:
            position = self.instance.get_transform().location
        return position

    def get_speed_now(self):
        if self.instance is None:
            roll_degrees = self.spawn_point.rotation.roll
            roll_rad = math.radians(roll_degrees)
            speed_x = self.speed * math.cos(roll_rad)
            speed_y = self.speed * math.sin(roll_rad)
            speed = carla.Vector3D(speed_x, speed_y, 0)
        else:
            speed = self.instance.get_velocity()
        return speed

    def set_instance(self, actor_vehicle):
        self.instance = actor_vehicle

    def get_waypoint(self, town_map):
        if self.instance is None:
            location = self.spawn_point.location
        else:
            location = self.instance.get_transform().location
        waypoint = town_map.get_waypoint(location, project_to_road=True,
                                         lane_type=carla.libcarla.LaneType.Driving)
        return waypoint

    def get_lane_width(self, town_map):
        return self.get_waypoint(town_map).lane_width

    def add_event(self, frame, event):
        self.event_list.append((frame, event))

    def splitting(self, town_map, actor_id):
        # split a car into two similar cars
        # return the new car
        while True:
            actor_loc = self.spawn_point.location
            x = 0
            y = 0
            while -2 <= x <= 2:
                x = random.uniform(-5, 5)
            while -2 <= y <= 2:
                y = random.uniform(-5, 5)
            new_speed = self.speed + random.uniform(-5, 5)
            location = carla.Location(x=actor_loc.x + x, y=actor_loc.y + y, z=actor_loc.z)
            waypoint = town_map.get_waypoint(location, project_to_road=True,
                                             lane_type=carla.libcarla.LaneType.Driving)
            new_car = Actor(self.actor_type, self.nav_type, waypoint.transform, self.dest_point, actor_id, new_speed,
                            self.ego_loc, self.ego_vel, self.spawn_frame, actor_bp=self.actor_bp,
                            spawn_stuck_frame=self.spawn_stuck_frame)
            new_car.fresh = True
            if new_car.safe_check(self):
                print("split:", self.actor_id, "to", self.actor_id, actor_id)
                return new_car


def calculate_safe_rectangle(position, speed, acceleration, lane_width, is_player=False):
    """
    :param position: the position of the vehicle
    :param speed: the speed of the vehicle
    :param acceleration: the acceleration of the vehicle
    :param lane_width: the width of the lane
    :param is_player: if the vehicle is ego vehicle
    :return: the four points of the rectangle

    calculate the safe rectangle points of vehicle in the next time step
    """
    t = math.sqrt(speed.x ** 2 + speed.y ** 2) / acceleration
    rect_length = acceleration * (t ** 2) / 2
    # add car length
    rect_length = rect_length + 2 * lane_width
    # # we don't want to cause ego's hard break,so we set the length to at least 10m
    # if is_player and rect_length < 10:
    #     rect_length = 10
    rect_width = 2 * lane_width
    rect_direction = math.atan2(speed.y, speed.x)
    rect_half_length = rect_length / 2
    rect_center = (position.x + speed.x * t / 2, position.y + speed.y * t / 2)
    rect_points = calculate_rectangle_points(rect_center, rect_half_length, rect_width, rect_direction)
    return rect_points


def calculate_rectangle_points(center, half_length, width, direction):
    """
    :param center: the center of the rectangle
    :param half_length: half of the length of the rectangle
    :param width: the width of the rectangle
    :param direction: the direction of the rectangle
    :return: the four points of the rectangle
    """
    dx = math.cos(direction) * half_length
    dy = math.sin(direction) * half_length
    point1 = (center[0] + dx - width / 2 * math.sin(direction),
              center[1] + dy + width / 2 * math.cos(direction))
    point2 = (center[0] + dx + width / 2 * math.sin(direction),
              center[1] + dy - width / 2 * math.cos(direction))
    point3 = (center[0] - dx + width / 2 * math.sin(direction),
              center[1] - dy - width / 2 * math.cos(direction))
    point4 = (center[0] - dx - width / 2 * math.sin(direction),
              center[1] - dy + width / 2 * math.cos(direction))
    return [point1, point2, point3, point4]
