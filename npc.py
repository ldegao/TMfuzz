import copy
import pdb
import random

import config
import constants as c
import math
from shapely.geometry import Polygon

from utils import _on_collision, _on_invasion

config.set_carla_api_path()
import carla


class NPC:
    npc_id: int
    npc_type: int
    npc_bp = carla.ActorBlueprint
    spawn_point = carla.Waypoint
    speed: int
    spawn_stuck_frame: int
    instance: carla.Actor
    ego_loc: carla.Location
    fresh: bool
    death_time: int

    sensor_collision: carla.Actor
    sensor_lane_invasion: carla.Actor

    def __init__(self, npc_type, spawn_point, npc_id=0, speed=0, ego_loc=None,
                 npc_bp=None, spawn_stuck_frame=0):
        self.npc_type = npc_type
        self.spawn_point = spawn_point
        self.npc_id = npc_id
        self.speed = speed
        self.ego_loc = ego_loc
        self.npc_bp = npc_bp
        self.spawn_stuck_frame = spawn_stuck_frame
        self.fresh = True
        self.instance = None
        self.sensor_collision = None
        self.sensor_lane_invasion = None
        self.stuck_duration = 0
        self.death_time = -1

    def __deepcopy__(self, memo):
        npc_copy = NPC(
            copy.deepcopy(self.npc_type, memo),
            copy.deepcopy(self.spawn_point, memo),
            copy.deepcopy(self.npc_id, memo),
            copy.deepcopy(self.speed, memo),
            copy.deepcopy(self.ego_loc, memo),
            copy.deepcopy(self.npc_bp, memo),
            copy.deepcopy(self.spawn_stuck_frame, memo),
        )
        return npc_copy

    def safe_check(self, another_npc, width=1.5, adjust=2):
        """
        :param another_npc: another npc
        :param width: the width of the vehicle
        :param adjust: to adjust of the HARD_ACC_THRES
        :return: True if safe, False if not safe

        check if two vehicles are safe to each other, if not, return False
        """
        # calculate points of two vehicles safe rectangle
        points_list1 = calculate_safe_rectangle(self.get_position_now(), self.get_speed_now(),
                                                c.HARD_ACC_THRES / 3.6 / adjust,
                                                width)
        points_list2 = calculate_safe_rectangle(another_npc.get_position_now(), another_npc.get_speed_now(),
                                                c.HARD_ACC_THRES / 3.6 / adjust, width)
        self_rect = Polygon(points_list1)
        another_rect = Polygon(points_list2)
        if self_rect.intersects(another_rect):
            # print("not safe")
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

    def set_instance(self, npc_vehicle):
        self.instance = npc_vehicle

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

    def attach_collision(self, world, sensors, state):
        # Attach collision detector
        blueprint_library = world.get_blueprint_library()
        collision_bp = blueprint_library.find('sensor.other.collision')
        sensor_collision = world.spawn_actor(collision_bp, carla.Transform(),
                                           attach_to=self.instance)
        sensor_collision.listen(lambda event: _on_collision(event, state))
        sensors.append(sensor_collision)
        self.sensor_collision = sensor_collision

    def attach_lane_invasion(self, world, sensors, state):
        # Attach lane invasion detector
        blueprint_library = world.get_blueprint_library()
        lane_invasion_bp = blueprint_library.find('sensor.other.lane_invasion')
        sensor_lane_invasion = world.spawn_actor(lane_invasion_bp, carla.Transform(),
                                               attach_to=self.instance)
        sensor_lane_invasion.listen(lambda event: _on_invasion(event, state))
        sensors.append(sensor_lane_invasion)
        self.sensor_lane_invasion = sensor_lane_invasion

    @classmethod
    def get_npc_by_one(cls, npc, town_map, npc_id):
        # split a vehicle into two similar vehicles
        # return the new vehicle
        while True:
            npc_loc = npc.spawn_point.location
            x = 0
            y = 0
            while -2 <= x <= 2:
                x = random.uniform(-5, 5)
            while -2 <= y <= 2:
                y = random.uniform(-5, 5)
            new_speed = npc.speed + random.uniform(-5, 5)
            location = carla.Location(x=npc_loc.x + x, y=npc_loc.y + y, z=npc_loc.z)
            waypoint = town_map.get_waypoint(location, project_to_road=True,
                                             lane_type=carla.libcarla.LaneType.Driving)
            new_vehicle = NPC(npc.npc_type, waypoint.transform, npc_id,
                          new_speed,
                          npc.ego_loc, npc_bp=npc.npc_bp,
                          spawn_stuck_frame=npc.spawn_stuck_frame)
            new_vehicle.fresh = True
            if new_vehicle.safe_check(npc):
                print("split:", npc.npc_id, "to", npc.npc_id, npc_id)
                return new_vehicle

    def npc_cross(self, adc2):
        pass


class Pedestrian(NPC):
    def __init__(self, npc_id, spawn_point, speed, ego_loc, spawn_stuck_frame):
        super().__init__(npc_type=c.PEDESTRIAN, spawn_point=spawn_point,
                         npc_id=npc_id, speed=speed, ego_loc=ego_loc,
                         spawn_stuck_frame=spawn_stuck_frame)


class Vehicle(NPC):
    def __init__(self, npc_id, spawn_point, speed, ego_loc, spawn_stuck_frame):
        super().__init__(npc_type=c.VEHICLE, spawn_point=spawn_point,
                         npc_id=npc_id, speed=speed, ego_loc=ego_loc,
                         spawn_stuck_frame=spawn_stuck_frame)


def calculate_safe_rectangle(position, speed, acceleration, lane_width):
    """
    :param position: the position of the vehicle,
    :param speed: the speed of the vehicle,
    :param acceleration: the acceleration of the vehicle,
    :param lane_width: the width of the lane,
    :return: the four points of the rectangle

    calculate the safe rectangle points of vehicle in the next time step
    """
    t = math.sqrt(speed.x ** 2 + speed.y ** 2) / acceleration
    rect_length = acceleration * (t ** 2) / 2
    # add car length
    rect_length = rect_length + 10
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
