import pdb

import config
import constants as c
import math

config.set_carla_api_path()
import carla


class Actor:
    id = 0
    actor_type = 0
    nav_type = 0
    spawn_point = None
    dest_point = None
    speed = 0
    weight = 0
    instance = None
    is_player = False
    fresh = True

    def __init__(self, actor_type, nav_type, spawn_point, dest_point=None, id=0, speed=0):
        self.actor_type = actor_type
        self.nav_type = nav_type
        self.spawn_point = spawn_point
        self.dest_point = dest_point
        self.speed = speed
        self.id = id

    def safe_check(self, another_actor, distance,t):
        if self.instance is None:
            position1 = self.spawn_point.location
            roll_degrees = self.spawn_point.rotation.roll
            roll_rad = math.radians(roll_degrees)
            speed_x = self.speed * math.cos(roll_rad)
            speed_y = self.speed * math.sin(roll_rad)
            speed1 = carla.Vector3D(speed_x, speed_y, 0)
        else:
            position1 = self.instance.get_transform().location
            speed1 = self.instance.get_velocity()
        if another_actor.instance is None:
            position2 = another_actor.spawn_point.location
            roll_degrees = another_actor.spawn_point.rotation.roll
            roll_rad = math.radians(roll_degrees)
            speed_x = another_actor.speed * math.cos(roll_rad)
            speed_y = another_actor.speed * math.sin(roll_rad)
            speed2 = carla.Vector3D(speed_x, speed_y, 0)
        else:
            position2 = another_actor.instance.get_transform().location
            speed2 = another_actor.instance.get_velocity()

        return not check_rectangle_intersection(position1, position2, speed1, speed2, 5, distance)

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


def check_rectangle_intersection(position1, position2, speed1, speed2, t, lane_width):
    rect1_length = math.sqrt(speed1.x ** 2 + speed1.y ** 2) * t
    rect1_width = lane_width

    rect2_length = math.sqrt(speed2.x ** 2 + speed2.y ** 2) * t
    rect2_width = lane_width

    rect1_direction = math.atan2(speed1.y, speed1.x)
    rect2_direction = math.atan2(speed2.y, speed2.x)

    rect1_half_length = rect1_length / 2
    rect2_half_length = rect2_length / 2

    rect1_center = (position1.x + speed1.x * t / 2, position1.y + speed1.y * t / 2)
    rect2_center = (position2.x + speed2.x * t / 2, position2.y + speed2.y * t / 2)

    rect1_points = calculate_rectangle_points(rect1_center, rect1_half_length, rect1_width, rect1_direction)
    rect2_points = calculate_rectangle_points(rect2_center, rect2_half_length, rect2_width, rect2_direction)

    if rectangles_intersect(rect1_points, rect2_points):
        return True
    else:
        return False


def calculate_rectangle_points(center, half_length, width, direction):
    dx = math.cos(direction) * half_length
    dy = math.sin(direction) * half_length
    point1 = (center[0] + dx - width / 2 * math.sin(direction), center[1] + dy + width / 2 * math.cos(direction))
    point2 = (center[0] + dx + width / 2 * math.sin(direction), center[1] + dy - width / 2 * math.cos(direction))
    point3 = (center[0] - dx + width / 2 * math.sin(direction), center[1] - dy - width / 2 * math.cos(direction))
    point4 = (center[0] - dx - width / 2 * math.sin(direction), center[1] - dy + width / 2 * math.cos(direction))
    return [point1, point2, point3, point4]


def rectangles_intersect(rect1_points, rect2_points):
    for point in rect1_points:
        if is_point_inside_rectangle(point, rect2_points):
            return True
    for point in rect2_points:
        if is_point_inside_rectangle(point, rect1_points):
            return True
    return False


def is_point_inside_rectangle(point, rect_points):
    x, y = point
    for i in range(len(rect_points)):
        j = (i + 1) % len(rect_points)
        x1, y1 = rect_points[i]
        x2, y2 = rect_points[j]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            return True
    return False
