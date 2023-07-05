import pdb

import config
import constants as c
import math

config.set_carla_api_path()
import carla


class Actor:
    actor_id = 0
    actor_type = 0
    nav_type = 0
    spawn_point = None
    dest_point = None
    speed = 0
    weight = 0
    spawn_frame = 0
    max_weight_frame = 0
    # Indicates the position relative to EGO when the vehicle weight is maximum
    max_weight_loc = -1
    max_weight_lane = -1
    instance = None
    is_player = False
    fresh = True
    ego_state = None

    def __init__(self, actor_type, nav_type, spawn_point, dest_point=None, actor_id=0, speed=0, ego_loc=None,
                 ego_vel=None, spawn_frame=0):
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

    def safe_check(self, another_actor, width=1.5):
        """
        :param another_actor: another actor
        :param width: the width of the vehicle
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
                                                c.HARD_ACC_THRES / 3.6/1.5,
                                                width, self_is_ego)
        points_list2 = calculate_safe_rectangle(another_actor.get_position_now(), another_actor.get_speed_now(),
                                                c.HARD_ACC_THRES / 3.6/1.5, width, another_is_ego)
        # calculate lines of two vehicles safe rectangle
        lines_list1 = []
        lines_list2 = []
        for i in range(4):
            lines_list1.append((points_list1[i], points_list1[(i + 1) % 4]))
            lines_list2.append((points_list2[i], points_list2[(i + 1) % 4]))
        # check if there is line cross
        for line1 in lines_list1:
            for line2 in lines_list2:
                if is_line_cross(line1, line2):
                    return False
        # check if there is point in rectangle
        for point in points_list1:
            if is_point_inside_rectangle(point, points_list2):
                return False
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
    # if rect_length<lane_width,change the length to 2*lane_width
    if rect_length < lane_width:
        rect_length = 2 * lane_width
    # we don't want to cause ego's hard break,so we set the length to at least 10m
    if is_player and rect_length < 10:
        rect_length = 10
    rect_width = lane_width
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
    point1 = carla.Vector2D(center[0] + dx - width / 2 * math.sin(direction),
                            center[1] + dy + width / 2 * math.cos(direction))
    point2 = carla.Vector2D(center[0] + dx + width / 2 * math.sin(direction),
                            center[1] + dy - width / 2 * math.cos(direction))
    point3 = carla.Vector2D(center[0] - dx + width / 2 * math.sin(direction),
                            center[1] - dy - width / 2 * math.cos(direction))
    point4 = carla.Vector2D(center[0] - dx - width / 2 * math.sin(direction),
                            center[1] - dy + width / 2 * math.cos(direction))
    return [point1, point2, point3, point4]


def is_point_inside_rectangle(point, rect_points):
    """
    :param point: the point to be checked
    :param rect_points: the four points of the rectangle
    """
    x, y = point.x, point.y
    x1, y1 = rect_points[0].x, rect_points[0].y
    x2, y2 = rect_points[1].x, rect_points[1].y
    x3, y3 = rect_points[2].x, rect_points[2].y
    x4, y4 = rect_points[3].x, rect_points[3].y
    if (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1) >= 0 and (x3 - x2) * (y - y2) - (y3 - y2) * (x - x2) >= 0 and \
            (x4 - x3) * (y - y3) - (y4 - y3) * (x - x3) >= 0 and (x1 - x4) * (y - y4) - (y1 - y4) * (x - x4) >= 0:
        return True
    else:
        return False


def is_point_on_line(p1, p2, p3):
    """
    :param p1: the first point of the line
    :param p2: the second point of the line
    :param p3: the point to be checked
    """
    if min(p1.x, p2.x) <= p3.x <= max(p1.x, p2.x) and min(p1.y, p2.y) <= p3.y <= max(p1.y, p2.y):
        return True
    else:
        return False


def is_line_cross(line1, line2):
    """
    :param line1: the first line
    :param line2: the second line
    :return: True if cross, False if not cross
    """
    p1, p2 = line1
    p3, p4 = line2
    if max(p1.x, p2.x) < min(p3.x, p4.x):
        return False
    if max(p1.y, p2.y) < min(p3.y, p4.y):
        return False
    if max(p3.x, p4.x) < min(p1.x, p2.x):
        return False
    if max(p3.y, p4.y) < min(p1.y, p2.y):
        return False
    if is_point_on_line(p1, p2, p3) or is_point_on_line(p1, p2, p4) or is_point_on_line(p3, p4, p1) or is_point_on_line(
            p3, p4, p2):
        return True
    return False
