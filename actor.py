import config
import constants as c
import math
config.set_carla_api_path()
import carla
class Actor:
    actor_type = 0
    nav_type = 0
    spawn_point = None
    dest_point = None
    speed = 0
    weight = 0
    instance = None

    def __init__(self, actor_type, nav_type, spawn_point, dest_point, speed):
        self.actor_type = actor_type
        self.nav_type = nav_type
        self.spawn_point = spawn_point
        self.dest_point = dest_point
        self.speed = speed

    def safe_check(self, another_actor, distance):
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
        position2 = another_actor.instance.get_transform().location
        speed2 = self.instance.get_velocity()
        return not check_rectangle_intersection(position1, position2, speed1, speed2, c.HARD_ACC_THRES,distance)

    def set_instance(self, actor_vehicle):
        self.instance = actor_vehicle


def check_rectangle_intersection(position1, position2, speed1, speed2, braking_acceleration, lane_width):
    braking_distance1 = (speed1.x ** 2 + speed1.y ** 2) / (2 * braking_acceleration)
    braking_distance2 = (speed2.x ** 2 + speed2.y ** 2) / (2 * braking_acceleration)

    rect1_length = braking_distance1
    rect1_width = lane_width

    rect2_length = braking_distance2
    rect2_width = lane_width

    rect1_left = position1.x - rect1_length / 2
    rect1_right = position1.x + rect1_length / 2
    rect1_top = position1.y + rect1_width / 2
    rect1_bottom = position1.y - rect1_width / 2

    rect2_left = position2.x - rect2_length / 2
    rect2_right = position2.x + rect2_length / 2
    rect2_top = position2.y + rect2_width / 2
    rect2_bottom = position2.y - rect2_width / 2

    if (rect1_left <= rect2_right and rect1_right >= rect2_left and
            rect1_bottom <= rect2_top and rect1_top >= rect2_bottom):
        return True
    else:
        return False
