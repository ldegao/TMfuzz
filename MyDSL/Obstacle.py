import pandas as pd
import numpy as np
from shapely.geometry import Polygon

from MyDSL.TTC import TTC


class Obstacle:
    def __init__(self, x, y, vx, vy, hx, hy, length, width):
        """
        Initialize an obstacle object.
        Parameters:
          x, y: Centroid (position) of the obstacle.
          vx, vy: Velocity components.
          hx, hy: Heading direction vector components.
          length, width: Dimensions of the obstacle.
        """
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.hx = hx
        self.hy = hy
        self.length = length
        self.width = width

    def update(self, position, prev_position, dt=1.0):
        if dt <= 0:
            dt = 1e-3  # avoid division by zero
        try:
            self.vx = (position[0] - prev_position[0]) / dt
            self.vy = (position[1] - prev_position[1]) / dt
            if not np.isfinite(self.vx) or not np.isfinite(self.vy):
                raise ValueError("Velocity invalid")
        except:
            self.vx, self.vy = 0.0, 0.0
        self.x, self.y = position
        return self.x, self.y, self.vx, self.vy

    def compute_ttc(self, other):
        """
        Compute Time-To-Collision (TTC) between this obstacle and another using an external TTC() function.
        Parameters:
            other: Another obstacle instance representing the ego vehicle.
        Returns:
            TTC value (float)
        """
        sample = pd.DataFrame([{
            'x_i': other.x, 'y_i': other.y,
            'vx_i': other.vx, 'vy_i': other.vy,
            'hx_i': other.hx, 'hy_i': other.hy,
            'length_i': other.length, 'width_i': other.width,
            'x_j': self.x, 'y_j': self.y,
            'vx_j': self.vx, 'vy_j': self.vy,
            'hx_j': self.hx, 'hy_j': self.hy,
            'length_j': self.length, 'width_j': self.width
        }])
        ttc_value = TTC(sample, toreturn='values')
        return float(ttc_value[0])

    def is_obstacle_in_lane(self, ego, t_safe=1.6):
        """
        Check if the obstacle will occupy the lane within a given time window (based on TTC).
        Parameters:
            ego: The ego vehicle object.
            t_safe: The time threshold for considering collision (e.g., 1.6 seconds).
        Returns:
            bool: True if the obstacle is occupying the lane, False otherwise.
        """
        # Calculate TTC between obstacle and ego vehicle
        ttc = self.compute_ttc(ego)

        # If TTC is less than safe threshold, it means the obstacle is in the lane
        if ttc < t_safe:
            return True
        else:
            return False
    def compute_trajectory_overlap_area(self, other, time_interval, time_step=0.1):
        """
        Compute the total overlap area between the trajectories of this obstacle and another over a time interval.
        Parameters:
            other: Another obstacle instance.
            time_interval: Total time to simulate (seconds).
            time_step: Time step for simulation (seconds).
        Returns:
            Total overlap area (float).
        """
        total_overlap_area = 0.0
        num_steps = int(time_interval / time_step)

        for step in range(num_steps):
            t = step * time_step
            ego_x_t = other.x + other.vx * t
            ego_y_t = other.y + other.vy * t
            obs_x_t = self.x + self.vx * t
            obs_y_t = self.y + self.vy * t

            ego_rect = self.get_vehicle_rectangle(ego_x_t, ego_y_t, other.hx, other.hy, other.length, other.width)
            obs_rect = self.get_vehicle_rectangle(obs_x_t, obs_y_t, self.hx, self.hy, self.length, self.width)

            overlap_area = ego_rect.intersection(obs_rect).area
            total_overlap_area += overlap_area * time_step

        return total_overlap_area

    def compute_trajectory_overlap_time(self, other, time_interval, time_step=0.1):
        """
        Compute the total overlap time between the trajectories of this obstacle and another over a time interval.
        Parameters:
            other: Another obstacle instance.
            time_interval: Total time to simulate (seconds).
            time_step: Time step for simulation (seconds).
        Returns:
            Total overlap time (float).
        """
        total_overlap_time = 0.0
        num_steps = int(time_interval / time_step)

        for step in range(num_steps):
            t = step * time_step
            ego_x_t = other.x + other.vx * t
            ego_y_t = other.y + other.vy * t
            obs_x_t = self.x + self.vx * t
            obs_y_t = self.y + self.vy * t

            ego_rect = self.get_vehicle_rectangle(ego_x_t, ego_y_t, other.hx, other.hy, other.length, other.width)
            obs_rect = self.get_vehicle_rectangle(obs_x_t, obs_y_t, self.hx, self.hy, self.length, self.width)

            if ego_rect.intersects(obs_rect):
                total_overlap_time += time_step

        return total_overlap_time

    @staticmethod
    def get_vehicle_rectangle(x, y, hx, hy, length, width):
        """
        Get the rectangular polygon representing the vehicle's position and orientation.
        Parameters:
            x, y: Center position of the vehicle.
            hx, hy: Heading direction vector components.
            length, width: Dimensions of the vehicle.
        Returns:
            Shapely Polygon representing the vehicle's rectangle.
        """
        angle = np.arctan2(hy, hx)
        half_length = length / 2
        half_width = width / 2
        corners = [
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width)
        ]
        rotated_corners = []
        for dx, dy in corners:
            rot_x = dx * np.cos(angle) - dy * np.sin(angle)
            rot_y = dx * np.sin(angle) + dy * np.cos(angle)
            rotated_corners.append((x + rot_x, y + rot_y))
        return Polygon(rotated_corners)

    def to_dict(self):
        """
        Convert obstacle data to a dictionary.
        """
        return {
            'x': self.x,
            'y': self.y,
            'vx': self.vx,
            'vy': self.vy,
            'hx': self.hx,
            'hy': self.hy,
            'length': self.length,
            'width': self.width
        }

