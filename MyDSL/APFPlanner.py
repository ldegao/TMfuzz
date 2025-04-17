import random

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cvxpy as cp
from Obstacle import Obstacle


class APFPlanner:
    def __init__(self, start, goal, obstacles, ego_speed=10, safe_width=1.8):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.ego_speed = ego_speed
        self.safe_width = safe_width
        self.lane_angle = None
        self.road_origin = None

        self.L = 4.5
        self.V = 10.0
        self.T = self.L / self.V

        self.ETA_ATT_NORM = 0.02  # Reduced for realism
        self.ETA_REP_OB_NORM = 1
        self.ETA_REP_EDGE_NORM = 0.1
        self.D0_NORM = 3.0
        self.STEP_LENGTH_NORM = 0.1

        self.K_SOLID_NORM = 200.0
        self.K_DASHED_NORM = 20
        self.VIOLATION_EXPONENT = 2.0

        self.eta_att = self.ETA_ATT_NORM * (self.V ** 2) / self.L
        self.eta_rep_ob = self.ETA_REP_OB_NORM * (self.V ** 2) * self.L
        self.eta_rep_edge = self.ETA_REP_EDGE_NORM * (self.V ** 2) / self.L
        self.d0 = self.D0_NORM * self.L
        self.step_length = self.STEP_LENGTH_NORM * self.L

        self.k_solid = self.K_SOLID_NORM * (self.V ** 2) / (self.L ** 3)
        self.k_dashed = self.K_DASHED_NORM * (self.V ** 2) / (self.L ** 3)
        self.k_tangent = 50  # Reduced tangent force for stability

        self.violation_exponent = self.VIOLATION_EXPONENT

        self.mass = 0.1 * (self.V ** 2)
        self.max_speed = self.V

        self.n = 1
        self.num_iter = int(100 / self.STEP_LENGTH_NORM)
        self.corridor_d = None
        self.platform_length = None
        self.lane_width = None
        self.num_lanes = None
        self.lane_line_types = None
        self.intended_lane_index = None
        self.lane_centers = None

        self.trajectory = None
        self.smoothed_trajectory = None

        self.ego = Obstacle(
            x=self.start[0],
            y=self.start[1],
            vx=self.ego_speed,
            vy=0.0,
            hx=1.0,
            hy=0.0,
            length=self.L,
            width=self.safe_width
        )

    def compute_lane_centers(self, lane_angle, road_origin):
        """
        :param lane_angle: Rotation angle from global x-axis to lane direction (radians)
        :param road_origin: Reference point on road centerline in global coordinates [x, y]
        :return: List of lane center positions in global coordinates
        """
        total_width = self.num_lanes * self.lane_width
        offsets = [-total_width / 2 + self.lane_width / 2 + i * self.lane_width for i in range(self.num_lanes)]

        lane_centers_global = [
            local_to_global([0, offset], road_origin, lane_angle) for offset in offsets
        ]
        return lane_centers_global

    def set_lane_info(self, road_width=None, num_lanes=None, lane_line_types=None,
                      intended_lane_index=1, lane_angle=0.0, road_origin=None):
        """
        Set or update lane information considering lane orientation.

        :param road_width: Total width of the road
        :param num_lanes: Number of lanes
        :param lane_line_types: List of lane line types between lanes
        :param intended_lane_index: Target lane index for the vehicle
        :param lane_angle: Angle of the lane with respect to global x-axis (radians)
        :param road_origin: Reference point for the road centerline (global coordinates)
        """
        if road_origin is None:
            road_origin = [0, 0]
        self.lane_width = road_width / num_lanes
        self.num_lanes = num_lanes
        self.lane_angle = lane_angle
        self.road_origin = road_origin

        assert len(lane_line_types) == self.num_lanes - 1, "Mismatch between lane count and lane line types."
        self.lane_line_types = lane_line_types
        self.intended_lane_index = intended_lane_index

        # Compute and update lane centers considering lane angle and road origin
        self.lane_centers = self.compute_lane_centers(lane_angle, road_origin)

        self.corridor_d = self.lane_width
        self.platform_length = self.num_lanes * self.lane_width

    def plan(self):
        trajectory = []
        velocities = []
        current_position = np.array(self.start[:2], dtype=np.float64)
        delta_init = np.array(self.goal[:2]) - np.array(self.start[:2])
        unit_init = delta_init / (np.linalg.norm(delta_init) + 1e-6)
        current_velocity = unit_init * self.ego_speed
        dt = 0.025

        lane_angle, road_origin = self.lane_angle, self.road_origin
        M = self.mass
        max_force = 30.0

        for i in range(self.num_iter):
            if np.linalg.norm(current_position - self.goal[:2]) < 1:
                break

            trajectory.append(current_position.copy())
            velocities.append(np.round(np.linalg.norm(current_velocity), 2))

            # --- Local Frame Info ---
            local_pos = global_to_local(current_position, road_origin, lane_angle)
            local_goal = global_to_local(self.goal[:2], road_origin, lane_angle)
            delta_goal = local_goal - local_pos
            dist_goal = np.linalg.norm(delta_goal)
            unit_goal = delta_goal / (dist_goal + 1e-6)

            local_velocity = global_to_local(current_velocity, [0, 0], lane_angle)

            # --- PID Attractive Force (local frame) ---
            target_velocity_local = unit_goal * self.V
            velocity_error = target_velocity_local - local_velocity

            kp = 0.5
            kd = 1.0
            F_att = kp * velocity_error * M - kd * local_velocity

            # --- Determine danger mode ---
            t_safe = 1.6
            danger_mode = False
            for obs in self.obstacles:
                obs_vec = np.array([obs.x - current_position[0], obs.y - current_position[1]])
                dist = np.linalg.norm(obs_vec)
                if dist < self.d0:
                    if hasattr(obs, 'compute_ttc'):
                        ttc = obs.compute_ttc(self.ego)
                        if 0 < ttc < t_safe:
                            danger_mode = True
                            break

            # --- Road edge TTC ---
            road_half_width = self.num_lanes * self.lane_width / 2.0 - 1
            y = local_pos[1]
            vy = local_velocity[1]
            if abs(y) >= road_half_width - 0.5 and vy * y > 0:
                ttc_edge = (road_half_width - abs(y)) / (abs(vy) + 1e-6)
                if ttc_edge < t_safe:
                    # print(f"[TTC] Road edge TTC = {ttc_edge:.2f}")
                    danger_mode = True


            # --- Radial Repulsive Force Only if danger ---
            F_rep = np.zeros(2)
            if danger_mode:
                for obs in self.obstacles:
                    obs_local = global_to_local([obs.x, obs.y], road_origin, lane_angle)
                    delta = local_pos - obs_local
                    dist = np.linalg.norm(delta)
                    if 0 < dist < self.d0:
                        repulsion = smooth_clipped_repulsion(delta, dist, self.d0, self.eta_rep_ob, repulsion_max=20.0)
                        F_rep += repulsion

            # --- Violation Force Only if danger ---
            F_vio = np.zeros(2)
            if danger_mode:
                try:
                    F_vio_global = self.compute_violation_cost(current_position, lane_angle, road_origin)
                    F_vio = global_to_local(F_vio_global, [0, 0], lane_angle)
                except Exception as e:
                    print(f"[WARN] Violation force failed at step {i}: {e}")
                    F_vio = np.zeros(2)

            # --- Combine All Forces (local frame) ---
            F_total = F_att + F_rep + F_vio
            acc_local = F_total / M

            # --- Velocity Integration ---
            local_velocity += acc_local * dt

            # --- Goal Damping + Snap Logic ---
            goal_slow_radius = 5.0
            if dist_goal < goal_slow_radius:
                damping_ratio = dist_goal / goal_slow_radius
                local_velocity *= damping_ratio
                guide_velocity = 1.0 * unit_goal * (1 - damping_ratio)
                local_velocity += guide_velocity

            goal_snap_radius = 5.0
            if dist_goal < goal_snap_radius:
                blend_ratio = (goal_snap_radius - dist_goal) / goal_snap_radius
                local_velocity *= (1 - blend_ratio)
                guide_velocity = 1.5 * unit_goal * blend_ratio
                local_velocity += guide_velocity

            # --- Update Position & Velocity ---
            local_next = local_pos + local_velocity * dt
            current_position = local_to_global(local_next, road_origin, lane_angle)
            current_velocity = local_to_global(local_velocity, [0, 0], lane_angle)

            F_att_global = local_to_global(F_att, [0, 0], lane_angle)
            F_rep_global = local_to_global(F_rep, [0, 0], lane_angle)
            F_vio_global = local_to_global(F_vio, [0, 0], lane_angle)
            F_total_global = local_to_global(F_total, [0, 0], lane_angle)

            # print(f"Step {i}: Pos {current_position}, Vel {current_velocity},")
            # print(
            #     f"         F_att {F_att_global}, F_rep {F_rep_global}, F_vio {F_vio_global}, Total {F_total_global}, Acc {acc_local}")

            self.ego.x, self.ego.y = current_position

            for obs in self.obstacles:
                prev_pos = np.array([obs.x, obs.y])
                new_pos = prev_pos + np.array([obs.vx, obs.vy]) * dt
                obs.update(new_pos, prev_pos, dt=dt)

        trajectory.append(self.goal[:2])
        self.trajectory = np.array(trajectory)
        trajectory_velocity = np.array(velocities)
        trajectory_velocity = np.append(trajectory_velocity, 0.0)

        self.smoothed_trajectory = self.smooth_trajectory(self.trajectory)
        return self.smoothed_trajectory, trajectory_velocity

    def compute_violation_cost(self, position, lane_angle, road_origin):

        if self.lane_width is None or self.intended_lane_index is None:
            return np.array([0.0, 0.0])

        local_position = global_to_local(position, road_origin, lane_angle)
        local_lane_center = global_to_local(self.lane_centers[self.intended_lane_index], road_origin, lane_angle)
        y_offset = local_position[1] - local_lane_center[1]
        half_lane = self.lane_width / 2.0

        # --- Base force from centerline deviation ---
        if abs(y_offset) <= half_lane:
            base_force = 0.0
        else:
            violation_distance = abs(y_offset) - half_lane
            line_type = self.lane_line_types[
                self.intended_lane_index] if y_offset > 0 and self.intended_lane_index < self.num_lanes - 1 else 0
            k = self.k_solid if line_type == 0 else self.k_dashed
            penalty = np.tanh(violation_distance / (self.L + 1e-6))
            base_force = -np.sign(y_offset) * k * penalty

        # --- Smart Edge Force (Spring + Damping) ---
        road_half_width = self.num_lanes * self.lane_width / 2.0
        edge_force = 0.0

        # Check if the ego vehicle is beyond the road boundary
        if abs(local_position[1]) > road_half_width:
            edge_violation = abs(local_position[1]) - road_half_width
            # Parameters for edge response
            k_edge = self.k_solid * 1.5  # Spring constant
            c_edge = 2.0  # Damping constant

            # Spring force: Based on the lateral deviation from the road center
            F_spring = k_edge * edge_violation

            # Damping force: Based on lateral velocity (v_lat)
            local_velocity = global_to_local([self.ego.vx, self.ego.vy], [0.0, 0.0], lane_angle)
            v_lat = local_velocity[1]  # Lateral velocity

            # Damping force is proportional to the lateral velocity
            F_damping = c_edge * v_lat

            # Combined force: Spring + Damping
            edge_force = -np.sign(local_position[1]) * (F_spring + F_damping)
            edge_force = np.clip(edge_force, -100, 100)  # Clip the force to prevent excessive values

        # --- Handed Tangent Force (TTC-based) ---
        tangent_force = np.zeros(2)

        # Get the closest obstacles based on their distance
        closest_obstacles = [tup[0] for tup in sorted(
            [(obs, np.linalg.norm([obs.x - self.ego.x, obs.y - self.ego.y])) for obs in self.obstacles],
            key=lambda tup: tup[1])[:3]]

        # Calculate the unit vector of the vehicle's current direction
        vehicle_direction = np.array([self.ego.vx, self.ego.vy])
        vehicle_speed = np.linalg.norm(vehicle_direction)
        if vehicle_speed > 1e-6:
            vehicle_direction /= vehicle_speed
        else:
            vehicle_direction = np.array([1.0, 0.0])  # Default to some direction if no velocity

        for obs in closest_obstacles:
            obs_vec = np.array([obs.x - self.ego.x, obs.y - self.ego.y])
            dist = np.linalg.norm(obs_vec)
            if dist < 1e-2:
                continue  # Skip if the obstacle is too close (to avoid singularity)

            # Unit vector from the vehicle to the obstacle
            unit_vec = obs_vec / dist

            # Calculate the leftward direction (perpendicular to the vehicle's motion)
            left_dir = np.array([-vehicle_direction[1], vehicle_direction[0]])

            # Compute the TTC for this obstacle
            ttc = obs.compute_ttc(self.ego)
            if ttc <= 0 or ttc > 5.0:
                continue  # Skip if no valid TTC

            # TTC-based urgency: the closer the TTC, the stronger the response
            urgency = np.tanh((1.6 - ttc) / 1.2)  # This can be adjusted for sharper reactions
            urgency = max(0.0, urgency)  # Only apply force if TTC is small enough

            # Calculate the angle between the vehicle's direction and the obstacle's direction
            angle_cos = np.dot(vehicle_direction, unit_vec)
            angle_sin = np.cross(vehicle_direction, unit_vec)  # Handedness (sign of the angle)

            # If the angle is negative, the obstacle is to the right of the vehicle's motion
            if angle_sin > 0:
                # Obstacle is to the left, so apply force to the right
                force_direction = -left_dir
            else:
                # Obstacle is to the right, so apply force to the left
                force_direction = left_dir

            # Apply tangent force in the chosen direction, scaled by urgency
            strength = self.k_tangent * urgency
            tangent_force += force_direction * strength

        # --- Time-To-Collision (TTC) Adjustment ---
        obs_distances = [(obs, np.linalg.norm([obs.x - self.ego.x, obs.y - self.ego.y])) for obs in self.obstacles]
        obs_distances.sort(key=lambda tup: tup[1])
        closest_obstacles = [tup[0] for tup in obs_distances[:3]]
        ttc_values = [obs.compute_ttc(self.ego) for obs in closest_obstacles]
        min_ttc = min(ttc_values) if ttc_values else float('inf')

        t_safe = 1.6
        ttc_std = 2.5 if self.ego_speed < 10 else (3.0 if self.ego_speed < 15 else 3.5)

        if 0 < min_ttc < t_safe:
            R = np.tanh(ttc_std / (min_ttc + 1e-3))
            ttc_penalty = (t_safe - min_ttc)
        else:
            R = 1.0
            ttc_penalty = 0.0

        # --- Combine All Lateral Forces ---
        force_y = (base_force + edge_force + tangent_force[1]) * R + ttc_penalty

        global_force = local_to_global([0.0, force_y], [0.0, 0.0], lane_angle)

        # print(f"[DEBUG] Debug info at step {debug_counter}:")
        # print(
        #     f"[DEBUG] base_force = {base_force:.3f}, edge_force = {edge_force:.3f}, tangent_force = {tangent_force}")
        # print(f"[DEBUG] Final global force: {global_force}")

        return global_force

    def smooth_trajectory(self, trajectory, weight_smooth=1.0, weight_jerk=0.1, weight_fidelity=1.0,
                          weight_direction=10.0, weight_curvature=10.0):
        if trajectory is None or len(trajectory) < 3:
            raise ValueError("Trajectory must have at least 3 points for smoothing.")

        N = len(trajectory)
        x = cp.Variable(N)
        y = cp.Variable(N)

        cost = 0
        constraints = [
            x[0] == trajectory[0, 0], y[0] == trajectory[0, 1],
            x[-1] == trajectory[-1, 0], y[-1] == trajectory[-1, 1]
        ]

        for i in range(1, N - 1):
            cost += weight_smooth * (
                    cp.square(x[i] - (x[i - 1] + x[i + 1]) / 2) +
                    cp.square(y[i] - (y[i - 1] + y[i + 1]) / 2)
            )

            if i < N - 2:
                cost += weight_jerk * (
                        cp.square(x[i - 1] - 3 * x[i] + 3 * x[i + 1] - x[i + 2]) +
                        cp.square(y[i - 1] - 3 * y[i] + 3 * y[i + 1] - y[i + 2])
                )

            dx_prev = x[i] - x[i - 1]
            dy_prev = y[i] - y[i - 1]
            dx_next = x[i + 1] - x[i]
            dy_next = y[i + 1] - y[i]

            cost += weight_direction * (
                    cp.square(dx_next - dx_prev) + cp.square(dy_next - dy_prev)
            )

            cost += weight_curvature * (
                    cp.square(x[i - 1] - 2 * x[i] + x[i + 1]) +
                    cp.square(y[i - 1] - 2 * y[i] + y[i + 1])
            )

        for i in range(N):
            cost += weight_fidelity * (
                    cp.square(x[i] - trajectory[i, 0]) +
                    cp.square(y[i] - trajectory[i, 1])
            )

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)

        if x.value is None or y.value is None:
            raise ValueError("Trajectory smoothing failed. Solver did not find a solution.")

        smoothed_traj = np.vstack((x.value, y.value)).T
        return smoothed_traj


def create_regular_obstacles(lane_centers, lane_angle=0.0):
    """
    Create obstacles arranged regularly along each lane,
    considering lane orientation.

    Parameters:
        lane_centers: List of lane center coordinates in global (x,y).
        lane_angle: Angle of the lane w.r.t. global x-axis (radians).

    Returns:
        A list of Obstacle instances.
    """
    obstacles = []
    num_obstacles = 5  # Number of obstacles per lane
    x_start = 15  # Initial x-coordinate along the lane direction
    x_interval = 15  # Spacing between obstacles along the lane direction
    length = 4.5
    width = 2.0

    # Create obstacles on each lane center line
    for lane_center in lane_centers:
        for i in range(num_obstacles):
            p = random.random()
            if p > 0.5:
                continue
            local_position = np.array([x_start + i * x_interval, 0])  # Position along lane
            global_position = local_to_global(local_position, lane_center, lane_angle)

            obs = Obstacle(
                x=global_position[0],
                y=global_position[1],
                vx=0, vy=0,
                hx=np.cos(lane_angle), hy=np.sin(lane_angle),
                length=length, width=width
            )
            obstacles.append(obs)

    return obstacles


def smooth_clipped_repulsion(delta_vec, dist, d0, eta, repulsion_max=20.0):
    """
    Smooth and bounded repulsive force.
    """
    if dist <= 0 or dist >= d0:
        return np.zeros_like(delta_vec)

    scale = (d0 - dist) / d0
    strength = eta * (scale ** 2)
    repulsion = strength * (delta_vec / (dist + 1e-6))

    # Apply maximum limit
    repulsion_norm = np.linalg.norm(repulsion)
    if repulsion_norm > repulsion_max:
        repulsion = repulsion / repulsion_norm * repulsion_max

    return repulsion


def rotate_vector(vec, angle):
    """
    Rotate a 2D vector by an angle (radians), without translation.
    """
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    rot_matrix = np.array([[cos_angle, -sin_angle],
                           [sin_angle, cos_angle]])
    return rot_matrix @ vec


def global_to_local(position, origin, lane_angle):
    """
    :param position: Global coordinates [x, y]
    :param origin: Origin of the local coordinate system in global coordinates [x, y]
    :param lane_angle: Rotation angle from global x-axis to lane direction (radians)
    :return: Local coordinates [x', y']
    """
    translation = np.array(position) - np.array(origin)
    cos_angle = np.cos(-lane_angle)
    sin_angle = np.sin(-lane_angle)
    rotation_matrix = np.array([[cos_angle, -sin_angle],
                                [sin_angle, cos_angle]])
    local_pos = rotation_matrix @ translation
    return local_pos


def local_to_global(local_position, origin, lane_angle):
    """
    :param local_position: Local coordinates [x', y']
    :param origin: Origin of the local coordinate system in global coordinates [x, y]
    :param lane_angle: Rotation angle from global x-axis to lane direction (radians)
    :return: Global coordinates [x, y]
    """
    cos_angle = np.cos(lane_angle)
    sin_angle = np.sin(lane_angle)
    rotation_matrix = np.array([[cos_angle, -sin_angle],
                                [sin_angle, cos_angle]])
    global_pos = rotation_matrix @ np.array(local_position) + np.array(origin)
    return global_pos


def test_scenario(start, goal, lane_angle, road_origin, scenario_name):
    planner = APFPlanner(
        start=start, goal=goal, obstacles=[],
        ego_speed=ego_speed, safe_width=safe_width
    )

    planner.set_lane_info(
        road_width=road_width,
        num_lanes=num_lanes,
        lane_line_types=lane_line_types,
        intended_lane_index=intended_lane_index,
        lane_angle=lane_angle,
        road_origin=road_origin
    )

    obstacle_lane_indices = [0, 1]
    obstacle_lane_centers = [planner.lane_centers[i] for i in obstacle_lane_indices]
    obstacles = create_regular_obstacles(
        lane_centers=obstacle_lane_centers,
        lane_angle=lane_angle,
    )
    planner.obstacles = obstacles

    trajectory, trajectory_velocity = planner.plan()

    trajectory = np.array(trajectory)
    plt.plot(trajectory[:, 0], trajectory[:, 1], '-o', label='Ego Trajectory', markersize=1)

    for idx, lane_center in enumerate(planner.lane_centers):
        plt.plot(lane_center[0], lane_center[1], 'x', markersize=8, label=f'Lane Center {idx}')

    for obs in obstacles:
        angle_deg = np.degrees(np.arctan2(obs.hy, obs.hx))

        obs_rect = patches.Rectangle(
            (obs.x - obs.length / 2, obs.y - obs.width / 2),
            obs.length, obs.width,
            angle=float(angle_deg),
            color='r', alpha=0.5
        )
        plt.gca().add_patch(obs_rect)
    # Plot goal point
    plt.plot(goal[0], goal[1], marker='*', color='red', markersize=5, label='Goal')
    plt.title(f"{scenario_name}")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    draw_lane_lines(planner)
    plt.show()


def draw_lane_lines(planner, road_length=100):
    total_width = planner.num_lanes * planner.lane_width
    left_y = -total_width / 2
    left_start = local_to_global([0, left_y], planner.road_origin, planner.lane_angle)
    left_end = local_to_global([road_length, left_y], planner.road_origin, planner.lane_angle)
    plt.plot([left_start[0], left_end[0]], [left_start[1], left_end[1]],
             color='black', linewidth=2.0, linestyle='-')

    for i in range(planner.num_lanes - 1):
        line_type = planner.lane_line_types[i]
        y_offset = (-planner.lane_width * planner.num_lanes / 2) + (i + 1) * planner.lane_width

        local_start = np.array([0, y_offset])
        local_end = np.array([road_length, y_offset])

        global_start = local_to_global(local_start, planner.road_origin, planner.lane_angle)
        global_end = local_to_global(local_end, planner.road_origin, planner.lane_angle)

        if line_type == 0:
            plt.plot(
                [global_start[0], global_end[0]],
                [global_start[1], global_end[1]],
                color='black', linewidth=1.5, linestyle='-'
            )
        elif line_type == 1:
            plt.plot(
                [global_start[0], global_end[0]],
                [global_start[1], global_end[1]],
                color='black', linewidth=1.0, linestyle='--'
            )
    right_y = total_width / 2
    right_start = local_to_global([0, right_y], planner.road_origin, planner.lane_angle)
    right_end = local_to_global([road_length, right_y], planner.road_origin, planner.lane_angle)
    plt.plot([right_start[0], right_end[0]], [right_start[1], right_end[1]],
             color='black', linewidth=2.0, linestyle='-')


if __name__ == "__main__":
    road_width = 15
    num_lanes = 3
    lane_line_types = [0, 1]
    intended_lane_index = 1
    ego_speed = 10
    safe_width = 2.0

    start_parallel = [0, 0, ego_speed, 0]
    goal_parallel = [100, 0, 0, 0]
    lane_angle_parallel = 0
    road_origin_parallel = [0, 0]

    start_rotated = [0, 0, ego_speed, 0]
    lane_angle_rotated = np.deg2rad(30)
    road_origin_rotated = [0, 0]
    goal_local_rotated = [100, 0]
    goal_rotated = local_to_global(goal_local_rotated, road_origin_rotated, lane_angle_rotated)

    test_scenario(start_parallel, goal_parallel, lane_angle_parallel, road_origin_parallel, "Parallel Scenario")
    test_scenario(start_rotated, goal_rotated, lane_angle_rotated, road_origin_rotated, "Rotated Scenario")
