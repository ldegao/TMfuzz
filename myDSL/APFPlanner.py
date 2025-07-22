if __name__ == "__main__" and __package__ is None:
    import sys
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    __package__ = "myDSL"
import random
from math import sqrt

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cvxpy as cp
import os
from math import sqrt, cos, sin, atan2
from myDSL.Obstacle import Obstacle
import pdb


class APFPlanner:
    def __init__(self, start, goal, surrounding_vehicles, centerline_points, centerline_headings, 
                 ego_speed=10, safe_width=1.8):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = [obs.copy() for obs in surrounding_vehicles]
        self.ego_speed = ego_speed
        self.safe_width = safe_width
        self.lane_angle = None
        self.road_origin = None
        self.centerline_points = centerline_points
        self.centerline_headings = centerline_headings

        self.L = 4.5
        self.V = 10.0
        self.T = self.L / self.V

        self.ETA_ATT_NORM = 0.02  # Reduced for realism
        self.ETA_REP_OB_NORM = 5 # represent obstacles（还原）
        self.ETA_REP_EDGE_NORM = 0.1
        self.D0_NORM = 1
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

        self.MAX_ACC = 3.0
        self.mass = 0.1 * (self.V ** 2)
        self.max_speed = self.V

        self.n = 1
        self.num_iter = 1000
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

    def find_nearest_centerline_index(self, point):
        # 必须有centerline数据，否则报错
        # 优化：使用numpy的向量化操作
        point_array = np.array(point)
        centerline_array = np.array(self.centerline_points)
        dists = np.linalg.norm(centerline_array - point_array, axis=1)
        return int(np.argmin(dists))

    # 删除：不再需要局部/全局坐标变换，因为：
    # 1. 所有计算现在都在全局坐标系中进行
    # 2. 弯道场景中动态局部坐标系过于复杂且不必要
    # 3. 基于lane_centers的距离计算更直观、统一

    def set_lane_info(self, road_width=None, num_lanes=None, lane_line_types=None,
                     intended_lane_index=1):
        """
        设置车道信息
        """
        if road_width is not None:
            self.road_width = road_width
        if num_lanes is not None:
            self.num_lanes = num_lanes
        if lane_line_types is not None:
            self.lane_line_types = lane_line_types
        if intended_lane_index is not None:
            self.intended_lane_index = intended_lane_index

        # 计算车道宽度
        if hasattr(self, 'road_width') and hasattr(self, 'num_lanes'):
            self.lane_width = self.road_width / self.num_lanes

        self.corridor_d = self.lane_width

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

        print('[DEBUG] plan: num_iter =', self.num_iter)
        for i in range(self.num_iter):
            # 关键异常监控：lateral_offset/lateral_speed
            idx = self.find_nearest_centerline_index(current_position)
            road_center = np.array(self.centerline_points[idx])
            road_heading = self.centerline_headings[idx]
            road_lateral = np.array([-np.sin(road_heading), np.cos(road_heading)])
            lateral_offset = np.dot(road_center - current_position, road_lateral)
            lateral_speed = np.dot(current_velocity, road_lateral)
            if abs(lateral_offset) > 20 or abs(lateral_speed) > 20:
                print(f"[PDB_TRIGGER] i={i}, lateral_offset={lateral_offset:.3f}, lateral_speed={lateral_speed:.3f}, current_velocity=({current_velocity[0]:.3f}, {current_velocity[1]:.3f})")
                pdb.set_trace()
            current_pos = current_position
            goal_pos = np.array(self.goal[:2])
            delta_goal = goal_pos - current_pos
            dist_goal = np.linalg.norm(delta_goal)
            unit_goal = delta_goal / (dist_goal + 1e-6)
            velocity = current_velocity

            # --- PID Attractive Force ---
            target_velocity = unit_goal * self.V
            velocity_error = target_velocity - velocity
            kp = 0.5
            kd = 1.0
            F_att = kp * velocity_error * M - kd * velocity
            # 新增吸引力调试
            print(f"[DEBUG_ATT] i={i}, delta_goal={delta_goal}, unit_goal={unit_goal}, target_velocity={target_velocity}, velocity_error={velocity_error}, F_att={F_att}")

            # --- Danger Mode Detection & TTC 合并遍历 ---
            ttc_danger_mode = False
            dist_danger_mode = False
            min_ttc = float('inf')
            for obs in self.obstacles:
                obs_pos = np.array([obs.x, obs.y])
                obs_yaw = np.arctan2(obs.hy, obs.hx)
                dist = distance_to_rectangle_edge(
                    current_pos,
                    obs_pos,
                    obs_yaw,
                    obs.length,
                    obs.width
                )
                if dist < self.d0:
                    dist_danger_mode = True
                if dist <= self.d0 and hasattr(obs, 'compute_ttc'):
                    ttc = obs.compute_ttc(self.ego)
                    if ttc > 0 and ttc < min_ttc:
                        min_ttc = ttc
                    if 0 < ttc < 1.5:
                        ttc_danger_mode = True
            # --- Radial Repulsive Force Only if dist_danger_mode ---
            F_rep = np.zeros(2)
            if dist_danger_mode:
                for obs in self.obstacles:
                    obs_pos = np.array([obs.x, obs.y])
                    delta = current_pos - obs_pos
                    obs_yaw = np.arctan2(obs.hy, obs.hx)
                    dist = distance_to_rectangle_edge(
                        current_pos,
                        obs_pos,
                        obs_yaw,
                        obs.length,
                        obs.width
                    )
                    if dist > self.d0:
                        continue
                    edge_dist = dist - sqrt(obs.length ** 2 + obs.width ** 2) / 2
                    repulsion = smooth_clipped_repulsion(delta, edge_dist, self.d0, self.eta_rep_ob,
                                                         repulsion_max=100.0)
                    F_rep += repulsion
            else:
                min_dist = float('inf')
                for obs in self.obstacles:
                    obs_pos = np.array([obs.x, obs.y])
                    obs_yaw = np.arctan2(obs.hy, obs.hx)
                    dist = distance_to_rectangle_edge(
                        current_pos,
                        obs_pos,
                        obs_yaw,
                        obs.length,
                        obs.width
                    )
                    if dist < min_dist:
                        min_dist = dist
            # --- Violation Force ---
            F_violation = self.compute_violation_force_global(current_pos, velocity, ttc_danger_mode)

            # --- Combine All Forces ---
            F_total = F_att + F_rep + F_violation
            acceleration = F_total / M
            acceleration = np.clip(acceleration, -self.MAX_ACC, self.MAX_ACC)

            # --- 详细的力分析调试输出 ---
            if i % 50 == 0:
                print(f'[FORCE_DEBUG] Step {i}:')
                print(f'  位置: ({current_pos[0]:.2f}, {current_pos[1]:.2f})')
                print(f'  速度: ({velocity[0]:.2f}, {velocity[1]:.2f}) m/s')
                print(f'  目标: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})')
                print(f'  距离目标: {dist_goal:.2f} m')
                print(f'  吸引力 F_att:      x={F_att[0]:.3f} N, y={F_att[1]:.3f} N, |F_att|={np.linalg.norm(F_att):.3f} N')
                print(f'  斥力   F_rep:      x={F_rep[0]:.3f} N, y={F_rep[1]:.3f} N, |F_rep|={np.linalg.norm(F_rep):.3f} N')
                print(f'  违规力 F_violation: x={F_violation[0]:.3f} N, y={F_violation[1]:.3f} N, |F_violation|={np.linalg.norm(F_violation):.3f} N')
                print(f'  合力   F_total:     x={F_total[0]:.3f} N, y={F_total[1]:.3f} N, |F_total|={np.linalg.norm(F_total):.3f} N')
                print(f'  加速度 acceleration: x={acceleration[0]:.3f} m/s², y={acceleration[1]:.3f} m/s², |a|={np.linalg.norm(acceleration):.3f} m/s²')
                print(f'  Danger mode: {dist_danger_mode}')
                print(f'  Number of obstacles: {len(self.obstacles)}')
                min_obs_dist = float('inf')
                for obs in self.obstacles:
                    obs_pos = np.array([obs.x, obs.y])
                    dist = np.linalg.norm(current_pos - obs_pos)
                    if dist < min_obs_dist:
                        min_obs_dist = dist
                print(f'  最近障碍物距离: {min_obs_dist:.2f} m')
                print(f'  距离目标: {dist_goal:.2f} m')
                print('')

            # --- Velocity Integration ---
            velocity += acceleration * dt

            # --- Goal Damping + Snap Logic ---
            goal_slow_radius = 5.0
            if dist_goal < goal_slow_radius:
                damping_ratio = dist_goal / goal_slow_radius
                velocity *= damping_ratio
                guide_velocity = 1.0 * unit_goal * (1 - damping_ratio)
                velocity += guide_velocity

            goal_snap_radius = 5.0
            if dist_goal < goal_snap_radius:
                blend_ratio = (goal_snap_radius - dist_goal) / goal_snap_radius
                velocity *= (1 - blend_ratio)
                guide_velocity = 1.5 * unit_goal * blend_ratio
                velocity += guide_velocity

            # --- Update Position & Velocity ---
            current_position = current_pos + velocity * dt
            current_velocity = velocity

            trajectory.append(current_position.copy())
            velocities.append(np.round(np.linalg.norm(current_velocity), 2))

            self.ego.x, self.ego.y = current_position
            self.ego.vx, self.ego.vy = current_velocity
            norm = np.linalg.norm(current_velocity)
            if norm > 1e-4:
                self.ego.hx, self.ego.hy = current_velocity / norm

            for obs in self.obstacles:
                prev_pos = np.array([obs.x, obs.y])
                new_pos = prev_pos + np.array([obs.vx, obs.vy]) * dt
                obs.x, obs.y = new_pos

            # --- 关键信息调试输出 ---
            if i % 10 == 0:
                min_obs_dist = float('inf')
                for obs in self.obstacles:
                    obs_pos = np.array([obs.x, obs.y])
                    dist = np.linalg.norm(current_pos - obs_pos)
                    if dist < min_obs_dist:
                        min_obs_dist = dist
                print(f'[KEY_DEBUG] Step {i}: min_obs_dist={min_obs_dist:.2f}, dist_goal={dist_goal:.2f}, ttc_danger_mode={ttc_danger_mode}, dist_danger_mode={dist_danger_mode}')

        trajectory.append(self.goal[:2])
        self.trajectory = np.array(trajectory)
        trajectory_velocity = np.array(velocities)
        trajectory_velocity = np.append(trajectory_velocity, 0.0)
        return self.trajectory, trajectory_velocity

    def compute_violation_force_global(self, position, velocity, ttc_danger_mode):
        """
        基于全局坐标系和道路中心线计算违规力
        """
        if self.lane_width is None:
            return np.zeros(2)
        
        current_pos = np.array(position)
        current_vel = np.array(velocity)
        
        # 1. 找到最近的道路中心线点
        idx = self.find_nearest_centerline_index(current_pos)
        road_center = np.array(self.centerline_points[idx])
        road_heading = self.centerline_headings[idx]
        
        # 2. 计算道路局部坐标系的单位向量
        road_forward = np.array([np.cos(road_heading), np.sin(road_heading)])
        road_lateral = np.array([-np.sin(road_heading), np.cos(road_heading)])
        
        # 3. 计算车辆到道路中心线的横向偏移
        to_road_center = road_center - current_pos
        # 修正：lateral_offset正负与物理空间一致
        lateral_offset = -np.dot(to_road_center, road_lateral)
        print(f"[DEBUG_LATERAL] current_pos={current_pos}, road_center={road_center}, road_lateral={road_lateral}, to_road_center={to_road_center}, lateral_offset={lateral_offset}")

        # 新增：横向速度分量，严格采用road_lateral方向
        lateral_speed = np.dot(current_vel, road_lateral)
        damping_coeff_max = 3.0  # 微调后的最大阻尼系数
        # 4. 计算基础违规力（基于横向偏移）- 降低触发阈值
        base_force = np.zeros(2)
        lane_violation_threshold = self.lane_width / 4.0
        force_magnitude = 15.0
        if abs(lateral_offset) > lane_violation_threshold:
            base_force = force_magnitude * (-lateral_offset) * road_lateral
        # 新增：横向速度阻尼项，分段增强
        road_half_width = 1.5 * self.lane_width
        if abs(lateral_offset) <= lane_violation_threshold:
            damping_coeff = 0.0
        elif abs(lateral_offset) >= road_half_width:
            damping_coeff = damping_coeff_max
        else:
            ratio = (abs(lateral_offset) - lane_violation_threshold) / (road_half_width - lane_violation_threshold)
            damping_coeff = damping_coeff_max * ratio
        damping_force = -damping_coeff * lateral_speed * road_lateral
        
        # 5. 边界力（防止车辆离开道路）- 降低触发阈值
        edge_force = np.zeros(2)
        if abs(lateral_offset) > road_half_width:
            edge_violation = abs(lateral_offset) - road_half_width
            edge_force = -np.sign(lateral_offset) * road_lateral * edge_violation * 8.0
        
        # 6. 切向力（TTC-based obstacle avoidance）
        tangent_force = np.zeros(2)
        if ttc_danger_mode:
            obstacle_distances = [(obs, np.linalg.norm([obs.x - current_pos[0], obs.y - current_pos[1]])) 
                                 for obs in self.obstacles]
            obstacle_distances.sort(key=lambda x: x[1])
            closest_obstacles = [x[0] for x in obstacle_distances[:3]]
            vehicle_speed = np.linalg.norm(current_vel)
            if vehicle_speed > 1e-6:
                vehicle_direction = current_vel / vehicle_speed
            else:
                vehicle_direction = road_forward
            for obs in closest_obstacles:
                obs_pos = np.array([obs.x, obs.y])
                obs_vec = obs_pos - current_pos
                dist = np.linalg.norm(obs_vec)
                if dist < 1e-2:
                    continue
                unit_vec = obs_vec / dist
                left_dir = np.array([-vehicle_direction[1], vehicle_direction[0]])
                # 计算TTC
                ttc = None
                if hasattr(obs, 'compute_ttc'):
                    ttc = obs.compute_ttc(self.ego)
                # 切向力与ttc成反比，ttc越小切向力越大，ttc>0才有意义
                if ttc is not None and ttc > 0 and ttc < 5.0:
                    # 线性反比关系，最大强度5.0，最小0.2
                    strength = max(0.2, min(5.0, 5.0 * (1.5 / (ttc + 1e-3))))
                    cross_product = np.cross(vehicle_direction, unit_vec)
                    if cross_product > 0:
                        force_direction = -left_dir
                    else:
                        force_direction = left_dir
                    tangent_force += force_direction * strength
        # 7. TTC调整力
        ttc_values = []
        if ttc_danger_mode:
            obstacle_distances = [(obs, np.linalg.norm([obs.x - current_pos[0], obs.y - current_pos[1]])) 
                                 for obs in self.obstacles]
            obstacle_distances.sort(key=lambda x: x[1])
            closest_obstacles = [x[0] for x in obstacle_distances[:3]]
            for obs in closest_obstacles:
                if hasattr(obs, 'compute_ttc'):
                    ttc = obs.compute_ttc(self.ego)
                    if ttc > 0:
                        ttc_values.append(ttc)
        min_ttc = min(ttc_values) if ttc_values else float('inf')
        t_safe = 1.6
        ttc_std = 2.5 if self.ego_speed < 10 else (3.0 if self.ego_speed < 15 else 3.5)
        ttc_adjustment = np.zeros(2)
        if 0 < min_ttc < t_safe:
            R = np.tanh(ttc_std / (min_ttc + 1e-3))
            ttc_penalty = (t_safe - min_ttc)
            ttc_adjustment = road_lateral * ttc_penalty * 0.5
        else:
            R = 1.0
        total_violation_force = (base_force + edge_force + tangent_force) * R + ttc_adjustment + damping_force
        if np.linalg.norm(total_violation_force) > 0.5:
            print(f"[VIOLATION_DEBUG] lateral_offset={lateral_offset:.3f}, lane_width={self.lane_width:.3f}, threshold={lane_violation_threshold:.3f}, road_half_width={road_half_width:.3f}")
            print(f"[VIOLATION_DEBUG] base_force={np.linalg.norm(base_force):.3f}, edge_force={np.linalg.norm(edge_force):.3f}, tangent_force={np.linalg.norm(tangent_force):.3f}, ttc_adjustment={np.linalg.norm(ttc_adjustment):.3f}, damping_force={np.linalg.norm(damping_force):.3f}")
            print(f'lateral_offset={lateral_offset:.2f}, base_force=({base_force[0]:.2f}, {base_force[1]:.2f}), damping_coeff={damping_coeff:.2f}, lateral_speed={lateral_speed:.2f}')
        return total_violation_force

    def smooth_trajectory(self, trajectory, weight_smooth=1.0, weight_jerk=0.1, weight_fidelity=1.0,
                          weight_direction=10.0, weight_curvature=10.0):
        if trajectory is None or len(trajectory) < 3:
            raise ValueError("Trajectory must have at least 3 points for smoothing.")

        # 下采样，减少优化复杂度
        if len(trajectory) > 200:
            print('[WARN] Downsampling trajectory for smoothing:', len(trajectory), '->', len(trajectory)//5)
            trajectory = trajectory[::5]

        N = len(trajectory)
        print('[DEBUG] smooth_trajectory: N =', N)
        x = cp.Variable(N)
        y = cp.Variable(N)

        cost = 0
        constraints = [
            x[0] == trajectory[0, 0], y[0] == trajectory[0, 1],
            x[-1] == trajectory[-1, 0], y[-1] == trajectory[-1, 1]
        ]

        for i in range(1, N - 1):
            # 移除频繁的调试输出，避免信息过载
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
            if i % 50 == 0:
                print(f'[DEBUG] smooth_trajectory fidelity at i={i}/{N}')
            cost += weight_fidelity * (
                    cp.square(x[i] - trajectory[i, 0]) +
                    cp.square(y[i] - trajectory[i, 1])
            )

        prob = cp.Problem(cp.Minimize(cost), constraints)
        print('[DEBUG] Before prob.solve, trajectory length:', N, 'variables:', x.shape)
        prob.solve(solver=cp.OSQP)
        print('[DEBUG] After prob.solve')

        if x.value is None or y.value is None:
            raise ValueError("Trajectory smoothing failed. Solver did not find a solution.")

        smoothed_traj = np.vstack((x.value, y.value)).T
        return smoothed_traj


def create_regular_obstacles(centerline_points, centerline_headings):
    """
    基于道路中心线创建障碍物
    """
    obstacles = []
    
    if centerline_points is None or len(centerline_points) < 2:
        return obstacles
    
    # 在道路中心线附近随机放置障碍物
    num_obstacles = 5  # 每个车道的障碍物数量减少到5
    num_lanes = 5  # 车道数量
    lane_width = 5.0  # 车道宽度
    
    for lane_idx in range(num_lanes):
        for i in range(num_obstacles):
            if random.random() > 0.7:  # 增加障碍物生成的随机性
                continue
                
            # 随机选择一个centerline点（避开起点和终点）
            idx = random.randint(5, len(centerline_points) - 5)
            centerline_point = centerline_points[idx]
            heading = centerline_headings[idx]
            
            # 计算车道偏移
            offset = (lane_idx - num_lanes//2) * lane_width
            
            # 计算障碍物位置
            obs_x = centerline_point[0] + offset * np.cos(heading + np.pi/2)
            obs_y = centerline_point[1] + offset * np.sin(heading + np.pi/2)
            
            # 创建障碍物
            obs = Obstacle(
                x=obs_x, y=obs_y,
                vx=0, vy=0,
                hx=np.cos(heading), hy=np.sin(heading),
                length=4.0, width=2.0
            )
            obstacles.append(obs)
    
    print(f"[DEBUG] Created {len(obstacles)} obstacles")
    return obstacles


def smooth_clipped_repulsion(delta_vec, dist, d0, eta, repulsion_max):
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


def test_scenario(start, goal, scenario_name, centerline_points, centerline_headings):
    """
    测试场景
    """
    # 创建障碍物
    obstacles = create_regular_obstacles(centerline_points, centerline_headings)
    
    # 创建规划器
    planner = APFPlanner(start, goal, obstacles, centerline_points, centerline_headings)
    
    # 设置车道信息
    planner.set_lane_info(road_width=15.0, num_lanes=5, 
                         lane_line_types=[0, 0, 0, 0], intended_lane_index=2)
    
    # 规划轨迹
    smoothed_trajectory, trajectory_velocity = planner.plan()
    
    # 绘制结果
    plt.figure(figsize=(15, 10))
    
    # 绘制背景道路
    draw_road_background(centerline_points, centerline_headings, lane_width=5.0, num_lanes=5)
    
    # 绘制轨迹
    if smoothed_trajectory is not None and len(smoothed_trajectory) > 0:
        plt.plot(smoothed_trajectory[:, 0], smoothed_trajectory[:, 1], 'b-', linewidth=2, label='Smoothed Trajectory')
    
    # 绘制起终点
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')
    
    # 绘制障碍物
    for obs in planner.obstacles:
        obs_rect = plt.Rectangle((obs.x - obs.length/2, obs.y - obs.width/2), 
            obs.length, obs.width,
                                angle=np.degrees(np.arctan2(obs.hy, obs.hx)),
                                alpha=0.7, color='red', label='Obstacle' if obs == planner.obstacles[0] else "")
        plt.gca().add_patch(obs_rect)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'{scenario_name} - APF Planning Result')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 保存图片到文件
    output_dir = "output_plots"
    os.makedirs(output_dir, exist_ok=True)
    filename = f"{output_dir}/{scenario_name.replace(' ', '_').lower()}_result.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"[INFO] Plot saved to: {filename}")
    plt.close()  # 关闭图形，释放内存


# 删除：draw_lane_lines函数不再需要，统一使用draw_lane_lines_centerline

def distance_to_rectangle_edge(point, rect_center, rect_yaw, length, width):
    """
    计算点到矩形边缘的距离
    """
    # 将点转换到矩形的局部坐标系
    dx = point[0] - rect_center[0]
    dy = point[1] - rect_center[1]
    
    # 旋转到矩形的局部坐标系
    cos_yaw = np.cos(-rect_yaw)
    sin_yaw = np.sin(-rect_yaw)
    
    local_x = dx * cos_yaw - dy * sin_yaw
    local_y = dx * sin_yaw + dy * cos_yaw

    # 计算到矩形边缘的距离
    half_length = length / 2
    half_width = width / 2

    # 最近点在矩形内部
    if abs(local_x) <= half_length and abs(local_y) <= half_width:
        # 到最近边缘的距离
        dist_to_edge = min(half_length - abs(local_x), half_width - abs(local_y))
        return dist_to_edge
    
    # 最近点在矩形外部
    closest_x = np.clip(local_x, -half_length, half_length)
    closest_y = np.clip(local_y, -half_width, half_width)
    
    dist = np.sqrt((local_x - closest_x)**2 + (local_y - closest_y)**2)
    return dist

def draw_road_background(centerline_points, centerline_headings, lane_width=5.0, num_lanes=5):
    """
    绘制背景道路
    """
    centerline_array = np.array(centerline_points)
    # 绘制道路中心线
    plt.plot(centerline_array[:, 0], centerline_array[:, 1], 'k--', linewidth=1, label='Road Centerline')
    # 绘制车道线
    for lane_idx in range(num_lanes):
        # 计算车道偏移
        offset = (lane_idx - num_lanes//2) * lane_width  # 从-2到2的车道
        xs = []
        ys = []
        for i, (point, heading) in enumerate(zip(centerline_points, centerline_headings)):
            lane_x = point[0] + offset * np.cos(heading + np.pi/2)
            lane_y = point[1] + offset * np.sin(heading + np.pi/2)
            xs.append(lane_x)
            ys.append(lane_y)
        # 只画普通车道线，不画中心车道蓝色线
        plt.plot(xs, ys, 'k-', linewidth=1, alpha=0.5)
    # 绘制道路边界
    for edge_offset in [-num_lanes//2 * lane_width, num_lanes//2 * lane_width]:
        xs = []
        ys = []
        for i, (point, heading) in enumerate(zip(centerline_points, centerline_headings)):
            edge_x = point[0] + edge_offset * np.cos(heading + np.pi/2)
            edge_y = point[1] + edge_offset * np.sin(heading + np.pi/2)
            xs.append(edge_x)
            ys.append(edge_y)
        plt.plot(xs, ys, 'k-', linewidth=3, alpha=0.8)

if __name__ == "__main__":
    road_width = 15
    num_lanes = 3
    lane_line_types = [0, 1]
    intended_lane_index = 1
    ego_speed = 10
    safe_width = 2.0
    # 场景1：水平直道
    start_parallel = [0, 0, ego_speed, 0]
    goal_parallel = [100, 0, 0, 0]
    lane_angle_parallel = 0
    road_origin_parallel = [0, 0]
    # 为水平直道创建centerline数据
    horizontal_points = 50
    centerline_points_horizontal = [[i * 2, 0] for i in range(horizontal_points)]
    centerline_headings_horizontal = [0.0] * horizontal_points
    print(f"[DEBUG] Centerline points (first 5): {centerline_points_horizontal[:5]}")
    print(f"[DEBUG] Centerline headings (first 5): {centerline_headings_horizontal[:5]}")
    print("[INFO] Running Parallel Scenario (Straight Road)")
    test_scenario(start_parallel, goal_parallel, "Parallel Scenario (Straight Road)",
                 centerline_points=centerline_points_horizontal,
                 centerline_headings=centerline_headings_horizontal)
    # 测试场景2：倾斜直道
    print("[INFO] Running Tilted Scenario (Tilted Straight Road)")
    start_tilted = [0, 0, ego_speed, 0]
    goal_tilted = [100 * np.cos(np.pi/6), 100 * np.sin(np.pi/6), 0, 0]
    centerline_points_tilted = [[i * 2 * np.cos(np.pi/6), i * 2 * np.sin(np.pi/6)] for i in range(horizontal_points)]
    centerline_headings_tilted = [np.pi/6] * horizontal_points
    test_scenario(start_tilted, goal_tilted, "Tilted Scenario (Tilted Straight Road)",
                 centerline_points=centerline_points_tilted,
                 centerline_headings=centerline_headings_tilted)
    
    # 测试场景3：弯道
    print("[INFO] Running Arc Scenario (Curved Road)")
    start_arc = [0, 0, ego_speed, 0]
    goal_arc = [100 * np.cos(np.pi/3), 100 * np.sin(np.pi/3), 0, 0]
    centerline_points_arc = [[i * 2 * np.cos(np.pi/3), i * 2 * np.sin(np.pi/3)] for i in range(horizontal_points)]
    centerline_headings_arc = [np.pi/3] * horizontal_points
    test_scenario(start_arc, goal_arc, "Arc Scenario (Curved Road)",
                 centerline_points=centerline_points_arc,
                 centerline_headings=centerline_headings_arc)
