import concurrent.futures
import math
import sys
if __name__ == "__main__" and __package__ is None:
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    __package__ = "myDSL"
import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
import heapq
from myDSL.RecordDealer import HeroPlanner
import matplotlib.patches as patches
import importlib
if "carla" not in sys.modules:
    carla = importlib.import_module("carla")
else:
    carla = sys.modules["carla"]


class SpeedPlanner:
    def __init__(self, trajectory, obstacles, T, dt, ds, lambda_acc, sigma, penalty_factor, ego):
        self.trajectory = trajectory
        self.obstacles = obstacles
        self.T = T
        self.dt = dt
        self.ds = ds
        self.lambda_acc = lambda_acc
        self.sigma = sigma
        self.penalty_factor = penalty_factor
        self.ego = ego  # Now we have ego as an instance variable

        # Correct s_total: compute actual arc length of the trajectory
        self.s_total = 0.0
        self.arc_lengths = [0.0]  # List to store the cumulative arc length at each point
        for i in range(1, len(trajectory)):
            dx = trajectory[i][0] - trajectory[i - 1][0]
            dy = trajectory[i][1] - trajectory[i - 1][1]
            dist = np.sqrt(dx ** 2 + dy ** 2)
            self.s_total += dist
            self.arc_lengths.append(self.s_total)  # Store the cumulative arc length

        # We will not compute st_regions here directly, as it depends on the ego vehicle
        self.st_regions = None
        self.update_st_regions()
        
        # 存储原始参数用于动态调整
        self.original_dt = dt
        self.original_ds = ds
        self.original_penalty_factor = penalty_factor

    def compute_s_range(self, obstacle, s_center):
        half_length = obstacle.length / 2.0
        return (s_center - half_length, s_center + half_length)

    def compute_obstacle_st(self, obstacle):
        """
        Compute the time-steps where obstacle is too close to trajectory.
        """
        st_regions = []
        for t in np.arange(0, self.T, self.dt):
            obs_x = obstacle.x + obstacle.vx * t
            obs_y = obstacle.y + obstacle.vy * t

            s_center, min_distance = self.project_to_trajectory(obs_x, obs_y)

            obstacle_radius = min(obstacle.length, obstacle.width) / 2
            ego_radius = min(self.ego.length, self.ego.width) / 2
            # threshold_distance = obstacle_radius + ego_radius
            threshold_distance = obstacle_radius + ego_radius / 2

            if min_distance < threshold_distance:
                s_range = self.compute_s_range(obstacle, s_center)
                st_regions.append((t, t + self.dt, s_range[0], s_range[1]))

        #     # Debug
        #     print(
        #         f"[DEBUG] t={t:.2f}, obs=({obs_x:.2f},{obs_y:.2f}), min_dist={min_distance:.2f}, threshold={threshold_distance:.2f}")
        #
        # print(f"[DEBUG] Found {len(st_regions)} st regions for this obstacle.")
        return st_regions

    def compute_all_obstacle_st(self):
        """
        Compute the time-steps at which all obstacles intersect with the ego vehicle's lane.
        Parameters:
            ego: The ego vehicle object.
        Returns:
            all_regions: List of time ranges for all obstacles during which they occupy the lane.
        """
        all_regions = []
        for obs in self.obstacles:
            # Call compute_obstacle_st for each obstacle
            regions = self.compute_obstacle_st(obs)
            all_regions.extend(regions)
        return all_regions

    def update_st_regions(self):
        """
        This method can be called when you want to recompute the obstacle st_regions.
        """
        self.st_regions = self.compute_all_obstacle_st()

    def project_to_trajectory(self, x, y):
        # Find the closest point to (x, y) based on the minimal distance
        dists = [np.hypot(pt[0] - x, pt[1] - y) for pt in self.trajectory]
        idx = np.argmin(dists)

        # Return the corresponding arc length from the lookup table
        return self.arc_lengths[idx], dists[idx]

    def compute_obstacle_penalty(self, times, s_vals):
        T, s_total = times[-1], s_vals[-1]
        obs_cost = np.zeros((len(times), len(s_vals)))
        for i, t in enumerate(times):
            norm_t = t / T
            for region in self.st_regions:
                t0, t1, s_start, s_end = region
                norm_t0, norm_t1 = t0 / T, t1 / T
                norm_s0, norm_s1 = s_start / s_total, s_end / s_total
                if t0 <= t < t1:
                    for j, s in enumerate(s_vals):
                        norm_s = s / s_total
                        if s_start <= s <= s_end:
                            obs_cost[i, j] = self.penalty_factor
                        else:
                            dt_norm = min(abs(norm_t - norm_t0), abs(norm_t - norm_t1))
                            ds_norm = min(abs(norm_s - norm_s0), abs(norm_s - norm_s1))
                            d = np.sqrt(dt_norm ** 2 + ds_norm ** 2)
                            if d < self.sigma:
                                obs_cost[i, j] = self.penalty_factor * (1 - d / self.sigma)
        return obs_cost

    def _adjust_parameters_based_on_scenario(self):
        """根据场景复杂度动态调整参数"""
        # 根据轨迹长度调整步长
        if self.s_total > 100:  # 长轨迹
            self.ds = min(self.original_ds * 2.0, 0.5)  # 增加空间步长，但不超过0.5m
            self.dt = min(self.original_dt * 1.5, 0.3)  # 增加时间步长，但不超过0.3s
            print(f"[A*-OPTIMIZE] Long trajectory detected, adjusted ds={self.ds:.3f}, dt={self.dt:.3f}")
        elif self.s_total > 50:  # 中等轨迹
            self.ds = self.original_ds * 1.5
            self.dt = self.original_dt * 1.2
            print(f"[A*-OPTIMIZE] Medium trajectory detected, adjusted ds={self.ds:.3f}, dt={self.dt:.3f}")
        
        # 根据障碍物数量调整惩罚因子
        num_obstacles = len(self.obstacles)
        if num_obstacles > 5:  # 高密度障碍物
            self.penalty_factor = self.original_penalty_factor * 0.8  # 降低惩罚，允许更多探索
            print(f"[A*-OPTIMIZE] High obstacle density ({num_obstacles}), reduced penalty factor to {self.penalty_factor}")
        elif num_obstacles > 3:  # 中等密度
            self.penalty_factor = self.original_penalty_factor * 0.9
            print(f"[A*-OPTIMIZE] Medium obstacle density ({num_obstacles}), adjusted penalty factor to {self.penalty_factor}")

    def _should_skip_planning(self, penalty_ratio, total_cells):
        """检查是否应该跳过规划"""
        # 场景太复杂，直接跳过
        if penalty_ratio > 0.6:  # 超过60%被阻挡
            print(f"[A*-SKIP] Too many obstacles ({penalty_ratio:.1%} blocked), skipping planning")
            return True
        
        # 搜索空间过大
        if total_cells > 100000:  # 超过10万个网格
            print(f"[A*-SKIP] Search space too large ({total_cells} cells), skipping planning")
            return True
        
        # 轨迹过长且障碍物密度高的组合
        if self.s_total > 150 and penalty_ratio > 0.3:
            print(f"[A*-SKIP] Long trajectory ({self.s_total:.1f}m) with high obstacle density ({penalty_ratio:.1%}), skipping planning")
            return True
        
        return False

    def _get_dynamic_node_limit(self, attempt, total_cells):
        """根据尝试次数和搜索空间动态调整节点限制"""
        base_limit = min(total_cells // 2, 15000)  # 基础限制不超过搜索空间的一半，最多15000
        
        if attempt < 5:  # 前5次尝试使用较小限制
            return min(base_limit // 2, 5000)
        elif attempt < 8:  # 中期尝试
            return min(base_limit, 10000)
        else:  # 后期尝试使用更大限制
            return min(base_limit * 2, 20000)

    def plan_speed_profile_astar(self):
        # 动态调整参数
        self._adjust_parameters_based_on_scenario()
        
        N_t = math.ceil(self.T / self.dt) + 1
        N_s = math.ceil(self.s_total / self.ds) + 1
        times = np.linspace(0, self.T, N_t)
        s_vals = np.linspace(0, self.s_total, N_s)
        obs_penalty = self.compute_obstacle_penalty(times, s_vals)

        # 添加调试信息
        print(f"[A*-DEBUG] Search space: N_t={N_t}, N_s={N_s}")
        print(f"[A*-DEBUG] Total time: {self.T:.2f}s, total distance: {self.s_total:.2f}m")
        print(f"[A*-DEBUG] Time step: {self.dt:.3f}s, space step: {self.ds:.3f}m")
        
        # 分析障碍物惩罚
        high_penalty_count = np.sum(obs_penalty >= self.penalty_factor)
        total_cells = N_t * N_s
        penalty_ratio = high_penalty_count / total_cells
        print(f"[A*-DEBUG] Obstacle penalty cells: {high_penalty_count}/{total_cells} ({penalty_ratio:.2%})")
        print(f"[A*-DEBUG] Penalty factor: {self.penalty_factor}")

        # 早期终止条件检查
        if self._should_skip_planning(penalty_ratio, total_cells):
            return None, obs_penalty

        max_attempts = 10
        base_max_speed = self.s_total / (self.T * 0.5)
        base_ds_range = 3
        print(f"[A*-DEBUG] Base max speed: {base_max_speed:.2f} m/s")

        for attempt in range(max_attempts + 1):
            print(f"[A*] Attempt {attempt + 1}...")
            visited = np.full((N_t, N_s), False)
            came_from = [[(-1, -1) for _ in range(N_s)] for _ in range(N_t)]
            cost = np.full((N_t, N_s), np.inf)

            heap = []
            cost[0, 0] = 0
            heapq.heappush(heap, (self.heuristic(0, 0, self.T, self.s_total), 0, 0, 0))

            if attempt < max_attempts:
                max_speed = base_max_speed * (1 + 0.5 * attempt)
                ds_limit = base_ds_range + attempt
                optimized = True
                print(f"[A*-DEBUG] Attempt {attempt + 1}: max_speed={max_speed:.2f}, ds_limit={ds_limit}")
            else:
                print("[A*] Fallback to Standard A* Mode.")
                optimized = False
                print(f"[A*-DEBUG] Standard mode: no speed/step limits")

            # 搜索过程监控
            nodes_explored = 0
            # 动态调整节点限制
            max_nodes_per_attempt = self._get_dynamic_node_limit(attempt, total_cells)
            print(f"[A*-DEBUG] Max nodes for attempt {attempt + 1}: {max_nodes_per_attempt}")
            
            while heap and nodes_explored < max_nodes_per_attempt:
                f, g, t_idx, s_idx = heapq.heappop(heap)
                if visited[t_idx, s_idx]:
                    continue
                visited[t_idx, s_idx] = True
                nodes_explored += 1
                
                # 每1000个节点打印一次进度
                if nodes_explored % 1000 == 0:
                    progress_t = t_idx / (N_t - 1) * 100
                    progress_s = s_idx / (N_s - 1) * 100
                    print(f"[A*-DEBUG] Explored {nodes_explored} nodes, progress: t={progress_t:.1f}%, s={progress_s:.1f}%")

                if optimized:
                    if s_idx >= N_s - 1 and t_idx >= N_t - 2:
                        print(f"[A*-DEBUG] Reached goal region: t_idx={t_idx}, s_idx={s_idx}")
                        break
                    ds_range = range(1, ds_limit)
                else:
                    if t_idx == N_t - 1 and s_idx == N_s - 1:
                        print(f"[A*-DEBUG] Reached exact goal: t_idx={t_idx}, s_idx={s_idx}")
                        break
                    ds_range = range(1, N_s - s_idx)

                valid_moves = 0
                blocked_moves = 0
                
                for ds_step in ds_range:
                    s_next_idx = s_idx + ds_step
                    t_next_idx = t_idx + 1
                    if t_next_idx >= N_t or s_next_idx >= N_s:
                        continue

                    if obs_penalty[t_next_idx, s_next_idx] >= self.penalty_factor:
                        blocked_moves += 1
                        continue

                    s_now = s_vals[s_idx]
                    s_next = s_vals[s_next_idx]
                    speed = (s_next - s_now) / self.dt

                    if optimized:
                        if speed < 0 or speed > max_speed:
                            continue
                        accel = speed / self.dt
                        transition_cost = self.lambda_acc * accel ** 2 * self.dt
                    else:
                        transition_cost = 1

                    new_g = g + transition_cost + obs_penalty[t_next_idx, s_next_idx]
                    h = self.heuristic(times[t_next_idx], s_vals[s_next_idx], self.T, self.s_total)

                    if new_g < cost[t_next_idx, s_next_idx]:
                        cost[t_next_idx, s_next_idx] = new_g
                        came_from[t_next_idx][s_next_idx] = (t_idx, s_idx)
                        heapq.heappush(heap, (new_g + h, new_g, t_next_idx, s_next_idx))
                        valid_moves += 1
                
                # 如果节点没有有效移动，记录调试信息
                if valid_moves == 0 and nodes_explored % 500 == 0:
                    print(f"[A*-DEBUG] Dead end at ({t_idx}, {s_idx}), blocked_moves={blocked_moves}")
            
            print(f"[A*-DEBUG] Attempt {attempt + 1} completed: explored {nodes_explored} nodes")
            
            if nodes_explored >= max_nodes_per_attempt:
                print(f"[A*-DEBUG] Hit node exploration limit ({max_nodes_per_attempt})")

            if optimized:
                goal_candidates = [(t_idx, s_idx) for t_idx in range(N_t - 3, N_t) for s_idx in range(N_s - 3, N_s) if
                                   visited[t_idx, s_idx]]
                if goal_candidates:
                    t_idx, s_idx = min(goal_candidates, key=lambda x: cost[x[0], x[1]])
                    break
            else:
                if visited[N_t - 1, N_s - 1]:
                    t_idx, s_idx = N_t - 1, N_s - 1
                    break
        else:
            print("[A*] Failed after all attempts.")
            return None, obs_penalty

        path = []
        while t_idx >= 0 and s_idx >= 0:
            path.append((times[t_idx], s_vals[s_idx]))
            t_idx, s_idx = came_from[t_idx][s_idx]
            if t_idx == -1 or s_idx == -1:
                break

        return path[::-1], obs_penalty

    def smooth_speed_profile_qp(self, dp_profile):
        if not dp_profile:
            return None
        times, s_dp, N = np.array([pt[0] for pt in dp_profile]), np.array([pt[1] for pt in dp_profile]), len(dp_profile)
        s = cp.Variable(N)
        obj = cp.sum_squares(s - s_dp) + 10.0 * cp.sum_squares(s[2:] - 2 * s[1:-1] + s[:-2])
        prob = cp.Problem(cp.Minimize(obj), [s[0] == 0, s[-1] == self.s_total])
        prob.solve()
        return list(zip(times, s.value))

    def heuristic(self, t, s, T, s_total):
        # Normalize time and space distances
        dt = T - t
        ds = s_total - s
        return np.sqrt((dt / T) ** 2 + (ds / s_total) ** 2)


def compute_time_and_s(trajectory, velocity, dt=0.025):
    times = [i * dt for i in range(len(trajectory))]
    s_vals = [0.0]
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i - 1][0]
        dy = trajectory[i][1] - trajectory[i - 1][1]
        s_vals.append(s_vals[-1] + np.sqrt(dx ** 2 + dy ** 2))
    if len(velocity) < len(trajectory):
        velocity = np.append(velocity, 0.0)
    return times, s_vals, velocity


def run_speed_planner(trajectory, trajectory_velocity, obstacles, ego, dt=0.2, ds=0.1, lambda_acc=0.2, sigma=0.1,
                      penalty_factor=1000):
    """
       Run speed planner to generate optimal velocity profile along given trajectory.

       Args:
           trajectory (list[list[float]]): List of (x, y) path points.
           trajectory_velocity (list[float]): Initial velocity at each trajectory point.
           obstacles (list[Obstacle]): List of obstacle objects in the environment.
           ego (dict): Ego vehicle data or configuration.
           dt (float): Time step for ST graph discretization.
           ds (float): Path step for ST graph discretization.
           lambda_acc (float): Acceleration penalty weight.
           sigma (float): Smoothness penalty weight.
           penalty_factor (float): Obstacle collision penalty weight.

       Returns:
           sp (SpeedPlanner): SpeedPlanner object for reference.
           times (list[float]): Original trajectory time stamps.
           trajectory_velocity (list[float]): Original trajectory velocity profile.
           smoothed_t (list[float]): Optimized time points from dynamic programming.
           v_vals_sp (list[float]): Optimized velocity values at each smoothed time point.
           dp_profile (list[tuple[float, float]]): Optimized (time, s) path from dynamic programming.
   """
    times, s_vals, trajectory_velocity = compute_time_and_s(trajectory, trajectory_velocity)
    print(f"[INFO] Trajectory length: {len(trajectory)}")
    print(f"[INFO] Total time: {times[-1]}, total path length: {s_vals[-1]}")

    sp = SpeedPlanner(
        trajectory=trajectory,
        obstacles=obstacles,
        T=times[-1],
        dt=dt,
        ds=ds,
        lambda_acc=lambda_acc,
        sigma=sigma,
        penalty_factor=penalty_factor,
        ego=ego  # Passing ego here; should be assigned accordingly
    )
    TIMEOUT = 5  # seconds
    print("[INFO] Running A* algorithm...")
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(sp.plan_speed_profile_astar)
        try:
            dp_profile, _ = future.result(timeout=TIMEOUT)
        except concurrent.futures.TimeoutError:
            print(f"[TIMEOUT] A* speed planning exceeded {TIMEOUT}s.")
            dp_profile = None
    if dp_profile is None:
        print("[RESULT] No feasible trajectory found.")
        return sp, times, trajectory_velocity, None, None, None

    # velocity profile from dp_profile
    s_vals_new = [pt[1] for pt in dp_profile]
    v_vals_sp = [0.0]
    for i in range(1, len(dp_profile)):
        ds = s_vals_new[i] - s_vals_new[i - 1]
        dt = dp_profile[i][0] - dp_profile[i - 1][0]
        v_vals_sp.append(round(ds / (dt + 1e-6), 2))
    smoothed_t = [pt[0] for pt in dp_profile]

    return sp, times, trajectory_velocity, smoothed_t, v_vals_sp, dp_profile


def plot_velocity_comparison(times, v_vals_hero, smoothed_t, v_vals_sp):
    if smoothed_t is None or v_vals_sp is None:
        print("[WARN] No replanned velocity available, skipping replanned curve.")

    plt.figure(figsize=(10, 6))
    plt.plot(times, v_vals_hero, 'b-', label='Original Velocity (HeroPlanner)')

    if smoothed_t is not None and v_vals_sp is not None:
        plt.plot(smoothed_t, v_vals_sp, 'g--', label='Replanned Velocity (SpeedPlanner)')

    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.title("Speed-Time (ST) Comparison")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_st_graph(sp, dp_profile, smoothed_t, trajectory_velocity):
    plt.figure(figsize=(10, 6))

    # Check if sp and its st_regions are valid
    if sp is not None and sp.st_regions is not None:
        for i, (t0, t1, s_start, s_end) in enumerate(sp.st_regions):
            plt.fill_between([t0, t1], s_start, s_end, color='red', alpha=0.3,
                             label='Obstacle' if i == 0 else "")
    else:
        print("[WARN] No ST regions available to plot obstacles.")

    # Plot DP Profile if available
    if dp_profile is not None:
        dp_profile_t = [pt[0] for pt in dp_profile]
        dp_profile_s = [pt[1] for pt in dp_profile]
        plt.plot(dp_profile_t, dp_profile_s, 'b--', label='DP Profile')

        if smoothed_t is not None:
            plt.plot(smoothed_t, dp_profile_s, 'g-', label='Smoothed Profile')
    else:
        print("[WARN] No DP profile available to plot.")
    # Plot HeroPlanner original ST curve
    if trajectory_velocity is not None and len(trajectory_velocity) > 0:
        s_hero = 0.0
        hero_st = [(0.0, 0.0)]
        for i in range(1, len(trajectory_velocity)):
            s_hero += trajectory_velocity[i] * 0.025
            hero_st.append((i * 0.025, s_hero))
        hero_t, hero_s = zip(*hero_st)
        plt.plot(hero_t, hero_s, 'm-.', label='Original ST (HeroPlanner)')
    else:
        print("[WARN] No trajectory velocity data to plot HeroPlanner ST curve.")

    plt.xlabel("Time (s)")
    plt.ylabel("Path Position (s)")
    plt.title("ST Graph with Obstacles")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()



def main():
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    recorder_path = "2025-04-22-19-58-17.log"
    frame_id = 580

    planner = HeroPlanner(client, recorder_path, frame_id)
    apf, trajectory, trajectory_velocity = planner.plan()

    sp, times, v_vals_hero, smoothed_t, v_vals_sp, dp_profile = run_speed_planner(
        trajectory, trajectory_velocity, planner.surrounding_vehicles, ego=planner.ego_data,
    )
    plot_st_graph(sp, dp_profile, smoothed_t, trajectory_velocity)

    if dp_profile is None:
        return

    plot_velocity_comparison(times, v_vals_hero, smoothed_t, v_vals_sp)


if __name__ == '__main__':
    main()
