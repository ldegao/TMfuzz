import glob
import os
import sys
import math
import re
import numpy as np
from scipy.interpolate import interp1d

try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from myDSL.RecordDealer import HeroPlanner
from myDSL.SpeedPlanner import run_speed_planner

import math
import carla


def set_vehicle_pose_and_speed(vehicle, target_position, next_position, target_speed):
    """
    Set vehicle transform and velocity directly.
    - target_position and next_position are numpy arrays: [x, y, (z)]
    - target_speed may be numpy type ? convert to float
    """
    target_speed = float(np.asarray(target_speed).flatten()[0])  # float

    # Calculate heading direction from current to next point
    dx = next_position[0] - target_position[0]
    dy = next_position[1] - target_position[1]
    if dx == 0 and dy == 0:
        yaw = vehicle.get_transform().rotation.yaw
    else:
        yaw = math.degrees(math.atan2(dy, dx))

    # Set vehicle transform
    transform = vehicle.get_transform()
    transform.location = carla.Location(
        x=target_position[0],
        y=target_position[1],
        z=transform.location.z
    )
    transform.rotation.yaw = yaw
    transform = carla.Transform(
        carla.Location(x=transform.location.x, y=transform.location.y, z=transform.location.z),
        carla.Rotation(yaw=yaw, pitch=0.0, roll=0.0)
    )
    vehicle.set_transform(transform)

    # Set vehicle velocity
    vx = float(target_speed * math.cos(math.radians(yaw)))
    vy = float(target_speed * math.sin(math.radians(yaw)))
    velocity = carla.Vector3D(vx, vy, 0.0)
    print(f"[INFO] Vehicle velocity set to: ({vx:.2f}, {vy:.2f}, 0.0) m/s")
    vehicle.set_target_velocity(velocity)

    print(f"[INFO] Vehicle set to pos: ({target_position[0]:.2f}, {target_position[1]:.2f}),{target_position[2]:.2f}) ")


def update_spectator(world, hero_vehicle):
    spectator = world.get_spectator()
    hero_transform = hero_vehicle.get_transform()

    distance_behind = 8.0
    height = 3.0
    angle_rad = math.radians(hero_transform.rotation.yaw)

    offset_x = -distance_behind * math.cos(angle_rad)
    offset_y = -distance_behind * math.sin(angle_rad)

    camera_location = carla.Location(
        x=hero_transform.location.x + offset_x,
        y=hero_transform.location.y + offset_y,
        z=hero_transform.location.z + height
    )

    spectator_transform = carla.Transform(
        camera_location,
        carla.Rotation(
            pitch=-10.0,
            yaw=hero_transform.rotation.yaw,
            roll=0.0
        )
    )

    spectator.set_transform(spectator_transform)


def compute_control(hero_vehicle, target_xy, next_target_xy, target_speed,
                    k=0.5, k_p=0.5, k_i=0.05, k_d=0.1, dt=0.05):
    """
    Stanley Controller + PID longitudinal controller + brake logic.
    Uses industrial standard heading error calculation (vehicle heading vs path tangent).

    Args:
        hero_vehicle: Carla vehicle actor.
        target_xy: (x, y) current target point on path.
        next_target_xy: (x, y) next point on path for heading calculation.
        target_speed: desired speed at this point (m/s).
        k: Stanley gain for cross-track error.
        k_p, k_i, k_d: PID controller gains.
        dt: time step for PID update (recommended = world.fixed_delta_seconds).
    Returns:
        carla.VehicleControl object.
    """

    transform = hero_vehicle.get_transform()
    location = transform.location
    yaw_vehicle = np.radians(transform.rotation.yaw)

    velocity = hero_vehicle.get_velocity()
    current_speed = np.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    # ----- Lateral control: Stanley -----
    # 1. heading error: vehicle heading vs path tangent
    dx_path = next_target_xy[0] - target_xy[0]
    dy_path = next_target_xy[1] - target_xy[1]
    path_heading = np.arctan2(dy_path, dx_path)

    heading_error = path_heading - yaw_vehicle
    heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

    # 2. cross-track error
    dx = target_xy[0] - location.x
    dy = target_xy[1] - location.y
    cte = -np.sin(path_heading) * dx + np.cos(path_heading) * dy  # signed distance to path
    effective_speed = max(current_speed, 2.0)
    steer_correction = np.arctan2(k * cte, effective_speed + 1e-5)
    steer = heading_error + steer_correction

    # ----- Longitudinal control: PID with brake -----
    if not hasattr(compute_control, "integral"):
        compute_control.integral = 0.0
        compute_control.prev_error = 0.0

    error = target_speed - current_speed
    compute_control.integral += error * dt
    derivative = (error - compute_control.prev_error) / dt
    compute_control.prev_error = error

    acc_cmd = k_p * error + k_i * compute_control.integral + k_d * derivative

    if acc_cmd >= 0:
        throttle = np.clip(acc_cmd / 3.5, 0.0, 1.0)  # assume max acc ~3.5 m/s²
        brake = 0.0
    else:
        throttle = 0.0
        brake = np.clip(-acc_cmd / 5.0, 0.0, 1.0)  # assume max dec ~5.0 m/s²

    control = carla.VehicleControl()
    control.steer = float(steer)
    control.throttle = float(throttle)
    control.brake = float(brake)
    print(f"[INFO] Control: steer={control.steer:.2f}, throttle={control.throttle:.2f}, brake={control.brake:.2f}")

    return control


def cut_trajectory_and_speed(trajectory, speed_profile, frame_count):
    """
    Cut trajectory and speed profile to match Carla tick frame count (no interpolation).
    trajectory: list or array of shape (N, 2) or (N, 3)
    speed_profile: list or array of shape (N,), already matched to trajectory
    """
    trajectory = np.array(trajectory)
    speed_profile = np.array(speed_profile)

    # Check valid size
    min_len = min(len(trajectory), len(speed_profile), frame_count)

    # Cut both to min_len
    cut_trajectory = trajectory[:min_len]
    cut_speed = speed_profile[:min_len]

    print(f"[INFO] Trajectory and speed cut to {min_len} frames for Carla replay.")
    return cut_trajectory, cut_speed


def interpolate_trajectory_and_speed(trajectory, dp_profile, frame_count):
    """
    Interpolate trajectory points and speed profile to match Carla tick frame count
    """
    trajectory = np.array(trajectory)
    x = trajectory[:, 0]
    y = trajectory[:, 1]

    s_vals = [0.0]
    for i in range(1, len(trajectory)):
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        s_vals.append(s_vals[-1] + np.hypot(dx, dy))
    s_vals = np.array(s_vals)

    s_target = np.linspace(0, s_vals[-1], frame_count)
    x_interp = interp1d(s_vals, x, kind='linear')(s_target)
    y_interp = interp1d(s_vals, y, kind='linear')(s_target)
    interpolated_trajectory = np.stack([x_interp, y_interp], axis=1)

    dp_times = [pt[0] for pt in dp_profile]
    dp_s = [pt[1] for pt in dp_profile]
    v_vals_sp = [0.0]
    for i in range(1, len(dp_profile)):
        ds = dp_s[i] - dp_s[i - 1]
        dt = dp_times[i] - dp_times[i - 1]
        v_vals_sp.append(ds / (dt + 1e-6))
    v_vals_sp = np.array(v_vals_sp)

    v_interp = interp1d(dp_s, v_vals_sp, kind='linear', fill_value="extrapolate")(s_target)

    return interpolated_trajectory, v_interp


def main():
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()

    recorder_path = "2025-04-22-19-58-17.log"
    frame_id = 500

    planner = HeroPlanner(client, recorder_path, frame_id)
    apf, trajectory, trajectory_velocity = planner.plan()

    sp, times, v_vals_hero, smoothed_t, v_vals_sp, dp_profile = run_speed_planner(
        trajectory, trajectory_velocity,
        planner.surrounding_vehicles,
        ego=planner.ego_data
    )

    if v_vals_hero is None:
        print("[WARN] Speed planning failed. Exiting.")
        return

    info = client.show_recorder_file_info(recorder_path, False)

    match_frames = re.search(r'Frames:\s+(\d+)', info)
    match_duration = re.search(r'Duration:\s+([0-9.]+)', info)
    frames = int(match_frames.group(1))
    duration = float(match_duration.group(1))
    fps = frames / duration if duration > 0 else 20.0
    fixed_delta_seconds = 1.0 / fps

    time_start = frame_id * fixed_delta_seconds
    frame_count = int((duration - time_start) / fixed_delta_seconds)

    print(f"[INFO] Replay FPS: {fps} ? fixed_delta_seconds: {fixed_delta_seconds}")
    print(f"[INFO] Total frames to control: {frame_count}")

    # interpolated_trajectory, interpolated_speeds = interpolate_trajectory_and_speed(
    #     trajectory, dp_profile, frame_count
    # )
    interpolated_trajectory, interpolated_speeds = cut_trajectory_and_speed(
        trajectory, v_vals_hero, frame_count
    )

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = fixed_delta_seconds
    world.apply_settings(settings)

    try:
        client.set_replayer_ignore_hero(True)
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

        start_x, start_y = interpolated_trajectory[0]
        next_x, next_y = interpolated_trajectory[2]

        dx = next_x - start_x
        dy = next_y - start_y
        yaw_rad = math.atan2(dy, dx)
        yaw_deg = math.degrees(yaw_rad)

        spawn_point = carla.Transform(
            carla.Location(x=start_x, y=start_y, z=0.5),
            carla.Rotation(yaw=yaw_deg)
        )

        hero_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        hero_vehicle.set_autopilot(False)
        hero_id = hero_vehicle.id
        update_spectator(world, hero_vehicle)
        for _ in range(50):
            world.tick()

        print(f"[INFO] Starting replay from t={time_start} s for {duration - time_start} s. Hero ID: {hero_id}")

        client.replay_file(recorder_path, time_start, duration - time_start, 0)

        # Tick loop using interpolated trajectory
        for i in range(frame_count):
            target = interpolated_trajectory[i]
            next_target = interpolated_trajectory[i + 1] if i + 1 < frame_count else target
            v_target = interpolated_speeds[i]

            set_vehicle_pose_and_speed(hero_vehicle, target, next_target, v_target)

            update_spectator(world, hero_vehicle)  # Update spectator view
            world.tick()  # Advance simulation

    finally:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[INFO] Finished replay. Synchronous mode disabled.")


if __name__ == '__main__':
    main()
