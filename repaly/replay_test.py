import glob
import os
import sys
import math
import re
import time

import numpy as np
from scipy.interpolate import interp1d

try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

from myDSL.RecordDealer import HeroPlanner
from myDSL.SpeedPlanner import run_speed_planner

import math
import carla


def set_vehicle_pose_and_speed(vehicle, trajectory, target_speed, carla_map, frame_idx, debug=False):
    target_speed = float(np.asarray(target_speed).flatten()[0])

    if frame_idx + 1 < len(trajectory):
        dx = trajectory[frame_idx + 1][0] - trajectory[frame_idx][0]
        dy = trajectory[frame_idx + 1][1] - trajectory[frame_idx][1]
    else:
        dx = trajectory[frame_idx][0] - trajectory[frame_idx - 1][0]
        dy = trajectory[frame_idx][1] - trajectory[frame_idx - 1][1]
    yaw = math.degrees(math.atan2(dy, dx)) if dx or dy else vehicle.get_transform().rotation.yaw

    waypoint = carla_map.get_waypoint(
        carla.Location(trajectory[frame_idx][0], trajectory[frame_idx][1], 0.0),
        project_to_road=True, lane_type=carla.LaneType.Driving
    )
    z_ground = waypoint.transform.location.z
    pitch = waypoint.transform.rotation.pitch
    roll = waypoint.transform.rotation.roll

    transform = carla.Transform(
        carla.Location(x=trajectory[frame_idx][0], y=trajectory[frame_idx][1], z=z_ground + 0.05),
        carla.Rotation(yaw=yaw, pitch=pitch, roll=roll)
    )
    vehicle.set_transform(transform)
    if debug:
        print(
            f"[POSE] Set transform=({trajectory[frame_idx][0]:.2f}, {trajectory[frame_idx][0]:.2f}, {z_ground:.2f}) m, yaw={yaw:.2f} degrees")

    vx = float(target_speed * math.cos(math.radians(yaw)))
    vy = float(target_speed * math.sin(math.radians(yaw)))
    velocity = carla.Vector3D(vx, vy, 0.0)
    vehicle.set_target_velocity(velocity)
    if debug:
        print(f"[VEL ] Set velocity=({vx:.2f}, {vy:.2f}) m/s")


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


def cut_trajectory_and_speed(trajectory, speed_profile, frame_count, debug=False):
    trajectory = np.array(trajectory)
    speed_profile = np.array(speed_profile)
    min_len = min(len(trajectory), len(speed_profile), frame_count)
    cut_trajectory = trajectory[:min_len]
    cut_speed = speed_profile[:min_len]
    if debug:
        print(f"[INFO] Trajectory and speed cut to {min_len} frames for Carla replay.")
    return cut_trajectory, cut_speed


def interpolate_trajectory_and_speed(trajectory, dp_profile, frame_count):
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


def world_reload(client, debug=False):
    world = client.get_world()
    actors = world.get_actors()
    for actor in actors:
        if not actor.is_alive:
            continue
        type_id = actor.type_id.lower()
        if 'vehicle.' in type_id or 'walker.pedestrian.' in type_id:
            try:
                actor.destroy()
            except Exception as e:
                if debug:
                    print(f"[WARN] Failed to destroy {type_id}: {e}")


def init_simulation(recorder_path: str, frame_id: int):
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    world = client.get_world()

    info = client.show_recorder_file_info(recorder_path, True)
    match_frames = re.search(r'Frames:\s+(\d+)', info)
    match_duration = re.search(r'Duration:\s+([0-9.]+)', info)
    frames = int(match_frames.group(1))
    duration = float(match_duration.group(1))
    fps = frames / duration if duration > 0 else 20.0
    fixed_delta_seconds = 1.0 / fps

    return {
        "client": client,
        "world": world,
        "fps": fps,
        "duration": duration,
        "frame_id": frame_id,
        "fixed_delta_seconds": fixed_delta_seconds,
        "recorder_path": recorder_path,
    }


def plan_trajectory(client, recorder_path, frame_id):
    planner = HeroPlanner(client, recorder_path, frame_id)
    apf, trajectory, trajectory_velocity = planner.plan()

    sp, times, v_vals_hero, smoothed_t, v_vals_sp, dp_profile = run_speed_planner(
        trajectory, trajectory_velocity,
        planner.surrounding_vehicles,
        ego=planner.ego_data
    )

    if v_vals_hero is None:
        raise RuntimeError("[ERROR] Speed planning failed.")

    return trajectory, v_vals_hero


def run_simulation(sim, debug=False):
    client = sim["client"]
    world = sim["world"]
    trajectory = sim["trajectory"]
    v_vals_hero = sim["v_vals_hero"]
    fixed_delta_seconds = sim["fixed_delta_seconds"]
    frame_id = sim["frame_id"]
    duration = sim["duration"]
    recorder_path = sim["recorder_path"]

    frame_count = int((duration - frame_id * fixed_delta_seconds) / fixed_delta_seconds)
    time_start = frame_id * fixed_delta_seconds

    world_reload(client, debug)
    time.sleep(1)
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    start_x, start_y = trajectory[0]
    next_x, next_y = trajectory[2]
    yaw_deg = math.degrees(math.atan2(next_y - start_x, next_x - start_x))

    waypoint = carla_map.get_waypoint(
        carla.Location(x=start_x, y=start_y, z=0.0),
        project_to_road=True, lane_type=carla.LaneType.Driving
    )
    spawn_point = carla.Transform(
        carla.Location(x=start_x, y=start_y, z=waypoint.transform.location.z + 0.5),
        carla.Rotation(yaw=yaw_deg)
    )

    hero_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    hero_vehicle.set_autopilot(False)
    hero_vehicle.set_simulate_physics()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = fixed_delta_seconds
    world.apply_settings(settings)
    client.set_replayer_ignore_hero(True)

    client.replay_file(recorder_path, time_start - 5 * fixed_delta_seconds,
                       duration - time_start + 5 * fixed_delta_seconds, 0)

    for _ in range(5):
        world.tick()
        time.sleep(fixed_delta_seconds)
        hero_vehicle.set_transform(spawn_point)
        update_spectator(world, hero_vehicle)

    collision_bp = blueprint_library.find('sensor.other.collision')
    collision_transform = carla.Transform(carla.Location(x=0, y=0, z=1.0))
    collision_sensor = world.spawn_actor(collision_bp, collision_transform, attach_to=hero_vehicle)

    collision_result = {"frame": None, "other_actor": None}

    def on_collision(event):
        type_id = event.other_actor.type_id.lower()
        if type_id.startswith("vehicle.") or type_id.startswith("walker.pedestrian."):
            if event.frame <= frame_id + frame_count:
                if debug:
                    print(f"[COLLISION] Ego collided with {type_id} at frame {event.frame}.")
                collision_result["frame"] = event.frame
                collision_result["other_actor"] = type_id

    collision_sensor.listen(on_collision)

    try:
        for i in range(len(trajectory)):
            v_target = v_vals_hero[i]
            set_vehicle_pose_and_speed(hero_vehicle, trajectory, v_target, carla_map, i, debug)
            update_spectator(world, hero_vehicle)
            time.sleep(fixed_delta_seconds)
            world.tick()
    finally:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        if collision_sensor:
            collision_sensor.stop()
            collision_sensor.destroy()
        if debug:
            print("[INFO] Finished replay. Synchronous mode disabled.")

    return collision_result["other_actor"]


if __name__ == '__main__':
    recorder_path = "2025-04-22-20-19-43.log"
    frame_id = 1600

    sim = init_simulation(recorder_path, frame_id)
    trajectory, v_vals_hero = plan_trajectory(sim["client"], sim["recorder_path"], sim["frame_id"])
    sim["trajectory"] = trajectory
    sim["v_vals_hero"] = v_vals_hero
    result = run_simulation(sim, debug=True)
    print(f"[RESULT] Collision with: {result if result else 'None'}")
