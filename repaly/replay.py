import glob
import json
import math
import os
import re
import sys
import traceback

import numpy as np
from scipy.interpolate import interp1d

from myDSL.ImportantFinder import ImportantFinder, parse_data
from myDSL.Obstacle import Obstacle
from myDSL.RecordDealer import HeroPlanner, plot_trajectory_with_obstacles, extract_collision_frame_and_type
from myDSL.SpeedPlanner import run_speed_planner

VIDEO_DIR = "/home/linshenghao/drivefuzz/save_autoware_6_20/replay"
try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def analyze_collision_and_replay(client, recorder_path, out_dir, log_file):
    try:
        recorder_info = client.show_recorder_file_info(recorder_path, True)
        frame_id, collision_vid, type_id = extract_collision_frame_and_type(recorder_info)
    except ValueError as e:
        msg = str(e)
        # log_file.write(f"[SKIP] {recorder_path} - Reason: {msg}\n")
        print(f"[INFO] Skipping {recorder_path} due to: {msg}")
        return {"collision": False, "replay_result": None, "error_type": msg}

    try:
        if type_id.startswith("vehicle.") or type_id.startswith("walker.pedestrian."):
            frame_id_ttc = ImportantFinder(client, recorder_path, mode=1)
        else:
            model_size_map = {}
            world = client.get_world()
            all_frame_data = parse_data(recorder_info, model_size_map, world)

            frame_obstacles = all_frame_data[frame_id]
            ego_obs = frame_obstacles.get("ego")

            ego_x = ego_obs.x
            ego_y = ego_obs.y
            ego_yaw_rad = math.atan2(ego_obs.hy, ego_obs.hx)
            ego_length = ego_obs.length

            obs_length = 4
            obs_width = 2.5
            distance = (ego_length + obs_length) / 2.0

            obs_x = ego_x - distance * math.cos(ego_yaw_rad)
            obs_y = ego_y - distance * math.sin(ego_yaw_rad)
            obs_yaw_rad = ego_yaw_rad + math.pi
            hx, hy = math.cos(obs_yaw_rad), math.sin(obs_yaw_rad)

            static_obs = Obstacle(
                x=obs_x, y=obs_y, vx=0.0, vy=0.0,
                hx=hx, hy=hy, length=obs_length, width=obs_width
            )
            frame_id_ttc = ImportantFinder(client, recorder_path, mode=4, static_obs=static_obs)

        important_frame_id = frame_id_ttc[0]

        pic_save_dir = os.path.join(out_dir, "replay_pic")
        os.makedirs(pic_save_dir, exist_ok=True)
        pic_name = os.path.splitext(os.path.basename(recorder_path))[0] + ".png"
        pic_save_path = os.path.join(pic_save_dir, pic_name)
        result = replay_test(client, important_frame_id, recorder_path, pic_save_path)
        return {"replay_result": result, "error_type": None}
    except Exception as e:
        err_msg = f"Replay failed: {e}"
        traceback.print_exc(file=sys.stdout)
        # log_file.write(f"[ERROR] {recorder_path} - {err_msg}\n")
        return {"replay_result": None, "error_type": "Replay failed"}


def init_simulation(client, recorder_path: str, frame_id: int):
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


def plan_trajectory(client, recorder_path, frame_id, pic_save_path=None):
    planner = HeroPlanner(client, recorder_path, frame_id)
    apf, trajectory, trajectory_velocity = planner.plan()
    if pic_save_path:
        plot_trajectory_with_obstacles(
            trajectory=trajectory,
            planner=planner,
            apf=apf,
            frame_id=frame_id,
            save_path=pic_save_path,
        )
    sp, times, v_vals_hero, smoothed_t, v_vals_sp, dp_profile = run_speed_planner(
        trajectory, trajectory_velocity,
        planner.surrounding_vehicles,
        ego=planner.ego_data
    )

    if v_vals_hero is None:
        raise RuntimeError("[ERROR] Speed planning failed.")

    return trajectory, v_vals_hero


def replay_test(client, frame_id, recorder_path, pic_save_path):
    sim = init_simulation(client, recorder_path, frame_id)
    trajectory, v_vals_hero = plan_trajectory(sim["client"], sim["recorder_path"], sim["frame_id"], pic_save_path)
    sim["trajectory"] = trajectory
    sim["v_vals_hero"] = v_vals_hero
    result = run_simulation(sim)
    return result["other_actor"]


def run_simulation(sim, debug=False):
    import os, time, math
    client = sim["client"]
    world = sim["world"]
    trajectory = sim["trajectory"]
    v_vals_hero = sim["v_vals_hero"]
    fixed_delta_seconds = sim["fixed_delta_seconds"]
    frame_id = sim["frame_id"]
    duration = sim["duration"]
    recorder_path = sim["recorder_path"]

    # frame_count = int((duration - frame_id * fixed_delta_seconds) / fixed_delta_seconds)
    frame_count = int(duration / fixed_delta_seconds)
    time_start = frame_id * fixed_delta_seconds

    preload_time = 2.0  # Seconds to preload replay before ego spawn
    start_replay_time = max(0.0, time_start - preload_time)
    preload_ticks = int(preload_time / fixed_delta_seconds)

    # Reload the world and wait briefly
    world_reload(client, debug)
    time.sleep(1)

    # Enable synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = fixed_delta_seconds
    world.apply_settings(settings)

    # Start replay from an earlier point to preload other actors
    client.replay_file(recorder_path,
                       start_replay_time,
                       duration + 2 * preload_time,
                       0)

    # Tick preload ticks to load replay actors
    for _ in range(preload_ticks):
        world.tick()
        time.sleep(fixed_delta_seconds)

    # Remove any replay-spawned hero vehicle
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get("role_name", "") in ["hero", "ego_vehicle"]:
            # print(f"[INFO] Destroying replay hero vehicle: id={actor.id}")
            actor.destroy()

    for _ in range(5):
        world.tick()
        time.sleep(fixed_delta_seconds)

    # Spawn ego vehicle at desired trajectory start point
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    start_x, start_y = trajectory[0]
    next_x, next_y = trajectory[2]
    yaw_deg = math.degrees(math.atan2(next_y - start_y, next_x - start_x))

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

    # Create and attach RGB top-down camera
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute("image_size_x", "800")
    camera_bp.set_attribute("image_size_y", "600")
    camera_bp.set_attribute("fov", "105")
    camera_bp.set_attribute("sensor_tick", str(fixed_delta_seconds))

    image_dir = f"/tmp/fuzzerdata/{os.getlogin()}/top"
    os.makedirs(image_dir, exist_ok=True)
    frame_counter = {"count": 0}
    camera_tf2 = carla.Transform(carla.Location(z=50.0), carla.Rotation(pitch=-90.0))

    def save_camera_image(image):
        filename = f"{image_dir}/top-replay-{frame_counter['count']:06d}.jpg"
        image.save_to_disk(filename)
        frame_counter["count"] += 1

    camera = world.spawn_actor(camera_bp, camera_tf2, attach_to=hero_vehicle)
    camera.listen(save_camera_image)

    # Tick several times to settle ego position
    for _ in range(5):
        world.tick()
        time.sleep(fixed_delta_seconds)
        hero_vehicle.set_transform(spawn_point)
        update_spectator(world, hero_vehicle)

    # Create collision sensor and attach to ego
    collision_bp = blueprint_library.find('sensor.other.collision')
    collision_transform = carla.Transform(carla.Location(x=0, y=0, z=1.0))
    collision_sensor = world.spawn_actor(collision_bp, collision_transform, attach_to=hero_vehicle)

    collision_result = {"frame": None, "other_actor": None}

    # Collision event callback
    def on_collision(event):
        type_id = event.other_actor.type_id.lower()
        if type_id.startswith("vehicle.") or type_id.startswith("walker.pedestrian."):
            if event.frame <= frame_id + frame_count:
                if debug:
                    print(f"[COLLISION] Ego collided with {type_id} at frame {event.frame}.")
                collision_result["frame"] = event.frame
                collision_result["other_actor"] = type_id

    collision_sensor.listen(on_collision)

    # Main simulation loop with trajectory + velocity control
    try:
        for i in range(len(trajectory)):
            v_target = v_vals_hero[i]
            set_vehicle_pose_and_speed(hero_vehicle, trajectory, v_target, carla_map, i, debug)
            update_spectator(world, hero_vehicle)
            time.sleep(fixed_delta_seconds)
            world.tick()
    finally:
        # Clean up settings and sensors
        settings.synchronous_mode = False
        world.apply_settings(settings)

        if collision_sensor:
            collision_sensor.stop()
            collision_sensor.destroy()

        # print("[INFO] Simulation finished. beginning to save replay video...")
        save_replay_video(sim["recorder_path"], VIDEO_DIR, camera_names=("top",))

        if debug:
            print("[INFO] Finished replay. Synchronous mode disabled.")

        return collision_result


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
    # if debug:
    #     print(
    #         f"[POSE] Set transform=({trajectory[frame_idx][0]:.2f}, {trajectory[frame_idx][0]:.2f}, {z_ground:.2f}) m, yaw={yaw:.2f} degrees")

    vx = float(target_speed * math.cos(math.radians(yaw)))
    vy = float(target_speed * math.sin(math.radians(yaw)))
    velocity = carla.Vector3D(vx, vy, 0.0)
    vehicle.set_target_velocity(velocity)
    # if debug:
    #     print(f"[VEL ] Set velocity=({vx:.2f}, {vy:.2f}) m/s")


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
    # if debug:
    #     print(f"[INFO] Trajectory and speed cut to {min_len} frames for Carla replay.")
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


def save_replay_video(log_filename, output_dir, camera_names=("top",), fps=20):
    username = os.getlogin()
    base_name = os.path.splitext(os.path.basename(log_filename))[0]
    for cam in camera_names:
        jpg_pattern = f"/tmp/fuzzerdata/{username}/top/{cam}-replay-*.jpg"
        mp4_path = os.path.join(output_dir, f"{base_name}-{cam}.mp4")

        # print(f"[VIDEO] Saving {cam} camera video to {mp4_path}")
        if os.path.exists(mp4_path):
            os.remove(mp4_path)

        cmd_cat = f"cat {jpg_pattern}"
        cmd_ffmpeg = " ".join([
            "ffmpeg",
            "-f image2pipe",
            f"-r {fps}",
            "-vcodec mjpeg",
            "-i -",
            "-vcodec libx264",
            "-crf 10",
            mp4_path
        ])
        full_cmd = f"{cmd_cat} | {cmd_ffmpeg} > /dev/null 2>&1"
        os.system(full_cmd)
        os.system(f"rm -f {jpg_pattern}")


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


def main():
    try:
        recorder_path = sys.argv[1]
        out_dir = sys.argv[2]
        result_path = os.path.join(out_dir, "replay_result.json")

        client = carla.Client('localhost', 4000)
        client.set_timeout(10.0)

        class DummyLogger:
            def write(self, msg):
                print(msg.strip())

        dummy_log = DummyLogger()
        result = analyze_collision_and_replay(client, recorder_path, out_dir, dummy_log)

        with open(result_path, "w") as f:
            json.dump(result, f)

        sys.exit(0)

    except Exception as e:
        err_result = {
            "replay_result": False,
            "error_type": f"Subprocess crash: {str(e)}"
        }

        result_path = os.path.join(sys.argv[2], "replay_result.json")
        with open(result_path, "w") as f:
            json.dump(err_result, f)

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
