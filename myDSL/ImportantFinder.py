import glob
import math
import os
import random
import re
import sys
import time

if __name__ == "__main__" and __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    __package__ = "myDSL"

try:
    sys.path.append(glob.glob('../../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

from myDSL.Obstacle import Obstacle


def get_actor_alive_status(recorder_info):
    creation_frame = {}
    destroy_frame = {}

    frame_positions = []
    for frame_match in re.finditer(r"Frame\s+(\d+)\s+at\s+([0-9\.]+)\s*seconds", recorder_info):
        frame_pos = frame_match.start()
        frame_id = int(frame_match.group(1))
        frame_positions.append((frame_pos, frame_id))
    frame_positions.sort()

    for match in re.finditer(r"(Create|Destroy)\s+(\d+):?", recorder_info):
        action = match.group(1)
        actor_id = int(match.group(2))
        match_pos = match.start()

        frame_id = None
        for i in range(len(frame_positions)):
            if frame_positions[i][0] > match_pos:
                break
            frame_id = frame_positions[i][1]

        if frame_id is None:
            continue

        if action == "Create":
            if actor_id not in creation_frame:
                creation_frame[actor_id] = frame_id
        elif action == "Destroy":
            destroy_frame[actor_id] = frame_id

    return creation_frame, destroy_frame, [fid for _, fid in frame_positions]


def extract_car_data(recorder_info):
    car_data = {}
    blocks = recorder_info.split("Create ")
    for block in blocks[1:]:
        lines = block.strip().splitlines()
        header = lines[0] if lines else ""
        header_match = re.match(r"(\d+): ([^\s]+)", header)
        if not header_match:
            continue
        vid = int(header_match.group(1))
        vehicle_model = header_match.group(2)
        role_match = re.search(r"role_name\s*=\s*([^\n]+)", block)
        if not role_match:
            continue
        role_name = role_match.group(1).strip()
        if vehicle_model is None or not vehicle_model.startswith("vehicle."):
            continue
        car_data[vid] = {
            'vehicle_model': vehicle_model,
            'role_name': role_name
        }
    return car_data


def parse_data(recorder_info, model_size_map, world):
    creation_frame, destroy_frame, frame_ids = get_actor_alive_status(recorder_info)
    car_data = extract_car_data(recorder_info)
    all_frame_data = {}

    for frame_id in frame_ids:
        frame_pattern = rf"(Frame {frame_id} at .*?)(?=Frame \d+ at |\Z)"
        frame_match = re.search(frame_pattern, recorder_info, re.DOTALL)
        frame_str = frame_match.group(0) if frame_match else ''
        static_pattern = r"Id:\s*(\d+)\s+Location:\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)\s*Rotation\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)"
        static_matches = re.findall(static_pattern, frame_str)

        static_data = {}
        for match in static_matches:
            vid = int(match[0])
            static_data[vid] = {
                'id': vid,
                'location': tuple(map(float, match[1:4])),
                'rotation': tuple(map(float, match[4:7]))
            }

        dynamic_pattern = r"Id:\s*(\d+)\s*linear_velocity:\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)\s*angular_velocity:\s*\(([^,]+),\s*([^,]+),\s*([^\)]+)\)"
        dynamic_matches = re.findall(dynamic_pattern, frame_str)
        for match in dynamic_matches:
            vid = int(match[0])
            if vid in static_data:
                static_data[vid].update({
                    'linear_velocity': tuple(map(float, match[1:4])),
                    'angular_velocity': tuple(map(float, match[4:7]))
                })

        frame_obstacles = {}
        obs_count = 1

        for vid, vdata in static_data.items():
            created = creation_frame.get(vid, 0)
            destroyed = destroy_frame.get(vid, float('inf'))
            if created > frame_id or destroyed <= frame_id:
                continue

            vehicle_model = car_data.get(vid, {}).get('vehicle_model', None)
            if vehicle_model is None or not vehicle_model.startswith("vehicle."):
                continue
            if vehicle_model in model_size_map:
                length, width = model_size_map[vehicle_model]
            else:
                length, width = 4.5, 2.0
                if vehicle_model:
                    bp = world.get_blueprint_library().find(vehicle_model)
                    spawn_points = world.get_map().get_spawn_points()
                    temp_actor = None
                    for _ in range(10):
                        spawn_point = random.choice(spawn_points)
                        time.sleep(0.1)
                        temp_actor = world.try_spawn_actor(bp, spawn_point)
                        if temp_actor:
                            break
                    if temp_actor:
                        time.sleep(0.2)
                        extent = temp_actor.bounding_box.extent
                        length = extent.x * 2
                        width = extent.y * 2
                        model_size_map[vehicle_model] = (length, width)
                        temp_actor.destroy()
                        time.sleep(0.1)

            yaw_rad = math.radians(vdata['rotation'][2])
            hx, hy = math.cos(yaw_rad), math.sin(yaw_rad)
            obs = Obstacle(
                x=vdata['location'][0] / 100,
                y=vdata['location'][1] / 100,
                vx=vdata.get('linear_velocity', (0, 0, 0))[0],
                vy=vdata.get('linear_velocity', (0, 0, 0))[1],
                hx=hx,
                hy=hy,
                length=length,
                width=width
            )
            role_name = car_data.get(vid, {}).get('role_name', '')
            if role_name == 'hero' or role_name == 'ego_vehicle':
                frame_obstacles['ego'] = obs
            else:
                frame_obstacles[f'obs_{obs_count}'] = obs
                obs_count += 1

        all_frame_data[frame_id] = frame_obstacles

    return all_frame_data


def compute_ttc_series(all_frame_data, ego_selector, obs_selector):
    """
    Generic TTC computation between selected ego and obstacle for each frame.

    Parameters:
        all_frame_data (dict): {frame_id: {'ego': Obstacle, 'obs_*': Obstacle, ...}}
        ego_selector (func): Given agents dict, returns ego Obstacle
        obs_selector (func): Given agents dict, returns iterable of obstacles to test

    Returns:
        List of tuples: [(frame_id, TTC value), ...]
    """
    ttc_results = []

    for frame_id, agents in all_frame_data.items():
        ego = ego_selector(agents)
        if ego is None:
            continue

        try:
            ttcs = [obs.compute_ttc(ego) for obs in obs_selector(agents)]
            ttcs = [t for t in ttcs if t >= 0]
            if ttcs:
                ttc_results.append((frame_id, min(ttcs)))
                # print(f"[INFO] Frame {frame_id}: TTC = {min(ttcs)}")
        except Exception as e:
            print(f"[WARN] TTC failed at frame {frame_id}: {e}")
            continue

    return ttc_results


def compute_all_ttc(all_frame_data):
    return compute_ttc_series(
        all_frame_data,
        ego_selector=lambda agents: agents.get('ego', None),
        obs_selector=lambda agents: [obs for key, obs in agents.items() if key != 'ego']
    )


def compute_ttc_with_obs(all_frame_data, obs):
    return compute_ttc_series(
        all_frame_data,
        ego_selector=lambda agents: agents.get('ego', None),
        obs_selector=lambda agents: [obs]
    )


def get_important_frame(ttc_results, mode, x, t, fixed_delta_seconds):
    """
    Find the most important frame based on TTC result list and selection mode.

    Parameters:
        ttc_results: List of (frame_id, ttc_value)
        mode:
            1 - First frame with TTC < x
            2 - Last TTC before crash (TTC == -1), going back t seconds
            3 - The first TTC in a continuous segment before crash where TTC < x
        x: Threshold TTC value (float)
        t: Time before crash in seconds (for mode 2 and 3)
        fixed_delta_seconds: Time per frame

    Returns:
        Tuple: (frame_id, ttc_value)
    """
    if not ttc_results:
        return None

    if mode == 1:
        for frame_id, ttc in ttc_results:
            if 0 < ttc < x:
                return (frame_id, ttc)
        return None

    if mode in [2, 3]:
        crash_index = next((i for i, (_, ttc) in enumerate(ttc_results) if ttc == -1), None)
        if crash_index is None:
            return None  # No crash found

        num_frames_back = int(t / fixed_delta_seconds)
        start_index = max(0, crash_index - num_frames_back)
        pre_crash_segment = ttc_results[start_index:crash_index]

        if mode == 2:
            return pre_crash_segment[0] if pre_crash_segment else None

        if mode == 3:
            segment = []
            for frame_id, ttc in pre_crash_segment:
                if 0 < ttc < x:
                    if not segment or segment[-1][0] + 1 == frame_id:
                        segment.append((frame_id, ttc))
                    else:
                        segment = [(frame_id, ttc)]
                elif segment:
                    break
            return segment[0] if segment else ttc_results[0]

    return None


def ImportantFinder(client, recorder_path, mode=1, x=2.0, t=3.0, static_obs=None):
    """
    Analyze a Carla recorder log and return the most important frame based on TTC criteria.

    Parameters:
        client (carla.Client): Carla client instance.
        recorder_path (str): Path to the Carla .log file.
        mode (int): Mode for selecting important frame:
            1 - First TTC < x
            2 - Frame t seconds before crash
            3 - Start of TTC < x segment before crash
            4 - Use given static_obs to compute TTC (no crash)
        x (float): TTC threshold for filtering
        t (float): Time window for pre-crash analysis
        static_obs (Obstacle): Optional static obstacle to compare with ego (only for mode 4)

    Returns:
        tuple: (frame_id, ttc_value)
    """
    recorder_info = client.show_recorder_file_info(recorder_path, True)
    match_frames = re.search(r'Frames:\s+(\d+)', recorder_info)
    match_duration = re.search(r'Duration:\s+([0-9.]+)', recorder_info)
    frames = int(match_frames.group(1))
    duration = float(match_duration.group(1))
    fps = frames / duration if duration > 0 else 20.0
    fixed_delta_seconds = 1.0 / fps

    model_size_map = {}
    world = client.get_world()
    all_frame_data = parse_data(recorder_info, model_size_map, world)
    sorted_keys = sorted(all_frame_data.keys())
    last_keys = sorted_keys[-200:]
    frame_data = {k: all_frame_data[k] for k in last_keys}

    if mode == 4 and static_obs is not None:
        ttc_results = compute_ttc_with_obs(frame_data, static_obs)
        important_frame = get_important_frame(
            ttc_results, mode=2, x=x, t=t, fixed_delta_seconds=fixed_delta_seconds
        )
    else:
        ttc_results = compute_all_ttc(frame_data)
        important_frame = get_important_frame(
            ttc_results, mode=mode, x=x, t=t, fixed_delta_seconds=fixed_delta_seconds
        )

    return important_frame


def actor_to_obstacle(actor):
    location = actor.get_location()
    rotation = actor.get_transform().rotation
    extent = actor.bounding_box.extent

    return Obstacle(
        x=location.x / 100.0,
        y=location.y / 100.0,
        vx=0.0,
        vy=0.0,
        hx=math.cos(math.radians(rotation.yaw)),
        hy=math.sin(math.radians(rotation.yaw)),
        length=extent.x * 2,
        width=extent.y * 2
    )


if __name__ == "__main__":
    # Connect to Carla
    client = carla.Client("localhost", 4000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Pick a static prop blueprint
    static_bp = blueprint_library.find("static.prop.streetbarrier")

    # Choose a spawn point
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)
    spawn_point.location.z += 0.5  # Slightly lift to avoid ground clipping

    # Spawn the static obstacle actor
    static_actor = world.try_spawn_actor(static_bp, spawn_point)
    if static_actor is None:
        print("[-] Failed to spawn static actor.")
        sys.exit(1)
    print(f"[INFO] Spawned static actor: {static_actor.type_id} (id={static_actor.id})")

    # Convert to Obstacle
    static_obs = actor_to_obstacle(static_actor)

    # Run TTC-based important frame search
    recorder_path = "2025-04-22-19-58-17.log"
    important_frame = ImportantFinder(
        client,
        recorder_path,
        mode=4,
        x=2.0,
        t=2.0,
        static_obs=static_obs
    )

    print(f"[RESULT] Important frame (static TTC): {important_frame}")

    # Optionally destroy the actor to clean up
    static_actor.destroy()
