import json
import math
import os
import pdb
import traceback

import config

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("[-] Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("    Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)

def save_json_to_file(json_data, output_dir, generation_id, scenario_id):
    """
    Saves the given JSON data to a file in the specified directory.

    :param json_data: The JSON data to be saved
    :param output_dir: The directory where the file should be saved
    :param generation_id: The generation ID to be included in the file name
    :param scenario_id: The scenario ID to be included in the file name

    :return: The full path of the saved JSON file
    """
    try:
        # Ensure the output directory exists, create it if necessary
        output_dir = output_dir + "/queue/"
        os.makedirs(output_dir, exist_ok=True)

        # Define the output file path
        output_file = os.path.join(output_dir, f"SceneDSL_cid{0}:gid:{generation_id}_sid:{scenario_id}.json")
        print(output_file)

        # Serialize and save all accumulated JSON data to the file
        try:
            with open(output_file, "w") as f:
                json.dump(json_data, f, indent=4)
        except FileNotFoundError:
            print(f"Error: FileNotFoundError. Unable to open or create the file '{output_file}'")
            traceback.print_exc()
        except PermissionError:
            print(f"Error: PermissionError. No write permission for the file '{output_file}'")
            traceback.print_exc()
        except Exception as e:
            print(f"Error saving scene to '{output_file}': {e}")
            traceback.print_exc()

        return output_file  # ? ????
    except Exception as e:
        print(f"Error during file operations: {e}")
        traceback.print_exc()
        return None  # ?????????? None



def make_unit_vector(velocity):
    """
    Returns a unit vector in the direction of the given velocity vector.

    :param velocity: A carla.Vector3D object representing the velocity vector.
    :return: A carla.Vector3D object representing the unit vector in the direction of the input velocity.
    """
    # Calculate the magnitude of the velocity vector
    magnitude = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    if magnitude == 0:
        # Return a zero vector if the magnitude is zero
        return carla.Vector3D(x=0, y=0, z=0)
    # Return the unit vector in the direction of the input velocity
    return carla.Vector3D(x=velocity.x / magnitude, y=velocity.y / magnitude, z=velocity.z / magnitude)


def initialize_vehicle_from_json(json_data, actor):
    """
    Initializes the vehicle (NPC or player) from a JSON configuration.
    This function modifies the velocity direction while keeping the speed intact.

    :param json_data: JSON data containing the vehicle's transform, velocity, and control properties.
    :param actor: The carla.Actor representing the vehicle (either NPC or player).
    """
    # # 1. Initialize current Velocity (existing velocity of the actor)
    # current_velocity = actor.get_velocity()
    # vx = current_velocity.x
    # vy = current_velocity.y
    # vz = current_velocity.z
    # current_speed = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    #
    # # 2. Initialize new velocity data from JSON
    # new_velocity_data = json_data['velocity']
    # new_velocity = carla.Vector3D(x=new_velocity_data['x'], y=new_velocity_data['y'], z=new_velocity_data['z'])
    #
    # # 3. Calculate the new direction (normalize the new velocity)
    # new_velocity_direction = make_unit_vector(new_velocity)
    #
    # # 4. Adjust the velocity to keep the original speed but change direction
    # adjusted_velocity = new_velocity_direction * current_speed  # Keep original speed, change direction
    #
    # # 5. Apply the adjusted velocity to the actor (vehicle)
    # actor.set_target_velocity(adjusted_velocity)
    new_velocity_data = json_data['velocity']
    actor.set_target_velocity(
        carla.Vector3D(x=new_velocity_data['x'], y=new_velocity_data['y'], z=new_velocity_data['z']))
    print(f"[DEBUG] Vehicle velocity adjusted: Speed {new_velocity_data['x']:.2f} | Direction {new_velocity_data}")
    # 6. Initialize Control (Throttle, Steer, Brake, Hand Brake, Reverse, Gear)
    control = actor.get_control()
    control_data = json_data['control']
    control.throttle = control_data['throttle']
    control.steer = control_data['steer']
    control.brake = control_data['brake']
    control.hand_brake = control_data['hand_brake']
    control.reverse = control_data['reverse']
    control.gear = control_data['gear']
    # 7. Apply control to the vehicle
    actor.apply_control(control)

    # # Debug log for adjusted velocity
    # print(f"[DEBUG] Vehicle velocity adjusted: Speed {current_speed:.2f} | Direction {adjusted_velocity}")

def find_timestamp(
        file_path: str,
        key_to_check: str = "TTCToNPCs",
        exclude_values=None,
        find_first: bool = True,
        find_first_first: bool = True
) -> float:
    """
    Reads the JSON file and finds the first or last frame in a continuous cluster
    where the specified key's value is not in the exclude_values.
    Allows finding the first or last such frame based on `find_first` and `find_first_first`.

    Args:
        file_path (str): The path to the JSON file.
        key_to_check (str): The key to check in the data (default: "TTCToNPCs").
        exclude_values (list): A list of values to exclude (default: ["very long", "long"]).
        find_first (bool): If True, finds the first matching frame. If False, finds the last matching frame.
        find_first_first (bool): If True, returns the first frame of a continuous cluster of matching frames.
                                 If False, returns the last frame of the cluster.

    Returns:
        float: The "timestamp" of the matching frame, or -1 if no match is found.
    """
    # Ensure the file exists before trying to open it
    if exclude_values is None:
        exclude_values = ["very long", "long"]
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")

    with open(file_path, 'r') as f:
        data = json.load(f)

    # Reverse the data order if looking for the last matching frame
    data = data if find_first else list(reversed(data))

    # Initialize cluster state
    in_cluster = False
    cluster_start_index = None

    # Search for the matching frame
    for i, datastamp in enumerate(data):
        # Ensure timestamp exists
        timestamp = datastamp.get("timestamp", None)
        if timestamp is None:
            continue  # Skip frames without a timestamp

        ads_data = datastamp.get("scene", {}).get("ADS", {})
        key_value = ads_data.get(key_to_check, "")

        # Check if the current frame is in the valid range (not in excluded values)
        if key_value not in exclude_values:
            if not in_cluster:
                # Start of a new matching cluster
                in_cluster = True
                cluster_start_index = i
        else:
            if in_cluster:
                # We just encountered a break in the cluster
                if find_first_first:  # Return the first frame of the cluster
                    return data[cluster_start_index]["timestamp"]
                else:  # Return the last frame of the cluster
                    return data[i-1]["timestamp"]
                in_cluster = False  # Reset for the next cluster

    # Handle case where the cluster ends at the end of the list
    if in_cluster:
        if find_first_first:
            return data[cluster_start_index]["timestamp"]
        else:
            return data[-1]["timestamp"]

    # If no matching frame is found, return -1
    return -1


def euclidean_distance(p1, p2):
    return math.sqrt((p1["x"] - p2["x"]) ** 2 + (p1["y"] - p2["y"]) ** 2)


def initialize_spawned_vehicles(snap_info, actor_vehicles, player_vehicle):
    """
    Initializes spawned vehicles and player using JSON data from snap_info.

    Args:
        snap_info: A dictionary containing the JSON data for the dangerous scenario.
        actor_vehicles: A list of carla.Actor objects representing spawned vehicles.
        player_vehicle: A carla.Actor object representing the player vehicle
    """

    # Initialize NPC vehicles
    npc_list = snap_info.get("NPC", [])
    for vehicle in actor_vehicles:
        # Find matching NPC configuration by vehicle waypoint
        vehicle_sp = vehicle.get_transform().location
        npc_data = None
        for npc in npc_list:
            npc_location = npc.get("transform", {}).get("location", {})
            distance = euclidean_distance(npc_location, {"x": vehicle_sp.x, "y": vehicle_sp.y})
            print(f"Distance between vehicle SP {vehicle_sp} and NPC location:{npc_location}: {distance}")
            if distance < 1:
                npc_data = npc
                break

        if npc_data is None:
            print(f"[!] No JSON data found for vehicle SP {vehicle_sp}. Skipping initialization.")
            continue

        # Call initialize_vehicle_from_json to set properties
        try:
            initialize_vehicle_from_json(npc_data, vehicle)
            print(f"[+] Initialized vehicle SP {vehicle_sp} with JSON data.")
        except Exception as e:
            print(f"[-] Failed to initialize vehicle SP {vehicle_sp}: {e}")

    # Initialize Player vehicle
    player_data = snap_info.get("player")
    if player_vehicle and player_data:
        try:
            # Call initialize_vehicle_from_json for the player
            initialize_vehicle_from_json(player_data, player_vehicle)
            print(f"[+] Initialized Player vehicle ID {player_vehicle.id} with JSON data.")
        except Exception as e:
            print(f"[-] Failed to initialize Player vehicle ID {player_vehicle.id}: {e}")
    elif not player_vehicle:
        print("[!] Player vehicle is not provided. Skipping Player initialization.")
    elif not player_data:
        print("[!] No Player data found in snap_info. Skipping Player initialization.")


def update_seed_from_snap_info(test_scenario_m, town_map):
    """
    Updates seed_data starting point (sp_x, sp_y, sp_z, pitch, yaw, roll)
    based on snap_info['player'] transform and updates wp_x, wp_y, wp_z from snap_info_last['player'].

    Args:
        test_scenario_m: An object with `snap_info`,"snap_info_last" and `seed_data` attributes.
    """
    if not hasattr(test_scenario_m, 'snap_info') or not hasattr(test_scenario_m, 'seed_data'):
        raise AttributeError("test_scenario_m must have 'snap_info' and 'seed_data' attributes.")

    snap_info = test_scenario_m.snap_info
    seed_data = test_scenario_m.seed_data

    # Ensure snap_info contains 'player' key
    if "player" not in snap_info or not isinstance(snap_info["player"], dict):
        raise ValueError("snap_info does not contain valid 'player' information.")

    player_info = snap_info["player"]
    transform = player_info.get("transform", {})

    # Extract location and rotation from snap_info (for sp_x, sp_y, sp_z, pitch, yaw, roll)
    location = transform.get("location", {})
    rotation = transform.get("rotation", {})

    # Validate location and rotation fields
    if not all(k in location for k in ["x", "y", "z"]) or not all(k in rotation for k in ["pitch", "yaw", "roll"]):
        raise ValueError("Player transform is missing location or rotation data.")

    # Update seed_data starting point fields
    seed_data["sp_x"] = location["x"]
    seed_data["sp_y"] = location["y"]
    seed_data["sp_z"] = location["z"]
    seed_data["pitch"] = rotation["pitch"]
    seed_data["yaw"] = rotation["yaw"]
    seed_data["roll"] = rotation["roll"]
    snap_info_last = test_scenario_m.snap_info_last

    # Check if snap_info_last is provided and contains valid player info
    if snap_info_last is not None:
        if "player" not in snap_info_last or not isinstance(snap_info_last["player"], dict):
            raise ValueError("snap_info_last does not contain valid 'player' information.")

        player_info_last = snap_info_last["player"]
        transform_last = player_info_last.get("transform", {})

        # Extract location from snap_info_last (for wp_x, wp_y, wp_z)
        location_last = transform_last.get("location", {})
        # if location_last is to close to the player, we should Extend driving distance
        if euclidean_distance(location, location_last) < 20:
            waypoint_last = town_map.get_waypoint(
                carla.Location(location_last["x"], location_last["y"], location_last["z"])).transform
            waypoint_last_Extend = waypoint_last.next(10)[0].location
            location_last["x"] = waypoint_last_Extend.x
            location_last["y"] = waypoint_last_Extend.y
            location_last["z"] = waypoint_last_Extend.z
        # Validate location fields
        if not all(k in location_last for k in ["x", "y", "z"]):
            raise ValueError("Player transform in snap_info_last is missing location data.")

        # Update seed_data wp fields
        seed_data["wp_x"] = location_last["x"]
        seed_data["wp_y"] = location_last["y"]
        seed_data["wp_z"] = location_last["z"]

        print("Seed data wp updated successfully:")
        print(f"  wp_x: {seed_data['wp_x']}, wp_y: {seed_data['wp_y']}, wp_z: {seed_data['wp_z']}")

    # Confirm update for the starting point (sp_x, sp_y, sp_z, pitch, yaw, roll)
    print("Seed data updated successfully:")
    print(f"  sp_x: {seed_data['sp_x']}, sp_y: {seed_data['sp_y']}, sp_z: {seed_data['sp_z']}")
    print(f"  pitch: {seed_data['pitch']}, yaw: {seed_data['yaw']}, roll: {seed_data['roll']}")
    print("Seed data update process completed.")


def update_actors_speed_and_spawn(test_scenario_m):
    """
    Updates only the 'speed' and 'spawn_point' fields of the test_scenario_m.actors list
    based on the information in test_scenario_m.snap_info['NPC'].

    Args:
        test_scenario_m: An object with `snap_info` and `actors` attributes.
    """
    if not hasattr(test_scenario_m, 'snap_info') or not hasattr(test_scenario_m, 'actors'):
        raise AttributeError("test_scenario_m must have 'snap_info' and 'actors' attributes.")

    snap_info = test_scenario_m.snap_info
    actors = test_scenario_m.actors

    # Ensure snap_info contains 'NPC' key
    if "NPC" not in snap_info or not isinstance(snap_info["NPC"], list):
        raise ValueError("snap_info does not contain valid 'NPC' information.")

    npc_list = snap_info["NPC"]

    # Update each actor that matches an NPC
    for actor in actors:
        actor_id = actor.get("id")
        # pdb.set_trace()
        if actor_id is None:
            continue

        # Search for matching NPC by id
        for npc in npc_list:
            if npc.get("id") == actor_id:
                print(f"Updating actor with ID: {actor_id}")

                # Debug: Print original values
                print("Original actor values:")
                for key in ["speed", "spawn_point"]:
                    print(f"  {key}: {actor.get(key)}")

                # Update only 'speed' and 'spawn_point'
                actor["speed"] = npc.get("velocity", {}).get("x", 0.0)  # Example: Use velocity x as speed

                transform = npc.get("transform", {})
                location = transform.get("location", {})
                rotation = transform.get("rotation", {})
                actor["spawn_point"] = (
                    (location.get("x", 0.0), location.get("y", 0.0), location.get("z", 0.0) + 1.0),
                    (rotation.get("roll", 0.0), rotation.get("pitch", 0.0), rotation.get("yaw", 0.0))
                )

                # Debug: Print updated values
                print("Updated actor values:")
                for key in ["speed", "spawn_point"]:
                    print(f"  {key}: {actor.get(key)}")

                # Break after finding the matching NPC
                break

    print("Actor speed and spawn_point update process completed.")


def get_dangerous_frame(file_path: str, dangerous_frame: int, find_first: bool = True):
    """
    Reads the JSON file and searches for the specified dangerous_frame.
    Depending on the `find_first` parameter, it retrieves either the first or last matching frame.

    Args:
        file_path (str): The path to the JSON file.
        dangerous_frame (int): The frame number to search for.
        find_first (bool): If True, finds the first matching frame. If False, finds the last matching frame.

    Returns:
        dict: The frame information if found, otherwise None.
    """
    # Ensure the file exists before trying to open it
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")

    with open(file_path, 'r') as f:
        data = json.load(f)

    # Convert data keys to a list and sort them if needed
    frame_keys = list(data.keys())

    if not find_first:  # Reverse the order for finding the last occurrence
        frame_keys = reversed(frame_keys)

    # Search for the matching frame
    for frame_key in frame_keys:
        if int(frame_key) == dangerous_frame:
            return data[frame_key]

    # If no matching frame is found, return None
    return None
