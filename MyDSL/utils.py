import json
import os
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


def save_json_to_file(json_data, output_dir, campaign_cnt, generation_id, scenario_id):
    """
    Saves the given JSON data to a file in the specified directory.

    :param json_data: The JSON data to be saved
    :param output_dir: The directory where the file should be saved
    :param campaign_cnt: The campaign count to be included in the file name
    :param generation_id: The generation ID to be included in the file name
    :param scenario_id: The scenario ID to be included in the file name

    :return: None

    """
    try:
        # Ensure the output directory exists, create it if necessary
        os.makedirs(output_dir, exist_ok=True)

        # Define the output file path
        output_file = os.path.join(output_dir, f"SceneDSL_cid{campaign_cnt}:gid:{generation_id}_sid:{scenario_id}.json")

        # Serialize and save all accumulated JSON data to the file
        try:
            with open(output_file, "w") as f:
                # Write the JSON data to the file
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
    except Exception as e:
        print(f"Error during file operations: {e}")
        traceback.print_exc()


def initialize_vehicle_from_json(json_data, actor):
    """
    Initializes the vehicle (NPC or player) from a JSON configuration.

    :param json_data: JSON data containing the vehicle's transform, velocity, and control properties.
    :param actor: The carla.Actor representing the vehicle (either NPC or player).
    """

    # 1. Initialize Transform (Location and Rotation)
    transform_data = json_data['transform']
    location_data = transform_data['location']
    rotation_data = transform_data['rotation']

    location = carla.Location(x=location_data['x'], y=location_data['y'], z=location_data['z'])
    rotation = carla.Rotation(pitch=rotation_data['pitch'], yaw=rotation_data['yaw'], roll=rotation_data['roll'])
    transform = carla.Transform(location, rotation)

    # 2. Initialize Velocity
    velocity_data = json_data['velocity']
    velocity = carla.Vector3D(x=velocity_data['x'], y=velocity_data['y'], z=velocity_data['z'])

    # 3. Initialize Control (Throttle, Steer, Brake, Hand Brake, Reverse, Gear)
    control_data = json_data['control']
    control = carla.VehicleControl(
        throttle=control_data['throttle'],
        steer=control_data['steer'],
        brake=control_data['brake'],
        hand_brake=control_data['hand_brake'],
        reverse=control_data['reverse'],
        gear=control_data['gear']
    )

    # Set Transform to the vehicle (NPC or player)
    actor.set_transform(transform)

    # Set the velocity by applying control with speed
    actor.set_target_velocity(velocity)

    # Apply control to the vehicle
    actor.apply_control(control)
