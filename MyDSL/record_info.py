import math
import pdb

import pandas as pd
import numpy as np
import config

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("[-] Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("    Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)

from MyDSL.TTC import TTC, TTC_with_zone


def record_DSL_data(state, world, town_map, player, FRAME_RATE, sampling_rate):
    try:
        # Parse the scene
        parser = DSL2Parser(world, town_map, sampling_rate)
        parser.set_ads(player)
        parser.get_actors()
        parser.previous_scene = state.DSLScene
        scene = parser.parse_scene()
    except Exception as e:
        # If an error occurs, use a default empty scene
        print(f"Error parsing scene: {e}")
        scene = {
            "Road": {},  # Default empty road information
            "Environment": {},  # Default empty environment information
            "NPCs": {},  # No NPCs
            "ADS": {}  # No ADS behavior
        }

    # Update the state with the parsed or default scene
    state.DSLScene = scene

    # Step 1: Create an in-memory JSON buffer if not exists
    if not hasattr(state, "json_data_buffer"):
        state.json_data_buffer = []  # Initialize the JSON data buffer

    # Step 2: Add the scene with timestamp to the memory buffer
    timestamp = state.num_frames / FRAME_RATE
    scene_with_timestamp = {
        "timestamp": timestamp,
        "scene": scene
    }
    state.json_data_buffer.append(scene_with_timestamp)


def is_lane_reachable(current_wp, target_wp):
    """
    Check if the target waypoint is reachable by sequential lane changes from the current waypoint.

    Parameters:
        current_wp (carla.Waypoint): The current waypoint.
        target_wp (carla.Waypoint): The target waypoint.

    Returns:
        bool: True if the target waypoint is reachable, False otherwise.
    """
    # Check if the two waypoints are on the same road
    if current_wp.road_id != target_wp.road_id:
        return False

    # Determine the lane change direction
    if current_wp.lane_id < target_wp.lane_id:
        # Move to the right
        direction = "right"
    elif current_wp.lane_id > target_wp.lane_id:
        # Move to the left
        direction = "left"
    else:
        # Same lane
        return True

    # Start from the current waypoint
    wp = current_wp

    while wp:
        # Get the next waypoint based on direction
        if direction == "right":
            next_wp = wp.get_right_lane()
            allowed_change = wp.lane_change in [carla.LaneChange.Right, carla.LaneChange.Both]
        elif direction == "left":
            next_wp = wp.get_left_lane()
            allowed_change = wp.lane_change in [carla.LaneChange.Left, carla.LaneChange.Both]
        else:
            break

        # If lane change is not allowed or there's a discontinuity, return False
        if not allowed_change or not next_wp or abs(next_wp.lane_id - wp.lane_id) != 1:
            return False

        # Check if we have reached the target lane
        if next_wp.lane_id == target_wp.lane_id:
            return True

        # Move to the next waypoint
        wp = next_wp

    # If the loop ends without finding the target lane, return False
    return False


def determine_moving_restricted_zone(vehicle, map, max_time=5, sampling_time_interval=0.5, debug=False):
    """
    Determine the nearest restricted zone the vehicle is moving towards based on its velocity and heading.

    Parameters:
        vehicle (carla.Vehicle): The vehicle actor.
        max_time (float): Maximum time (in seconds) to sample along the velocity and heading direction.
        sampling_time_interval (float): Time interval (in seconds) for sampling points.
        debug (bool): If True, prints debug information.

    Returns:
        carla.Waypoint or None: The waypoint of the nearest restricted zone, or None if none found.
    """

    # Get vehicle's location, velocity, rotation, and map
    location = vehicle.get_location()
    velocity = vehicle.get_velocity()
    carla_map = map
    rotation = vehicle.get_transform().rotation

    # Get the current waypoint
    current_waypoint = carla_map.get_waypoint(location)

    # Calculate velocity vector and magnitude
    velocity_vector = np.array([velocity.x, velocity.y])
    velocity_magnitude = np.linalg.norm(velocity_vector)

    # Normalize velocity direction
    velocity_direction = velocity_vector / velocity_magnitude if velocity_magnitude > 0 else np.array([0, 0])

    # Calculate heading direction
    hx, hy = get_heading_direction(rotation.yaw)

    # Sample points in velocity direction and heading direction
    forward_points = [
        carla.Location(
            x=location.x + velocity_direction[0] * velocity_magnitude * t,
            y=location.y + velocity_direction[1] * velocity_magnitude * t,
            z=location.z
        )
        for t in np.arange(0, max_time, sampling_time_interval)
    ]
    heading_points = [
        carla.Location(
            x=location.x + hx * velocity_magnitude * t,
            y=location.y + hy * velocity_magnitude * t,
            z=location.z
        )
        for t in np.arange(0, max_time, sampling_time_interval)
    ]

    # Iterate through sampled points
    for idx, point in enumerate(forward_points + heading_points):
        waypoint = carla_map.get_waypoint(point, project_to_road=False)
        if not waypoint:
            continue  # Skip if there's no valid waypoint
        if waypoint.is_junction:
            continue  # Skip junction temporary
        # Check if the lane is of Driving type
        if waypoint.lane_type != carla.LaneType.Driving:
            if debug:
                print("Debug: Restricted Zone Found (Not 'Driving' Lane)")
                print(
                    f" - Location: ({waypoint.transform.location.x}, {waypoint.transform.location.y}, {waypoint.transform.location.z})")
                print(f" - Lane ID: {waypoint.lane_id}, Road ID: {waypoint.road_id}")
                print(f" - Lane Type: {waypoint.lane_type}")
            return waypoint  # Return the restricted zone waypoint

        # Check if the lane is not reachable
        if not is_lane_reachable(current_waypoint, waypoint):
            if debug:
                print("Debug: Restricted Zone Found (Not Reachable)")
                print(
                    f" - Location: ({waypoint.transform.location.x}, {waypoint.transform.location.y}, {waypoint.transform.location.z})")
                print(f" - Lane ID: {waypoint.lane_id}, Road ID: {waypoint.road_id}")
            return waypoint  # Return the restricted zone waypoint

    # No restricted zone found
    if debug:
        print("Debug: No restricted zones found.")
    return None  # No restricted zone found


def get_traffic_light_for_waypoint(world, waypoint, search_distance=10.0):
    """
    Get the traffic light corresponding to a given waypoint.

    Parameters:
        world (carla.World): The CARLA world object.
        waypoint (carla.Waypoint): The waypoint to find the traffic light for.
        search_distance (float): The search radius for finding nearby traffic lights.

    Returns:
        str: The state of the closest traffic light, or "None" if no traffic light is found.
    """
    # Get all traffic lights in the world
    traffic_lights = world.get_actors().filter("*traffic_light*")

    # Initialize variables to track the closest traffic light
    closest_traffic_light = None
    closest_distance = search_distance

    # Iterate over all traffic lights
    for traffic_light in traffic_lights:
        # Get the location of the traffic light
        light_location = traffic_light.get_location()

        # Calculate the distance between the traffic light and the waypoint
        distance = waypoint.transform.location.distance(light_location)

        # Check if this traffic light is closer than the current closest
        if distance < closest_distance:
            closest_distance = distance
            closest_traffic_light = traffic_light
    if not closest_traffic_light:
        return "None"
    else:
        return str(closest_traffic_light.state)


def determine_prior_action(current_behavior, previous_behavior):
    """
    Determine whether the prior action is "Maneuver" or "Stable".

    Parameters:
        current_behavior (dict): Current behavior of the vehicle (ADS or NPC).
        previous_behavior (dict): Previous behavior of the vehicle.

    Returns:
        str: "Maneuver" if any of the conditions are met, otherwise "Stable".
    """
    # Ensure there is a valid previous_scene
    if not previous_behavior:
        return "Stable"  # Default to Stable if no previous data is available

    # Define the conditions to check for "Maneuver"
    conditions = [
        # Condition 1: HeadDirection mismatch
        current_behavior["HeadDirection"] != previous_behavior["HeadDirection"],
        # Condition 2: SpeedDirection mismatch
        current_behavior["SpeedDirection"] != previous_behavior["SpeedDirection"],
        # Condition 3: Acceleration is not constant-speed
        current_behavior["Accelerate"] != "constant-speed",
        # Condition 4: MovingToWhichWay mismatch
        current_behavior["MovingToWhichWay"] != previous_behavior["MovingToWhichWay"],
        # Condition 5: HeadDirection and SpeedDirection mismatch
        current_behavior["HeadDirection"] != current_behavior["SpeedDirection"],
        # Condition 6: MovingOnWhichWay and MovingToWhichWay mismatch
        current_behavior["MovingOnWhichWay"] != current_behavior["MovingToWhichWay"],
    ]
    # Return "Maneuver" if any condition is true, otherwise "Stable"
    return "Maneuver" if any(conditions) else "Stable"


def get_max_steer_angle(vehicle):
    """
    Get the maximum steering angle for the vehicle.

    Parameters:
        vehicle (carla.Vehicle): The vehicle actor.

    Returns:
        float: Maximum steering angle (in degrees) for the vehicle's wheels.
    """
    # Get the vehicle's physics control
    physics_control = vehicle.get_physics_control()

    # Extract the wheels' max_steer_angle
    max_steer_angles = [wheel.max_steer_angle for wheel in physics_control.wheels]

    # Return the maximum steering angle among all wheels
    return max(max_steer_angles)


def steer_to_angle_radians(control_steer, max_steer_angle=45):
    """
    Map the normalized steer value to an actual steering angle in radians.

    Parameters:
        control_steer (float): Normalized steer value, range [-1.0, 1.0].
        max_steer_angle (float): Maximum steering angle in degrees.

    Returns:
        float: Steering angle in radians.
    """
    control_steer = max(-1.0, min(1.0, control_steer))
    steering_angle_degrees = control_steer * max_steer_angle
    return steering_angle_degrees


def get_heading_direction(yaw):
    """
    Convert the yaw (rotation around Z-axis in degrees) to the heading direction vector.

    Parameters:
        yaw (float): The yaw angle in degrees (Z-axis rotation).

    Returns:
        tuple: (hx_i, hy_i), the x and y coordinates of the heading direction unit vector.
    """
    # Convert yaw from degrees to radians
    yaw_radians = math.radians(yaw)

    # Calculate heading direction vector
    hx_i = math.cos(yaw_radians)  # x component
    hy_i = math.sin(yaw_radians)  # y component

    return hx_i, hy_i


def classify_ttc(ttc, divide=1.0):
    """Classify TTC into categories."""
    if ttc < divide:
        return "very short"
    elif ttc < divide * 2:
        return "short"
    elif ttc < divide * 3:
        return "moderate"
    elif ttc < divide * 5:
        return "long"
    else:
        return "very long"


def angle_to_direction(yaw):
    """Convert a yaw angle to a compass direction."""
    yaw = yaw % 360
    directions = ["north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest"]
    index = int((yaw + 22.5) % 360 // 45)
    return directions[index]


def velocity_to_direction(velocity):
    """
    Convert velocity vector to a compass direction.

    Parameters:
        velocity (carla.Vector3D): The velocity vector.

    Returns:
        str: Compass direction ("north", "northeast", etc.) or "stationary".
    """
    # Check if the velocity is effectively zero
    if np.linalg.norm([velocity.x, velocity.y, velocity.z]) < 0.1:
        return "stationary"

    # Calculate the angle in degrees from the velocity vector
    angle = np.degrees(np.arctan2(velocity.y, velocity.x)) % 360

    # Use angle_to_direction to map angle to compass direction
    return angle_to_direction(angle)


def weather_to_text(weather):
    """Convert weather data to a descriptive text."""
    if weather.precipitation > 0:
        return "rainy"
    if weather.cloudiness > 50:
        return "cloudy"
    if weather.fog_density > 0:
        return "foggy"
    return "sunny"


def get_all_lanes(waypoint):
    """
    Get all lanes at the waypoint based on lane_id.

    Parameters:
        waypoint (carla.Waypoint): The starting waypoint.

    Returns:
        list: A list of waypoints representing all lanes (left, right, and current).
    """
    # Initialize a set to track unique lane IDs and a list to store all lane waypoints
    lane_ids = {waypoint.lane_id}
    all_lanes = [waypoint]  # Start with the current lane's waypoint

    # Check right lanes
    right_wp = waypoint.get_right_lane()
    while right_wp and right_wp.lane_type == carla.LaneType.Driving:
        if right_wp.lane_id in lane_ids:  # Skip if lane_id is already visited
            break
        lane_ids.add(right_wp.lane_id)
        all_lanes.append(right_wp)
        right_wp = right_wp.get_right_lane()

    # Check left lanes
    left_wp = waypoint.get_left_lane()
    while left_wp and left_wp.lane_type == carla.LaneType.Driving:
        if left_wp.lane_id in lane_ids:  # Skip if lane_id is already visited
            break
        lane_ids.add(left_wp.lane_id)
        all_lanes.append(left_wp)
        left_wp = left_wp.get_left_lane()

    # Return all lane waypoints
    return all_lanes


def get_vehicle_dimensions(vehicle):
    """
    Get the dimensions (length and width) of a vehicle actor.

    Parameters:
        vehicle (carla.Actor): A CARLA vehicle actor.

    Returns:
        tuple: (length, width) of the vehicle in meters.
    """
    # Get the bounding box of the vehicle
    bounding_box = vehicle.bounding_box

    # Bounding box extent gives half-dimensions
    length = 2 * bounding_box.extent.x  # Total length (x-axis)
    width = 2 * bounding_box.extent.y  # Total width (y-axis)

    return length, width


def get_road_type(waypoint):
    """
    Determine the RoadType based on the waypoint and its surroundings.
    Returns one of intersection, divided, undivided, two-way.
    """
    # Check if the waypoint is part of an intersection
    if waypoint.is_junction:
        return "intersection"
    # Check a lane type and driving direction
    left_lane = waypoint.get_left_lane()
    right_lane = waypoint.get_right_lane()

    if left_lane and right_lane:
        if left_lane.lane_type == carla.LaneType.Driving and right_lane.lane_type == carla.LaneType.Driving:
            # Check if the road is divided
            if left_lane.lane_id * waypoint.lane_id < 0:
                return "divided"
            else:
                return "undivided"
    # Default to two-way for bidirectional roads without specific division
    return "two-way"


def get_road_slope(waypoint):
    """
    Calculate the RoadSlope based on the waypoint's elevation and nearby points.
    Returns one of flat, uphill, downhill, level.
    """
    # Get the current waypoint location
    current_location = waypoint.transform.location

    # Get a waypoint slightly ahead on the same lane
    forward_waypoint = waypoint.next(5.0)[0]  # 5 meters ahead
    forward_location = forward_waypoint.transform.location

    # Calculate slope using the difference in z-coordinates
    slope = (forward_location.z - current_location.z) / 5.0  # Slope per meter

    # Classify the slope
    if abs(slope) < 0.01:  # Small threshold for flat
        return "flat"
    elif slope > 0.01:
        return "uphill"
    elif slope < -0.01:
        return "downhill"
    else:
        return "level"


class DSL2Parser:
    def __init__(self, world, town_map, sampling_rate=1):
        self.world = world
        self.map = town_map
        self.npcs = []
        self.ads = None
        self.previous_scene = None
        self.sampling_rate = sampling_rate

    def set_ads(self, ads_vehicle):
        """Set the ADS vehicle."""
        self.ads = ads_vehicle

    def get_actors(self, radius=50.0):
        """
        Retrieve all vehicles (NPCs) in the scene within a given radius of the ADS vehicle.

        Parameters:
            radius (float): The maximum distance (in meters) to include NPCs.
        """
        self.npcs = []
        all_vehicles = self.world.get_actors().filter("*vehicle.*")
        ads_location = self.ads.get_location()

        for vehicle in all_vehicles:
            # Skip the ADS vehicle itself
            if vehicle.id == self.ads.id:
                continue

            # Calculate the distance from the ADS vehicle
            distance = ads_location.distance(vehicle.get_location())

            # Include the vehicle if it's within the radius
            if distance <= radius:
                self.npcs.append(vehicle)

    def get_road_info(self):
        """Extract road information including RoadType, Lanes, RoadSlope, and SpeedLimit."""
        map_data = self.map
        waypoint = map_data.get_waypoint(self.ads.get_location())
        lanes = get_all_lanes(waypoint)
        road_info = {
            "RoadType": get_road_type(waypoint),
            "Lanes": [f"L{lane.lane_id}_R{lane.road_id}" for lane in lanes],
            "RoadSlope": get_road_slope(waypoint),
            "SpeedLimit": 30
        }
        return road_info

    def parse_scene(self):
        """Parse the entire scene and return the DSL2 description."""
        scene_data = {
            "Road": self.get_road_info(),
            "Environment": self.get_environment_info(),
            "NPCs": {npc.id: self.get_npc_behavior(npc) for npc in self.npcs},
            "ADS": self.get_ads_behavior()
        }
        return scene_data

    def get_lane_departure_status(self, vehicle):
        """Calculate the lane departure status."""
        vehicle_location = vehicle.get_location()
        waypoint = self.map.get_waypoint(vehicle_location)
        lane_center = waypoint.transform.location
        lane_departure_distance = np.linalg.norm([
            vehicle_location.x - lane_center.x,
            vehicle_location.y - lane_center.y
        ])
        lane_width = waypoint.lane_width
        align_threshold = lane_width / 6
        deviate_threshold = lane_width / 3
        if lane_departure_distance <= align_threshold:
            return "Align"
        elif lane_departure_distance <= deviate_threshold:
            return "Deviate"
        else:
            return "Depart"

    def get_speed_limit(self):
        """Get the speed limit near the ADS vehicle."""
        waypoint = self.map.get_waypoint(self.ads.get_location())
        return waypoint.get_speed_limit()

    def get_environment_info(self):
        """Extract environment information."""
        weather = self.world.get_weather()
        waypoint = self.map.get_waypoint(self.ads.get_location())
        road_condition = "wet" if weather.precipitation > 0 else "dry"
        if weather.fog_density > 50:
            road_condition = "icy"
        return {
            "Weather": weather_to_text(weather),
            "RoadCondition": road_condition,
            "TrafficSignals": get_traffic_light_for_waypoint(self.world, waypoint)
        }

    def determine_moving_lane(self, vehicle, max_time=3, sampling_time_interval=1):
        """
        Determine which lane the vehicle is moving towards based on its velocity.

        Parameters:
            vehicle (carla.Vehicle): The vehicle actor.
            max_time (float): The maximum time (in seconds) to sample along the velocity direction.
            sampling_time_interval (float): Time interval (in seconds) for sampling points.

        Returns:
            carla.Waypoint: The waypoint of the lane the vehicle is moving towards.
        """
        # Get vehicle's location, velocity, and map
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        carla_map = self.map
        current_waypoint = carla_map.get_waypoint(location)

        # Get the current lane_id and road_id
        lane_id = current_waypoint.lane_id
        road_id = current_waypoint.road_id

        # Calculate the velocity vector and magnitude
        velocity_vector = np.array([velocity.x, velocity.y])
        velocity_magnitude = np.linalg.norm(velocity_vector)

        # If the vehicle is stationary or nearly stationary, return the current waypoint
        if velocity_magnitude < 1e-3:
            return current_waypoint

        # Normalize the velocity direction vector
        velocity_direction = velocity_vector / velocity_magnitude

        # Sample points based on time intervals
        forward_points = [
            carla.Location(
                x=location.x + velocity_direction[0] * velocity_magnitude * t,
                y=location.y + velocity_direction[1] * velocity_magnitude * t,
                z=location.z
            )
            for t in np.arange(0, max_time, sampling_time_interval)
        ]

        # Iterate through the sampled points and find the first valid lane
        for point in forward_points:
            waypoint = carla_map.get_waypoint(point, project_to_road=False)
            if waypoint and waypoint.lane_id != lane_id:
                if waypoint.road_id == road_id:
                    return waypoint
                return waypoint
        # If no new lane is found in the sampled points, return the current waypoint
        return current_waypoint

    def get_npc_behavior(self, npc):
        """Extract the behavior of a single NPC."""
        transform = npc.get_transform()
        velocity = npc.get_velocity()
        control = npc.get_control()
        lane = self.map.get_waypoint(transform.location)
        steeringAngle = steer_to_angle_radians(control.steer, get_max_steer_angle(npc))
        # Determine the type of NPC
        npc_type = "motorized vehicle"
        if velocity.x == 0 and velocity.y == 0 and velocity.z == 0:
            npc_type = "roadblock"  # Static obstacles
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        if self.previous_scene:
            previous_behavior = self.previous_scene["NPCs"].get(npc.id)
        else:
            previous_behavior = None
        if previous_behavior:
            acceleration = (speed - previous_behavior["Speed"]) / self.sampling_rate
        else:
            acceleration = 0
        MovingToWhichWaypoint = self.determine_moving_lane(npc)
        # Behavior information
        behavior = {
            "Type": npc_type,
            "ID": npc.id,
            "SideToADS": self.get_side_to_ads(npc),
            "HeadDirection": angle_to_direction(transform.rotation.yaw),
            "SpeedDirection": velocity_to_direction(velocity),
            "Speed": speed,
            "MovingOnWhichWay": f'L{lane.lane_id}_R{lane.road_id}',
            "MovingToWhichWay": f'L{MovingToWhichWaypoint.lane_id}_R{MovingToWhichWaypoint.road_id}',
            "SteeringAngle": steeringAngle,
            "Accelerate": "Accelerating" if acceleration > 1 else "Braking" if acceleration < -1 else "constant-speed",
            "LaneDeparture": self.get_lane_departure_status(npc),
            "TTCToADS": self.calculate_ttc_to_ads(npc)
        }

        behavior["PriorAction"] = determine_prior_action(behavior, previous_behavior)
        return behavior

    def get_ads_behavior(self):
        """Extract the behavior of the ADS vehicle."""
        transform = self.ads.get_transform()
        velocity = self.ads.get_velocity()
        control = self.ads.get_control()
        lane = self.map.get_waypoint(transform.location)
        steeringAngle = steer_to_angle_radians(control.steer, get_max_steer_angle(self.ads))
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        if self.previous_scene:
            previous_behavior = self.previous_scene["ADS"]
        else:
            previous_behavior = None
        if previous_behavior:
            acceleration = (speed - previous_behavior["Speed"]) / self.sampling_rate
        else:
            acceleration = 0
        MovingToWhichWaypoint = self.determine_moving_lane(self.ads)
        # Behavior information
        behavior = {
            "ID": self.ads.id,
            "HeadDirection": angle_to_direction(transform.rotation.yaw),
            "SpeedDirection": velocity_to_direction(velocity),
            "Speed": speed,
            "MovingOnWhichWay": f'L{lane.lane_id}_R{lane.road_id}',
            "MovingToWhichWay": f"L{MovingToWhichWaypoint.lane_id}_R{MovingToWhichWaypoint.road_id}",
            "SteeringAngle": steeringAngle,
            "Accelerate": "Accelerating" if acceleration > 1 else "Braking" if acceleration < -1 else "constant-speed",
            "LaneDeparture": self.get_lane_departure_status(self.ads),
            "TTCToNPCs": self.calculate_ttc_to_npcs(),
            "TTCToRestrictedZone": self.calculate_ttc_to_restricted_zones()
        }

        behavior["PriorAction"] = determine_prior_action(behavior, previous_behavior)
        return behavior

    def calculate_ttc_to_ads(self, npc):
        """
        Calculate the Time-to-Collision (TTC) between the given NPC and the ADS vehicle using the TTC function.
        """

        # Extract NPC information
        npc_location = npc.get_location()
        npc_velocity = npc.get_velocity()
        npc_transform = npc.get_transform()
        npc_heading = get_heading_direction(npc_transform.rotation.yaw)
        npc_length = get_vehicle_dimensions(npc)[0]
        npc_width = get_vehicle_dimensions(npc)[1]

        # Extract ADS (ego vehicle) information
        ads_location = self.ads.get_location()
        ads_velocity = self.ads.get_velocity()
        ads_transform = self.ads.get_transform()
        ads_heading = get_heading_direction(ads_transform.rotation.yaw)
        ads_length = get_vehicle_dimensions(self.ads)[0]
        ads_width = get_vehicle_dimensions(self.ads)[1]

        # Create a DataFrame for the single NPC-ADS pair
        sample = pd.DataFrame([{
            'x_i': npc_location.x,
            'y_i': npc_location.y,
            'vx_i': npc_velocity.x,
            'vy_i': npc_velocity.y,
            'hx_i': npc_heading[0],
            'hy_i': npc_heading[1],
            'length_i': npc_length,
            'width_i': npc_width,
            'x_j': ads_location.x,
            'y_j': ads_location.y,
            'vx_j': ads_velocity.x,
            'vy_j': ads_velocity.y,
            'hx_j': ads_heading[0],
            'hy_j': ads_heading[1],
            'length_j': ads_length,
            'width_j': ads_width
        }])

        # Compute TTC using the TTC function
        ttc_values = TTC(sample, 'values')

        # Extract the TTC value for this pair (single value in this case)
        ttc = ttc_values[0] if len(ttc_values) > 0 else float('inf')
        # Return the classified TTC value
        return classify_ttc(ttc)

    def calculate_ttc_to_npcs(self, debug=False):
        """
        Calculate the minimum Time-to-Collision (TTC) between the ADS and any NPC using the TTC function.

        Parameters:
            debug (bool): If True, prints debug information.

        Returns:
            str: Classification of the minimum TTC as per the defined categories.
        """
        # If there are no NPCs, return "very long"
        if not self.npcs:
            if debug:
                print("Debug: No NPCs detected.")
            return "very long"

        # ADS (ego vehicle) information
        ego_location = self.ads.get_location()
        ego_velocity = self.ads.get_velocity()
        ego_transform = self.ads.get_transform()
        ego_heading = get_heading_direction(ego_transform.rotation.yaw)
        ego_length, ego_width = get_vehicle_dimensions(self.ads)

        # Debug: Print ADS vehicle information
        if debug:
            print("Debug: ADS Vehicle Info:")
            print(f" - Location: (x={ego_location.x}, y={ego_location.y})")
            print(f" - Velocity: (vx={ego_velocity.x}, vy={ego_velocity.y})")
            print(f" - Heading: (hx={ego_heading[0]}, hy={ego_heading[1]})")
            print(f" - Dimensions: Length={ego_length}, Width={ego_width}")

        # Prepare data for all NPCs
        npc_data = []
        for idx, npc in enumerate(self.npcs):
            npc_location = npc.get_location()
            npc_velocity = npc.get_velocity()
            npc_transform = npc.get_transform()
            npc_heading = get_heading_direction(npc_transform.rotation.yaw)
            npc_length, npc_width = get_vehicle_dimensions(npc)

            # Debug: Print NPC information
            if debug:
                print(f"Debug: NPC {idx + 1} Info:")
                print(f" - Location: (x={npc_location.x}, y={npc_location.y})")
                print(f" - Velocity: (vx={npc_velocity.x}, vy={npc_velocity.y})")
                print(f" - Heading: (hx={npc_heading[0]}, hy={npc_heading[1]})")
                print(f" - Dimensions: Length={npc_length}, Width={npc_width}")

            # Append ego and NPC information to the data list
            npc_data.append({
                'x_i': ego_location.x,
                'y_i': ego_location.y,
                'vx_i': ego_velocity.x,
                'vy_i': ego_velocity.y,
                'hx_i': ego_heading[0],
                'hy_i': ego_heading[1],
                'length_i': ego_length,
                'width_i': ego_width,
                'x_j': npc_location.x,
                'y_j': npc_location.y,
                'vx_j': npc_velocity.x,
                'vy_j': npc_velocity.y,
                'hx_j': npc_heading[0],
                'hy_j': npc_heading[1],
                'length_j': npc_length,
                'width_j': npc_width
            })

        # Convert to pandas DataFrame
        samples = pd.DataFrame(npc_data)

        # Debug: Print samples DataFrame
        if debug:
            print("Debug: Samples DataFrame:")
            print(samples)

        # Compute TTC values
        ttc_results = TTC(samples, 'values')

        # Debug: Print TTC results
        if debug:
            print("Debug: TTC Results:")
            print(ttc_results)

        # Find the minimum TTC value
        min_ttc = ttc_results.min() if len(ttc_results) > 0 else float('inf')

        # Debug: Print the minimum TTC value
        if debug:
            print(f"Debug: Minimum TTC: {min_ttc}")

        # Return the classification based on the minimum TTC
        classified_ttc = classify_ttc(min_ttc)
        if debug:
            print(f"Debug: Classified TTC: {classified_ttc}")
        return classified_ttc

    def calculate_ttc_to_restricted_zones(self, debug=False):
        """
        Calculate Time-to-Collision (TTC) between the ADS and restricted zones,
        including specific restricted waypoints and nearby obstacles.

        Parameters:
            debug (bool): If True, prints debug information.

        Returns:
            str: Classification of the TTC as per the defined categories.
        """
        # Step 1: Get the restricted zone waypoint
        restricted_zone_wp = determine_moving_restricted_zone(
            vehicle=self.ads,
            map=self.map,
            max_time=5,
            sampling_time_interval=0.5,
            debug=debug
        )

        # # Step 2: Get obstacles from the environment
        # world = self.ads.get_world()
        # obstacle_objects = world.get_environment_objects(carla.CityObjectLabel.Walls) + \
        #                    world.get_environment_objects(carla.CityObjectLabel.Poles)
        #
        # # Filter obstacles within 20 meters
        # nearby_obstacles = [
        #     obstacle for obstacle in obstacle_objects
        #     if self.ads.get_location().distance(obstacle.transform.location) <= 20
        # ]

        # Step 3: Get ADS parameters
        ego_location = self.ads.get_location()
        ego_velocity = self.ads.get_velocity()
        ego_rotation = self.ads.get_transform().rotation
        ego_length, ego_width = get_vehicle_dimensions(self.ads)
        ego_heading = get_heading_direction(ego_rotation.yaw)

        # Step 4: Compute TTC for restricted zone waypoint
        samples = pd.DataFrame({
            "x_i": [ego_location.x],
            "y_i": [ego_location.y],
            "vx_i": [ego_velocity.x],
            "vy_i": [ego_velocity.y],
            "hx_i": [ego_heading[0]],
            "hy_i": [ego_heading[1]],
            "length_i": [ego_length],
            "width_i": [ego_width],
        })
        ttc_values = []

        if restricted_zone_wp:
            ttc_zone = TTC_with_zone(samples, restricted_zone_wp, toreturn="values")[0]
            ttc_values.append((ttc_zone, restricted_zone_wp))  # Append tuple of TTC and source
        #
        # # Step 5: Compute TTC for nearby obstacles
        # npc_data = []
        # for obstacle in nearby_obstacles:
        #     obstacle_location = obstacle.bounding_box.location
        #     obstacle_rotation = obstacle.bounding_box.rotation
        #     hx, hy = get_heading_direction(obstacle_rotation.yaw)
        #     npc_data.append({
        #         'x_i': ego_location.x,
        #         'y_i': ego_location.y,
        #         'vx_i': ego_velocity.x,
        #         'vy_i': ego_velocity.y,
        #         'hx_i': ego_heading[0],
        #         'hy_i': ego_heading[1],
        #         'length_i': ego_length,
        #         'width_i': ego_width,
        #         'x_j': obstacle_location.x,
        #         'y_j': obstacle_location.y,
        #         'vx_j': 0,
        #         'vy_j': 0,
        #         'hx_j': hx,
        #         'hy_j': hy,
        #         'length_j': obstacle.bounding_box.extent.x * 2,
        #         'width_j': obstacle.bounding_box.extent.y * 2
        #     })
        #
        # # Convert to DataFrame and compute TTC for obstacles
        # if npc_data:
        #     npc_samples = pd.DataFrame(npc_data)
        #     ttc_obstacles = TTC(npc_samples, 'values')
        #     for ttc, obstacle in zip(ttc_obstacles, nearby_obstacles):
        #         ttc_values.append((ttc, obstacle))  # Append tuple of TTC and source

        # Step 6: Get the minimum TTC value and its source
        if ttc_values:
            min_ttc, min_source = min(ttc_values, key=lambda x: x[0])
        else:
            min_ttc, min_source = float('inf'), None

        # Debug: Print only key information for min_ttc
        if debug and min_source:
            if isinstance(min_source, carla.Waypoint):
                print("Debug: Final Minimum TTC Source (Waypoint)")
                print(f" - TTC: {min_ttc}")
                print(f" - Position: ({min_source.transform.location.x}, {min_source.transform.location.y})")
                print(f" - Lane Width: {min_source.lane_width}")
            elif isinstance(min_source, carla.EnvironmentObject):
                print("Debug: Final Minimum TTC Source (Obstacle)")
                print(f" - TTC: {min_ttc}")
                print(f" - Obstacle ID: {min_source.id}")
                print(f" - Location: ({min_source.bounding_box.location.x}, {min_source.bounding_box.location.y})")
                print(f" - Extent: ({min_source.bounding_box.extent.x}, {min_source.bounding_box.extent.y})")

        # Step 7: Return classified TTC
        return classify_ttc(min_ttc) if min_ttc != float('inf') else "very long"

    def get_side_to_ads(self, npc):
        """
        Determine the relative position of the NPC to the ADS.
        Returns one of front, rear, left, right, front-left, front-right, rear-left, rear-right.
        """
        ads_location = self.ads.get_location()
        npc_location = npc.get_location()
        diff = npc_location - ads_location

        # Calculate the angle in degrees from ADS to NPC
        angle = np.degrees(np.arctan2(diff.y, diff.x)) % 360

        # Define direction categories
        directions = [
            "front", "front-right", "right", "rear-right",
            "rear", "rear-left", "left", "front-left"
        ]

        # Map the angle to one of the 8 categories
        index = int((angle + 22.5) % 360 // 45)
        return directions[index]
