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


def determine_moving_restricted_zone(vehicle, max_time=5, sampling_time_interval=0.5):
    """
    Determine the nearest restricted zone the vehicle is moving towards based on its velocity and heading.

    Parameters:
        vehicle (carla.Vehicle): The vehicle actor.
        max_time (float): Maximum time (in seconds) to sample along the velocity and heading direction.
        sampling_time_interval (float): Time interval (in seconds) for sampling points.

    Returns:
        carla.Waypoint or None: The waypoint of the nearest restricted zone, or None if none found.
    """
    # Get vehicle's location, velocity, rotation, and map
    location = vehicle.get_location()
    velocity = vehicle.get_velocity()
    world = vehicle.get_world()
    carla_map = world.get_map()
    rotation = vehicle.get_transform().rotation

    # Calculate velocity vector and magnitude
    velocity_vector = np.array([velocity.x, velocity.y])
    velocity_magnitude = np.linalg.norm(velocity_vector)

    # Normalize velocity direction
    if velocity_magnitude > 0:
        velocity_direction = velocity_vector / velocity_magnitude
    else:
        velocity_direction = np.array([0, 0])  # Stationary vehicle

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

    # Find the first restricted zone along the sampled points
    for point in forward_points + heading_points:
        waypoint = carla_map.get_waypoint(point, project_to_road=False)
        if waypoint and waypoint.lane_type != carla.LaneType.Driving:
            return waypoint  # Return the first restricted zone waypoint

    return None  # No restricted zone found


def get_traffic_light_for_waypoint(world, waypoint, search_distance=10.0):
    """
    Get the traffic light corresponding to a given waypoint.

    Parameters:
        world (carla.World): The CARLA world object.
        waypoint (carla.Waypoint): The waypoint to find the traffic light for.
        search_distance (float): The search radius for finding nearby traffic lights.

    Returns:
        carla.TrafficLight or None: The nearest traffic light affecting the waypoint, or None if none found.
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

    return closest_traffic_light


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


def determine_moving_lane(vehicle, max_time=5, sampling_time_interval=0.5):
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
    world = vehicle.get_world()
    carla_map = world.get_map()
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
    def __init__(self, world):
        self.world = world
        self.npcs = []
        self.ads = None
        self.previous_scene = None

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
        map_data = self.world.get_map()
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
        waypoint = self.world.get_map().get_waypoint(vehicle_location)
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
        waypoint = self.world.get_map().get_waypoint(self.ads.get_location())
        return waypoint.get_speed_limit()

    def get_environment_info(self):
        """Extract environment information."""
        weather = self.world.get_weather()
        waypoint = self.world.get_map().get_waypoint(self.ads.get_location())
        road_condition = "wet" if weather.precipitation > 0 else "dry"
        if weather.fog_density > 50:
            road_condition = "icy"
        return {
            "Weather": weather_to_text(weather),
            "RoadCondition": road_condition,
            "TrafficSignals": get_traffic_light_for_waypoint(self.world, waypoint)
        }

    def get_npc_behavior(self, npc):
        """Extract the behavior of a single NPC."""
        transform = npc.get_transform()
        velocity = npc.get_velocity()
        control = npc.get_control()
        lane = self.world.get_map().get_waypoint(transform.location)
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
            acceleration = (speed - previous_behavior["Speed"])
        else:
            acceleration = 0
        MovingToWhichWaypoint = determine_moving_lane(npc)
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
        lane = self.world.get_map().get_waypoint(transform.location)
        steeringAngle = steer_to_angle_radians(control.steer, get_max_steer_angle(self.ads))
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        if self.previous_scene:
            previous_behavior = self.previous_scene["ADS"]
        else:
            previous_behavior = None
        if previous_behavior:
            acceleration = (speed - previous_behavior["Speed"])
        else:
            acceleration = 0
        MovingToWhichWaypoint = determine_moving_lane(self.ads)
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
            "TTCToRestrictedZone": self.calculate_ttc_to_restricted_zone()
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

    def calculate_ttc_to_npcs(self):
        """
        Calculate the minimum Time-to-Collision (TTC) between the ADS and any NPC using the TTC function.
        """
        if not self.npcs:
            return "very long"
        # ADS (ego vehicle) information
        ego_location = self.ads.get_location()
        ego_velocity = self.ads.get_velocity()
        ego_transform = self.ads.get_transform()
        ego_heading = get_heading_direction(ego_transform.rotation.yaw)
        ego_length = get_vehicle_dimensions(self.ads)[0]
        ego_width = get_vehicle_dimensions(self.ads)[1]

        # Prepare data for all NPCs
        npc_data = []
        for npc in self.npcs:
            npc_location = npc.get_location()
            npc_velocity = npc.get_velocity()
            npc_transform = npc.get_transform()
            npc_heading = get_heading_direction(npc_transform.rotation.yaw)
            npc_length = get_vehicle_dimensions(npc)[0]
            npc_width = get_vehicle_dimensions(npc)[1]

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

        # Compute TTC values
        ttc_results = TTC(samples, 'values')

        # Find the minimum TTC value
        min_ttc = ttc_results.min() if len(ttc_results) > 0 else float('inf')
        # Return the classification based on the minimum TTC
        return classify_ttc(min_ttc)

    def calculate_ttc_to_restricted_zone(self):
        """
        Calculate Time-to-Collision (TTC) between the ADS and the nearest restricted zone.

        Returns:
            str: Classification of the TTC as per the defined categories.
        """
        # Get the nearest restricted zone waypoint
        restricted_zone_wp = determine_moving_restricted_zone(
            vehicle=self.ads,
            max_time=5,
            sampling_time_interval=0.5
        )

        # If no restricted zone is found, return "very long"
        if not restricted_zone_wp:
            return "very long"

        # Get ADS parameters
        vehicle_location = self.ads.get_location()
        vehicle_velocity = self.ads.get_velocity()
        vehicle_rotation = self.ads.get_transform().rotation
        vehicle_length, vehicle_width = get_vehicle_dimensions(self.ads)

        # Calculate heading direction
        hx_i, hy_i = get_heading_direction(vehicle_rotation.yaw)

        # Build the samples DataFrame
        samples = pd.DataFrame({
            "x_i": [vehicle_location.x],
            "y_i": [vehicle_location.y],
            "vx_i": [vehicle_velocity.x],
            "vy_i": [vehicle_velocity.y],
            "hx_i": [hx_i],
            "hy_i": [hy_i],
            "length_i": [vehicle_length],
            "width_i": [vehicle_width],
        })

        # Compute TTC using the restricted zone waypoint
        ttc_values = TTC_with_zone(samples, restricted_zone_wp, toreturn="values")

        # # Debug: Print detailed information
        # print(
        #     f"Restricted Zone Info: Position: ({restricted_zone_wp.transform.location.x}, {restricted_zone_wp.transform.location.y})")
        # print(
        #     f" - Heading (Yaw): {restricted_zone_wp.transform.rotation.yaw}, Lane Width: {restricted_zone_wp.lane_width}")
        # print("TTC Values:", ttc_values)

        # Get the minimum TTC value
        min_ttc = ttc_values[0]

        # # Debug: Print the final minimum TTC
        # print(f"Final Minimum TTC to Restricted Zone: {min_ttc}")

        # Return classified TTC
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


# Example usage
if __name__ == "__main__":
    client = carla.Client("localhost", 5000)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)
    DSLScene = None
    vehicles = world.get_actors().filter("*vehicle.*")
    ads_vehicle = vehicles[0]
    parser = DSL2Parser(world)
    parser.set_ads(ads_vehicle)
    parser.get_actors()
    parser.previous_scene = DSLScene
    scene = parser.parse_scene()
    DSLScene = scene
