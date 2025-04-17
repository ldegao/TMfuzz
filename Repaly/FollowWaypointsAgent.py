from agents.navigation.behavior_agent import BehaviorAgent


class FollowWaypointsAgent:
    def __init__(self, vehicle, waypoints, behavior="normal"):
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.agent = BehaviorAgent(vehicle, behavior)
        self.current_index = 0

    def update_destination(self):
        if self.current_index < len(self.waypoints):
            destination = self.waypoints[self.current_index]
            self.agent.set_destination(destination.location)
            self.current_index += 1
            return destination
        else:
            return None

    def run_step(self):
        control = self.agent.run_step()
        return control
