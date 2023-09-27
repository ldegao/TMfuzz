""" Global States """
import signal


class ExecState:
    def __init__(self):
        # exec states
        self.client = None
        self.world = None
        self.G = None
        self.first_frame_id = 0
        self.first_sim_elapsed_time = 0
        self.sim_start_time = 0
        self.num_frames = 0
        self.elapsed_time = 0
        self.end = False
        self.proc_state = None


class ScenarioState:
    """
    A state object stores raw data and events captured from the simulator,
    including vehicular states.
    Pass a reference to this object to Scenario.run_test() as a parameter.
    """

    def __init__(self):
        # exec states
        self.client = None
        self.world = None
        self.G = None

        self.first_frame_id = 0
        self.first_sim_elapsed_time = 0
        self.sim_start_time = 0
        self.num_frames = 0
        self.elapsed_time = 0
        self.end = False

        self.other = None
        self.mutation = 0

        # failure states
        self.spawn_failed = False
        self.spawn_failed_object = None

        # error states
        self.crashed = False
        self.collision_event = None
        self.stuck = False
        self.stuck_duration = 0
        self.laneinvaded = False
        self.laneinvasion_event = []
        self.speeding = False
        self.speed = []
        self.speed_lim = []
        self.on_red = False
        self.on_red_speed = []
        self.red_violation = False

        # other error states, e.g., segfault
        self.other_error = False
        self.other_error_val = 0
        self.signal = 0

        # control states
        self.cont_throttle = []
        self.cont_brake = []
        self.cont_steer = []
        self.steer_angle_list = []
        self.yaw_list = []
        self.yaw_rate_list = []
        self.lat_speed_list = []
        self.lon_speed_list = []

        self.min_dist = 99998

        self.autoware_cmd = ""
        self.autoware_goal = ""
        self.drawn_points = set()

    def sig_handler(self, signum, frame):
        print("[-] something happened: {}".format(signal.signum.name))
        self.other = True
        self.signal = signum
