#!/usr/bin/env python3.8
import socket
import threading
import json
import time

'''
json:
{
    "cmd": 0,
    "data":{
        "pose":{
            "x":0.0,
            "y":0.0,
            "z":0.0,
            "pitch":0.0,
            "yaw":0.0,
            "roll":0.0,
        },
        "current-state":0,
        "engage":true,
        "msg":""
    } 
}
'''


class CMD:
    RESPONSE = 0
    PUB_INITIAL_POSE = 1
    PUB_GOAL_POSE = 3
    RESET = 4
    ENGAGE = 5
    STATE_CHANGE = 6


class AUTOWARE_STATE:
    UNLOADED = 0
    INITIALIZING = 1
    WAITING_FOR_ROUTE = 2
    PLANNING = 3
    WAITING_FOR_ENGAGE = 4
    DRIVING = 5
    ARRIVED_GOAL = 6
    FINALIZING = 7
    
state_string=["UNLOADED","INITIALIZING",
              "WAITING_FOR_ROUTE","PLANNING",
              "WAITING_FOR_ENGAGE","DRIVING",
              "ARRIVED_GOAL","FINALIZING"]


class RemoteControllerClient:
    current_state = AUTOWARE_STATE.UNLOADED
    engage_when_ready = False

    def __init__(self, host, port) -> None:
        self.host = host
        self.port = port
        self.mission_completed_callback = None
        self.initial_finish_callback = None

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_to_remote_server()

        listen_thread = threading.Thread(
            target=self.wait_msg_thread)
        listen_thread.start()

    def connect_to_remote_server(self, retry_after=2, retry_times=10):
        print(f"[+] Connecting to {self.host}:{self.port}")
        retries = 0
        self.connected = False

        while not self.connected and retries < retry_times:
            try:
                self.client_socket = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.host, self.port))
                self.connected = True
            except ConnectionRefusedError:
                print("[-] Connection refused. Retrying in {} seconds...".format(retry_after))
                time.sleep(retry_after)
                retries += 1

        if self.connected:
            print("[+] Connected successfully!")
        else:
            print("[-] Failed to connect after {} retries.".format(retry_times))

        listen_thread = threading.Thread(
            target=self.wait_msg_thread)
        listen_thread.start()
        

    def wait_msg_thread(self):
        try:
            while True:
                recv_data = self.client_socket.recv(1024).decode('utf-8')
                if not recv_data:
                    print("Server closed the connection.")
                    self.connected = False
                    break
                try:
                    recv_dict = json.loads(recv_data)
                    # print("Received JSON data:", recv_dict)
                    self.handle_recv_dict(recv_dict)
                except json.JSONDecodeError as e:
                    print("Error decoding JSON data:", e)
        finally:
            self.client_socket.close()

    def handle_recv_dict(self, recv_dict):
        cmd = recv_dict["cmd"]
        if cmd == CMD.RESPONSE:
            self.current_state = recv_dict["data"]["current-state"]
        elif cmd == CMD.STATE_CHANGE:
            last_state = self.current_state
            self.current_state = recv_dict["data"]["current-state"]
            self.state_change_callback(self.current_state, last_state)
            print(f"[+] state changed to {state_string[self.current_state]}")

    def set_goal(self, x, y, z, roll, pitch, yaw):
        data_dict = {
            "cmd": CMD.PUB_GOAL_POSE,
            "data": {
                "pose": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "pitch": pitch,
                    "yaw": yaw,
                    "roll": roll,
                }
            },
        }
        try:
            self.client_socket.send(
                bytes(json.dumps(data_dict, indent=4), encoding="utf-8"))
        except Exception as e:
            print(f"Error: {e}")

    def set_initialpose(self, x, y, z, roll, pitch, yaw):
        data_dict = {
            "cmd": CMD.PUB_INITIAL_POSE,
            "data": {
                "pose": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "pitch": pitch,
                    "yaw": yaw,
                    "roll": roll,
                }
            },
        }
        try:
            self.client_socket.send(
                bytes(json.dumps(data_dict, indent=4), encoding="utf-8"))
        except Exception as e:
            print(f"Error: {e}")

    def set_engage(self, engage):
        data_dict = {
            "cmd": CMD.ENGAGE,
            "data": {
                "engage": "true" if engage else "false"
            },
        }
        try:
            self.client_socket.send(
                bytes(json.dumps(data_dict, indent=4), encoding="utf-8"))
        except Exception as e:
            print(f"Error: {e}")

    def set_start_driving(self):
        if self.current_state == AUTOWARE_STATE.WAITING_FOR_ENGAGE:
            self.set_engage(True)
        elif self.current_state == AUTOWARE_STATE.INITIALIZING:
            print("[-] initial not done")
        elif self.current_state == AUTOWARE_STATE.WAITING_FOR_ROUTE:
            print("[-] goal not published")
        elif self.current_state == AUTOWARE_STATE.PLANNING:
            print("[-] planning")
        elif self.current_state == AUTOWARE_STATE.DRIVING:
            print("[-] already in driving")
        elif self.current_state == AUTOWARE_STATE.ARRIVED_GOAL:
            print("[-] initial not done")

    def set_goal_and_engage(self, pose, timeout=5):
        self.set_goal(pose)
        wait_time = 0
        self.engage_when_ready = True
        while self.current_state != AUTOWARE_STATE.WAITING_FOR_ENGAGE:
            time.sleep(0.01)
            wait_time += 0.01
            if wait_time >= timeout:
                print("[-] timeout while planning")
                self.engage_when_ready = False
                return False
        return True

    def state_change_callback(self, current_state, last_state):
        if current_state == AUTOWARE_STATE.ARRIVED_GOAL and \
                last_state == AUTOWARE_STATE.DRIVING:
            if self.mission_completed_callback != None:
                self.mission_completed_callback()
            print("[+] Mission completed!")
        if current_state == AUTOWARE_STATE.WAITING_FOR_ENGAGE and \
                last_state == AUTOWARE_STATE.WAITING_FOR_ROUTE:
            if self.engage_when_ready:
                self.set_start_driving()

    def wait_autoware_launch_ready(self, timeout=20):
        wait_time = 0
        while self.current_state == AUTOWARE_STATE.UNLOADED:
            time.sleep(0.01)
            wait_time += 0.01
            print("[*] Waitting for autoware to launch" +
                  "."*int(wait_time*0.5) + "\r", end="")
            if wait_time >= timeout:
                print("\n[-] timeout while waitting launch")
                return False
            if self.connected == False:
                print("\n[-] connection lost while waitting launch")
                return False
        print("\n[+] Autoware universe has been launched")
        return True

    def wait_autoware_initial_ready(self, timeout=20):
        wait_time = 0
        if self.current_state != AUTOWARE_STATE.INITIALIZING:
            print("[-] not initializing.")
            return False
        while self.current_state == AUTOWARE_STATE.INITIALIZING:
            time.sleep(0.01)
            wait_time += 0.01
            print("[*] Waitting for autoware to initialize" +
                  "."*int(wait_time*0.5) + "\r", end="")
            if wait_time >= timeout:
                print("\n[-] timeout while waitting initializing")
                return False
            if self.connected == False:
                print("\n[-] connection lost while waitting initializing")
                return False
                
        print("\n[+] Autoware universe has been initialized")
        return True

    def set_mission_completed_callback(self, callback_func):
        self.mission_completed_callback = callback_func

    def set_initial_finish_callback(self, callback_func):
        self.initial_finish_callback = callback_func


def main():
    remote_client = RemoteControllerClient("172.17.0.2", 60052)
    remote_client.wait_autoware_launch_ready()
    remote_client.wait_autoware_initial_ready()
    remote_client.set_engage(True)
    while True:
        pass


if __name__ == '__main__':
    main()
