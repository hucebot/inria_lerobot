import os
import time
import json
import numpy as np
import cv2
import threading
import queue

from scservo_sdk import *

from .utils.colors import *
from .utils.feetech_motor import FeetechMotor
from .utils.constants import *
from .configuration.configs import *
import pathlib

class So100Robot:
    """
    Manages the control of a follower robotic arm and optionally a leader arm.
    Optionally captures and saves camera frames to a dataset.
    """
    def __init__(self, require_leader_arm=True):
        base = pathlib.Path(__file__).parent.resolve() / "configuration" / "calibration"
        with open(base / "main_leader.json", "r") as f:
            self.calib_leader = json.load(f)
        with open(base / "main_follower.json", "r") as f:
            self.calib_follower = json.load(f)

        motor_names = self.calib_follower["motor_names"]
        self.ids = list(range(1, len(motor_names) + 1))


        self.require_leader_arm = require_leader_arm
        self.port_follower = PortHandler(PORT_FOLLOWER)
        if not self.port_follower.openPort():
            print(bcolors.FAIL + "Failed to open follower port." + bcolors.ENDC)
            quit()
        self.port_follower.setBaudRate(BAUDRATE)
        print(bcolors.OKGREEN + "Follower port opened successfully." + bcolors.ENDC)
        if self.require_leader_arm:
            self.port_leader = PortHandler(PORT_LEADER)
            if not self.port_leader.openPort():
                print(bcolors.FAIL + "Failed to open leader port." + bcolors.ENDC)
                quit()
            self.port_leader.setBaudRate(BAUDRATE)
            print(bcolors.OKGREEN + "Leader port opened successfully." + bcolors.ENDC)
        else:
            self.port_leader = None
        self.leader_arm, self.follower_arm = self.create_arms()
        self.initial_positions_leader = []
        self.initial_positions_follower = []
        self.dataset_records = []
        self.current_follower_positions = []
        self.current_follower_speeds = []
        self.current_target_angles = []
        self.video_captures = []
        self.cameras_in_use = []
        self.latest_frames = None
        self.frame_queue = queue.Queue()
        self.stop_threads = False
        self.camera_thread = None
        self.robot_thread = None
        self.init_cameras()
        self.read_initial_state()

    def create_arms(self):
        follower_arm = [
            FeetechMotor(
                ID,
                self.port_follower,
                PacketHandler(PROTOCOL_END),
                calibration=self.calib_follower,
                calib_idx=ID-1
            )
            for ID in self.ids
        ]
        if self.require_leader_arm:
            leader_arm = [
                FeetechMotor(
                    ID,
                    self.port_leader,
                    PacketHandler(PROTOCOL_END),
                    calibration=self.calib_leader,
                    calib_idx=ID-1
                )
                for ID in self.ids
            ]
        else:
            leader_arm = []
        return leader_arm, follower_arm


    def init_cameras(self):
        """Initializes cameras defined in config.py."""
        if not CAMERAS:
            print(bcolors.WARNING + "No cameras defined in config.py." + bcolors.ENDC)
            return
        for cam_conf in CAMERAS:
            cap = cv2.VideoCapture(cam_conf['camera_id'])
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_conf['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_conf['height'])
            cap.set(cv2.CAP_PROP_FPS, cam_conf['fps'])
            if cap.isOpened():
                self.video_captures.append(cap)
                self.cameras_in_use.append(cam_conf['name'])
                print(bcolors.OKGREEN + f"Camera '{cam_conf['name']}' opened." + bcolors.ENDC)
            else:
                print(bcolors.FAIL + f"Error opening camera '{cam_conf['name']}'." + bcolors.ENDC)

    def camera_thread_func(self):
        """Captures frames from cameras and puts them in a queue."""
        while not self.stop_threads:
            frames = []
            for idx, cap in enumerate(self.video_captures):
                ret, frame = cap.read()
                if ret:
                    cv2.imshow(self.cameras_in_use[idx], frame)
                frames.append(frame if ret else None)
            cv2.waitKey(1)
            self.frame_queue.put(frames)
            time.sleep(1 / RATE)

    def get_leader_positions(self):
        """Returns the current angles of leader motors or zeros if disabled."""
        if not self.require_leader_arm:
            return [0.0] * len(self.ids)
        return [motor.read_position() for motor in self.leader_arm]

    def get_follower_positions(self):
        """Returns the current angles of follower motors."""
        return [motor.read_position() for motor in self.follower_arm]

    def get_follower_speeds(self):
        """Returns the current speeds of follower motors."""
        return [motor.read_speed() for motor in self.follower_arm]

    def set_follower_positions(self, angles):
        """Converts angles to raw values and writes them to follower motors."""
        for motor, angle in zip(self.follower_arm, angles):
            target_raw = motor.angle_to_raw(angle)
            motor.write_position(target_raw)

    def robot_thread_func(self, record_dataset=False, dataset_path=""):
        """Main loop for controlling the robot."""
        while not self.stop_threads:
            try:
                try:
                    camera_frames = self.frame_queue.get_nowait()
                except queue.Empty:
                    camera_frames = []

                leader_positions = self.get_leader_positions()
                self.set_follower_positions(leader_positions)
                follower_positions = self.get_follower_positions()
                follower_speeds    = self.get_follower_speeds()
                self.current_follower_positions = follower_positions
                self.current_follower_speeds    = follower_speeds

                for idx, motor_id in enumerate(self.ids):
                    lp = leader_positions[idx] if self.require_leader_arm else 0.0
                    fp = follower_positions[idx]
                    print(
                        bcolors.OKBLUE + f"Motor {motor_id}:" + bcolors.ENDC,
                        f"Leader → {lp:6.2f}° |",
                        bcolors.OKGREEN + f"Follower → {fp:6.2f}°" + bcolors.ENDC
                    )

                if record_dataset:
                    self.record_dataset(camera_frames)
                    status = bcolors.WARNING + " Recording " + bcolors.ENDC
                else:
                    status = bcolors.WARNING + " Teleoperation " + bcolors.ENDC

                print("-" * 10 + status + "-" * 10)
                time.sleep(1 / RATE)

            except KeyboardInterrupt:
                print(bcolors.WARNING + "Keyboard interruption. Ending loop..." + bcolors.ENDC)
                break
            except Exception as e:
                print(bcolors.FAIL + "Error in robot loop: " + str(e) + bcolors.ENDC)
                break

        self.close_ports()
        if record_dataset:
            self.save_dataset_to_npy(dataset_path=dataset_path)


    def record_dataset(self, camera_frames):
        """Records a single step: timestamp, target angles, speeds, and camera frames."""
        timestamp = time.time()
        data_tuple = (
            timestamp,
            self.current_target_angles.copy(),
            self.current_follower_speeds.copy(),
            camera_frames
        )
        self.dataset_records.append(data_tuple)

    def save_dataset_to_npy(self, dataset_path="datasets/"):
        """Saves dataset (timestamps, positions, speeds, frames) as an .npy file."""
        if not os.path.exists(dataset_path):
            os.makedirs(dataset_path)
        existing_npy_files = [f for f in os.listdir(dataset_path) if f.endswith('.npy')]
        total_files = len(existing_npy_files)
        zero_padded_filename = f"{total_files:05d}.npy"
        full_path = os.path.join(dataset_path, zero_padded_filename)
        timestamps = [rec[0] for rec in self.dataset_records]
        positions  = [rec[1] for rec in self.dataset_records]
        speeds     = [rec[2] for rec in self.dataset_records]
        all_frames = [rec[3] for rec in self.dataset_records]
        data_dict = {
            "timestamps": timestamps,
            "positions": positions,
            "speeds": speeds,
            "camera_frames": all_frames
        }
        np.save(full_path, data_dict, allow_pickle=True)
        print(bcolors.OKGREEN + f"Dataset saved to '{full_path}' with {len(self.dataset_records)} records." + bcolors.ENDC)

    def replay_episode(self, replay_episode, dataset_path):
        """
        Replays a recorded episode from a .npy file, setting follower positions over time
        and displaying the saved camera frames.
        """
        if not os.path.exists(dataset_path):
            print(bcolors.FAIL + f"Dataset not found at: {dataset_path}" + bcolors.ENDC)
            return
        print(bcolors.WARNING + f"Replaying episode from: {dataset_path}" + bcolors.ENDC)
        data_dict = np.load(f"{dataset_path}/{replay_episode}", allow_pickle=True).item()
        timestamps = data_dict["timestamps"]
        all_positions = data_dict["positions"]
        all_frames = data_dict["camera_frames"]
        start_time = time.time()
        base_timestamp = timestamps[0]
        for i in range(len(timestamps)):
            current_timestamp = timestamps[i]
            offset = current_timestamp - base_timestamp
            now = time.time()
            expected_time = start_time + offset
            wait_time = expected_time - now
            if wait_time > 0:
                time.sleep(wait_time)
            frames = all_frames[i]
            if frames:
                for idx, frame in enumerate(frames):
                    if frame is not None:
                        cv2.imshow(f"Replay_Camera_{idx}", frame)
                cv2.waitKey(1)
            self.set_follower_positions(all_positions[i])
            print(bcolors.WARNING + "[Replay] " + bcolors.ENDC + bcolors.OKBLUE +
                  f"Step {i+1}/{len(timestamps)} at offset {offset:.2f}s" +
                  bcolors.ENDC)
        self.close_ports()
        cv2.destroyAllWindows()

    def read_initial_state(self):
        """Reads the initial angles of leader (if any) and follower motors."""
        try:
            if self.require_leader_arm:
                for motor in self.leader_arm:
                    self.initial_positions_leader.append(motor.read_position())
            else:
                self.initial_positions_leader = [0.0]*len(self.ids)
            for motor in self.follower_arm:
                self.initial_positions_follower.append(motor.read_position())
            print(bcolors.OKGREEN + "Initial leader angles:  " + bcolors.ENDC, self.initial_positions_leader)
            print(bcolors.OKGREEN + "Initial follower angles:" + bcolors.ENDC, self.initial_positions_follower)
        except Exception as e:
            print(bcolors.FAIL + "Error reading initial state: " + str(e) + bcolors.ENDC)
            self.close_ports()
            quit()

    def close_ports(self):
        """Creates and runs So100Robot based on arguments."""
        try:
            for cap in self.video_captures:
                cap.release()
            for motor in self.follower_arm:
                motor.disable_torque()
            if self.port_leader is not None:
                self.port_leader.closePort()
            self.port_follower.closePort()
            cv2.destroyAllWindows()
            print(bcolors.WARNING + "Ports and cameras closed." + bcolors.ENDC)
        except Exception as e:
            print(bcolors.FAIL + "Error closing ports or cameras: " + str(e) + bcolors.ENDC)

    def start_threads(self, record_dataset=False, dataset_path=""):
        """Starts the camera and robot threads."""
        self.camera_thread = threading.Thread(target=self.camera_thread_func)
        self.robot_thread = threading.Thread(target=self.robot_thread_func, args=(record_dataset, dataset_path))
        self.camera_thread.start()
        self.robot_thread.start()

    def join_threads(self):
        """Waits for the camera and robot threads to finish."""
        self.camera_thread.join()
        self.robot_thread.join()

def run_robot(mode, record_dataset, replay_episode, dataset_path):
    if record_dataset and not dataset_path:
        print(bcolors.FAIL + "Error: --dataset_path cannot be empty if --record_dataset is True." + bcolors.ENDC)
        exit(1)
    require_leader_arm = (not replay_episode)
    robot = So100Robot(require_leader_arm=require_leader_arm)
    if replay_episode:
        robot.replay_episode(replay_episode, dataset_path)
    elif mode == "teleoperation":
        robot.start_threads(record_dataset=record_dataset, dataset_path=dataset_path)
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            robot.stop_threads = True
            robot.join_threads()
    else:
        print(bcolors.FAIL + f"Unknown mode: {mode}. Exiting..." + bcolors.ENDC)
        robot.close_ports()
