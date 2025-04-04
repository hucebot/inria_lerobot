import os
import time
import json
import argparse
import numpy as np

from scservo_sdk import *
from utils.colors import *
from utils.feetech_motor import FeetechMotor
from config import *
from utils.constants import *

with open("/inria_lerobot/calibration/main_follower.json", "r") as f:
    calib_follower = json.load(f)

motor_names = calib_follower["motor_names"]
IDs = list(range(1, len(motor_names) + 1))

class So100Robot:
    """
    This class manages the control of a follower robotic arm, and optionally a leader arm.
    It handles serial port connections, motor initialization, data recording, and replaying
    of episodes based on saved data.
    """

    def __init__(self, require_leader_arm: bool = True):
        """
        Initialize the So100Robot.

        :param require_leader_arm: Indicates whether to use a leader arm. If True, creates and
                                   connects the leader arm. If False, only uses the follower arm.
        """
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

        self.read_initial_state()

    def create_arms(self):
        """
        Create FeetechMotor objects for the follower arm, and for the leader arm if required.

        :return: A tuple (leader_arm, follower_arm).
                 leader_arm: List of FeetechMotor objects (empty if leader arm is not used).
                 follower_arm: List of FeetechMotor objects for the follower arm.
        """
        follower_arm = [
            FeetechMotor(ID, self.port_follower, PacketHandler(PROTOCOL_END)) for ID in IDs
        ]

        if self.require_leader_arm:
            leader_arm = [
                FeetechMotor(ID, self.port_leader, PacketHandler(PROTOCOL_END)) for ID in IDs
            ]
        else:
            leader_arm = []

        return leader_arm, follower_arm

    def get_leader_positions(self):
        """
        Retrieve the current position in degrees of each motor in the leader arm.
        If the leader arm is not used, return a list of zeros.

        :return: A list of angles (floats) for each motor of the leader arm, or zeros if disabled.
        """
        if not self.require_leader_arm:
            return [0.0] * len(IDs)
        return [motor.read_position() for motor in self.leader_arm]

    def get_follower_positions(self):
        """
        Retrieve the current position in degrees of each motor in the follower arm.

        :return: A list of angles (floats) for each motor of the follower arm.
        """
        return [motor.read_position() for motor in self.follower_arm]

    def get_follower_speeds(self):
        """
        Retrieve the current speed of each motor in the follower arm.

        :return: A list of speeds (integers) for each motor of the follower arm.
        """
        speeds = []
        for motor in self.follower_arm:
            speed_value = motor.read_speed()
            speeds.append(speed_value)
        return speeds

    def set_follower_positions(self, angles):
        """
        Convert angles in degrees to raw position values and send them to the follower motors.

        :param angles: List of angles in degrees to move each follower motor to.
        """
        for motor, angle in zip(self.follower_arm, angles):
            target_raw = motor.angle_to_raw(angle)
            motor.write_position(target_raw)

    def main_loop(self, record_dataset=False, dataset_task=""):
        """
        Main control loop. Continuously reads positions from the leader arm (if enabled),
        calculates the difference relative to the initial leader angles, and sets the
        follower arm to match that difference. Prints current positions and speeds.

        :param record_dataset: If True, data will be recorded every iteration.
        :param dataset_task: Task name or label used for dataset organization.
        """
        while True:
            try:
                leader_positions = self.get_leader_positions()

                delta_angles = [
                    current - initial
                    for current, initial in zip(leader_positions, self.initial_positions_leader)
                ] if self.require_leader_arm else [0]*len(IDs)

                target_angles = [
                    init_follower + delta
                    for init_follower, delta in zip(self.initial_positions_follower, delta_angles)
                ]

                self.set_follower_positions(target_angles)

                follower_positions = self.get_follower_positions()
                follower_speeds = self.get_follower_speeds()
                self.current_follower_positions = follower_positions
                self.current_follower_speeds = follower_speeds

                for motor_i, motor_id in enumerate(IDs):
                    print(
                        bcolors.OKBLUE + f"Motor {motor_id}: " + bcolors.ENDC +
                        (
                            f"Leader angle: {leader_positions[motor_i]:.2f}, "
                            if self.require_leader_arm else "Leader angle: [disabled], "
                        ) +
                        bcolors.OKBLUE + f"Target angle: {target_angles[motor_i]:.2f}, " + bcolors.ENDC +
                        (
                            f"Delta angle: {delta_angles[motor_i]:.2f}, "
                            if self.require_leader_arm else ""
                        ) +
                        bcolors.OKBLUE + f"Follower angle: {follower_positions[motor_i]:.2f}, " + bcolors.ENDC +
                        bcolors.OKBLUE + f"Follower speed: {follower_speeds[motor_i]}" + bcolors.ENDC
                    )

                print("-" * 70)

                if record_dataset:
                    self.record_dataset(dataset_task=dataset_task)

                time.sleep(1 / RATE)

            except KeyboardInterrupt:
                print(bcolors.WARNING + "Keyboard interruption. Ending main loop..." + bcolors.ENDC)
                break
            except Exception as e:
                print(bcolors.FAIL + "Error in main loop: " + str(e) + bcolors.ENDC)
                break

        self.close_ports()

        if record_dataset:
            self.save_dataset_to_npy(dataset_task=dataset_task)

    def record_dataset(self, dataset_task=""):
        """
        Record a single dataset entry (timestamp, follower positions, follower speeds) to
        the internal list `dataset_records`.

        :param dataset_task: Task name or label for consistency (not used in the filename here).
        """
        timestamp = time.time()
        data_tuple = (
            timestamp,
            self.current_follower_positions.copy(),
            self.current_follower_speeds.copy()
        )
        self.dataset_records.append(data_tuple)

    def save_dataset_to_npy(self, dataset_task=""):
        """
        Save all recorded data (timestamps, positions, speeds) to a .npy file. The file
        will be placed in '/inria_lerobot/datasets/<dataset_task>/', with an incremented
        numeric filename.

        :param dataset_task: Task name or label, used to determine the folder path.
        """
        dataset_task = dataset_task.lower().replace(" ", "_")
        dir_path = os.path.join('/inria_lerobot/datasets/', dataset_task)
        
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

        existing_npy_files = [
            f for f in os.listdir(dir_path) if f.endswith('.npy')
        ]
        total_files = len(existing_npy_files)

        zero_padded_filename = f"{total_files:05d}.npy"
        full_path = os.path.join(dir_path, zero_padded_filename)

        timestamps = [rec[0] for rec in self.dataset_records]
        positions  = [rec[1] for rec in self.dataset_records]
        speeds     = [rec[2] for rec in self.dataset_records]

        data_dict = {
            "timestamps": timestamps,
            "positions": positions,
            "speeds": speeds,
        }

        np.save(full_path, data_dict, allow_pickle=True)
        
        print(bcolors.OKGREEN
            + f"Dataset saved to '{full_path}' with {len(self.dataset_records)} records."
            + bcolors.ENDC)

    def replay_episode(self, replay_episode, dataset_task):
        """
        Replay a recorded episode from a .npy file, moving the follower arm to the
        recorded positions at the correct timing.

        :param replay_episode: Name of the .npy file (without extension) to replay.
        :param dataset_task: Task name or label, used to determine the folder path.
        """
        dataset_task = dataset_task.lower().replace(" ", "_")
        dataset_path = os.path.join('/inria_lerobot/datasets/', dataset_task, replay_episode + ".npy")
        if not os.path.exists(dataset_path):
            print(bcolors.FAIL + f"Dataset not found at: {dataset_path}" + bcolors.ENDC)
            return
        print(bcolors.WARNING + f"Replaying episode from: {dataset_path}" + bcolors.ENDC)

        data_dict = np.load(dataset_path, allow_pickle=True).item()
        timestamps = data_dict["timestamps"]
        all_positions = data_dict["positions"]

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

            self.set_follower_positions(all_positions[i])

            print(bcolors.OKBLUE + f"[Replay] Step {i+1}/{len(timestamps)} at offset {offset:.2f}s" + bcolors.ENDC)

        self.close_ports()
        print(bcolors.OKGREEN + "Replay finished. Ports closed." + bcolors.ENDC)

    def read_initial_state(self):
        """
        Read and store the initial angles of the leader (if used) and follower arms.
        These angles serve as reference points for differential control in the main loop.
        """
        try:
            if self.require_leader_arm:
                for motor in self.leader_arm:
                    angle = motor.read_position()
                    self.initial_positions_leader.append(angle)
            else:
                self.initial_positions_leader = [0.0]*len(IDs)

            for motor in self.follower_arm:
                angle = motor.read_position()
                self.initial_positions_follower.append(angle)

            print(bcolors.OKGREEN + "Initial leader angles:  " + bcolors.ENDC, self.initial_positions_leader)
            print(bcolors.OKGREEN + "Initial follower angles:" + bcolors.ENDC, self.initial_positions_follower)

        except Exception as e:
            print(bcolors.FAIL + "Error reading initial state: " + str(e) + bcolors.ENDC)
            self.close_ports()
            quit()

    def close_ports(self):
        """
        Close the serial ports for both the leader (if exists) and follower arms.
        """
        try:
            if self.port_leader is not None:
                self.port_leader.closePort()
            self.port_follower.closePort()
            print(bcolors.WARNING + "Ports closed." + bcolors.ENDC)
        except:
            pass

def run_robot(mode: str, record_dataset: bool, replay_episode: str, dataset_task: str):
    """
    Create and run the So100Robot, depending on the mode and other arguments.

    :param mode: String indicating the operation mode (e.g., "teleoperation").
    :param record_dataset: Boolean, if True then the dataset is recorded during teleoperation.
    :param replay_episode: String name (without extension) of the .npy file to replay.
    :param dataset_task: String name or label for the dataset task (used for saving or replaying).
    """
    if record_dataset and not dataset_task:
        print(bcolors.FAIL + "Error: --dataset_task cannot be empty if --record_dataset is True." + bcolors.ENDC)
        exit(1)

    require_leader_arm = (not replay_episode)

    robot = So100Robot(require_leader_arm=require_leader_arm)

    if replay_episode:
        robot.replay_episode(replay_episode, dataset_task)
    elif mode == "teleoperation":
        print(bcolors.WARNING + "Running in TELEOPERATION mode." + bcolors.ENDC)
        robot.main_loop(record_dataset=record_dataset, dataset_task=dataset_task)
    else:
        print(bcolors.FAIL + f"Unknown mode: {mode}. Exiting..." + bcolors.ENDC)
        robot.close_ports()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control the So100 Robot.")
    parser.add_argument(
        "--mode",
        type=str,
        default="teleoperation",
        help="Choose the mode of operation (e.g., teleoperation or inference)."
    )
    parser.add_argument(
        "--record_dataset",
        type=bool,
        default=False,
        help="Record dataset? (true/false)"
    )
    parser.add_argument(
        "--dataset_task",
        type=str,
        default="",
        help="Task name or description, used to organize dataset files (e.g. 'Grasp a red cube'): /inria_lerobot/datasets/<task>/"
    )
    parser.add_argument(
        "--replay_episode",
        type=str,
        default="",
        help="Replay the specified .npy file in /inria_lerobot/datasets/<task>/ (without extension)."
    )

    args = parser.parse_args()

    run_robot(
        mode=args.mode,
        record_dataset=args.record_dataset,
        replay_episode=args.replay_episode,
        dataset_task=args.dataset_task
    )
