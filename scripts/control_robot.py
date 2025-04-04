import os
import time
import json
import argparse

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
    def __init__(self):
        """
        Initializes the So100Robot class, setting up the leader and follower arms.
        """
        self.port_leader = PortHandler(PORT_LEADER)
        self.port_follower = PortHandler(PORT_FOLLOWER)

        if not self.port_leader.openPort() or not self.port_follower.openPort():
            print(bcolors.FAIL + "Failed to open one or both ports." + bcolors.ENDC)
            quit()

        print(bcolors.OKGREEN + "Both ports opened successfully." + bcolors.ENDC)

        self.port_leader.setBaudRate(BAUDRATE)
        self.port_follower.setBaudRate(BAUDRATE)

        self.leader_arm, self.follower_arm = self.create_arms()

        self.initial_positions_leader = []
        self.initial_positions_follower = []

        self.current_follower_positions = []
        self.current_follower_speeds = []

        self.read_initial_state()

    def create_arms(self):
        """
        Creates FeetechMotor objects for both the leader and follower arms using the global IDs.
        """
        leader_arm = [
            FeetechMotor(ID, self.port_leader, PacketHandler(PROTOCOL_END)) for ID in IDs
        ]
        follower_arm = [
            FeetechMotor(ID, self.port_follower, PacketHandler(PROTOCOL_END)) for ID in IDs
        ]
        return leader_arm, follower_arm

    def get_leader_positions(self):
        """
        Reads and returns the current angles (in degrees) of each motor on the leader arm.
        """
        return [motor.read_position() for motor in self.leader_arm]

    def get_follower_positions(self):
        """
        Reads and returns the current angles (in degrees) of each motor on the follower arm.
        """
        return [motor.read_position() for motor in self.follower_arm]

    def get_follower_speeds(self):
        """
        Reads and returns the current speeds of each motor on the follower arm.

        Note: This assumes you've implemented 'read_speed()' in FeetechMotor.
        """
        speeds = []
        for motor in self.follower_arm:
            speed_value = motor.read_speed()
            speeds.append(speed_value)
        return speeds

    def set_follower_positions(self, angles):
        """
        Writes the target positions (in degrees) to each motor in the follower arm.
        """
        for motor, angle in zip(self.follower_arm, angles):
            target_raw = motor.angle_to_raw(angle)
            motor.write_position(target_raw)

    def main_loop(self, record_dataset=False):
        """
        Main loop that continuously reads the leader arm's positions, calculates the target angles
        for the follower arm, sets those positions, and prints the current state.

        Args:
            record_dataset (bool): If True, records the dataset during operation.
        """
        while True:
            try:
                leader_positions = self.get_leader_positions()

                delta_angles = [
                    current - initial
                    for current, initial in zip(leader_positions, self.initial_positions_leader)
                ]

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
                        f"Leader angle: {leader_positions[motor_i]:.2f}, " +
                        bcolors.OKBLUE + f"Target angle: {target_angles[motor_i]:.2f}, " + bcolors.ENDC +
                        f"Delta angle: {delta_angles[motor_i]:.2f}, " +
                        bcolors.OKBLUE + f"Follower angle: {follower_positions[motor_i]:.2f}, " + bcolors.ENDC +
                        bcolors.OKBLUE + f"Follower speed: {follower_speeds[motor_i]}" + bcolors.ENDC
                    )

                if record_dataset:
                    self.record_dataset()

                print("-" * 70)
                time.sleep(1 / RATE)

            except Exception as e:
                print(bcolors.FAIL + "Error in main loop: " + str(e) + bcolors.ENDC)
                self.close_ports()
                quit()

    def record_dataset(self):
        """
        Placeholder for dataset recording functionality.
        """
        pass

    def read_initial_state(self):
        """
        Reads and stores the initial positions (in degrees) for both leader and follower arms.
        """
        try:
            for motor in self.leader_arm:
                angle = motor.read_position()
                self.initial_positions_leader.append(angle)

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
        Closes both leader and follower ports properly.
        """
        self.port_leader.closePort()
        self.port_follower.closePort()
        print(bcolors.WARNING + "Ports closed." + bcolors.ENDC)

def run_robot(mode: str, record_dataset: bool):
    """
    Depending on 'mode' and 'record_dataset', run the appropriate behavior.
    """
    robot = So100Robot()

    if mode == "teleoperation":
        print(bcolors.WARNING + "Running in TELEOPERATION mode." + bcolors.ENDC)
        try:
            robot.main_loop(record_dataset=record_dataset)

        except KeyboardInterrupt:
            pass
        finally:
            robot.close_ports()

        robot.close_ports()
    else:
        print(bcolors.FAIL + f"Unknown mode: {mode}. Exiting..." + bcolors.ENDC)
        robot.close_ports()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control the So100 Robot.")
    parser.add_argument(
        "--mode",
        type=str,
        default="teleoperation",
        help="Choose mode of operation (e.g., teleoperation or inference)."
    )
    parser.add_argument(
        "--record_dataset",
        action="store_true",
        default=False,
        help="If set, records dataset during operation."
    )

    args = parser.parse_args()

    run_robot(mode=args.mode, record_dataset=args.record_dataset)
