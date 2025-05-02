from scservo_sdk import *
from .constants import *

class FeetechMotor:
    def __init__(self, ID, port, packetHandler, *, calibration=None, calib_idx=None):
        self.ID = ID
        self.port = port
        self.packetHandler = packetHandler
        self.calibration = calibration
        self.calib_idx = calib_idx

    def read_position(self):
        pos, comm, err = self.packetHandler.read4ByteTxRx(
            self.port, self.ID, ADDR_STS_PRESENT_POSITION
        )
        if comm == COMM_SUCCESS and err == 0:
            raw_pos = SCS_LOWORD(pos)
            return self.raw_to_angle(raw_pos)
        else:
            return 0.0

    def raw_to_angle(self, raw_value):
        if self.calibration is not None:
            cal = self.calibration
            idx = self.calib_idx
            mode = cal["calib_mode"][idx]

            if mode == "DEGREE":
                drive = cal["drive_mode"][idx]
                homing = cal["homing_offset"][idx]
                signed = raw_value if raw_value < 2**31 else raw_value - 2**32
                if drive:
                    signed = -signed
                steps = signed + homing
                angle = steps / 2048.0 * 180.0
                return angle
            else:
                start = cal["start_pos"][idx]
                end = cal["end_pos"][idx]
                return (raw_value - start) / (end - start) * 100.0

        return 360.0 * (raw_value / 4095.0)

    def angle_to_raw(self, angle):
        if self.calibration is not None:
            cal = self.calibration
            idx = self.calib_idx
            mode = cal["calib_mode"][idx]

            if mode == "DEGREE":
                drive = cal["drive_mode"][idx]
                homing = cal["homing_offset"][idx]
                steps = angle / 180.0 * 2048.0
                steps = steps - homing
                if drive:
                    steps = -steps
                raw = int(steps) & 0xFFFFFFFF
                return raw
            else:
                start = cal["start_pos"][idx]
                end = cal["end_pos"][idx]
                raw = angle / 100.0 * (end - start) + start
                return int(raw)

        clamped = max(0.0, min(360.0, angle))
        return max(0, min(4095, int((clamped / 360.0) * 4095.0)))

    def write_position(self, position):
        self.packetHandler.write2ByteTxRx(
            self.port, self.ID, ADDR_STS_GOAL_POSITION, position
        )

    def enable_torque(self):
        self.packetHandler.write1ByteTxRx(
            self.port, self.ID, ADDR_SCS_TORQUE_ENABLE, 1
        )

    def disable_torque(self):
        self.packetHandler.write1ByteTxRx(
            self.port, self.ID, ADDR_SCS_TORQUE_ENABLE, 0
        )

    def read_speed(self):
        data, result, error = self.packetHandler.read4ByteTxRx(
            self.port, self.ID, ADDR_STS_PRESENT_SPEED
        )
        if result == COMM_SUCCESS and error == 0:
            present_speed_raw = SCS_HIWORD(data)
            present_speed_signed = SCS_TOHOST(present_speed_raw, 15)
            return present_speed_signed
        else:
            return 0.0
