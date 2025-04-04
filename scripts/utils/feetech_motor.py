from scservo_sdk import *
from utils.constants import *


class FeetechMotor:
    def __init__(self, ID, port, packetHandler):
        self.ID = ID
        self.port = port
        self.packetHandler = packetHandler

    def read_position(self):
        pos, comm, err = self.packetHandler.read4ByteTxRx(self.port, self.ID, ADDR_STS_PRESENT_POSITION)
        if comm == COMM_SUCCESS and err == 0:
            raw_pos = SCS_LOWORD(pos)
            angle = self.raw_to_angle(raw_pos)
            return angle
        else:
            return 0.0
        
    def raw_to_angle(self, raw_value):
        return 360.0 * (raw_value / 4095.0)
    
    def angle_to_raw(self, angle):
        clamped_angle = max(0, min(360, angle))
        return max(0, min(4095, int((clamped_angle / 360.0) * 4095)))
    
    def write_position(self, position):
        self.packetHandler.write2ByteTxRx(self.port, self.ID, ADDR_STS_GOAL_POSITION, position)

    def enable_torque(self):
        self.packetHandler.write1ByteTxRx(self.port, self.ID, ADDR_SCS_TORQUE_ENABLE, 1)

    def read_speed(self):
        data, result, error = self.packetHandler.read4ByteTxRx(self.port, self.ID, ADDR_STS_PRESENT_POSITION)
        if result == COMM_SUCCESS and error == 0:
            present_speed_raw = SCS_HIWORD(data)
            present_speed_signed = SCS_TOHOST(present_speed_raw, 15)
            return present_speed_signed
        else:
            return 0.0


        
