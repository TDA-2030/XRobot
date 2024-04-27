from pathlib import Path
import sys
from platform import system

DIR = Path(__file__).resolve().parent
if str(DIR) not in sys.path:
    sys.path.append(str(DIR)) 

import motormaster

from utils import logger as print

# Document: http://doc.rmaxis.com/docs/sdk/api_v2/

class Gripper:
    def __init__(self) -> None:
        self.is_connected = False

    def connect(self, ip:str, port:int)->bool:
        print(f"Connecting Gripper at {ip}:{port}")
        try:
            if system() == "Windows" and ip.upper().startswith('COM') and int(ip[3:]) > 8:
                ip = '\\\\.\\' + ip # the "\\.\COMx" format is required for devices other than COM1-COM8
            self.axis = motormaster.create_axis_modbus_rtu(ip, port, 2)
            # self.axis = motormaster.create_axis_modbus_tcp(ip, port, 2)
            print(f"RM Version: {self.axis.get_version()}")
            self.is_connected = True
            return True
        except:
            print("Gripper connect failed")
        return False

    def disconnect(self):
        motormaster.destroy_axis(self.axis)
        self.is_connected = False

    def sucker(self, is_enable:bool):
        """
        is_enable: Used to indicate whether the solenoid valve is enabled. True means enabled (open the solenoid valve), False means disabled (closed solenoid valve).
        """
        cmd_index = 1 if is_enable else 2
        self.axis.trig_command(cmd_index)

    def _position_calibrate(self, position:float)->float:
        """
        position: 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130
        real gap: 146, 140.47, 135.4, 129.1, 122.46, 115.2, 107.8, 99.3, 90.5, 80.9, 70.4, 58.5, 45.8, 30

        y = -0.0053x2 - 0.1661x + 138.76
        """
        bending_width = 15 # The sum of the inner widths of the finger bends
        if position >= 146:
            return 0
        if position <= 30:
            return 130
        return -0.0037*(position*position) - 0.3908*position + 144.8 - bending_width

    def push(self, force: float, distance: float, velocity: float):
        """
        force: 0~100
        """
        self.axis.push(force, distance, velocity)

    def move_absolute(self, position: float, velocity: float, acceleration: float, deacceleration: float, band: float):
        position = self._position_calibrate(position)
        self.axis.move_absolute(position, velocity, acceleration, deacceleration, band)

    def move_relative(self, position: float, velocity: float, acceleration: float, deacceleration: float, band: float):
        self.axis.move_relative(position, velocity, acceleration, deacceleration, band)

    def get_position(self) -> float:
        return self.axis.position()

if __name__ == "__main__":
    import time
    g = Gripper()
    g.connect("COM10", 115200)
    g.axis.set_servo_on_off(True)
    print(f"position={g.axis.position():.2f}")
    print(f"velocity={g.axis.velocity():.2f}")
    print(f"torque={g.axis.torque():.2f}")
    # print(f"force={g.axis.force_sensor():.2f}")
    while True:
        try:
            for i in range(1, 11):
                f=i*10
                print(f"push with {f}% force")
                g.push(f, 100, 400)
                g.sucker(True)
                time.sleep(3)
                g.move_absolute(0, 400, 3000, 3000, 0.1)
                time.sleep(1)
                g.sucker(False)
                time.sleep(3)
        except KeyboardInterrupt:
            g.sucker(False)
            break
    g.disconnect()
