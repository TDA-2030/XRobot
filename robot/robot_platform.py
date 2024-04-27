

from typing import Tuple
from . import belt
from . import gripper
from . import robot_driver
from .common import get_serial_port
import time

def scan_correct_port():
    ports = get_serial_port()
    _ok = [False, False]
    res={}

    for p in ports:
        com = p['device']
        if not _ok[0]:
            d = gripper.Gripper()
            if d.connect(com, 115200):
                time.sleep(0.1)
                d.disconnect()
                res['gripper'] = com
                _ok[0] = True
                continue
        if not _ok[1]:
            d = belt.BeltEncoder(200, 10, 1024, 50)
            try:
                d.connect(com, 38400)
                time.sleep(0.1)
                d.disconnect()
                res['encoder'] = com
                _ok[1] = True
                continue
            except:
                pass
    return res

class Platform():
    def __init__(self) -> None:
        self.robot = robot_driver.RobotDriver()
        self.gripper = gripper.Gripper()
        self.belt = belt.Belt(6)
        self.encoder = belt.BeltEncoder(200, 10, 1024, 50)
        self.is_connected = False

    def connect(self, robot_ip:Tuple[str, int], gripper_ip:Tuple[str, int], belt_ip:Tuple[str, int], encoder_ip:Tuple[str, int]):
        """
        Connecting the robot platform
        robot_ip: OMRON Robot ACE server IP
        """
        ret_msg = ""

        while True:
            if self.gripper.connect(*gripper_ip):
                self.gripper.axis.set_servo_on_off(True)
            else:
                ret_msg += 'Unable to connect gripper'
                break 

            if self.belt.connect(*belt_ip):
                pass
            else:
                ret_msg += 'Unable to connect conveyor belt'
                self.gripper.disconnect()
                break

            if self.encoder.connect(*encoder_ip):
                pass
            else:
                ret_msg += 'Unable to connect encoder'
                self.gripper.disconnect()
                self.belt.disconnect()
                break

            self.robot.set_ipaddress(*robot_ip)
            if self.robot.connect():
                time.sleep(0.2)
                self.robot.info()
                if not self.robot.power_state:
                    print("init robot power")
                    self.robot.set_power(True)
            else:
                ret_msg += 'Unable to connect robot'
                self.gripper.disconnect()
                self.belt.disconnect()
                self.encoder.disconnect()
                break

            self.is_connected = True
            break

        return ret_msg

    def disconnect(self):
        """
        Disconnect the robot platform
        """
        if self.robot.is_connected:
            self.robot.disconnect()
        if self.gripper.is_connected:
            self.gripper.disconnect()
        if self.belt.is_connected:
            self.belt.disconnect()
        if self.encoder.is_connected:
            self.encoder.disconnect()
        self.is_connected = False

