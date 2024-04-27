import socket, threading, traceback
from queue import Queue, Empty
from math import cos, sin
import numpy as np
import time

MODE_INC = 0
MODE_ABS = 1

ACK_OK = "ok"
ACK_ERROR = "error"
from utils import logger as print


class TCPClient:

    def __init__(self) -> None:
        self.connected = False
        self.recv_queue = Queue(10)
        self.lock = threading.Lock()

    def set_ipaddress(self, ip: str, port: int):
        self.addr = (ip, port)

    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.3)
            self.sock.connect(self.addr)
            self.sock.settimeout(None)
            self.recv_queue.queue.clear()
            self.thread = threading.Thread(target=self._recv, name='tcprecv')
            self.thread.setDaemon(True)
            self.thread.start()
            self.connected = True
            return True
        except Exception as e:
            traceback.print_exc()
        return False

    @property
    def is_connected(self) -> bool:
        return self.connected

    def _recv(self):
        while True:
            try:
                data = self.sock.recv(1024)
                if data:
                    self.recv_queue.put(data)

            except ConnectionAbortedError:
                print("disconnected")
                break
            except Exception as e:
                traceback.print_exc()
                break

    def _send(self, msg: str, timeout:float = 10) -> tuple:
        if not self.is_connected:
            print.warning("not connected")
            return ("", "")
        self.lock.acquire()
        data = "<{}>".format(msg.strip()).encode()
        self.sock.send(data)
        try:
            data: str = self.recv_queue.get(block=True, timeout=timeout)
            data = data.decode().strip()
            if not data.startswith('<') or not data.endswith('>'):
                print.error(f"cmd reply error:{data}")
            data = data[1:-1].split('|')
        except Empty:
            print.error("command no ack")
            data[0], data[1] = "", ""
        self.lock.release()
        return (data[0], data[1])

    def disconnect(self):
        self.sock.close()
        self.thread.join()
        self.connected = False


class MotionParam():
    def __init__(self, speed, accel, decel, wait=True, scurve=0) -> None:
        self.speed = speed
        self.accel = accel
        self.decel = decel
        self.wait = wait
        self.scurve = scurve

class RobotDriver(TCPClient):

    def __init__(self) -> None:
        super().__init__()
        self.robot_num: int = 0
        self.model_number: str = ""
        self.serial_numer: str = ""
        self.power_state: bool = False
        self.enable_status: bool = False

    def move(self, mode: int, **kwargs) -> bool:
        """
        positional arguments:
            mode: 0-relative position; 1-absolute position
        kwargs: 
            dx: The x value
            dy: The y value
            dz: The z value
            dr: The r value
            speed: The speed of movement, 100 represents the rating
            accel: Motion acceleration, 100 represents the rating
            decel: Motion deceleration, 100 represents the rating
            scurve: S-Curve [0, 8]
            straight: default to True
            wait: Whether to wait for the motion to finish before returning; defaults to True
        
        """
        if mode == MODE_INC:
            cmd = "MI:"
        else:
            if not ('dx' in kwargs or 'dx' in kwargs or 'dx' in kwargs):
                print(
                    'The absolute value mode must give three coordinates of xyz at the same time'
                )
                return False
            cmd = "MA:"

        timeout = 10.0
        if 'dx' in kwargs:
            cmd = cmd + f",x{kwargs['dx']}"
        if 'dy' in kwargs:
            cmd = cmd + f",y{kwargs['dy']}"
        if 'dz' in kwargs:
            cmd = cmd + f",z{kwargs['dz']}"
        if 'dr' in kwargs:
            cmd = cmd + f",r{kwargs['dr']}"

        if 'wait' in kwargs:
            cmd = cmd + f",w{1 if kwargs['wait'] else 0}"
            timeout = timeout if kwargs['wait'] else 5
        if 'speed' in kwargs:
            cmd = cmd + f",f{kwargs['speed']}"
        if 'accel' in kwargs:
            cmd = cmd + f",a{kwargs['accel']}"
        if 'decel' in kwargs:
            cmd = cmd + f",d{kwargs['decel']}"
        if 'scurve' in kwargs:
            cmd = cmd + f",s{kwargs['scurve']}"
        if 'straight' in kwargs:
            cmd = cmd + f",l{kwargs['straight']}"
        cmd = cmd.replace(":,", ":")  # remove redundant comma
        state, msg = self._send(cmd)
        if ACK_OK != state:
            print.error(f"Robot motion error: {msg}\n traceback:{traceback.format_stack()}")
        return True if ACK_OK == state else False

    def halt(self) -> bool:
        """
        Stop current movement
        """
        state, msg = self._send(f"HALT:{0}")
        return True if ACK_OK == state else False

    def set_power(self, enable: bool) -> bool:
        """
        Control the high-voltage power supply of the robot
        """
        state, msg = self._send(f"POWER:{int(enable)}")
        return True if ACK_OK == state else False

    def info(self) -> bool:
        """
        Read robot information
        """
        state, msg = self._send(f"R:i")
        msg: str
        msg = msg.split(",")
        for i in msg:
            if i.startswith("Robot Number:"):
                self.robot_num = int(i.split(': ')[1])
            elif i.startswith("Enable Status:"):
                self.enable_status = True if 'True' == i.split(
                    ': ')[1] else False
            elif i.startswith("Model Number:"):
                self.model_number = i.split(': ')[1]
            elif i.startswith("Serial Number:"):
                self.serial_numer = i.split(': ')[1]
            elif i.startswith("Power:"):
                self.power_state = True if 'True' == i.split(
                    ': ')[1] else False
        return True if ACK_OK == state else False

    def get_encoder(self)->int:
        """
        Get Conveyor encoder value
        !!! NotImplemented on Robot server !!!
        """
        raise NotImplementedError
        state, msg = self._send(f"R:e")
        msg: str
        if ACK_OK == state:
            msg = msg.split(': ')[1].split(' ')

        return True if ACK_OK == state else False 

    def get_world_position(self) -> np.ndarray:
        """
        Get the current pose of the robot
        """
        state, msg = self._send(f"R:p")
        msg: str
        if ACK_OK == state:
            msg = msg.split(': ')[1].split(' ')
            pos = np.array(msg, dtype=np.float32)
            return pos
        return None
