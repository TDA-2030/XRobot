from threading import Lock
# from multiprocessing import Lock
# from functools import wraps
from time import sleep

import minimalmodbus
import serial
import serial.tools.list_ports
import traceback
from utils import logger as print


def get_serial_port():
    port = serial.tools.list_ports.comports()
    return [p.__dict__ for p in port]


class RS485:
    def __init__(self, slave_addr=6) -> None:
        self.slave_addr = slave_addr
        self.is_connected:bool = False
        self.lock = Lock()

    def connect(self, ip, port):
        try:
            print(f"Connecting RS485 addr:{self.slave_addr} {ip}:{port}")
            # self.master = modbus_tcp.TcpMaster(ip, port)
            # self.master.set_timeout(5.0)
            self.ip = ip
            self.port = port
            self.ser = serial.Serial(ip, port, timeout=0.5)
            self.master = minimalmodbus.Instrument(self.ser, self.slave_addr)

            self.is_connected = True
            return True
        except:
            traceback.print_exc()
            return False

    def disconnect(self):
        self.ser.close()
        del self.master
        del self.ser
        self.is_connected = False

    def read_register(self, addr: int):
        ret = self.master.read_register(addr)
        return ret

    def write_register(self, addr: int, value: int):
        ret = self.master.write_register(addr, value, functioncode=6)
        return ret

# def RS485_locker(func):
#     """
#     Multiple devices are mount on a 485 bus, and access to devices on the bus requires locking
#     """
#     @wraps(func)
#     def wrapper(*args, **kwargs):
#         lock.acquire()
#         # print("acquire")
#         r = func(*args, **kwargs)
#         # sleep(0.05) 
#         lock.release()
#         # print("release")
#         return r
#     return wrapper
