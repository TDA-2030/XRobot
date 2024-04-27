
from pathlib import Path
import sys
import time
import serial
import threading, traceback
DIR = Path(__file__).resolve().parent
if str(DIR) not in sys.path:
    sys.path.append(str(DIR)) 

from common import RS485, get_serial_port
from utils import logger as print


class Belt(RS485):

    def __init__(self, slave_addr=6) -> None:
        super().__init__(slave_addr)
        self.is_run:int = 0

    def disconnect(self):
        if self.is_run:
            self.stop()
        super().disconnect()

    def get_freq(self):
        self.lock.acquire()
        v = self.read_register(int(0x2103))
        self.lock.release()
        return v

    def move(self, dir:bool, speed:int):
        if speed > 100 or speed < -100:
            print("speed out range")
            return
        self.is_run = 1 if dir else -1
        dir = int(0x0012) if dir else int(0x0022)
        self.lock.acquire()
        self.write_register(int(0x2001), speed * 100)
        self.write_register(int(0x2000), dir)
        self.lock.release()

    def stop(self):
        self.lock.acquire()
        self.write_register(int(0x2000), int(0x0001))
        self.lock.release()
        self.is_run = 0

    def get_info(self):
        pass

    def get_error_code(self)->int:
        self.lock.acquire()
        v = self.read_register(int(0x2101))
        self.lock.release()
        return v


class BeltEncoder(RS485):
    def __init__(self, wheel_perimeter:float=200, slave_addr=10, resolution:int=1024, number_of_turns:int=50) -> None:
        super().__init__(slave_addr)
        self.wheel_perimeter = wheel_perimeter
        self.resolution = resolution
        self.max_num = resolution * number_of_turns
        self.last_dis = 0.
        self.last_time = 0
        self._distance = 0.
        self._speed = 0.

    def _run(self):
        while self.is_connected:
            try:
                self._get_distance()
            except:
                # traceback.print_exc()
                print.warning("Encoder read error!")
                # super().disconnect()
                time.sleep(0.3)
                # super().connect(self.ip, self.port)
                pass
            time.sleep(0.03)

    def disconnect(self):
        super().disconnect()
        self.thread.join()
        self._distance = 0.

    def connect(self, ip, port):
        ret = super().connect(ip, port)
        if ret:
            self.setup()
            self.thread = threading.Thread(target=self._run, name='encoder_run')
            self.thread.setDaemon(True)
            self.thread.start()
        return ret

    def write_register(self, addr:int, value:int):
        super().write_register(addr, value)
        time.sleep(0.2) # 延时一下，等待编码器响应

    def setup(self):
        self.lock.acquire()
        self.write_register(6, 0) #查询模式
        self.write_register(9, 1) #CCW 逆时针递增（编码器输出轴朝向观察者）
        self.write_register(10, 1000) #角速度采样时间 1000ms
        self.lock.release()

    @property
    def distance(self)->float:
        return self._distance

    def _get_distance(self)->float:
        t = time.perf_counter()
        l = self.read_register(0)
        h = self.read_register(1)
        v = h<<16 | l
        a = v * 360 / self.resolution # calculate angle
        a = a * self.wheel_perimeter / 360

        dd = a - self.last_dis
        if abs(dd) > 999:
            if dd < 0:
                dd += self.wheel_perimeter * (self.max_num / self.resolution)
            else:
                dd -= self.wheel_perimeter * (self.max_num / self.resolution)
                dd = -dd
        self._distance += dd
        self._speed = dd / (t - self.last_time)
        self.last_dis = a
        self.last_time = t

    @property
    def speed(self)->float:
        return self._speed
        # l = self.read_register(int(0x0020))
        # h = self.read_register(int(0x0021))
        # v = h<<16 | l
        # a = v / 32768 / 1  # calculate angle
        # return a * self.wheel_perimeter/1000
    
    def zero(self):
        self.lock.acquire()
        self.write_register(8, 1)
        self.lock.release()



if __name__ == "__main__":
    b = Belt()
    b.connect("COM9", 38400)
    print(f"frequecy:{b.get_freq()}Hz")
    while True:
        try:
            b.move(True, 10)
            time.sleep(5)
            b.stop()
            time.sleep(1)
            b.move(False, 10)
            time.sleep(5)
            b.stop()
            time.sleep(1)
        except KeyboardInterrupt:
            b.stop()
            break

