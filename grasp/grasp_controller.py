import time
from typing import Callable
from robot import robot_platform
from robot.robot_driver import MODE_ABS, MODE_INC, MotionParam
from .calibration import RobotCalibration
import numpy as np
import threading
import traceback
from queue import Queue, Full, Empty
from utils import logger, bbox_iou_eval
from utils import rotate_ract_in_imgframe
from utils.settings import Setting

class GraspController():
    TOOL_GRIPPER = 0
    TOOL_SUCKER = 1

    def __init__(self, platform:robot_platform.Platform, calibration:RobotCalibration, setting:Setting) -> None:
        self.cal = calibration
        self.pf = platform
        self.setting = setting
        self.task_que = Queue(20)
        self.grasp_thread = threading.Thread(target=self.grasp_task, name="grasp_task")
        self.grasp_thread.setDaemon(True)
        self.grasp_thread.start()
        self.is_run = False
        self.current_grasp = None

        self.speed_f = 1.0
        self.mp_travel = MotionParam(setting.s_platform.travel_speed, setting.s_platform.travel_accel, setting.s_platform.travel_decel)
        self.mp_close = MotionParam(setting.s_platform.close_speed, setting.s_platform.close_accel, setting.s_platform.close_decel)
        self.trace_dis = 70
        self.obj_min_gap = 55
        self.latency_dis = 30
        self.last_move_time = time.perf_counter()

        self.home_pos = np.array([0,   0, -790,    0.   ,  180.   ,    0.001])
        self.place_pos = np.array([
            [-473.292,   96.086, -810,    0.   ,  180.   ,    0.001],
            [-343.642,   -106.890, -810,    0.   ,  180.   ,    0.001],
            [-235.620,   -309.547, -810,    0.   ,  180.   ,    0.001],
            [-131.420,   -485.123, -810,    0.   ,  180.   ,    0.001],
        ])

    def grasp(self, grasp_rect:np.ndarray, encoder_cap:float, places:np.ndarray, tools:np.ndarray, _class:np.ndarray, img:np.ndarray=None):
        """
        抓取一张图中的目标，按照输入抓取矩形的顺序抓取
        grasp_rect: shape=(nx6) [cx, cy, angle, width, height],一张图片上的抓取矩形序列
        [
            [1, 2, 3, 0, 0, 0],
            [2, 3, 5, 0, 0, 0],
            ...
        ]
        encoder_cap: 拍摄图片时的编码器值
        places: shape=(n,)每个目标放置的位置编号,长度必须和目标个数相等
        [0, 1, ...]
        tools: 每个目标使用的工具 0:"Gripper", 1:"Sucker"
        img: 拍摄的照片
        """
        if np.max(places) >= len(self.place_pos) or np.min(places) < 0:
            logger.error("place index error")
            return
        if np.max(tools) >= 2 or np.min(tools) < 0:
            logger.error("tool index error")
            return

        if len(places) != len(grasp_rect):
            logger.error("place number is not equal to pos number")
            return
        if len(tools) != len(grasp_rect):
            logger.error("tools number is not equal to pos number")
            return
        if len(_class) != len(grasp_rect):
            logger.error("class number is not equal to pos number")
            return
        pos_in_cam, _ = self.grasp_rect_to_world_pos(grasp_rect)
        dis = self.get_cal_distance(encoder_cap)
        pos_in_belt = self.cal.get_pos_in_beltframe(pos_in_cam, dis) # 转换到传送带坐标系
        del_index = []
        for tq in self.task_que.queue:
            pc, _ = self.grasp_rect_to_world_pos(tq['grasp_rect'])
            dis = self.get_cal_distance(tq['encoder'])
            pbs = self.cal.get_pos_in_beltframe(pc, dis) # 转换到传送带坐标系
            for pb in pbs:
                for i, p in enumerate(pos_in_belt):
                    d = np.linalg.norm(p[:2]-pb[:2])
                    if d < self.obj_min_gap:
                        # print(f"[{i}] - p1:{p}, p2:{pb}, d:{d}")
                        del_index.append(i)
        if self.current_grasp:
            pc, _ = self.grasp_rect_to_world_pos(self.current_grasp['grasp_rect'])
            dis = self.get_cal_distance(self.current_grasp['encoder'])
            pbs = self.cal.get_pos_in_beltframe(pc, dis) # 转换到传送带坐标系
            for pb in pbs:
                for i, p in enumerate(pos_in_belt):
                    d = np.linalg.norm(p[:2]-pb[:2])
                    if d < self.obj_min_gap:
                        # print(f"[{i}] - p1:{p}, p2:{pb}, d:{d}")
                        del_index.append(i)
        grasp_rect = np.delete(grasp_rect, del_index, axis = 0)
        # encoder_cap = np.delete(encoder_cap, del_index)
        places = np.delete(places, del_index)
        tools = np.delete(tools, del_index)
        _class = np.delete(_class, del_index)
        if len(grasp_rect)==0:
            return
        pos_in_cam, _ = self.grasp_rect_to_world_pos(grasp_rect)
        dis = self.get_cal_distance(encoder_cap)
        pos_in_belt = self.cal.get_pos_in_beltframe(pos_in_cam, dis) # 转换到传送带坐标系
        index = pos_in_belt[:,0].argsort() # 按照x轴坐标排序后的索引
        grasp_rect = grasp_rect[index]
        # encoder_cap = encoder_cap[index]
        places = places[index]
        tools = tools[index]
        _class = _class[index]
        task = {"grasp_rect":grasp_rect, "encoder":encoder_cap, "places":places, "tools":tools, "class":_class, "img":img}
        try:
            logger(f"put {len(grasp_rect)} grasp into queue, places={task['places']}, tools={task['tools']}")
            self.task_que.put(task, block=False)
        except Full:
            logger.error("task_que full")
        except:
            traceback.print_exc()

    def robot_move(self, pos:np.ndarray, **kwargs):
        # print(f"grasp: bot move to {pos}")
        self.last_move_time = time.perf_counter()
        self.pf.robot.move(MODE_ABS,
                        dx=pos[0],
                        dy=pos[1],
                        dz=pos[2],
                        dr=pos[5],
                        **kwargs)

    def get_cal_distance(self, encoder:float)->float:
        ret = self.pf.encoder.distance - encoder
        return ret * self.cal.Ke

    @property
    def last_move_elapse(self)->float:
        return time.perf_counter() - self.last_move_time
    
    def gripper_pick(self, tool:int, distance:int):
        if tool == self.TOOL_GRIPPER:
            self.pf.gripper.push(13, distance, 500)
        elif tool == self.TOOL_SUCKER:
            self.pf.gripper.sucker(True)

    def gripper_place(self, tool:int, position:float=146):
        if tool == self.TOOL_GRIPPER:
            self.pf.gripper.move_absolute(position, 1000, 3000, 3000, 0.1)
        elif tool == self.TOOL_SUCKER:
            self.pf.gripper.sucker(False)

    def grasp_rect_to_world_pos(self, grasp_rect:np.ndarray)->np.ndarray:
        """
        转换像素坐标的
        grasp_rect: 抓取矩形 [cx, cy, angle, width, height]
            [[1196, 576, 60, 100, 400],
            [435, 576, 30, 100, 500],
            [435, 913, 90, 100, 400],
            [700, 600, 0, 100, 400],
            [1368, 913, 120, 100, 400]])
        """
        _pos_6d = np.zeros((grasp_rect.shape[0], 6))
        _gripper_width = np.zeros((grasp_rect.shape[0]))
        for i, obj in enumerate(grasp_rect):
            points = rotate_ract_in_imgframe(obj)
            vup = np.array([[points[0]],[points[1]], [obj[:2]]], dtype=np.float32)
            pos = self.cal.get_world_cartesian(self.cal.Rt_matrix[-1], vup)
            dx, dy = pos[1][0] - pos[0][0], pos[1][1] - pos[0][1]
            _pos_6d[i][:3] = pos[2] # 中心点坐标
            _pos_6d[i][5] = np.rad2deg(np.arctan2(dy, dx)) -120 # 角度
            _gripper_width[i] = np.linalg.norm(pos[1] - pos[0]) + 32 # 加上一个冗余宽度
        return _pos_6d, _gripper_width

    def set_grasp_end_callback(self, callback: Callable[[int, bool], None]) -> None:
        self.grasp_end_callback = callback

    def set_grasp_start_callback(self, callback: Callable[[int], None]) -> None:
        self.grasp_start_callback = callback

    def grasp_task(self):
        while self.pf.is_connected:
            self.is_run = False
            task:dict = self.task_que.get(block=True)
            self.current_grasp = task
            pos_in_cam, gripper_width = self.grasp_rect_to_world_pos(task["grasp_rect"])
            encoder:float = task["encoder"]
            places:np.ndarray = task["places"]
            tools:np.ndarray = task["tools"]
            _class:np.ndarray = task["class"]
            self.gripper_place(self.TOOL_GRIPPER)
            self.gripper_place(self.TOOL_SUCKER)
            logger(f"take {len(pos_in_cam)} grasp actions")
            self.is_run = True
            self.latency_dis = self.pf.encoder.speed * self.setting.s_platform.grasp_prepare_latency_ms / 1000
            self.trace_dis = self.pf.encoder.speed * ((self.setting.s_platform.grasp_close_time_ms / 1000) + 0.5) # 最后加上一个时间保证跟踪距离足够
            logger(f"latency_dis={self.latency_dis}, trace_dis={self.trace_dis}")
            for pos, place, tool, cls, gw in zip(pos_in_cam, places, tools, _class, gripper_width):
                logger(f"start to grasp {pos} put into place:{place} with tool:{tool} gw:{gw:.1f}mm")
                step = 0
                if hasattr(self, "grasp_start_callback"):
                    self.grasp_start_callback(cls)
                if tool==self.TOOL_GRIPPER:
                    self.gripper_place(tool, gw)
                while True:
                    dis = self.get_cal_distance(encoder)
                    p = self.cal.get_pos_in_beltframe(pos, dis)[0] # 获取传送带坐标系下的坐标
                    # print(f"p:{p}")
                    if p[0] >= (self.cal.L1+self.cal.L2-self.trace_dis-self.latency_dis):
                        logger.warning(f"object out of the area")
                        break

                    if step == 0:
                        if p[0] > (self.cal.L1 - 0):
                            # if p[0] > self.cal.L1:
                            bot_pos = self.cal.get_pos_in_robotframe_withtool(pos, dis, True if tool==self.TOOL_GRIPPER else False)[0]
                            # else:
                            #     bot_pos = self.cal.get_pos_in_robotframe_withtool(pos, self.cal.L1, True if tool==self.TOOL_GRIPPER else False)[0]
                            bot_pos[2] = self.home_pos[2]
                            self.robot_move(bot_pos, speed = self.mp_travel.speed, accel = self.mp_travel.accel, decel = self.mp_travel.decel, wait=1)
                            step = 1
                            if p[0] < self.cal.L1:
                                continue
                    if step == 1:
                        if p[0] >= self.cal.L1: # 目标在上下游线之间，可以抓取
                            bs = self.pf.encoder.speed
                            dis = self.get_cal_distance(encoder)
                            bot_pos = self.cal.get_pos_in_robotframe_withtool(pos, dis+self.latency_dis, True if tool==self.TOOL_GRIPPER else False)[0]
                            self.robot_move(bot_pos, speed = self.mp_close.speed, accel = self.mp_close.accel, decel = self.mp_close.decel, wait=1)

                            # time.sleep(0.3)
                            # self.pf.robot.halt()
                            
                            dis = self.get_cal_distance(encoder)
                            end_pos = self.cal.get_pos_in_robotframe_withtool(pos, dis+self.trace_dis, True if tool==self.TOOL_GRIPPER else False)[0]
                            self.robot_move(end_pos, speed = bs * 0.034, accel = 400, decel = 400, wait=False)
                            self.gripper_pick(tool, 130)
                            time.sleep(self.setting.s_platform.grasp_close_time_ms/1000 if tool==self.TOOL_GRIPPER else self.setting.s_platform.sucker_close_time_ms/1000)
                            self.pf.robot.halt()
                            step = 2
                    if step == 2:
                        pos = self.pf.robot.get_world_position()
                        pos[3:] = end_pos[3:] # 保持姿态角度不变
                        pos[2] = self.home_pos[2]
                        self.robot_move(pos, speed = self.mp_close.speed, accel = self.mp_close.accel, decel = self.mp_close.decel, wait=0)
                        # time.sleep(0.3)
                        # self.pf.robot.halt()
                        self.robot_move(self.place_pos[place], speed = self.mp_travel.speed, accel = self.mp_travel.accel, decel = self.mp_travel.decel, wait=1)
                        if hasattr(self, "grasp_end_callback"):
                            is_successful = False if self.pf.gripper.get_position() > 120 else True
                            self.grasp_end_callback(cls, is_successful)
                        self.gripper_place(tool, gw)
                        time.sleep(0.1)
                        step = 0
                        break

                    time.sleep(0.03)
                
                #抓取完了一个目标，删除对应抓取矩形
                self.current_grasp["grasp_rect"] = self.current_grasp["grasp_rect"][1:]

            self.robot_move(self.home_pos, wait=False)
            self.current_grasp = None

        logger.warning("exit from grasp task because platform disconnected")


