# from .visualize import Visualizer
from .logs import *
from pathlib import Path
import os
import numpy as np
import os, sys, contextlib
from shapely.geometry import Polygon, MultiPoint, mapping
from shapely.errors import TopologicalError
import psutil
import datetime
import threading

logger = MyLog() # Create a default logger

def increment_path(path, exist_ok=False, sep='', mkdir=False):
    # Increment file or directory path, i.e. runs/exp --> runs/exp{sep}2, runs/exp{sep}3, ... etc.
    path = Path(path)  # os-agnostic
    if path.exists() and not exist_ok:
        path, suffix = (path.with_suffix(''), path.suffix) if path.is_file() else (path, '')

        # Method 1
        for n in range(2, 9999):
            p = f'{path}{sep}{n}{suffix}'  # increment path
            if not os.path.exists(p):  #
                break
        path = Path(p)

        # Method 2 (deprecated)
        # dirs = glob.glob(f"{path}{sep}*")  # similar paths
        # matches = [re.search(rf"{path.stem}{sep}(\d+)", d) for d in dirs]
        # i = [int(m.groups()[0]) for m in matches if m]  # indices
        # n = max(i) + 1 if i else 2  # increment number
        # path = Path(f"{path}{sep}{n}{suffix}")  # increment path

    if mkdir:
        path.mkdir(parents=True, exist_ok=True)  # make directory

    return path




def bbox_iou_eval(box1, box2):
    box1 = np.array(box1).reshape(4, 2)
    poly1 = Polygon(box1).convex_hull #POLYGON ((0 0, 0 2, 2 2, 2 0, 0 0))
    print(type(mapping(poly1)['coordinates'])) # (((0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)),)
    poly_arr = np.array(poly1)
    box2 = np.array(box2).reshape(4, 2)
    poly2 = Polygon(box2).convex_hull
    if not poly1.intersects(poly2):  # 如果两四边形不相交
        iou = 0
    else:
        try:
            inter_area = poly1.intersection(poly2).area  # 相交面积
            iou = float(inter_area) / (poly1.area + poly2.area - inter_area)
        except TopologicalError:
            print('shapely.geos.TopologicalError occured, iou set to 0')
            iou = 0
    return iou


def RM_2D(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

def rotate_ract_in_imgframe(result:np.ndarray):
    """
    result: [cx, cy, angle_deg, width, height]
    ```
      p0 ------------- p1  
      |                |  
      |       c        |  
      |                |  
      p3 ------------- p2
    ```
    """
    # 下面几个参数，可能需要根据自己的数据进行调整
    x = int(result[0])  # 矩形框的中心点x
    y = int(result[1])  # 矩形框的中心点y
    angle = result[2]      # 矩形框的倾斜角度（长边相对于水平）
    width, height = int(result[3]), int(result[4])  # 矩形框的宽和高

    # 旋转矩形的四个角点，注意：旋转基于图像坐标系
    # ○-------> X
    # |
    # |
    # ↓
    # Y
    angle = np.deg2rad(angle)
    cosA = np.cos(angle)
    sinA = np.sin(angle)
    y1 = y + width / 2 * sinA
    x1 = x - width / 2 * cosA
    y2 = y - width / 2 * sinA
    x2 = x + width / 2 * cosA

    y0n, x0n = y1 - height/2 * cosA, x1 - height/2 * sinA
    y1n, x1n = y2 - height/2 * cosA, x2 - height/2 * sinA
    y2n, x2n = y2 + height/2 * cosA, x2 + height/2 * sinA
    y3n, x3n = y1 + height/2 * cosA, x1 + height/2 * sinA

    return np.array(((x0n, y0n), (x1n, y1n), (x2n, y2n), (x3n, y3n)), dtype=np.uint16)

def rotate_img_point(points: np.ndarray, resolution: np.ndarray, x_mirror: bool, y_mirror: bool, swap: bool) -> np.ndarray:
    """_summary_

    Args:
        point (np.ndarray): shape[nx2]
        resolution (np.ndarray): _description_
        x_mirror (bool): _description_
        y_mirror (bool): _description_
        swap (bool): _description_

    Returns:
        np.ndarray: shape[nx2]
    """
    if x_mirror:
        points[:, 0] = resolution[0] - points[:, 0]
    if y_mirror:
        points[:, 1] = resolution[1] - points[:, 1]
    if swap:
        points = points[:, [1, 0]]
    return points

def print_runtime():
   
    # 1 获取线程ID,NAME
    t = threading.currentThread()
    #线程ID
    print('Thread id : %d' % t.ident)
    #线程NAME
    print('Thread name : %s' % t.getName())

    pid = os.getpid()
    p = psutil.Process(pid)
    print('----------------')
    #进程ID
    print('Process id : %d' % pid)
    #进程NAME
    print('Process name : %s' % p.name())
    #获取进程bin路径
    print('Process bin  path : %s' % p.exe())
    #获取pid对应的路径
    print('Process path : %s' % p.cwd())
    #进程状态
    print('Process status : %s' % p.status())
    #进程运行时间
    print('Process creation time : %s' % datetime.datetime.fromtimestamp(p.create_time()).strftime("%Y-%m-%d %H:%M:%S"))
    #CPU使用情况
    print(p.cpu_times())
    #内存使用情况
    print('Memory usage : %s%%' % p.memory_percent())
    #硬盘读取信息
    print(p.io_counters())
    #打开进程socket的namedutples列表
    print(p.connections())
    #此进程的线程数
    print('Process number of threads : %s' % p.num_threads())

@contextlib.contextmanager
def hiddenPrint():
    try:
        original_stdout = sys.stdout
        original_stderr = sys.stderr
        sys.stdout = open(os.devnull, 'w')
        sys.stderr = open(os.devnull, 'w')
        yield
    # 用这个来指示打印
    except Exception as e:
        sys.stdout.close()
        sys.stdout = original_stdout
        print(e.args[0])
        sys.stdout = open(os.devnull, 'w')
    finally:
        sys.stdout.close()
        sys.stderr.close()
        sys.stdout = original_stdout
        sys.stderr = original_stderr
