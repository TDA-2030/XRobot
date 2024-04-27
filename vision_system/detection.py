import sys
from multiprocessing import Process, Queue
from typing import Any
import numpy as np
from pathlib import Path
import cv2

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # root directory

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT)) 

DIR = Path(__file__).resolve().parent/'..'
if str(DIR) not in sys.path:
    sys.path.append(str(DIR)) 
from utils.visual import draw_ract

from yolo.detector import Detector as YOLO
from misc import draw_bbox

BACKEND_CPU = 'cpu'
BACKEND_CUDA = 'cuda'

def get_detector(model_name:str):
    return eval(model_name)


class Predictor():

    def __init__(self, model_name:str, onnx_model: str, backend: str = BACKEND_CPU) -> None:
        self.q = Queue(1)
        self.qout = Queue(1)
        self.p = Process(target=self.run, args=(self.q, self.qout, model_name, onnx_model, backend), daemon=True)
        self.p.start()
        

    def __del__(self):
        self.p.terminate()
        self.p.join()

    @staticmethod
    def get_grasp_rect_from_bbox(bboxs, size):
        grasp_rect = []
        for i, box in enumerate(bboxs):
            box = box.astype(np.int16)
            if box[0] < 0 or box[1] < 10 or box[2] > (size[1]) or box[3] > (size[0]-10):
                # logger.warning(f"skip object: {box[:4]}")
                continue # 触碰到边缘的框跳过
            g = [0]*6
            g[0] = int((box[0] + box[2]) / 2)
            g[1] = int((box[1] + box[3]) / 2)
            g[3] = int(box[2] - box[0])
            g[4] = int(box[3] - box[1])
            g[5] = box[5]
            if g[4] > g[3]:
                g[2] = 0
            else:
                g[2] = 90
                g[3] = g[4]
                g[4] = int(box[2] - box[0])
            grasp_rect.append(g)
        grasp_rect = np.array(grasp_rect)
        return grasp_rect

    @staticmethod
    def grasp_filter(gr, size):
        grasp_rect = []
        for i, (cx, cy, angle, width, height, cls) in enumerate(gr):
            x0 = cx-width/2
            x1 = cx+width/2
            if x0 < 130+80 or x1 > (size[1]-130-80):
                # print(f"skip edge object: {gr[i]}")
                continue # 触碰到边缘的框跳过
            area = width * height
            if area < 14*14:
                # print(f"skip little object: {gr[i]}")
                continue
            
            grasp_rect.append(gr[i])

        grasp_rect = np.array(grasp_rect)
        return grasp_rect

    def run(self, q: Queue, qout: Queue, model_name:str, onnx_model: str, backend: str):
        self.det = get_detector(model_name)(onnx_model, backend)
        while True:
            img = q.get()
            results = self.det.detect(img)
            if isinstance(img, dict):
                img = img['rgb']
            # if isinstance(self.det, GGCNN):
            #     bboxs = None
            # else:
            bboxs = results["result"]
            # print(f"Time:{results['time']}")
            grasp_rect = self.get_grasp_rect_from_bbox(bboxs, img.shape[:2])
            grasp_rect = self.grasp_filter(grasp_rect, img.shape[:2])

            out = {"grasp_rect":grasp_rect, "bbox":bboxs}
            qout.put(out)

    def inference(self, img: np.ndarray)->dict:
        self.q.put(img)
        return self.qout.get()


def trackbar_cb(value):
    global set_h, set_s, set_v
    set_h = cv2.getTrackbarPos('H', 'video')
    set_s = cv2.getTrackbarPos('S', 'video')
    set_v = cv2.getTrackbarPos('V', 'video')


if __name__ == '__main__':

    from camera import VideoGrab
    cam = VideoGrab()
    cameras = cam.scan_all_camera()
    print("find %d camera" % len(cameras))
    if len(cameras) == 0:
        exit()
    cam.open_camera(1)

    # cap.cap.set(cv2.CAP_PROP_POS_FRAMES, 5900)
    cv2.namedWindow('video', cv2.WINDOW_NORMAL|cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar('H', 'video', 0, 100, trackbar_cb)
    cv2.createTrackbar('S', 'video', 0, 100, trackbar_cb)
    cv2.createTrackbar('V', 'video', 0, 100, trackbar_cb)

    # p = Predictor(str(ROOT/"yolo"/'yolov5s.onnx'))
    DIR = Path(__file__).resolve().parent/'ggcnn3'
    if str(DIR) not in sys.path:
        sys.path.append(str(DIR)) 
    p = Predictor(str(ROOT/"ggcnn3"/'epoch_34_iou_0.78-rgb.pkl'), BACKEND_CUDA)

    while True:
        img = cam.get_camera_frame()
        results = p.inference(img)
        
        img = img['rgb']
        # img = draw_bbox(img, results["bbox"], GGCNN._CLASS)
        for g in results["grasp_rect"]:
            # print(p)
            draw_ract(img, g, GGCNN._CLASS)
        cv2.imshow("video", img)
        # print(f"infer time:{p.det.timer[0].dt:.3f}, {p.det.timer[1].dt:.3f}, {p.det.timer[2].dt:.3f}")
        # ret_img = ret_img[:, :, ::-1]
        # ret_img = cv2.cvtColor(ret_img, cv2.COLOR_BGR2RGB)
        k = cv2.waitKey(1)
        if k >= 0:
            break

    cam.close_camera()
