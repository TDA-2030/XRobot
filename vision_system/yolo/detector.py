import onnxruntime as ort
import numpy as np
import cv2
import time, sys
from pathlib import Path

# FILE = Path(__file__).resolve()
# ROOT = FILE.parents[0]  # root directory
# if str(ROOT/"..") not in sys.path:
#     sys.path.append(str(ROOT/"..")) 

from misc import numpy_nms, xywh2xyxy, Profile, draw_bbox
"""
Cudnn安装: https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installwindows
"""

BACKEND_CPU = 'cpu'
BACKEND_CUDA = 'cuda'


def nms(pred: np.ndarray, conf_thres: float, iou_thres: float) -> np.ndarray:
    # Checks
    assert 0 <= conf_thres <= 1, f'Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0'
    assert 0 <= iou_thres <= 1, f'Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0'

    # pred.shape (25200, 85)
    # center_x,center_y, width, height, obj_conf, cls1_conf,cls2_conf, ... ,cls80_conf
    # 置信度抑制，小于置信度阈值则删除
    xc = pred[..., 4] > conf_thres  # candidates
    grid = pred[xc]
    grid[:, :4] = xywh2xyxy(grid[:, :4])
    # grid[:, 5:] *= grid[:, 4:5]  # conf = obj_conf * cls_conf
    output = np.empty((0, 6))
    if not grid.shape[0]:
        return output

    agnostic = True
    max_wh = 7680  # (pixels) maximum box width and height
    grid[..., 5] = np.argmax(grid[..., 5:], axis=1)
    # Batched NMS
    offset = grid[:, 5:6] * (0 if agnostic else max_wh)  # classes
    boxes, scores = grid[:, :4] + offset, grid[:, 4]  # boxes (offset by class), scores
    i = numpy_nms(boxes, scores, iou_thres)  # NMS
    output = grid[i]
    return output


class Detector(object):

    _CLASS = ('person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
          'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant',
          'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
          'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle',
          'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
          'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
          'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
          'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush')
    _PLACE = {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1, 7: 1, 8: 1, 9: 1, 10: 1, 11: 1, 12: 1, 13: 1, 14: 1, 15: 1, 16: 1, 17: 1, 18: 1, 19: 1, 20: 1,
            21: 1, 22: 1, 23: 1, 24: 1, 25: 1, 26: 1, 27: 1, 28: 1, 29: 1, 30: 1, 31: 1, 32: 1, 33: 1, 34: 1, 35: 1, 36: 1, 37: 1, 38: 1, 39: 1, 40: 1,
            41: 1, 42: 1, 43: 1, 44: 1, 45: 1, 46: 1, 47: 1, 48: 1, 49: 1, 50: 1, 51: 1, 52: 1, 53: 1, 54: 1, 55: 1, 56: 1, 57: 1, 58: 1, 59: 1, 60: 1,
            61: 1, 62: 1, 63: 1, 64: 1, 65: 1, 66: 1, 67: 1, 68: 1, 69: 1, 70: 1, 71: 1, 72: 1, 73: 1, 74: 1, 75: 1, 76: 1, 77: 1, 78: 1, 79: 1}

    def __init__(self, onnx_model: str, backend: str = BACKEND_CPU) -> None:
        self.height, self.width = 640, 640
        self.timer = Profile(), Profile(), Profile()
        self._load_model(onnx_model, backend)

    def set_device(self, device):
        self.device = device
        return True
    
    def _load_model(self, onnx_model: str, backend: str = BACKEND_CPU):
        backend = backend.lower()
        if backend == BACKEND_CPU:
            providers = [("CPUExecutionProvider", {"cudnn_conv_use_max_workspace": '1'})]
        else:
            providers = [("CUDAExecutionProvider", {"cudnn_conv_use_max_workspace": '1'})]
        sess_options = ort.SessionOptions()
        self.sess = ort.InferenceSession(onnx_model, sess_options=sess_options, providers=providers)
        self.input_name = self.sess.get_inputs()[0].name
        self.label_name = self.sess.get_outputs()[0].name

    def _preprocess(self, img0: np.ndarray):
        if isinstance(img0, dict):
            img0 = img0['rgb']
        x_scale = img0.shape[1] / self.width
        y_scale = img0.shape[0] / self.height
        img = cv2.resize(img0, (self.height, self.width))  # 尺寸变换
        img = img / 255.
        img = img[:, :, ::-1].transpose((2, 0, 1))  # HWC转CHW
        data = np.expand_dims(img, axis=0)  # 扩展维度至[1,3,640,640]
        return data, x_scale, y_scale
    
    def _postprocess(self, pred, xf, yf):
        out = nms(pred, 0.3, 0.45)
        out[:, 0:3:2] *= xf
        out[:, 1:4:2] *= yf
        return out

    def detect(self, img: np.ndarray):
        with self.timer[0]:
            data, xf, yf = self._preprocess(img)
        with self.timer[1]:
            pred_onx = self.sess.run([self.label_name], {self.input_name: data.astype(np.float32)})[0]
            pred = np.squeeze(pred_onx)  #(1, 25200, 85)
        with self.timer[2]:
            res = self._postprocess(pred, xf, yf)
        
        out = {"result":res, "time":(round(self.timer[0].dt, 3), round(self.timer[1].dt, 3), round(self.timer[2].dt, 3))}
        return out

if __name__ == '__main__':
    if str(ROOT.parent) not in sys.path:
        sys.path.append(str(ROOT.parent))
    import camera
    cap = camera.USBCamera()
    cap.open_camera(0)
    # cap.cap.set(cv2.CAP_PROP_POS_FRAMES, 5900)

    p = Detector(str(ROOT/'yolov5s.onnx'))

    while True:
        img = cap.get_camera_frame()
        out = p.detect(img)
        ret_img = draw_bbox(img, out, _CLASS)
        print(f"infer time:{p.timer[0].dt:.3f}, {p.timer[1].dt:.3f}, {p.timer[2].dt:.3f}")
        ret_img = ret_img[:, :, ::-1]
        ret_img = cv2.cvtColor(ret_img, cv2.COLOR_BGR2RGB)
        cv2.imshow("out", ret_img)
        k = cv2.waitKey(1)
        if k >= 0:
            break

    cap.close_camera()
