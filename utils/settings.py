import json
from pathlib import Path


class SetItem():
    def __init__(self, name:str) -> None:
        self.setting_name = name

class PlatformSetting(SetItem):
    def __init__(self) -> None:
        super().__init__("platfrom")
        self.robot_ip = "127.0.0.1:43000"
        self.belt_port = "COM5:38400"
        self.encoder_port = "COM3:38400"
        self.gripper_port = "COM4:115200"
        self.belt_speed = 10
        self.travel_speed = 180
        self.travel_accel = 170
        self.travel_decel = 170
        self.close_speed = 140
        self.close_accel = 130
        self.close_decel = 130
        self.grasp_prepare_latency_ms = 738
        self.grasp_close_time_ms = 600
        self.sucker_close_time_ms = 200

class DetectionSetting(SetItem):
    def __init__(self) -> None:
        super().__init__("detection")
        self.camera_param = "cal_param_realsense.json"
        self.robot_param = "robot_param_realsense.json"
        self.detection_model = "YOLO"
        self.model = "yolo/yolov5s.onnx"
        self.detection_device = 0
        self.camera_latency_ms = 180

class Setting():
    def __init__(self, setting_file:str) -> None:
        self.s_platform = PlatformSetting()
        self.s_detection = DetectionSetting()
        
        self.save_filename = Path(setting_file)
        if self.save_filename.is_file():
            self.load(self.save_filename)
        else:
            # save a default settings
            print("Can't find settings file, save a default settings")
            self.save(self.save_filename)


    def save(self, path: str) -> bool:
        if isinstance(path, Path):
            path = str(path)
        mate = {"s_platform":self.s_platform.__dict__,
                "s_detection":self.s_detection.__dict__,}
        jsonString = json.dumps(mate, indent=2, ensure_ascii=True)
        with open(path, "w") as f:
            f.write(jsonString)
            print('parameters saved')
            return True
        return False

    def load(self, path: str) -> bool:
        if isinstance(path, Path):
            path = str(path)
        try:
            with open(path, "r") as f:
                js:dict = json.loads(f.read())
        except FileNotFoundError:
            print(f'File "{path}" not found')
            return False
        
        for k, v in js["s_platform"].items():
            setattr(self.s_platform, k, v)

        for k, v in js["s_detection"].items():
            setattr(self.s_detection, k, v)

        return True
    
if __name__ == "__main__":
    s = Setting()
    print(s.s_detection.__dict__)
