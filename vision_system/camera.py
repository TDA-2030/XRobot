import cv2
import numpy as np
import time

class USBCamera():

    def __init__(self) -> None:
        self.opened = False

    @property
    def is_open(self) -> bool:
        return self.opened

    @classmethod
    def scan_camera(cls) -> list:
        cams = []

        from pygrabber.dshow_graph import FilterGraph
        graph = FilterGraph()
        allcams = graph.get_input_devices() # list of camera device
        for cam in allcams:
            device = graph.get_input_devices().index(cam)
            cams.append({
                        'cls': cls,
                        'dev': device,
                        'ModelName': "USB-"+cam,
                    })

        return cams

    def get_camera_frame(self) -> np.ndarray:
        assert self.opened, "camera not open"
        flag, img = self.cap.read()
        return img

    def open_camera(self, cam) -> bool:
        self.cap = cv2.VideoCapture()
        if self.cap.open(cam):
            self.opened = True
        return self.opened

    def close_camera(self):
        self.opened = False
        self.cap.release()

try:
    from pypylon import pylon
    from pypylon import genicam

    class BaslerCamera():
        # https://github.com/basler/pypylon/tree/master/samples

        def __init__(self) -> None:
            self.opened = False

        @property
        def is_open(self) -> bool:
            return self.opened

        # search device and get device
        @classmethod
        def scan_camera(cls) -> list:
            cams = []
            tl_factory = pylon.TlFactory.GetInstance()
            for dev_info in tl_factory.EnumerateDevices():
                print("DeviceClass:", dev_info.GetDeviceClass())
                if dev_info.GetDeviceClass() == 'BaslerGigE':
                    cams.append({'cls': cls, 'dev': dev_info, 'ModelName': dev_info.GetModelName(), 'IP': dev_info.GetIpAddress()})
                    break

            return cams

        def get_camera_frame(self) -> np.ndarray:
            assert self.opened, "camera not open"
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                # Access the image data
                image = self.converter.Convert(grabResult)
                img = image.GetArray()

            grabResult.Release()
            return img

        def open_camera(self, cam) -> bool:
            # conecting to the first available camera
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(cam))
            # self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

            # self.camera.RegisterConfiguration(ConfigurationEventPrinter(), pylon.RegistrationMode_Append, pylon.Cleanup_Delete)

            self.camera.Open()

            self.camera.Width = self.camera.Width.Max
            self.camera.Height = self.camera.Height.Max
            # self.camera.ExposureTimeAbs.SetValue(22670)
            # d = self.camera.ExposureTimeAbs.GetValue()
            # print(d)

            # Grabing Continusely (video) with minimal delay
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.converter = pylon.ImageFormatConverter()

            # converting to opencv bgr format
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

            self.opened = True
            return self.opened

        def close_camera(self):
            self.opened = False
            self.camera.StopGrabbing()
except:
    pass


try:
    from pykinect2 import PyKinectV2
    from pykinect2.PyKinectV2 import *
    from pykinect2 import PyKinectRuntime

    class Kinect2():

        def __init__(self) -> None:
            self.opened = False

        @property
        def is_open(self) -> bool:
            return self.opened

        @classmethod
        def scan_camera(cls) -> list:
            cams = []
            device = 0
            cams.append({
                'cls': cls,
                'dev': device,
                'ModelName': "KinectV2%s" % device,
            })

            return cams
    
        def open_camera(self, cam) -> bool:
            self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Infrared)
            self.opened = True
            return True

        def close_camera(self):
            self.opened = False
            del self.kinect

        def get_camera_frame(self) -> tuple:
            assert self.opened, "camera not open"
            
            #records and saves colour and depth frames from the Kinect
            #initialise kinect recording, and some time variables for tracking the framerate of the recordings
            while not (self.kinect.has_new_depth_frame() and self.kinect.has_new_color_frame() and self.kinect.has_new_infrared_frame()):
                time.sleep(0.1)
                
            #read kinect colour and depth data (somehow the two formats below differ, think one is and one isnt ctypes)
            depthframe = self.kinect.get_last_depth_frame() #data for display
            depthframeD = self.kinect._depth_frame_data
            colourframe = self.kinect.get_last_color_frame()
            colourframeD = self.kinect._color_frame_data
            irframe = self.kinect.get_last_infrared_frame()
            irframeD = self.kinect._infrared_frame_data

            #reformat the other depth frame format for it to be displayed on screen
            # depthframe = depthframe.astype(np.uint8)
            # print(depthframe.shape)
            depthframe = np.reshape(depthframe, (424, 512))
            # print(depthframe.dtype)
            dimg = depthframe.copy()
            dimg = dimg.astype(np.uint8)
            dimg = cv2.cvtColor(dimg, cv2.COLOR_GRAY2RGB)
            
            #Reslice to remove every 4th colour value, which is superfluous
            
            colourframe = np.reshape(colourframe, (2073600, 4))
            # colourframe = np.reshape(colourframe, (217088, 4))
            colourframe = colourframe[:,0:3]

            #extract then combine the RBG data
            colourframeR = colourframe[:,0]
            colourframeR = np.reshape(colourframeR, (1080, 1920))
            colourframeG = colourframe[:,1]
            colourframeG = np.reshape(colourframeG, (1080, 1920))
            colourframeB = colourframe[:,2]
            colourframeB = np.reshape(colourframeB, (1080, 1920))

            # colourframeR = colourframe[:,0]
            # colourframeR = np.reshape(colourframeR, (424, 512))
            # colourframeG = colourframe[:,1]
            # colourframeG = np.reshape(colourframeG, (424, 512))
            # colourframeB = colourframe[:,2]
            # colourframeB = np.reshape(colourframeB, (424, 512))
            framefullcolour = cv2.merge([colourframeR, colourframeG, colourframeB])

            irframe = np.reshape(irframe,(424, 512))
            irframe = np.uint8(irframe/256)
            jpgIRFrame = np.zeros((424,512,3), np.uint8)
            jpgIRFrame[:,:,0]  =  irframe
            jpgIRFrame[:,:,1]  =  irframe
            jpgIRFrame[:,:,2]  =  irframe

            return framefullcolour, depthframe, dimg, irframe, jpgIRFrame
except:
    pass

try:
    import pyrealsense2 as rs

    class RealSenseCamera:
        def __init__(self,
                    width=640,
                    height=480,
                    fps=30):
            self.width = width
            self.height = height
            self.fps = fps

            self.pipeline = None
            self.scale = None
            self.intrinsics = None
            self.opened = False

        @property
        def is_open(self) -> bool:
            return self.opened

        @classmethod
        def scan_camera(cls) -> list:
            cams = []
            for d in rs.context().devices:
                if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                    cams.append({
                        'cls': cls,
                        'dev': d.get_info(rs.camera_info.serial_number),
                        'ModelName': d.get_info(rs.camera_info.name),
                    })

            return cams

        def open_camera(self, cam) -> bool:
            # Start and configure
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(cam)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
            cfg = self.pipeline.start(config)
            
            sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
            # sensor.set_option(rs.option.auto_exposure_priority, True)
            # sensor.set_option(rs.option.enable_auto_exposure, 1)
            sensor.set_option(rs.option.enable_auto_exposure, True)
            # rgb_sensor.set_option(rs.option.enable_auto_white_balance, True)
            sensor.set_option(rs.option.enable_auto_white_balance, False)
            # sensor.set_option(rs.option.exposure, 700)
            # sensor.set_option(rs.option.exposure, 600)
            # sensor.set_option(rs.option.gain,80)
            # print(sensor.get_option(rs.option.exposure))
            # print(sensor.get_option(rs.option.gain))

            # Determine intrinsics
            rgb_profile = cfg.get_stream(rs.stream.color)
            self.intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()

            # Determine depth scale
            self.scale = cfg.get_device().first_depth_sensor().get_depth_scale()            
            self.opened = True
            return True

        def close_camera(self):
            if self.opened:
                self.pipeline.stop()
                self.opened = False

        def get_camera_frame(self) -> tuple:
            assert self.opened, "camera not open"

            frames = self.pipeline.wait_for_frames()

            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.first(rs.stream.color)
            aligned_depth_frame = aligned_frames.get_depth_frame()

            depth_image = np.asarray(aligned_depth_frame.get_data(), dtype=np.float32)
            depth_image *= self.scale
            color_image = np.asanyarray(color_frame.get_data())

            depth_image = np.expand_dims(depth_image, axis=2)

            return {
                'rgb': color_image,
                'aligned_depth': depth_image,
            }

except:
    pass


class VideoGrab:

    def __init__(self) -> None:
        self.camears = []

    @property
    def is_open(self) -> bool:
        if hasattr(self, 'cam'):
            return self.cam.is_open
        return False

    def scan_all_camera(self) -> list:
        print("Scaning the Camera...")
        self.camears = []
        try:
            self.camears += BaslerCamera.scan_camera()
        except:
            pass

        try:
            self.camears += USBCamera.scan_camera()
        except:
            pass

        try:
            self.camears += Kinect2.scan_camera()
        except:
            pass

        try:
            self.camears += RealSenseCamera.scan_camera()
        except:
            pass
        for i, c in enumerate(self.camears):
            print(f"Camera[{i}] name:{c['ModelName']}")
        return self.camears

    def get_scanned_cameras(self)->list:
        return self.camears

    def open_camera(self, index) -> bool:
        cam = self.camears[index]
        self.cam = cam['cls']()
        return self.cam.open_camera(cam['dev'])

    def get_camera_frame(self) -> np.ndarray:
        return self.cam.get_camera_frame()

    def close_camera(self):
        self.cam.close_camera()

    def trans_video(self,image_path,video_name,fps,res,type):
        import glob as gb
        img_path = gb.glob(image_path+"/*.png")
        videoWriter = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'DIVX'), fps, res)
        for path in img_path:
            img = cv2.imread(path)
            img = cv2.resize(img, res)
            videoWriter.write(img)
        print('transform '+type+' video done!')



if __name__ == "__main__":

    cam = VideoGrab()
    cameras = cam.scan_all_camera()
    print("find %d camera" % len(cameras))
    if len(cameras) > 0:
        cam.open_camera(0)
        while True:
            img = cam.get_camera_frame()
            if isinstance(img, dict):
                depth = img['aligned_depth']
                img = img['rgb']
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.imshow("depth", depth)
            cv2.imshow("rgb", img)
            k = cv2.waitKey(1)
            if k > 0:
                break
        cam.close_camera()
