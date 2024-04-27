# XRobot

XRobot is a control software specifically designed for the robotic sorting platform we have constructed.

**Features**

- [x] Robot Motion Control
- [x] Camera Image Acquisition
- [x] Camera Calibration and Rectification
- [x] Robot and conveyor Calibration
- [x] Grasping Detection / Control Robot to Complete Grasping
- [x] Sorting Statistics

## Demonstration video 
[<img src="https://embed.wave.video/YGyijsPmgWabDyc9/preview.jpg?width=1120&height=630&play=true&color=%2330AEF2" alt="video" width=560 />](https://watch.wave.video/YGyijsPmgWabDyc9)

## File structure
```
├─ace_src                # OMRON ACE source code
├─calibration            # Data files of camera and robot calibration
├─grasp                  # System calibration and grasping control
│  └─V4-sdk-standalone   # SDK for our gripper controller
├─resource               # Resource files for GUI
├─robot                  # Drivers of sorting platform control
├─scripts                # Some useful scripts
├─utils                  # Ancillary functions such as logging and Settings
└─vision_system          # Vision system include camera drivers and detection algorithm
   └─yolo

```

## Requirements
The software is designed to run in a Python 3.8 environment.

**Installation**
1. Get the source code:
```shell
git clone https://github.com/TDA-2030/XRobot.git
cd XRobot
```

2. Install the required Python packages:
```shell
pip install -r requirements.txt
```

3. Install OMRON ACE-4.5.3 software for the robot control, you can download it from https://automation.omron.com/en/us/products/family/Robot%20Software

## Usage


1. **Connect devices:** Follow the diagram below to connect all devices including robot, camera, encoder, etc.
    ```
    ------------
                |<------ USB camera
                |<------ Ethernet ----- Robot Controller
                |      
    Computer    |      
                |<------| USB To RS485 Adapter |----- Conveyor
                |<------| USB To RS485 Adapter |----- Encoder
                |<------| USB To RS485 Adapter |----- Gripper
    ------------ 
    ```

2. **Launch robot motion Server:** Double-click to open the `ace_src/waste-sorting.awp2` file, Execute the `server` C# program within your workspace.
3. **Launch XRobot interface:** Run the Python script `xrobot.py` to open the interface.

All system settings are stored in the `sys-settings.json` file. Modify the file directly to change the settings.

## System calibration

The system supports the use of both RGB and RGBD cameras. The default camera used is the `Intel Realsense D455`. For newly installed cameras, calibration within the system is required:
1. **Download and Print Calibration Pattern:** Download and print an [A3](grasp/chressboard-A3.pdf) 5x7 chessboard pattern.
2. **Calibration Process:** Navigate to the calibration tab on the interface and follow the prompts in sequence.
