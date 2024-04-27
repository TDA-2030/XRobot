import time, shutil
from pathlib import Path
import sys, os
from typing import Dict, List

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

import mainwindow, manual
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QMainWindow,
    QTextBrowser,
    QAction,
    QMessageBox,
    QLabel,
    QListWidgetItem,
    QTableWidget,
    QTableWidgetItem,
    QFileDialog,
    QPushButton,
    QSplashScreen,
    QShortcut,
)
from PyQt5.QtGui import QCloseEvent, QIcon, QPixmap, QImage, QCursor, QKeyEvent, QMovie, QKeySequence
from PyQt5.QtCore import QTimer, Qt, QModelIndex, QThread, pyqtSignal, QWaitCondition, QMutex, QTranslator, QCoreApplication
import cv2
import pandas as pd
import numpy as np
from robot import robot_platform
from robot.robot_driver import MODE_ABS, MODE_INC
from vision_system import detection
from vision_system.camera import VideoGrab
from grasp.grasp_controller import GraspController
from grasp.calibration import RobotCalibration
from utils.settings import Setting
from utils.visual import draw_ract
from utils import logger
from utils import hiddenPrint, rotate_ract_in_imgframe

import multiprocessing
import atexit


class MainWin(QMainWindow):

    def __init__(self, platform: robot_platform.Platform, video: VideoGrab, setting: Setting, parent=None):
        super(QMainWindow, self).__init__(parent)
        self.ui = mainwindow.Ui_mainwin()
        self.ui.setupUi(self)

        self.trans = QTranslator()
        self.trans.load(str(ROOT / "resource/en.qm"))
        _app = QApplication.instance()
        _app.installTranslator(self.trans)
        self.ui.retranslateUi(self)
        self._translate = QCoreApplication.translate

        self.m_flag = False
        # style 1: window can be stretched
        # self.setWindowFlags(Qt.CustomizeWindowHint | Qt.WindowStaysOnTopHint)

        # style 2: window can not be stretched
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint | Qt.WindowSystemMenuHint | Qt.WindowMinimizeButtonHint | Qt.WindowMaximizeButtonHint)
        # self.setWindowOpacity(0.85)  # Transparency of window

        self.ui.minButton.clicked.connect(self.showMinimized)
        self.ui.maxButton.clicked.connect(self.max_or_restore)
        self.ui.closeButton.clicked.connect(self.close)

        self.ui.listWidget.currentItemChanged.connect(self._solt_list_clicked)
        self.ui.listWidget.setCurrentRow(0)

        self.ui.btn_export.clicked.connect(self._solt_export)
        self.ui.btn_scan_cam.clicked.connect(self._solt_scan_cam)

        self.ui.Sliderspeed.valueChanged.connect(self._solt_set_speed_change)
        self.ui.Slider_acc_thres.valueChanged.connect(self._solt_set_acc_change)
        self.ui.Slider_camera_fps.valueChanged.connect(self._solt_set_cam_fps_change)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_cb)
        self.ui.pushButton_camera.clicked.connect(self._solt_camera)
        self.ui.pushButton_sorting.clicked.connect(self._solt_sorting)
        self.ui.pushButton_connect.clicked.connect(self._solt_connect)
        self.ui.pushButton_manual.clicked.connect(self._solt_manual)
        self.ui.pushButton_manual.setEnabled(False)

        self.platform = platform
        self.videocap = video
        self.setting = setting
        _translate = QCoreApplication.translate
        self.ui.comboBox_cam.clear()
        for i in self.videocap.get_scanned_cameras():
            self.ui.comboBox_cam.addItem(QIcon(":/mian_background/cameraopen.png"), i["ModelName"])
        self.ui.ipaddress_edit.setText(_translate("mainwin", self.setting.s_platform.robot_ip))
        self.ui.lineEdit_belt_ip.setText(_translate("mainwin", self.setting.s_platform.belt_port))
        self.ui.lineEdit_encoder_ip.setText(_translate("mainwin", self.setting.s_platform.encoder_port))
        self.ui.lineEdit_gripper_ip.setText(_translate("mainwin", self.setting.s_platform.gripper_port))
        self.ui.Sliderspeed.setValue(self.setting.s_platform.belt_speed)
        self.ui.comboBox_backend.setCurrentIndex(self.setting.s_detection.detection_device)
        self.ui.comboBox_model.clear()
        self.ui.comboBox_model.addItem(QIcon(":/mian_background/model.png"), self.setting.s_detection.detection_model)
        self.ui.comboBox_model.setCurrentIndex(0)
        self.ui.pushButton_open_model.setText(self.setting.s_detection.model)

        self.is_sorting = False
        self.calibra = RobotCalibration()
        self.calgui = CalGUI(self, self.videocap, self.platform, self.calibra)
        self.calibra.load(str(self.calgui.cal_dir / self.setting.s_detection.camera_param))
        self.calibra.load_bot_param(str(self.calgui.cal_dir / self.setting.s_detection.robot_param))

        self.stat_timer = QTimer(self)
        self.stat_timer.timeout.connect(self.stat_timer_cb)
        self.stat_timer.start(2000)
        self._temp = None
        atexit.register(self.my_clean)

    def my_clean(self):
        print("my_clean")
        if self.platform.belt.is_run:
            self.platform.belt.stop()

    def _solt_set_speed_change(self):
        self.ui.set_label_speed.setText(f"{self.ui.Sliderspeed.value()}%")

    def _solt_set_acc_change(self):
        self.ui.set_label_acc_thres.setText(f"{self.ui.Slider_acc_thres.value()}%")

    def _solt_set_cam_fps_change(self):
        self.ui.set_label_exposure.setText(f"{self.ui.Slider_camera_fps.value()}%")

    def _solt_scan_cam(self):
        self.videocap.scan_all_camera()
        self.ui.comboBox_cam.clear()
        for i in self.videocap.get_scanned_cameras():
            self.ui.comboBox_cam.addItem(QIcon(":/mian_background/cameraopen.png"), i["ModelName"])

    def camera_open(self) -> bool:
        index = self.ui.comboBox_cam.currentIndex()
        if len(self.videocap.get_scanned_cameras()) > 0:
            return self.videocap.open_camera(index)
        else:
            return False

    def _solt_list_clicked(self, item: QListWidgetItem):
        index = self.ui.listWidget.currentRow()
        self.ui.stackedWidget.setCurrentIndex(index)
        if index == 3:
            self.ui.stackedWidget_cal.setCurrentIndex(0)

    def _solt_manual(self, qaction: QAction):
        if not hasattr(self, "manualDlg"):
            logger("open manual")
            self.manualDlg = ManualDialog(self.platform, self)
            self.manualDlg.show()

    def _solt_sorting(self):
        if self.is_sorting:
            self.ui.pushButton_sorting.setText(self._translate("mainwin", "开始分拣"))
            self.is_sorting = False
            if self.platform.belt.is_run:
                self.platform.belt.stop()
        else:
            self.ui.pushButton_sorting.setText(self._translate("mainwin", "停止分拣"))
            self.is_sorting = True
            self.platform.belt.move(True, self.setting.s_platform.belt_speed)
            if hasattr(self, "obj_statistics") and hasattr(self, "grasp_controller"):
                self.grasp_controller.set_grasp_end_callback(self.obj_statistics.add_grasping)
                self.grasp_controller.set_grasp_start_callback(self.obj_statistics.add_obj)

    def _solt_camera(self):
        if self.videocap.is_open:
            self.inferthread.stopRun()
            self.inferthread.wait()
            self.inferthread.sinOut.disconnect()  # The wait() call does not mean that the thread is complete.We have to disconnect the signal
            del self.inferthread
            del self.objvis
            del self.obj_statistics
            if self.platform.belt.is_run:
                self.platform.belt.stop()
            self.videocap.close_camera()
            self.ui.pushButton_camera.setText(self._translate("mainwin", "打开相机"))
            self.ui.label_view.setPixmap(QPixmap(""))
            self.ui.label_view.setStyleSheet("#label_view{\n" "    border-image:url(:/mian_background/cover.jpg);\n" "border-radius:15px;\n" "}")
        else:
            if not self.camera_open():
                QMessageBox.warning(self, "Warning", "请检测相机与电脑是否连接正确", buttons=QMessageBox.Ok)
            else:
                self.ui.pushButton_camera.setText(self._translate("mainwin", "关闭相机"))
                backend = self.ui.comboBox_backend.currentText().lower()
                self.last_grasp_time = time.perf_counter()
                self.objvis = ObjectVisual(self.calibra)
                self.obj_statistics = ObjStatistics(detection.get_detector(self.setting.s_detection.detection_model)._CLASS)

                # self.platform.belt.move(True, self.setting.s_platform.belt_speed)
                def proc_result(res: dict):
                    # 运行在 MainThread
                    img = res["img"].copy()
                    if self.is_sorting:
                        self.last_grasp_time = time.perf_counter()
                        encoder = res["encoder"] - (self.platform.encoder.speed * self.setting.s_detection.camera_latency_ms / 1000)
                        grasp_rect = res["pred"]["grasp_rect"]
                        if len(grasp_rect) > 0:
                            tools = np.zeros((grasp_rect.shape[0]), dtype=np.uint8)
                            place = np.zeros((grasp_rect.shape[0]), dtype=np.uint8)
                            _classes = np.zeros((grasp_rect.shape[0]), dtype=np.uint8)
                            for i, g in enumerate(grasp_rect):
                                class_ID = int(g[5])
                                place[i] = detection.get_detector(self.setting.s_detection.detection_model)._PLACE[class_ID]
                                _classes[i] = class_ID
                            self.grasp_controller.grasp(grasp_rect, encoder, place, tools, _classes, img=img)

                    self.show_img_in_label(img, self.ui.label_view, False)

                model_path = str(ROOT / "vision_system" / self.setting.s_detection.model)
                self.inferthread = InferThread(self.setting.s_detection.detection_model, model_path, backend, self.videocap, self.platform, 20)
                self.inferthread.sinOut.connect(lambda res: proc_result(res))
                self.inferthread.start()
                self.ui.label_view.setStyleSheet("#label_view{\n" "    background-color: rgba(149, 149, 149, 0);\n" "}")

    def _solt_connect(self) -> None:

        if self.platform.is_connected:
            logger("disconnect")
            self.platform.disconnect()
            self.ui.ipaddress_edit.setEnabled(True)
            self.ui.pushButton_manual.setEnabled(False)
            self.ui.pushButton_connect.setText(self._translate("mainwin", "连接"))
            self.ui.pushButton_connect.setIcon(QIcon(QPixmap(":/mian_background/connect.png")))
            self.timer.stop()
            self.ui.label_state_bar.setText("")
            self.ui.label_robot_pos.setText("")
            self.ui.label_conveyer_speed.setText("")
        else:
            text = self.ui.lineEdit_gripper_ip.text().strip()
            text = text.split(":")
            text_bot = self.ui.ipaddress_edit.text().strip()
            text_bot = text_bot.split(":")
            text_belt = self.ui.lineEdit_belt_ip.text().strip()
            text_belt = text_belt.split(":")
            text_encoder = self.ui.lineEdit_encoder_ip.text().strip()
            text_encoder = text_encoder.split(":")
            msg = self.platform.connect((text_bot[0], int(text_bot[1])), (text[0], int(text[1])), (text_belt[0], int(text_belt[1])), (text_encoder[0], int(text_encoder[1])))
            if msg:
                with hiddenPrint():
                    ports = robot_platform.scan_correct_port()
                logger(ports)
                QMessageBox.warning(self, "Warning", f"{msg}，扫描设备为{ports}，请检测是否连接正确", buttons=QMessageBox.Ok)
                return

            self.grasp_controller = GraspController(self.platform, self.calibra, self.setting)
            self.ui.ipaddress_edit.setEnabled(False)
            self.ui.pushButton_manual.setEnabled(True)
            self.ui.pushButton_connect.setText(self._translate("mainwin", "断开"))
            self.ui.pushButton_connect.setIcon(QIcon(QPixmap(":/mian_background/disconnect.png")))
            self.timer.start(1000)
            logger("connect succeed")
            np.set_printoptions(precision=3, suppress=True)

    def timer_cb(self):
        enc = self.platform.encoder.distance
        if hasattr(self, "objvis"):  # Update the detected target display graph
            self.objvis.clear()
            if self.grasp_controller.current_grasp:  # Complement the target currently being grasped
                g = self.grasp_controller.current_grasp
                dis = (enc - g["encoder"]) * self.calibra.Ke
                self.objvis.add_object(g["img"], g["grasp_rect"], dis)
            for g in self.grasp_controller.task_que.queue:
                dis = (enc - g["encoder"]) * self.calibra.Ke
                self.objvis.add_object(g["img"], g["grasp_rect"], dis)
            img_map = self.objvis.get_map()
            self.show_img_in_label(img_map, self.ui.label_view_objects, False)

        t = self.platform.robot.get_world_position()
        bs = self.platform.encoder.speed
        robot_s = f"Robot:({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}, {t[3]:.3f}, {t[4]:.3f}, {t[5]:.3f})"
        if hasattr(self, "inferthread"):
            detection_s = f"Detection: FPS:{self.inferthread.get_real_fps():.1f}  Objnum:{self.objvis.object_num}, stat:{self.obj_statistics.stat}"
        else:
            detection_s = ""
        self.ui.label_state_bar.setText(detection_s)
        self.ui.label_robot_pos.setText(robot_s)
        self.ui.label_conveyer_speed.setText(f"Conveyor:{enc:.2f}mm Speed:{bs:.2f}mm/s")

    def stat_timer_cb(self):
        table = self.ui.tableWidget
        total = 0
        if hasattr(self, "inferthread"):
            cls_num = len(self.obj_statistics.stat)
            table.clearContents()
            table.setColumnCount(4)
            table.setRowCount(cls_num)
            for ii, (_k, _v) in enumerate(self.obj_statistics.stat.items()):
                table.setVerticalHeaderItem(ii, QTableWidgetItem(_k))
                table.setItem(ii, 0, QTableWidgetItem(str(_v[0])))
                table.setItem(ii, 1, QTableWidgetItem(str(_v[1])))
                table.setItem(ii, 2, QTableWidgetItem())
                table.setItem(ii, 3, QTableWidgetItem())
            table.resizeColumnsToContents()

        for row in range(table.rowCount()):
            total += int(table.item(row, 0).text())

        __sortingEnabled = table.isSortingEnabled()
        table.setSortingEnabled(False)
        for row in range(table.rowCount()):
            n = int(table.item(row, 0).text())
            r = int(table.item(row, 1).text()) / (n + 1e-6)
            table.item(row, 2).setText(f"{r:.2f}")
            table.item(row, 3).setText(f"{n/total:.2f}")
        table.setSortingEnabled(__sortingEnabled)

    def _solt_export(self):
        fname = f"{ROOT / ('grasp_result_'+time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime())+'.csv')}"
        # fname, ftype = QFileDialog.getSaveFileName(self, "save file", "./", "ALL (*.csv)")
        if fname:
            self.exportToExcel(self.ui.tableWidget, fname)
            logger(f"export {fname}")

    def exportToExcel(self, table: QTableWidget, filename: str):
        # wb = openpyxl.Workbook()
        columnHeaders = []
        # create column header list
        columnHeaders.append("class")
        for j in range(table.columnCount()):
            columnHeaders.append(table.horizontalHeaderItem(j).text())
        df = pd.DataFrame(columns=columnHeaders)

        # create dataframe object recordset
        for row in range(table.rowCount()):
            for col in range(table.columnCount()):
                item = table.item(row, col)
                df.at[row, columnHeaders[col+1]] = item.text() if item is not None else ""
            df.at[row, "class"] = table.verticalHeaderItem(row).text()
        df.to_csv(filename, index=False, encoding="utf-8_sig")

    @staticmethod
    def show_img_in_label(img: np.ndarray, label: QLabel, is_bgr: bool = True):
        ih, iw, _ = img.shape
        fx = (label.width() - 8) / iw
        fy = (label.height() - 8) / ih
        f = min(fx, fy)
        img = cv2.resize(img, None, fx=f, fy=f)
        if is_bgr:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        showImage = QImage(img.data, img.shape[1], img.shape[0], img.shape[2] * img.shape[1], QImage.Format_RGB888)
        label.setPixmap(QPixmap.fromImage(showImage))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos() 
            event.accept()
            self.setCursor(QCursor(Qt.OpenHandCursor))

    def mouseMoveEvent(self, QMouseEvent):
        if Qt.LeftButton and self.m_flag:
            self.move(QMouseEvent.globalPos() - self.m_Position)
            QMouseEvent.accept()

    def mouseReleaseEvent(self, QMouseEvent):
        self.m_flag = False
        self.setCursor(QCursor(Qt.ArrowCursor))

    def max_or_restore(self):
        if self.ui.maxButton.isChecked():
            self.showMaximized()
        else:
            self.showNormal()

    def closeEvent(self, a0: QCloseEvent) -> None:
        logger("close")
        if hasattr(self, "inferthread") and self.inferthread.isRun:
            self.inferthread.stopRun()
            self.inferthread.wait()
        if self.videocap.is_open:
            self.videocap.close_camera()
        if self.platform.is_connected:
            self.timer.stop()
            self.platform.disconnect()
            time.sleep(0.5)
        return super().closeEvent(a0)

class ObjStatistics:
    def __init__(self, class_names:tuple) -> None:
        self.stat: Dict[str, List[int]] = {}
        self.class_names = class_names
    def add_obj(self, _class: int) -> None:
        _class = self.class_names[_class]
        logger(f"====add obj {_class}")
        if not _class in self.stat:
            self.stat[_class] = [0, 0]
        self.stat[_class][0] += 1

    def add_grasping(self, _class: int, _successful:bool) -> None:
        _class = self.class_names[_class]
        logger(f"====add grasping {_class} {_successful}")
        if _successful:
            self.stat[_class][1] += 1

class ObjectVisual:
    def __init__(self, cal: RobotCalibration) -> None:
        self.cal = cal
        cam_w, cam_h = cal.resolution[1], cal.resolution[0]
        vup = np.array([[[0, 0]], [[cam_w - 1, 0]], [[cam_w - 1, cam_h - 1]], [[0, cam_h - 1]]], dtype=np.float32)
        if cam_w == 1600:  ## Baslar camera
            pass
        else:  # Realsense
            vup = vup[[1, 2, 3, 0]]
        pos = self.cal.get_world_cartesian(self.cal.Rt_matrix[-1], vup)
        #  ^ X
        #  |
        #  p1 -------- p2  ^
        #  |            |  | Conveyor belt direction
        #  |            |  |
        #  p4 -------- p3 ---> Y
        _pos = np.column_stack((pos, np.zeros_like(pos)))
        p1, p2, p3 ,p4 = self.cal.get_pos_in_beltframe(_pos, 0)[:, :3]
        self.fov_width = max(p2[1] - p1[1], p3[1] - p4[1])
        self.fov_height = max(p2[0] - p3[0], p1[0] - p4[0])
        self.x_offset = -p4[0]  # Offset the origin to the point p4
        self.y_offset = -p4[1]
        self.f = 250 / self.fov_width
        self.indicate_map = np.zeros((int((cal.L1 + cal.L2 + self.x_offset) * self.f), int(self.fov_width * self.f), 3), dtype=np.uint8)
        self.clear()

    def clear(self):
        self.object_num = 0
        self.indicate_map.fill(35)
        cv2.line(self.indicate_map, (0, int((self.cal.L1 + self.x_offset) * self.f)), (self.indicate_map.shape[1] - 1, int((self.cal.L1 + self.x_offset) * self.f)), (0, 255, 0), 3)

    def add_object(self, img: np.ndarray, grasp_rect: np.ndarray, dis: float):
        """
        Extract the object from the image to the actual conveyor belt location
        img: image of camera
        grasp_rect:  [cx, cy, angle, width, height]
        dis: The distance from the location where the photo was taken to the current location
        """
        for i, obj in enumerate(grasp_rect):
            self.object_num += 1
            points = rotate_ract_in_imgframe(obj)
            p1 = np.min(points[:, 0]), np.min(points[:, 1])
            p2 = np.max(points[:, 0]), np.max(points[:, 1])
            roi = img[p1[1] : p2[1], p1[0] : p2[0]]
            # cv2.imshow(f"roi", roi)
            # cv2.waitKey(0)
            p_in_img = [[p1], [[p2[0], p1[1]]], [p2], [[p1[0], p2[1]]], [obj[:2]]]
            vup = np.array(p_in_img, dtype=np.float32)
            pos = self.cal.get_world_cartesian(self.cal.Rt_matrix[-1], vup)
            pc = self.cal.get_pos_in_beltframe(np.array([pos[4][0], pos[4][1], pos[4][2], 0, 0, 0]), dis)[0]
            newsize = (int(np.linalg.norm(pos[0] - pos[1]) * self.f), int(np.linalg.norm(pos[2] - pos[1]) * self.f))
            roi = cv2.resize(roi, newsize)
            cy = int((pc[1] + self.y_offset) * self.f)
            cx = int((pc[0] + self.x_offset) * self.f)
            xx, yy = np.array((cx - roi.shape[0] / 2, cx + roi.shape[0] / 2), dtype=np.int16), np.array((cy - roi.shape[1] / 2, cy + roi.shape[1] / 2), dtype=np.int16)
            # xx, yy = np.clip(xx, 0, self.indicate_map.shape[0]), np.clip(yy, 0, self.indicate_map.shape[1])
            try:
                self.indicate_map[xx[0] : xx[1], yy[0] : yy[1], :] = roi
            except:
                # traceback.print_exc()
                logger.warning(f"indicate_map error {xx, yy}, {newsize}")

    def get_map(self):
        return self.indicate_map


class CalGUI:

    def __init__(self, main_win: MainWin, videocap: VideoGrab, pt: robot_platform.Platform, calibration: RobotCalibration) -> None:
        self.ui = main_win.ui
        self.main_win = main_win
        self.videocap = videocap
        self.camcal = calibration
        self.pt = pt
        self.cal_dir = ROOT / "calibration"
        self.ui.cal_btn_next.clicked.connect(self._solt_cal_next)
        self.ui.cal_btn_prev.clicked.connect(self._solt_cal_prev)
        self.ui.pb_cal_rd1.clicked.connect(lambda: self._solt_cal_rd_pos(1))
        self.ui.pb_cal_rd2.clicked.connect(lambda: self._solt_cal_rd_pos(2))
        self.ui.pb_cal_rd3.clicked.connect(lambda: self._solt_cal_rd_pos(3))
        self.ui.pb_cal_rd4.clicked.connect(lambda: self._solt_cal_rd_pos(4))
        self.ui.pb_cal_rd5.clicked.connect(lambda: self._solt_cal_rd_pos(5))
        self.ui.pb_cal_rd6.clicked.connect(lambda: self._solt_cal_rd_pos(6))
        self.ui.pb_cal_rd7.clicked.connect(lambda: self._solt_cal_rd_pos(7))
        self.ui.pb_cal_rd8.clicked.connect(lambda: self._solt_cal_rd_pos(8))
        self.ui.pb_cal_rd9.clicked.connect(lambda: self._solt_cal_rd_pos(9))
        self.ui.pb_cal_rd10.clicked.connect(lambda: self._solt_cal_rd_pos(10))
        self.ui.pb_cal_rd11.clicked.connect(lambda: self._solt_cal_rd_pos(11))
        self.ui.pb_cal_rd12.clicked.connect(lambda: self._solt_cal_rd_pos(12))
        self.ui.pb_cal_rd13.clicked.connect(lambda: self._solt_cal_rd_pos(13))
        self.ui.pb_cal_rd14.clicked.connect(lambda: self._solt_cal_rd_pos(14))
        self.ui.pb_cal_rd15.clicked.connect(lambda: self._solt_cal_rd_pos(15))
        self.ui.pb_cal_rd16.clicked.connect(lambda: self._solt_cal_rd_pos(16))
        self.ui.pb_cal_rd1d.clicked.connect(lambda: self._solt_cal_rd_pos(17))
        QShortcut(Qt.Key_1, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(1))
        QShortcut(Qt.Key_2, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(2))
        QShortcut(Qt.Key_3, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(3))
        QShortcut(Qt.Key_4, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(4))
        QShortcut(Qt.Key_5, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(5))
        QShortcut(Qt.Key_6, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(6))
        QShortcut(Qt.Key_7, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(7))
        QShortcut(Qt.Key_8, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(8))
        QShortcut(Qt.Key_9, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(9))
        QShortcut(Qt.Key_0, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(10))
        QShortcut(Qt.Key_Minus, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(11))
        QShortcut(Qt.Key_Plus, self.ui.step3).activated.connect(lambda: self._solt_cal_rd_pos(12))
        QShortcut(Qt.Key_1, self.ui.step4).activated.connect(lambda: self._solt_cal_rd_pos(13))

        self.ui.cal_btn_shot.clicked.connect(self._solt_cal_shot)
        self.ui.stackedWidget_cal.setCurrentIndex(1)  # The currentChanged signal is triggered only when the page is set to another page
        self.ui.stackedWidget_cal.currentChanged.connect(self._solt_cal_changed)

    def camera_open(self) -> bool:
        index = self.ui.comboBox_cam.currentIndex()
        if len(self.videocap.get_scanned_cameras()) > 0:
            return self.videocap.open_camera(index)
        else:
            return False

    def _solt_cal_shot(self):
        fname = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime(time.time())) + "_src.jpg"
        name = str(self.cal_dir / fname)
        self.cal_imgs.append(name)
        self.ui.cal_imglist.addItem(str(self.ui.cal_imglist.count() + 1) + ". " + fname)
        cv2.imwrite(name, self.img)
        self.cal_encoder[0] = self.pt.encoder.distance

    def camera_cal_timer_cb(self):
        self.img = self.videocap.get_camera_frame()
        if isinstance(self.img, dict):
            self.img = self.img["rgb"]
            self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        img = self.camcal.find_and_draw_corner(self.img.copy(), 12)
        MainWin.show_img_in_label(img, self.ui.label_cam_cal)

    def _solt_cal_camera_open(self):
        if not self.videocap.is_open:
            if not self.camera_open():
                QMessageBox.warning(self, "Warning", "请检测相机与电脑是否连接正确", buttons=QMessageBox.Ok)
            else:
                # camera connected
                self.ui.cal_btn_camopen.setText(self.main_win._translate("mainwin", "已连接"))
                self.ui.cal_btn_next.setEnabled(True)

    def _solt_cal_rd_pos(self, i):
        if i == 1:  # upstream line
            self.cal_encoder[1] = self.pt.encoder.distance
        elif i == 17:  # downstream line
            self.cal_encoder[2] = self.pt.encoder.distance

        c = self.cal_rpos[i - 1] = self.pt.robot.get_world_position()
        if i == 17:
            self.ui.label_cal_p1d.setText(f"x:{c[0]:.2f} y:{c[1]:.2f} z:{c[2]:.2f}")  # show the coordinates of the robot in the downstream coordinate system
        else:
            label: QLabel = eval(f"self.ui.label_cal_p{i}")
            label.setText(f"x:{c[0]:.2f} y:{c[1]:.2f} z:{c[2]:.2f}")  # show the coordinates

    def _solt_cal_changed(self):
        cur = self.ui.stackedWidget_cal.currentIndex()
        logger(f"cal cur={cur}")
        if cur == 1:
            old_path = str(self.cal_dir.parent / (self.cal_dir.stem + "_old"))
            if self.cal_dir.exists() and not len(os.listdir(self.cal_dir)) == 0:
                shutil.rmtree(old_path, ignore_errors=True)
                shutil.move(str(self.cal_dir), old_path)
            self.cal_dir.mkdir(exist_ok=True)
            self.cal_imgs = []
            self.cal_encoder = np.zeros((3,))  # The three encoder values are the calibration board located at the camera, the upstream line, and the downstream line
            self.cal_rpos = np.zeros((17, 6))
            self.ui.cal_imglist.clear()
            self.timer_camera = QTimer(self.ui.page_4)
            self.timer_camera.start(100)
            self.timer_camera.timeout.connect(self.camera_cal_timer_cb)
        else:
            if hasattr(self, "timer_camera"):
                self.timer_camera.stop()
                self.videocap.close_camera()

        if cur == 0:
            index = self.ui.comboBox_cam.currentIndex()
            self.ui.label_cal_camera.setText("Camera: " + self.videocap.get_scanned_cameras()[index]["ModelName"])
            if not self.videocap.is_open:
                self.ui.cal_btn_camopen.setText(self.main_win._translate("mainwin", "连接"))
                self.ui.cal_btn_next.setEnabled(False)
                self.ui.cal_btn_camopen.clicked.connect(self._solt_cal_camera_open)

        if cur == 2:
            img = cv2.imread(self.cal_imgs[-1])
            img = self.camcal.find_and_draw_corner(img, 12)
            MainWin.show_img_in_label(img, self.ui.label_cal_point1234)

        if cur == 3:
            img = cv2.imread(self.cal_imgs[-1])
            img = self.camcal.find_and_draw_corner(img, 1)
            MainWin.show_img_in_label(img, self.ui.label_cal_point1)

        if cur == 4:
            pics = list(self.cal_dir.glob("*_src.jpg"))
            cam_err = self.camcal.calibration(pics)
            self.camcal.save(str(self.cal_dir / "cal_param.json"))
            bot_err = self.camcal.calibration_robot(self.cal_encoder, self.cal_rpos)
            self.camcal.save_bot_param(str(self.cal_dir / "robot_param.json"))
            self.ui.label_cal_L.setText(f"L1:{self.camcal.L1}mm, L2:{self.camcal.L2}mm")
            self.ui.label_cal_angle.setText(f"Angle of Robot to Belt:{np.rad2deg(self.camcal.angle)}°")

            s = ""
            for k, v in cam_err.items():
                s += f"{k}:{v:.4f} "
            self.ui.label_cal_cam_err.setText(s)
            s = ""
            for k, v in bot_err.items():
                if not len(v) == 0:
                    s += f"{k}:{np.mean(v):.4f} "
            self.ui.label_cal_bot_err.setText(s)
            self.ui.cal_btn_next.setText(self.main_win._translate("mainwin", "完成"))
        else:
            self.ui.cal_btn_next.setText(self.main_win._translate("mainwin", "下一步"))

    def _solt_cal_next(self):
        cnt = self.ui.stackedWidget_cal.count()
        cur = self.ui.stackedWidget_cal.currentIndex()
        if cur + 1 < cnt:
            self.ui.stackedWidget_cal.setCurrentIndex(cur + 1)
        else:
            self.ui.stackedWidget_cal.setCurrentIndex(0)

    def _solt_cal_prev(self):
        cur = self.ui.stackedWidget_cal.currentIndex()
        if cur > 0:
            self.ui.stackedWidget_cal.setCurrentIndex(cur - 1)


class InferThread(QThread):
    sinOut = pyqtSignal(dict)

    def __init__(self, model_name: str, model: str, backend: str, videocap: VideoGrab, platform: robot_platform.Platform, max_fps: int = 20, parent=None):
        super(InferThread, self).__init__(parent)
        self.videocap = videocap
        self.pf = platform
        self.interval = 1.0 / max_fps
        self.last_t = time.time()
        self.elapse = 30
        self.isRun = True
        self.mutex = QMutex()
        self.cond = QWaitCondition()
        self.detector = detection.Predictor(model_name, model, backend=backend)
        self.model_name = model_name

    def __del__(self):
        self.wait()

    def stopRun(self):
        self.isRun = False

    def get_real_fps(self) -> float:
        return 1.0 / self.elapse

    def run(self):
        while self.isRun:
            t0 = time.time()
            img = self.videocap.get_camera_frame()
            dis = self.pf.encoder.distance
            out: dict = self.detector.inference(img)
            if isinstance(img, dict):
                img = img["rgb"]
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # Here, the image is BGR channel
            for obj in out["grasp_rect"]:
                draw_ract(img, obj, detection.get_detector(self.model_name)._CLASS)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            res = {"img": img, "pred": out, "encoder": dis}
            self.sinOut.emit(res)
            t = time.time()
            self.elapse = t - self.last_t
            self.last_t = t
            delay = max(0, self.interval - (t - t0))
            self.msleep(int(delay * 1000))


class ManualDialog(QDialog):

    def __init__(self, pt: robot_platform.Platform, parent=None):
        super(QDialog, self).__init__(parent)
        self.pt = pt
        self.mainwin: MainWin = parent
        self.ui = manual.Ui_dlg_manual()
        self.ui.setupUi(self)
        self.belt_speed = 10
        self.ui.Slider_speed.setValue(10)
        self.ui.Slider_dis.setValue(40)
        self.ui.Slider_belt_speed.setValue(self.belt_speed)
        self.ui.Slider_belt_speed.valueChanged.connect(self._solt_vchanged)

        self.ui.pushButton_home.pressed.connect(self.btn_home_callback)
        self.ui.pushButton_home.released.connect(self.btn_home_callback)
        self.ui.pushButton_xp.pressed.connect(self.btn_right_callback)
        self.ui.pushButton_xp.released.connect(self.btn_right_callback)
        self.ui.pushButton_xm.pressed.connect(self.btn_left_callback)
        self.ui.pushButton_xm.released.connect(self.btn_left_callback)
        self.ui.pushButton_yp.pressed.connect(self.btn_backward_callback)
        self.ui.pushButton_yp.released.connect(self.btn_backward_callback)
        self.ui.pushButton_ym.pressed.connect(self.btn_forward_callback)
        self.ui.pushButton_ym.released.connect(self.btn_forward_callback)
        self.ui.pushButton_zp.pressed.connect(self.btn_up_callback)
        self.ui.pushButton_zp.released.connect(self.btn_up_callback)
        self.ui.pushButton_zm.pressed.connect(self.btn_down_callback)
        self.ui.pushButton_zm.released.connect(self.btn_down_callback)
        self.ui.pushButton_rp.pressed.connect(self.btn_cw_callback)
        self.ui.pushButton_rp.released.connect(self.btn_cw_callback)
        self.ui.pushButton_rm.pressed.connect(self.btn_ccw_callback)
        self.ui.pushButton_rm.released.connect(self.btn_ccw_callback)

        self.ui.Slider_speed.valueChanged.connect(self._solt_vchanged)
        self.ui.Slider_accel.valueChanged.connect(self._solt_vchanged)
        self.ui.Slider_curve.valueChanged.connect(self._solt_vchanged)
        self.ui.Slider_dis.valueChanged.connect(self._solt_vchanged)
        self._solt_vchanged()

        self.ui.pushButton_belt_m.clicked.connect(self.btn_belt_m)
        self.ui.pushButton_belt_p.clicked.connect(self.btn_belt_p)
        self.ui.pushButton_belt_stop.clicked.connect(self.btn_belt_stop)

        self.ui.pushButton_gripper_m.clicked.connect(self.btn_gripper_m)
        self.ui.pushButton_gripper_p.clicked.connect(self.btn_gripper_p)

        self.ui.pushButton_sucker.clicked.connect(self.btn_sucker)

        self.suker_en: bool = False

    def btn_belt_m(self):
        if self.pt.belt.is_run == -1:
            speed = self.ui.Slider_belt_speed.value()
            self.ui.Slider_belt_speed.setValue(speed + 10)
        else:
            self.ui.Slider_belt_speed.setValue(self.belt_speed)
        speed = self.ui.Slider_belt_speed.value()
        self.pt.belt.move(False, speed)

    def btn_belt_p(self):
        if self.pt.belt.is_run == 1:
            speed = self.ui.Slider_belt_speed.value()
            self.ui.Slider_belt_speed.setValue(speed + 10)
        else:
            self.ui.Slider_belt_speed.setValue(self.belt_speed)
        speed = self.ui.Slider_belt_speed.value()
        self.pt.belt.move(True, speed)

    def btn_belt_stop(self):
        self.pt.belt.stop()
        self.ui.Slider_belt_speed.setValue(self.belt_speed)

    def btn_gripper_m(self):
        self.pt.gripper.move_absolute(146, 1000, 3000, 3000, 0.1)

    def btn_gripper_p(self):
        self.pt.gripper.push(13, 120, 500)

    def btn_sucker(self):
        if self.suker_en:
            self.pt.gripper.sucker(False)
            self.suker_en = False
        else:
            self.suker_en = True
            self.pt.gripper.sucker(True)

    def keyPressEvent(self, QKeyEvent: QKeyEvent):  # Called when a key is pressed on the keyboard
        if QKeyEvent.isAutoRepeat():
            return

        k = QKeyEvent.key()
        if k == Qt.Key_A:
            self.ui.pushButton_xm.setDown(True)
            self.btn_left_callback()  # setdown() does not trigger the `clicked` signal, so we need to call it manually
        elif k == Qt.Key_D:
            self.ui.pushButton_xp.setDown(True)
            self.btn_right_callback()
        elif k == Qt.Key_W:
            self.ui.pushButton_yp.setDown(True)
            self.btn_backward_callback()
        elif k == Qt.Key_S:
            self.ui.pushButton_ym.setDown(True)
            self.btn_forward_callback()
        elif k == Qt.Key_Q:
            self.ui.pushButton_zp.setDown(True)
            self.btn_up_callback()
        elif k == Qt.Key_Z:
            self.ui.pushButton_zm.setDown(True)
            self.btn_down_callback()
        elif k == Qt.Key_E:
            self.ui.pushButton_rp.setDown(True)
            self.btn_cw_callback()
        elif k == Qt.Key_C:
            self.ui.pushButton_rm.setDown(True)
            self.btn_ccw_callback()

        elif k == Qt.Key_Comma:
            self.ui.pushButton_belt_m.click()
        elif k == Qt.Key_Period:
            self.ui.pushButton_belt_p.click()
        elif k == Qt.Key_Slash:
            self.ui.pushButton_belt_stop.click()

        elif k == Qt.Key_G:
            self.ui.pushButton_gripper_p.click()
        elif k == Qt.Key_H:
            self.ui.pushButton_gripper_m.click()
        elif k == Qt.Key_F:
            self.ui.pushButton_sucker.click()

    def keyReleaseEvent(self, QKeyEvent: QKeyEvent):  # Called when a key is released on the keyboard
        if QKeyEvent.isAutoRepeat():
            return
        k = QKeyEvent.key()
        if k == Qt.Key_A:
            self.ui.pushButton_xm.setDown(False)
            self.btn_left_callback()
        elif k == Qt.Key_D:
            self.ui.pushButton_xp.setDown(False)
            self.btn_right_callback()
        elif k == Qt.Key_W:
            self.ui.pushButton_yp.setDown(False)
            self.btn_backward_callback()
        elif k == Qt.Key_S:
            self.ui.pushButton_ym.setDown(False)
            self.btn_forward_callback()
        elif k == Qt.Key_Q:
            self.ui.pushButton_zp.setDown(False)
            self.btn_up_callback()
        elif k == Qt.Key_Z:
            self.ui.pushButton_zm.setDown(False)
            self.btn_down_callback()
        elif k == Qt.Key_E:
            self.ui.pushButton_rp.setDown(False)
            self.btn_cw_callback()
        elif k == Qt.Key_C:
            self.ui.pushButton_rm.setDown(False)
            self.btn_ccw_callback()

    def _solt_vchanged(self):
        self.speed = self.ui.Slider_speed.value()
        self.accel = self.ui.Slider_accel.value()
        self.curve = self.ui.Slider_curve.value()
        self.distance = self.ui.Slider_dis.value()
        self.ui.label_speed.setText(f"速度:{self.speed}")
        self.ui.label_accel.setText(f"加速度:{self.accel}")
        self.ui.label_dis.setText(f"位移:{self.distance}")
        self.ui.label_curve.setText(f"曲线:{self.curve}")

        bs = self.ui.Slider_belt_speed.value()
        self.ui.label_belt_speed.setText(f"传送带速度:{bs}")

    def btn_home_callback(self):
        if not self.ui.pushButton_home.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_ABS, dx=0, dy=0, dz=-800, dr=0, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_right_callback(self):
        if not self.ui.pushButton_xp.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dx=self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_left_callback(self):
        if not self.ui.pushButton_xm.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dx=-self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_backward_callback(self):
        if not self.ui.pushButton_yp.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dy=self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_forward_callback(self):
        if not self.ui.pushButton_ym.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dy=-self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_up_callback(self):
        if not self.ui.pushButton_zp.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dz=-self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_down_callback(self):
        if not self.ui.pushButton_zm.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dz=self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_cw_callback(self):
        if not self.ui.pushButton_rp.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dr=self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def btn_ccw_callback(self):
        if not self.ui.pushButton_rm.isDown():
            self.pt.robot.halt()
        else:
            self.pt.robot.move(MODE_INC, dr=-self.distance, wait=0, speed=self.speed, accel=self.accel, decel=self.accel, scurve=self.curve)

    def closeEvent(self, a0: QCloseEvent) -> None:
        logger("close manual")
        del self.mainwin.manualDlg
        return super().closeEvent(a0)


class MySplashScreen(QSplashScreen):
    def __init__(self):
        super(MySplashScreen, self).__init__()

        self.movie = QMovie(":/mian_background/splash_bg.png")
        self.movie.frameChanged.connect(lambda: self.setPixmap(self.movie.currentPixmap()))
        self.movie.start()

    def mousePressEvent(self, QMouseEvent):
        pass


if __name__ == "__main__":
    multiprocessing.set_start_method("spawn")
    log_path = Path("run.log")
    log_path.mkdir(parents=True, exist_ok=True)
    logger.set_file_dir(log_path)
    _setting = Setting(ROOT / "sys-settings.json")
    myapp = QApplication(sys.argv)

    splash = MySplashScreen()
    splash.show()
    splash.showMessage(f"Loading...", Qt.AlignLeft | Qt.AlignBottom, Qt.white)

    myapp.processEvents()
    _pf = robot_platform.Platform()
    _video = VideoGrab()
    _video.scan_all_camera()
    splash.showMessage(f"All Done", Qt.AlignLeft | Qt.AlignBottom, Qt.white)
    win = MainWin(_pf, _video, _setting)
    win.show()
    splash.finish(win)
    splash.movie.stop()
    splash.deleteLater()
    sys.exit(myapp.exec_())
