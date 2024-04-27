import cv2
import numpy as np
from pathlib import Path
import sys
from math import sin, cos, pi
import json
ROOT = Path(__file__).resolve().parent
DIR = ROOT/'..'
if str(DIR) not in sys.path:
    sys.path.append(str(DIR))
from utils import logger

def GetRigidTrans3D(srcPoints: np.ndarray, dstPoints: np.ndarray) -> tuple:
    """
    srcPoints: n x 3 matrix
    dstPoints: n x 3 matrix
    pointsNum: number of paired points

    example:
    p0 = np.array([[0, 0, 0], [3, 0, 0], [0, 3, 0]])
    p1 = np.array([[0, 0, 0], [0, 3, 0], [3, 0, 0]])
    GetRigidTrans3D(p0, p1)
    """

    assert len(srcPoints) == len(dstPoints)
    pointsNum = srcPoints.shape[0]  # Total points

    srcMeans = srcPoints.mean(axis=0)
    dstMeans = dstPoints.mean(axis=0)

    srcMat = np.zeros((3, pointsNum))
    dstMat = np.zeros((3, pointsNum))

    for i in range(pointsNum):
        srcMat[0][i] = srcPoints[i][0] - srcMeans[0]
        srcMat[1][i] = srcPoints[i][1] - srcMeans[1]
        srcMat[2][i] = srcPoints[i][2] - srcMeans[2]

        dstMat[0][i] = dstPoints[i][0] - dstMeans[0]
        dstMat[1][i] = dstPoints[i][1] - dstMeans[1]
        dstMat[2][i] = dstPoints[i][2] - dstMeans[2]

    matS = np.matmul(srcMat, dstMat.T)
    matU, Sigma, matVt = np.linalg.svd(matS, full_matrices=False)
    matR = matVt.T @ matU.T
    matT = dstMeans - matR @ srcMeans

    hm = np.eye(4)
    hm[:3, 3] = matT.T
    hm[:3, :3] = matR
    # print("hm:\n", hm)

    errs = np.zeros((pointsNum,))
    for i in range(pointsNum):
        s = np.concatenate((srcPoints[i], np.ones(1)), axis=0)
        p = (hm @ s)[:3]
        err = np.linalg.norm(p-dstPoints[i])
        errs[i] = err
        logger(f'point err: {err}')
    return hm, errs


#Used to calculate the rotation matrix in terms of Euler angles
def _RPY2Rmat(x, y, z, to_hm:bool=False):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx # RPY 角
    if to_hm:
        hm = np.eye(4)
        hm[:3, :3] = R
        return hm
    return R

# Calculates rotation matrix to euler angles
def RM_2_RPY(R:np.ndarray) :

    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
 
    return np.array([np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)])


#It is used to calculate the transformation matrix based on the pose

def pose_robot(Tx, Ty, Tz, Rx, Ry, Rz):
    thetaX = Rx / 180 * pi
    thetaY = Ry / 180 * pi
    thetaZ = Rz / 180 * pi
    R = _RPY2Rmat(thetaX, thetaY, thetaZ)
    t = np.array([[Tx], [Ty], [Tz]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0, 0, 0, 1])))
    return RT1


class CameraCalibration:

    def __init__(self, cb_h:int=10, cb_w:int=7, cb_d:float=20) -> None:
        np.set_printoptions(precision=3, suppress=True)
        self.h, self.w, self.d = cb_h, cb_w, cb_d  # The height, width, and spacing of a checkerboard
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.h * self.w, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.h, 0:self.w].T.reshape(-1, 2)
        self.objp = objp * self.d

    def find_and_draw_corner(self, img: np.ndarray, draw_corner_num: int) -> np.ndarray:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (self.h, self.w), None)  # Find the chess board corners
        if ret == True:
            # Sub-pixel intersection detection shape=(70, 1, 2)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            cv2.drawChessboardCorners(img, (self.h, self.w), corners2, ret)
            if draw_corner_num > 0:
                points = corners.reshape(self.w, self.h, 2)[0::3, 0::3].reshape(-1, 2)
                for i, p in enumerate(points):
                    if i >= draw_corner_num:
                        break
                    center = p.squeeze().astype(np.int16)
                    cv2.circle(img, center, 20, (0, 0, 200), 5)  # draw the original point
                    cv2.circle(img, center, 10, (0, 0, 120), 4)  # draw the original point
                    cv2.putText(img, str(i + 1), center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 255), 4)
                    cv2.putText(img, str(i + 1), center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        return img

    def calibration(self, pictures: list):
        # From https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html
        # https://docs.opencv.org/4.7.0/d9/d0c/group__calib3d.html
        pictures = [Path(f) for f in pictures]

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        for i, f in enumerate(pictures):
            print(f"Processing {i+1}/{len(pictures)} - {str(f)}", end=' ')
            img = cv2.imread(str(f))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (self.h, self.w), None)  # Find the chess board corners
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(self.objp)
                # Sub-pixel intersection detection shape=(70, 1, 2)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img, (self.h, self.w), corners2, ret)
                center = corners2[0].squeeze().astype(np.int16)
                cv2.circle(img, center, 10, (255, 100, 140), 4)  # draw the original point
                cv2.imwrite(str(f.parent / f.stem) + '_corner.jpg', img)
                print("> OK")
            else:
                print("> Fail to find corner")

        self.resolution = img.shape[:2] # (1200, 1600, 3)
        ret, self.cameraMatrix, self.distcoeff, self.rvecs, self.tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)
        self.cameraMatrix: np.ndarray
        self.distcoeff: np.ndarray
        self.rvecs: np.ndarray
        self.tvecs: np.ndarray
        self.Rt_matrix = []
        for (rvec, tvec) in zip(self.rvecs, self.tvecs):
            R_matrix, _ = cv2.Rodrigues(rvec)
            self.Rt_matrix.append(np.concatenate((R_matrix, tvec), axis=1))
        self.Rt_matrix: np.ndarray = np.asarray(self.Rt_matrix)
        self.rvecs = np.asarray(self.rvecs)
        self.tvecs = np.asarray(self.tvecs)
        self.imgpoints = np.asarray(imgpoints)

        # Re-projection Error
        mean_error = [0, 0]
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], self.rvecs[i], self.tvecs[i], self.cameraMatrix, self.distcoeff)
            world = self.get_world_cartesian(self.Rt_matrix[i], imgpoints[i])
            error1 = cv2.norm(objpoints[i], world, cv2.NORM_L2) / len(imgpoints2)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error[0] += error
            mean_error[1] += error1
        errs = {'distort_err':mean_error[0] / len(objpoints), 'world_err':mean_error[1] / len(objpoints)}
        logger(f"Reproject mean error: {errs}")

        # Generate undistort image
        for f in pictures:
            img = cv2.imread(str(f))
            dst = self.undistort_img(img)
            cv2.imwrite(str(f.parent / f.stem) + '_cal.jpg', dst)

        return errs


    def undistort_img(self, img: np.ndarray) -> np.ndarray:

        if not hasattr(self, "mapx"):
            print("Prepare undistort maps")
            h, w = img.shape[:2]
            newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.cameraMatrix, self.distcoeff, (w, h), 1, (w, h))
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.cameraMatrix, self.distcoeff, None, newcameramtx, (w, h), 5)

        dst = cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)
        # crop the image
        x, y, w, h = self.roi
        dst = dst[y:y + h, x:x + w]
        return dst

    def get_world_cartesian(self, Rt_matrix: np.ndarray, uvpoints: np.ndarray) -> np.ndarray:
        assert uvpoints.shape[1] == 1 and uvpoints.shape[2] == 2, f"input shape:{uvpoints.shape}"
        uvpoints = cv2.undistortPoints(uvpoints, self.cameraMatrix, self.distcoeff, None, self.cameraMatrix)
        uvPoint = uvpoints.squeeze(axis=1).T
        uvPoint = np.vstack((uvPoint, np.ones((1, uvPoint.shape[1]))))  # uvPoint.shape (3, 70)

        # From https://stackoverflow.com/questions/34140150/reverse-of-opencv-projectpoints
        R = Rt_matrix[:, :3]
        t = Rt_matrix[:, 3].reshape(3, -1)

        # Lcam = self.cameraMatrix.dot(Rt_matrix)
        # Z = 0
        # a = np.hstack((Lcam[:, 0:2], -1 * uvPoint))
        # o = (-Z * Lcam[:, 2] - Lcam[:, 3]).reshape(3, -1)
        # X = np.linalg.inv(a).dot(o)

        # From https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point
        R_inv = np.linalg.inv(R)
        M_inv = np.linalg.inv(self.cameraMatrix)
        leftSideMat = R_inv @ M_inv @ uvPoint
        rightSideMat = R_inv @ t
        s = rightSideMat[2, 0] / leftSideMat[2, :]  # s.shape (70,)
        for i in range(uvPoint.shape[1]):
            uvPoint[:, i] = uvPoint[:, i] * s[i]
        X = R_inv @ M_inv @ uvPoint - rightSideMat
        return X.astype(np.float32).T

    def save(self, path: str) -> bool:
        meta = {}
        meta['cameraMatrix'] = self.cameraMatrix.tolist()
        meta['distcoeff'] = self.distcoeff.tolist()
        meta['rvecs'] = self.rvecs.tolist()
        meta['tvecs'] = self.tvecs.tolist()
        meta['imgpoints'] = self.imgpoints.tolist()
        meta["Rt_matrix"] = self.Rt_matrix.tolist()
        meta["resolution"] = list(self.resolution)
        jsonString = json.dumps(meta, indent=2, ensure_ascii=True)
        with open(path, "w") as f:
            f.write(jsonString)
            logger('parameters saved')
            return True
        return False

    def load(self, path: str) -> bool:
        try:
            with open(path, "r") as f:
                js = json.loads(f.read())
        except FileNotFoundError:
            logger(f'File "{path}" not found')
            return False
        logger(f'parameters load:{js.keys()}')
        self.cameraMatrix = np.array(js['cameraMatrix'])
        self.distcoeff = np.array(js['distcoeff'])
        self.rvecs = np.array(js['rvecs'])
        self.tvecs = np.array(js['tvecs'])
        self.imgpoints = np.array(js['imgpoints'])
        self.Rt_matrix = np.array(js['Rt_matrix'])
        self.resolution = np.array(js['resolution'])
        return True

class RobotCalibration(CameraCalibration):

    def __init__(self) -> None:
        super().__init__(cb_h=6, cb_w=4, cb_d=40)

    def calibration_robot(self, encoder: np.ndarray, postion: np.ndarray):
        """
        encoder: shape:(3,)编码器的三个值, [相机校准处的值， 上游线的值，下游线的值]
        postion: shape:(17, 6)机器人的17个姿态，前4个是圆形标记坐标，其后12个计算误差用，[指向上游线原点（点1）的坐标, 指向上游线点2的坐标, 指向上游线点3的坐标, 指向上游线点17的坐标, 指向下游线原点的坐标]
        
        for example:
        encoder = np.array([1120.507, 1957.617, 2560.742])
        postion = np.array([
            [  -5.062,  333.684, -937.981,    0.   ,  180.   ,   -0.003],
            [   8.824,  468.984, -937.979,    0.   ,  180.   ,   -0.003],
            [ -94.728,  343.425, -937.974,    0.   ,  180.   ,   -0.003],
            [ -80.351,  478.277, -937.978,    0.   ,  180.   ,   -0.003],
            [ 308.359, -187.718, -937.97 ,    0.   ,  180.   ,   -0.003],
        ])
        """
        self.encoder ,self.postion = encoder, postion
        e1, e2, e3 = encoder
        self.L2 = np.linalg.norm(postion[0, :3] - postion[-1, :3]) # 上下游之间的距离
        self.Ke = self.L2 / (e3 - e2)  # 编码器系数=dx/de
        self.L1 = self.Ke * (e2-e1) # 相机坐标系到上游坐标系的平移距离
        logger(f"L1:{self.L1}, L2:{self.L2}, Ke={self.Ke}")

        def find_vaild_point(postion, obj_point):
            _bot_pos = []
            _obj_pos = []
            for i, _p in enumerate(postion):
                if not _p[2]==0: # 根据z轴坐标判断当前坐标是否有效
                    _bot_pos.append(_p)
                    d, m = divmod(i, divmod((self.h-1), 3)[0]+1) # 点行数
                    _obj_pos.append(obj_point[d*3*self.h+m*3])
            return np.array(_bot_pos), np.array(_obj_pos)

        # _bp, _op = find_vaild_point(postion[:12], self.objp)
        _bp = postion[:4]
        _op = self.objp.take([0, self.h - 1, (self.w - 1) * self.h, self.h * self.w - 1], axis=0)
        _op[0][:2] -= self.d/2
        _op[1][0] += self.d/2
        _op[1][1] -= self.d/2
        _op[2][0] -= self.d/2
        _op[2][1] += self.d/2
        _op[3][:2] += self.d/2

        # print("points:", _bp, _op)
        Tur, ur_errs = GetRigidTrans3D(_op, _bp[:, :3]) # 求上游线位置坐标系到机器人坐标系的变换矩阵
        p5_in_u = np.linalg.inv(Tur) @ np.concatenate((postion[-1, :3], np.ones(1)), axis=0) # 求得下游坐标系原点在上游坐标系的坐标
        logger(f"p5_in_u={p5_in_u}")
        self.angle = np.arctan2(p5_in_u[1], p5_in_u[0])
        logger(f"angle:{np.rad2deg(self.angle)}")
        self.Tu1_u = _RPY2Rmat(0, 0, self.angle, to_hm=True)
        # print("==>", self.Tu_u1 @ np.array([135, 90, 0, 1]))
        self.Tu1_r = Tur @ self.Tu1_u # 传送带坐标系原点在上游位置的标定板原点，x轴平行于传送带运行方向
        # print("==>", self.Tr_u1 @ np.array([-80.351,  478.277, -937.978, 1]))
        # print("==>", np.linalg.inv(self.Tr_u1) @ np.array([-162.2, 9.7, 0, 1]))
        self.Tb_u1 = np.eye(4)
        self.Tb_u1[0, 3] = -self.L1 # x轴上的平移
        # print("==>", self.Tb_u1 @ np.array([-162.2, 9.7, 0, 1]))
        self.Tb_r = self.Tu1_r @ self.Tb_u1  # 传送带坐标系到机器人坐标系
        # print("==>", self.Tr_b @ np.array([-80.351,  478.277, -937.978, 1]))

        # 工具坐标系
        self.Te_tg = pose_robot(0, 0, 0, 0, 0, -30) # 夹爪
        self.Te_ts = pose_robot(157, 0, -20, 0, 0, -30) # 吸盘

        # 计算误差
        logger("----------------------")
        vup = self.imgpoints[-1]
        pos = self.get_world_cartesian(self.Rt_matrix[-1], vup)
        # print(np.linalg.norm(self.objp-pos, axis=1, keepdims=True))
        # print("----------------------")
        _bp, _op = find_vaild_point(postion[4:16], pos)

        bot_err=np.zeros((len(_op),))
        for i, point in enumerate(_op):
            p = self.get_pos_in_robotframe_withtool(np.array([point[0], point[1], point[2], 0, 0, 0]), self.L1, True)[0]
            err = np.linalg.norm(p[:3]-_bp[i][:3])
            bot_err[i] = err
            logger(f"p{point}, image to robot error={err}")
        logger("----------------------")

        return {'Tur_err':ur_errs, 'bot_err':bot_err}

    def save_bot_param(self, path: str) -> bool:
        meta = {}
        meta['encoder'] = self.encoder.tolist()
        meta['postion'] = self.postion.tolist()
        meta['Tu1_u'] = self.Tu1_u.tolist()
        meta['Tb_r'] = self.Tb_r.tolist()
        meta['Te_tg'] = self.Te_tg.tolist()
        meta['Te_ts'] = self.Te_ts.tolist()
        meta['Ke'] = self.Ke
        meta['L1'] = self.L1
        meta['L2'] = self.L2
       
        jsonString = json.dumps(meta, indent=2, ensure_ascii=True)
        with open(path, "w") as f:
            f.write(jsonString)
            logger('parameters saved')
            return True
        return False

    def load_bot_param(self, path: str) -> bool:
        try:
            with open(path, "r") as f:
                js = json.loads(f.read())
        except FileNotFoundError:
            logger(f'File "{path}" not found')
            return False
        logger(f'parameters load:{js.keys()}')
        self.encoder = np.array(js['encoder'])
        self.postion = np.array(js['postion'])
        self.Tu1_u = np.array(js['Tu1_u'])
        self.Tb_r = np.array(js['Tb_r'])
        self.Te_tg = np.array(js['Te_tg'])
        self.Te_ts = np.array(js['Te_ts'])
        self.Ke = float(js['Ke'])
        self.L1 = float(js['L1'])
        self.L2 = float(js['L2'])
       
        return True

    def xyz2hm(self, pos:np.ndarray):
        if not pos.shape == (3,):
            logger(f"shape error {pos.shape}")
        p = np.ones((4, 1))
        p[:3, 0] = pos.reshape((3,))
        return p

    def get_pos_in_beltframe(self, pos_in_cam:np.ndarray, dis:float):
        """
        pos_in_cam: 相机坐标系下的位姿
        dis: 传送带相对相机坐标系原点运动的距离
        """
        if len(pos_in_cam.shape) == 1:
            pos_in_cam = np.expand_dims(pos_in_cam, axis=0)
        out = np.zeros_like(pos_in_cam)
        for i, p in enumerate(pos_in_cam):
            p = self.Tu1_u.T @ pose_robot(p[0], p[1], p[2], p[3], p[4], p[5])
            p[0][3] = p[0][3] + dis
            rpy = RM_2_RPY(p[:3, :3])
            out[i] = np.array([p[0][3], p[1][3], p[2][3], rpy[0], rpy[1], rpy[2]])
        return out

    def get_pos_in_robotframe_withtool(self, pos_in_cam:np.ndarray, dis:float, tool_grasp:bool):
        pos_in_belt = self.get_pos_in_beltframe(pos_in_cam, dis)
        out = np.zeros_like(pos_in_belt)
        for i, p in enumerate(pos_in_belt):
            p = self.Tb_r @ pose_robot(p[0], p[1], p[2], p[3], p[4], p[5])
            if tool_grasp:
                p = p @ self.Te_tg
            else:
                p = p @ self.Te_ts

            rpy = RM_2_RPY(p[:3, :3])
            if rpy[2] < -180:
                rpy[2] += 180
            out[i] = np.array([p[0][3], p[1][3], p[2][3], rpy[0], rpy[1], rpy[2]])
        return out


if __name__ == "__main__":

    c = RobotCalibration()
    
    c.load(str(ROOT/"../calibration/cal_param.json"))
    c.load_bot_param(str(ROOT/"../calibration/robot_param.json"))

    d1 = np.linalg.norm(c.postion[0][:3]-c.postion[1][:3])
    d2 = np.linalg.norm(c.postion[1][:3]-c.postion[3][:3])
    d3 = np.linalg.norm(c.postion[2][:3]-c.postion[3][:3])
    d4 = np.linalg.norm(c.postion[0][:3]-c.postion[2][:3])
    print(f"d={d1, d2, d3, d4}")
    m = np.mgrid[0:c.h, 0:c.w].T.reshape(-1, 2)
    # c.calibration_robot(c.encoder, c.postion)
    # p = c.get_pos_in_beltframe(np.array([0, 0, 0, 0, 0, 0]), c.L1)
    # print(f"b=== {p}")
    # p = c.get_pos_in_robotframe(np.array([135, 0, 10, 0, 0, 0]), c.L1+c.L2)
    # print(f"r=== {p}")
    exit()

    c = CameraCalibration()
    c.calibration('calibration')
    c.save("./cal_param.json")
    c.load("./cal_param.json")