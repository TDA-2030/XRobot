import cv2
import math
import numpy as np

def draw_ract(img:np.ndarray, result:np.ndarray, class_names:tuple=None):
    """
    img: BGR color channel
    result: [cx, cy, angle_deg, width, height, cls]
    ```
      p0 ------------- p1  
      |                |  
      |       c        |  
      |                |  
      p3 ------------- p2
    ```
    """
    x = int(result[0])  # 矩形框的中心点x
    y = int(result[1])  # 矩形框的中心点y
    angle = result[2]      # 矩形框的倾斜角度（长边相对于水平）
    width, height = int(result[3]), int(result[4])  # 矩形框的宽和高
    if class_names:
        cls = result[5]

    angle = np.deg2rad(angle)
    cosA = np.cos(angle)
    sinA = np.sin(angle)
    y1 = y + width / 2 * sinA
    x1 = x - width / 2 * cosA
    y2 = y - width / 2 * sinA
    x2 = x + width / 2 * cosA

    y0n, x0n = int(y1 - height/2 * cosA), int(x1 - height/2 * sinA)
    y1n, x1n = int(y2 - height/2 * cosA), int(x2 - height/2 * sinA)
    y2n, x2n = int(y2 + height/2 * cosA), int(x2 + height/2 * sinA)
    y3n, x3n = int(y1 + height/2 * cosA), int(x1 + height/2 * sinA)

    # 根据得到的点，画出矩形框
    if not isinstance(img, type(None)):
        if class_names:
            cv2.putText(img, f"{class_names[int(cls)]}", (x0n, y0n),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0),
                            thickness=1,
                            lineType=cv2.LINE_AA)
        cv2.line(img, (x0n, y0n), (x1n, y1n), (0, 100, 200), 2, 4)
        cv2.line(img, (x1n, y1n), (x2n, y2n), (0, 100, 0), 2, 4)
        cv2.line(img, (x2n, y2n), (x3n, y3n), (0, 100, 200), 2, 4)
        cv2.line(img, (x0n, y0n), (x3n, y3n), (0, 100, 0), 2, 4)

        cv2.arrowedLine(img, (int((x0n+x3n)/2), int((y0n+y3n)/2)), (x, y), (0, 100, 200), 4, 4, tipLength=0.08)
        cv2.arrowedLine(img, (int((x2n+x1n)/2), int((y2n+y1n)/2)), (x, y), (0, 100, 200), 4, 4, tipLength=0.08)

def show_calibration_error():

    from pyecharts import options as opts
    from pyecharts.globals import ThemeType
    from pyecharts.charts import Bar3D
    from pyecharts.faker import Faker

    objp = np.zeros((10 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:10, 0:7].T.reshape(-1, 2)
    a = objp * 20
    a=a.reshape(7, 10, 3)
    p = a[0::3, 0::3]

    err = [0.8379426813758691,
    0.5836392491008828,
    0.30877032058446174,
    1.0173956616803577,
    0.601276357707165,
    0.4999557909974235,
    0.43409120980788246,
    0.3537586868179601,
    0.829762635469704,
    0.07320422057150901,
    0.37577258782907946,
    0.5831382058377675,]

    data = p.reshape(-1, 3)
    for i, d in enumerate(data):
        d /= 60
        d[2] = err[i]
    print(data)
    c = (
        Bar3D(init_opts=opts.InitOpts(theme=ThemeType.WHITE))
        .add(
            "",
            data.tolist(),
            shading = "lambert",
            xaxis3d_opts=opts.Axis3DOpts([0, 60, 120, 180], type_="category"),
            yaxis3d_opts=opts.Axis3DOpts([0, 60, 120], type_="category"),
            zaxis3d_opts=opts.Axis3DOpts(type_="value"),
        )
        .set_global_opts(
            visualmap_opts=opts.VisualMapOpts(max_=20),
            title_opts=opts.TitleOpts(title="Bar3D-基本示例"),
        )
        .render("bar3d_base.html")
    )