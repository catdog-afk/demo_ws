"""OpenCV 彩色方块识别。输入只有 BGR 像素与固定相机标定，不读取场景真值。"""
import cv2
import numpy as np
from machining_demo import layout
from machining_demo.vision_geometry import (WIDTH, HEIGHT, FOCAL, CAMERA_Z, TOP_Z,
                                           project, unproject, validate_centers)

HSV_RANGES = (((4, 110, 90), (18, 255, 255)),
              ((19, 110, 90), (36, 255, 255)),
              ((125, 90, 80), (165, 255, 255)))


def detect(image):
    if image.dtype != np.uint8 or image.shape != (HEIGHT, WIDTH, 3):
        raise ValueError('相机图像尺寸/编码不符合固定标定')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    centers, boxes = [], []
    expected = FOCAL * layout.WORKPIECE_SIZE / (CAMERA_Z - TOP_Z)
    for low, high in HSV_RANGES:
        mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        candidates = []
        for i in range(1, count):
            x, y, w, h, area = stats[i]
            if area < 25:
                continue
            # 拒绝合并物体、严重遮挡、大小不符及旋转方块；本版本仅支持平移。
            if (abs(w - expected) > 4 or abs(h - expected) > 4 or
                    area / float(w * h) < 0.90):
                raise ValueError('物料轮廓异常或遮挡，不能确认可抓取位姿')
            candidates.append((centroids[i], (int(x), int(y), int(w), int(h))))
        if len(candidates) != 1:
            raise ValueError('物料颜色缺失或存在多个同色目标')
        pixel, box = candidates[0]
        centers.append(unproject(float(pixel[0]), float(pixel[1])))
        boxes.append(box)
    validate_centers(centers)
    annotated = image.copy()
    for i, ((x, y, w, h), xyz) in enumerate(zip(boxes, centers)):
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (255, 255, 255), 1)
        cv2.putText(annotated, '%d: %.3f,%.3f' % (i, xyz[0], xyz[1]),
                    (x - 35, y - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return centers, annotated


def render(objects):
    """仅虚拟相机使用：将场景的轴对齐箱体投影成像，最近表面最后绘制。"""
    image = np.full((HEIGHT, WIDTH, 3), 35, dtype=np.uint8)
    for name, xyz, size in sorted(objects, key=lambda obj: obj[1][2] + obj[2][2] / 2):
        x, y, z = xyz
        dx, dy, dz = (a / 2 for a in size)
        corners = np.array([project(x + a, y + b, z + dz)
                            for a, b in ((-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy))])
        color = layout.OBJECT_COLORS.get(name, (0.5, 0.5, 0.5, 1.0))
        bgr = tuple(round(255 * v) for v in color[2::-1])
        cv2.fillConvexPoly(image, np.rint(corners).astype(np.int32), bgr)
    return image
