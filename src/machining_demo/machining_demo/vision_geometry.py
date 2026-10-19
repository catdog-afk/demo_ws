"""固定俯视针孔相机标定与平面定位约束，不包含任何工件真实坐标查询。"""
import math
from machining_demo import layout

WIDTH = HEIGHT = 800
FOCAL = 900.0
CX = CY = 399.5
CAMERA_X, CAMERA_Y, CAMERA_Z = 0.45, 0.25, 1.0
FRAME = 'overhead_camera_optical_frame'
TRAY_CENTER_Z = 0.062
TOP_Z = TRAY_CENTER_Z + layout.WORKPIECE_SIZE / 2
MIN_CENTER_DISTANCE = 0.12


def project(x, y, z):
    depth = CAMERA_Z - z
    if not all(math.isfinite(v) for v in (x, y, z)) or depth <= 0:
        raise ValueError('点位不在相机前方')
    return CX + FOCAL * (x - CAMERA_X) / depth, CY - FOCAL * (y - CAMERA_Y) / depth


def unproject(u, v):
    if not all(math.isfinite(a) for a in (u, v)) or not (0 <= u < WIDTH and 0 <= v < HEIGHT):
        raise ValueError('像素坐标无效')
    depth = CAMERA_Z - TOP_Z
    return CAMERA_X + (u - CX) * depth / FOCAL, CAMERA_Y - (v - CY) * depth / FOCAL, TRAY_CENTER_Z


def validate_centers(centers):
    if len(centers) != layout.NUM_SLOTS:
        raise ValueError('必须同时识别三件物料，缺失或遮挡时不抓取')
    for point in centers:
        if len(point) != 3 or not all(math.isfinite(a) for a in point):
            raise ValueError('视觉位置包含无效数值')
        x, y, z = point
        if abs(z - TRAY_CENTER_Z) > 0.001:
            raise ValueError('物料不在已标定的料盘平面')
        for axis, value in enumerate((x, y)):
            half = layout.TRAY['size'][axis] / 2 - layout.WORKPIECE_SIZE / 2 - 0.003
            if abs(value - layout.TRAY['pos'][axis]) > half:
                raise ValueError('物料超出允许的料盘抓取区域')
    for i, a in enumerate(centers):
        for b in centers[i + 1:]:
            if math.hypot(a[0] - b[0], a[1] - b[1]) < MIN_CENTER_DISTANCE:
                raise ValueError('物料间距不足 12cm，请分开物料后重试')
    return centers


def initial_centers(offsets):
    values = [float(v) for v in offsets.split(',')]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('workpiece_offsets 需为6个有限数值 dx0,dy0,dx1,dy1,dx2,dy2')
    centers = []
    for slot in range(3):
        x, y, z = layout.slot_xyz(slot)
        centers.append((x + values[slot * 2], y + values[slot * 2 + 1], z))
    return validate_centers(centers)
