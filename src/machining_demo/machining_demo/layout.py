"""场景布局与共享常量。

坐标系说明：
    所有坐标均位于机械臂基座坐标系 panda_link0（即 MoveIt 的规划坐标系）下。
    机器人基座位于原点，桌面高度 z=0，机械臂工作空间约 0.85m。
"""
import math

# ---------- 工件与抓取参数 ----------
WORKPIECE_SIZE = 0.04        # 工件边长（正方体，单位 m）
NUM_SLOTS = 3                # 料盘槽位数
APPROACH_HEIGHT = 0.12       # 预抓取/预放置高度（工件上方，单位 m）
# 手指关节位于掌心下方 0.0584，指尖约在掌心下方 0.112 处；
# 掌心在工件中心上方 0.10 时，指尖到达工件中下部且高于料盘面约 8mm，
# 既保证抓取姿态合理，又避免指尖与料盘碰撞
GRASP_Z_OFFSET = 0.10        # 抓取时手部坐标系原点（掌心）相对工件中心的 z 偏移
ATTACH_Z_OFFSET = 0.10       # 附着时工件中心在手部坐标系（z 朝下）中的 z 位置

# 末端（panda_hand）z 轴朝下（自上而下抓取）对应的四元数（x, y, z, w）
GRASP_QUAT = (1.0, 0.0, 0.0, 0.0)

# ---------- 场景物体 ----------
# 桌面：顶面高度 z=0
TABLE = {'name': 'table', 'size': (1.2, 1.2, 0.10), 'pos': (0.25, 0.0, -0.05)}
# 料盘：顶面高度 z=0.04
TRAY = {'name': 'tray', 'size': (0.32, 0.32, 0.04), 'pos': (0.45, 0.25, 0.02)}
# 加工台（模拟机床）：顶面高度 z=0.10
MACHINE = {'name': 'machine', 'size': (0.34, 0.30, 0.10), 'pos': (0.10, -0.25, 0.05)}

# RGBA 颜色与碰撞对象 ID 对应；取料、附着和放回时保持一致。
OBJECT_COLORS = {
    'table': (0.40, 0.44, 0.52, 1.0),      # 深灰桌面
    'tray': (0.15, 0.45, 0.85, 1.0),       # 蓝色料盘
    'machine': (0.85, 0.85, 0.88, 1.0),    # 浅灰加工台
    'workpiece_0': (1.0, 0.45, 0.08, 1.0), # 橙色工件
    'workpiece_1': (1.0, 0.78, 0.12, 1.0), # 黄色工件
    'workpiece_2': (0.70, 0.30, 0.85, 1.0), # 紫色工件
}

# ---------- 关键点位 ----------
# 工件中心 z 比台面高 2mm（料盘顶 0.04 -> 0.062，加工台顶 0.10 -> 0.122）：
# 若箱子正好坐在台面上，抓取附着后箱子与台面"恰好接触"会被 FCL 判为碰撞，
# 导致提起/放置时起始状态碰撞、规划失败。留 2mm 间隙（视觉上不可见，
# 等价于真实工件与台面间的倒角/间隙）
def slot_xyz(slot):
    """料盘槽位 slot（0 起）处工件中心的世界坐标。"""
    return (0.45, 0.17 + 0.08 * slot, 0.062)


def machine_xyz():
    """加工台上工件放置位置（世界坐标）。"""
    return (0.10, -0.25, 0.122)


def workpiece_name(slot):
    return 'workpiece_%d' % slot


# ---------- 状态名称（日志与显示用） ----------
TASK_STATE_NAMES = {
    0: 'IDLE', 1: 'PICK', 2: 'PLACE', 3: 'WAIT_MACHINING', 4: 'RETRIEVE',
    5: 'RETURN', 6: 'DONE', 7: 'WAIT_STATION', 8: 'TIMEOUT', 9: 'ABORTED',
}

MACHINE_STATE_NAMES = {0: 'READY', 1: 'BUSY', 2: 'MACHINING', 3: 'DONE'}

# 末端执行器与接触连杆
EE_LINK = 'panda_hand'
TOUCH_LINKS = ['panda_hand', 'panda_leftfinger', 'panda_rightfinger']

# 机械臂规划组
ARM_GROUP = 'panda_arm'
HAND_GROUP = 'hand'

# 机械臂 "ready" 姿态（panda SRDF 中的命名姿态）
HOME_POSE_NAME = 'ready'
