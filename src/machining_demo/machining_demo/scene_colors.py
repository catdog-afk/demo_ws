"""MoveIt 场景物体的独立颜色，在工件附着/分离时重复发送。"""
from moveit_msgs.msg import ObjectColor
from machining_demo import layout


def object_color(name):
    color = ObjectColor()
    color.id = name
    color.color.r, color.color.g, color.color.b, color.color.a = layout.OBJECT_COLORS[name]
    return color
