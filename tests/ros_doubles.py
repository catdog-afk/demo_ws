"""Minimal ROS transport doubles for testing the real node callbacks on Windows/CI.

These doubles do not simulate MoveIt motion or ROS middleware.
"""
import copy
import sys
import types
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'machining_demo'))


def ns(**values):
    return types.SimpleNamespace(**values)


class Future:
    def __init__(self):
        self.complete = False
        self.error = None
        self.value = None
        self.cancelled = False

    def done(self):
        return self.complete

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def resolve(self, success=True, message='ok'):
        self.complete = True
        self.value = ns(success=success, message=message)

    def cancel(self):
        self.cancelled = self.complete = True


class Client:
    def __init__(self):
        self.ready = True
        self.calls = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def wait_for_service(self, timeout_sec):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        future = Future()
        self.futures.append(future)
        return future


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class Instant:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1e9)

    def to_msg(self):
        return ns(sec=self.nanoseconds // 1000000000, nanosec=self.nanoseconds % 1000000000)

    def __sub__(self, other):
        return ns(nanoseconds=self.nanoseconds - other.nanoseconds)


class Clock:
    def __init__(self):
        self.seconds = 100.0

    def now(self):
        return Instant(self.seconds)

    def advance(self, seconds):
        self.seconds += seconds


class Node:
    overrides = {}

    def __init__(self, name):
        self.params = dict(self.overrides)
        self.clock = Clock()
        self.logger = Mock()
        self.publishers = {}
        self.subscriptions = {}
        self.clients = {}
        self.services = {}

    def declare_parameter(self, name, default):
        self.params.setdefault(name, default)

    def get_parameter(self, name):
        return ns(value=self.params[name])

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def create_publisher(self, cls, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, cls, topic, callback, qos, **kwargs):
        self.subscriptions[topic] = callback
        return callback

    def create_client(self, cls, topic, **kwargs):
        self.clients[topic] = Client()
        return self.clients[topic]

    def create_service(self, cls, topic, callback):
        self.services[topic] = callback
        return callback

    def create_timer(self, interval, callback):
        return ns(interval=interval, callback=callback)

    def destroy_timer(self, timer):
        pass

    def destroy_node(self):
        return True


class Pose:
    def __init__(self):
        self.position = ns(x=0.0, y=0.0, z=0.0)
        self.orientation = ns(x=0.0, y=0.0, z=0.0, w=0.0)


class Marker:
    TEXT_VIEW_FACING, ADD = 9, 0

    def __init__(self):
        self.header = ns(frame_id='', stamp=None)
        self.pose = Pose()
        self.scale = ns(x=0.0, y=0.0, z=0.0)
        self.color = ns(r=0.0, g=0.0, b=0.0, a=0.0)


class TaskState:
    IDLE, PICK, PLACE, WAIT_MACHINING, RETRIEVE, RETURN = range(6)
    DONE, WAIT_STATION, TIMEOUT, ABORTED = range(6, 10)

    def __init__(self, state=0, slot=0, detail=''):
        self.state, self.slot, self.detail = state, slot, detail
        self.stamp = None


class MachineStatus:
    READY, BUSY, MACHINING, DONE = range(4)

    def __init__(self, state=0, progress=0.0, detail=''):
        self.state, self.progress, self.detail = state, progress, detail


class TrayOccupancy:
    def __init__(self, occupied=None):
        self.occupied = [True] * 3 if occupied is None else occupied
        self.num_slots = len(self.occupied)


class Request:
    def __init__(self):
        self.scene = None
        self.command = ''
        self.slot = 0
        self.occupied = False


class Service:
    Request = Request


class CollisionObject:
    ADD, REMOVE = 0, 1

    def __init__(self):
        self.header = ns(frame_id='')
        self.pose = Pose()
        self.primitives, self.primitive_poses = [], []
        self.id = ''


class PlanningScene:
    def __init__(self):
        self.world = ns(collision_objects=[])
        self.robot_state = ns(is_diff=False, attached_collision_objects=[])
        self.object_colors = []


class ObjectColor:
    def __init__(self):
        self.id = ''
        self.color = ns(r=0.0, g=0.0, b=0.0, a=0.0)


class Primitive:
    BOX, SPHERE = 1, 2


class Data:
    def __init__(self, data=None, **kwargs):
        self.data = data
        self.__dict__.update(kwargs)


def install():
    definitions = {
        'rclpy': dict(ok=lambda: True, spin_until_future_complete=Mock(), spin_once=Mock()),
        'rclpy.node': dict(Node=Node),
        'rclpy.qos': dict(QoSProfile=lambda **kwargs: ns(**kwargs),
                          DurabilityPolicy=ns(TRANSIENT_LOCAL=1)),
        'rclpy.action': dict(ActionClient=Mock),
        'rclpy.callback_groups': dict(MutuallyExclusiveCallbackGroup=Mock),
        'rclpy.executors': dict(MultiThreadedExecutor=Mock),
        'visualization_msgs.msg': dict(Marker=Marker),
        'std_msgs.msg': dict(Bool=Data, String=Data),
        'sensor_msgs.msg': dict(JointState=Data),
        'geometry_msgs.msg': dict(Pose=Pose, TransformStamped=Data),
        'shape_msgs.msg': dict(SolidPrimitive=Primitive),
        'moveit_msgs.msg': dict(PlanningScene=PlanningScene, CollisionObject=CollisionObject,
                               AttachedCollisionObject=Data, ObjectColor=ObjectColor),
        'moveit_msgs.action': dict(MoveGroup=Data),
        'moveit_msgs.srv': dict(ApplyPlanningScene=Service),
        'demo_interfaces.msg': dict(TaskState=TaskState, MachineStatus=MachineStatus,
                                   TrayOccupancy=TrayOccupancy),
        'demo_interfaces.srv': dict(StartTask=Service, CancelTask=Service, ArmCommand=Service,
                                   SetOccupancy=Service, ResetMachine=Service),
        'tf2_ros': dict(StaticTransformBroadcaster=Mock),
    }
    for name in ('MotionPlanRequest', 'PlanningOptions', 'Constraints', 'PositionConstraint',
                 'OrientationConstraint', 'JointConstraint', 'BoundingVolume', 'RobotState'):
        definitions['moveit_msgs.msg'][name] = Data
    for name, values in definitions.items():
        for index in range(1, len(name.split('.')) + 1):
            key = '.'.join(name.split('.')[:index])
            if key not in sys.modules:
                sys.modules[key] = types.ModuleType(key)
        sys.modules[name].__dict__.update(values)
    for name in definitions:
        if '.' in name:
            parent, child = name.rsplit('.', 1)
            setattr(sys.modules[parent], child, sys.modules[name])
