"""真实图像渲染/识别与控制器接入检查；不模拟 MoveIt 轨迹或 DDS。"""
import unittest
from unittest.mock import Mock
import cv2
import numpy as np
from ros_doubles import Node, ns, install
install()
from machining_demo import layout
from machining_demo.vision_geometry import (initial_centers, validate_centers,
                                           project, unproject, WIDTH, HEIGHT)
from machining_demo.vision_image import detect, render
from machining_demo.arm_controller import ArmController
from test_arm_linear import completed
from machining_demo.vision_locator import VisionLocator
from machining_demo.virtual_camera import VirtualCamera


OFFSETS = '0.012,0.006,-0.009,-0.007,0.006,0.005'


def image_for(centers):
    objects = [(item['name'], item['pos'], item['size']) for item in (layout.TABLE, layout.TRAY)]
    return render(objects + [(layout.workpiece_name(i), xyz, (0.04,) * 3)
                             for i, xyz in enumerate(centers)])


class ImageTests(unittest.TestCase):
    def test_displaced_objects_are_located_from_pixels_not_nominal_slots(self):
        actual = initial_centers(OFFSETS)
        measured, annotated = detect(image_for(actual))
        for i, (a, b) in enumerate(zip(actual, measured)):
            np.testing.assert_allclose(a, b, atol=0.001)
            self.assertGreater(np.linalg.norm(np.array(b) - layout.slot_xyz(i)), 0.005)
        self.assertEqual(annotated.shape, (HEIGHT, WIDTH, 3))

    def test_many_safe_offsets_keep_submillimeter_pixel_localization(self):
        rng = np.random.default_rng(42)
        for _ in range(40):
            offsets = ','.join(str(v) for v in rng.uniform(-0.008, 0.008, 6))
            actual = initial_centers(offsets)
            measured, _ = detect(image_for(actual))
            np.testing.assert_allclose(actual, measured, atol=0.001)

    def test_missing_occluded_or_duplicate_target_is_not_accepted(self):
        original = image_for(initial_centers(OFFSETS))
        for altered in (np.zeros_like(original), original.copy(), original.copy()):
            if altered.any():
                x, y = project(*initial_centers(OFFSETS)[0][:2], 0.082)
                cv2.rectangle(altered, (int(x)-10, int(y)-30), (int(x)+30, int(y)+30), (0,0,0), -1)
            with self.assertRaises(ValueError):
                detect(altered)
        duplicate = original.copy()
        cv2.rectangle(duplicate, (450, 450), (489, 489), (20,115,255), -1)
        with self.assertRaises(ValueError):
            detect(duplicate)

    def test_too_close_or_outside_tray_is_rejected(self):
        for offsets in ('0,0.03,0,-0.03,0,0', '0.5,0,0,0,0,0', 'nan,0,0,0,0,0'):
            with self.assertRaises(ValueError):
                initial_centers(offsets)

    def test_projection_round_trip_and_wrong_image_geometry(self):
        for x, y, z in initial_centers(OFFSETS):
            measured = unproject(*project(x, y, z + 0.02))
            np.testing.assert_allclose((x,y,z), measured, atol=1e-12)
        for image in (np.zeros((50,50,3),dtype=np.uint8), np.zeros((800,800,3))):
            with self.assertRaises(ValueError):
                detect(image)


class ControllerVisionTests(unittest.TestCase):
    def setUp(self):
        Node.overrides = {}
        self.node = ArmController()
        self.centers = initial_centers(OFFSETS)
        self.node._apply_scene = Mock(return_value=True)
        self.node._gripper = Mock(return_value=True)
        self.node._goto_pose = Mock(return_value=True)
        self.node._goto_linear = Mock(return_value=True)
        self.node._locate_workpieces = Mock(return_value=self.centers)

    def test_pick_and_return_follow_observed_position_and_update_all_obstacles(self):
        self.assertTrue(self.node._pick(1))
        approach = self.node._goto_pose.call_args.args[0].position
        self.assertAlmostEqual(approach.x, self.centers[1][0])
        self.assertAlmostEqual(approach.y, self.centers[1][1])
        scene = self.node._apply_scene.call_args_list[0].args[0]
        self.assertEqual(len(scene.world.collision_objects), 3)
        for obj, xyz in zip(scene.world.collision_objects,self.centers):
            self.assertEqual(obj.primitive_poses[0].position.y, xyz[1])
        self.assertTrue(self.node._return(1))
        detached = self.node._apply_scene.call_args.args[0].world.collision_objects[0]
        self.assertAlmostEqual(detached.primitive_poses[0].position.y, self.centers[1][1])

    def test_vision_failure_never_falls_back_to_nominal_pick(self):
        self.node._locate_workpieces.return_value = None
        self.assertFalse(self.node._pick(0))
        self.node._goto_pose.assert_not_called()
        self.node._gripper.assert_not_called()

    def test_motion_during_approach_stops_before_descent_or_attachment(self):
        changed = list(self.centers)
        changed[0] = (changed[0][0] + 0.01, *changed[0][1:])
        self.node._locate_workpieces.side_effect = [self.centers, changed]
        self.assertFalse(self.node._pick(0))
        self.node._goto_linear.assert_not_called()
        self.assertIsNone(self.node._attached_name)

    def test_stale_wrong_frame_partial_and_failed_service_responses_are_rejected(self):
        del self.node._locate_workpieces
        def response(stamp=100, frame='panda_link0'):
            return ns(success=True, centers=[ns(header=ns(frame_id=frame,stamp=ns(sec=stamp,nanosec=0)),
                pose=ns(position=ns(x=x,y=y,z=z))) for x,y,z in self.centers])
        for result in (response(95), response(101), response(frame='camera'),
                       ns(success=True,centers=[]), ns(success=False,message='not found')):
            self.node._vision_client.call_async = Mock(return_value=completed(result))
            self.assertIsNone(self.node._locate_workpieces())
        self.node._vision_client.call_async = Mock(return_value=completed(response()))
        self.assertEqual(self.node._locate_workpieces(), self.centers)
        self.assertIsNone(self.node._locate_workpieces(newer_than=100000000000))

    def test_fast_repeated_pick_waits_for_a_new_camera_frame(self):
        del self.node._locate_workpieces
        self.node.clock.advance(0.3)
        def observation(nanoseconds):
            return ns(success=True,centers=[ns(header=ns(frame_id='panda_link0',
                stamp=ns(sec=100,nanosec=nanoseconds)), pose=ns(position=ns(x=x,y=y,z=z)))
                for x,y,z in self.centers])
        self.node._vision_client.call_async = Mock(side_effect=[
            completed(observation(0)), completed(observation(200000000))])
        self.assertEqual(self.node._locate_workpieces(newer_than=100000000000),self.centers)
        self.assertEqual(self.node._vision_stamp,100200000000)


class SensorNodeTests(unittest.TestCase):
    def setUp(self):
        Node.overrides = {}
        self.node = VisionLocator()
        self.msg = ns(header=ns(frame_id='overhead_camera_optical_frame',
                               stamp=ns(sec=100,nanosec=0)),
                      image=image_for(initial_centers(OFFSETS)))

    def test_image_node_outputs_pixel_estimates_with_original_capture_stamp(self):
        self.node.image_cb(self.msg)
        result = self.node.locate(ns(), ns(centers=[]))
        self.assertTrue(result.success)
        self.assertEqual(result.centers[0].header.stamp.sec, 100)
        self.assertEqual(result.centers[0].header.frame_id,'panda_link0')
        self.assertEqual(set(self.node.clients), set())  # 识别节点没有场景/真值客户端。
        self.assertEqual(set(self.node.subscriptions), {'/camera/image_raw'})

    def test_no_image_old_capture_old_delivery_and_wrong_frame_fail_closed(self):
        self.assertFalse(self.node.locate(ns(),ns(centers=[])).success)
        for frame, seconds in (('camera',100), ('overhead_camera_optical_frame',90),
                               ('overhead_camera_optical_frame',101)):
            self.msg.header.frame_id = frame
            self.msg.header.stamp.sec = seconds
            self.node.image_cb(self.msg)
            self.assertFalse(self.node.locate(ns(),ns(centers=[])).success)
        self.node.image_cb(self.msg)
        self.node.latest = (self.msg, 0.0)
        self.assertFalse(self.node.locate(ns(),ns(centers=[])).success)

    def test_missing_object_clears_previous_boxes_in_live_display(self):
        self.node.image_cb(self.msg)
        self.msg.image = np.zeros_like(self.msg.image)
        self.node.image_cb(self.msg)
        self.assertFalse(self.node.locate(ns(),ns(centers=[])).success)
        np.testing.assert_array_equal(self.node.pub.messages[-1].image, self.msg.image)

    def test_camera_uses_query_time_not_delivery_time_and_only_world_objects(self):
        camera = VirtualCamera()
        camera.tick()
        future = camera.future
        objects = []
        from ros_doubles import CollisionObject, Pose
        for i, xyz in enumerate(initial_centers(OFFSETS)):
            obj = CollisionObject()
            obj.id, obj.header.frame_id = layout.workpiece_name(i), 'panda_link0'
            obj.pose.orientation.w = 1.0
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xyz
            pose.orientation.w = 1.0
            obj.primitive_poses = [pose]
            obj.primitives = [ns(dimensions=[0.04]*3)]
            objects.append(obj)
        future.complete = True
        future.value = ns(scene=ns(world=ns(collision_objects=objects)))
        camera.clock.advance(2)
        camera.tick()
        self.assertEqual(camera.pub.messages[-1].header.stamp.sec,100)
        self.assertEqual(camera.info.messages[-1].k[0],900.0)
        self.node.clock.advance(2)
        self.node.image_cb(camera.pub.messages[-1])
        self.assertFalse(self.node.locate(ns(),ns(centers=[])).success)


if __name__ == '__main__':
    unittest.main()
