#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import time
import rclpy
import queue
import signal
import threading
import numpy as np
import sdk.pid as pid
import sdk.fps as fps
import sdk.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from kinematics import transform
from dt_apriltags import Detector
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import ColorsInfo, ColorDetect
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetString
from kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position

class KcfTrackNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.z_dis = 0.28
        self.y_dis = 500
        self.x_init = 0.18
        self.bridge = CvBridge()  # Used for the conversion between ROS Image messages and OpenCV images.
        self.center = None
        self.tracker = None
        self.enable_select = False
        self.running = True
        self.start = False
        self.name = name
        self.image_queue = queue.Queue(maxsize=2)
        self.pid_z = pid.PID(0.00006, 0.0, 0.0)
        self.pid_y = pid.PID(0.05, 0.0, 0.0)
        self.fps = fps.FPS() 

        self.params = cv2.TrackerNano_Params()
        self.params.backbone = os.path.join('/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/resources/models/nanotrack_backbone_sim.onnx')
        self.params.neckhead = os.path.join('/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/resources/models/nanotrack_head_sim.onnx')
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)
        
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.init_action()
        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
            time.sleep(1.8)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start kcf track")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop kcf track")
        self.start = False
        response.success = True
        response.message = "stop"
        return response
    
    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)

    def main(self):
        while self.running:
            rgb_image = self.image_queue.get()
            result_image = np.copy(rgb_image)
            factor = 4
            rgb_image = cv2.resize(rgb_image, (int(result_image.shape[1]/ factor), int(result_image.shape[0]/ factor)))

            if self.tracker is None:
                if self.enable_select:
                    roi = cv2.selectROI("result", result_image, False)
                    if roi != (0, 0, 0, 0):  
                        self.tracker = cv2.TrackerNano_create(self.params)
                        
                        roi_scaled = tuple(int(i / factor) for i in roi)
                        
                        self.tracker.init(rgb_image, roi_scaled)
            else:
                status, box = self.tracker.update(rgb_image)
                if status:
                    p1 = int(box[0] * factor), int(box[1] * factor)
                    p2 = p1[0] + int(box[2] * factor), p1[1] + int(box[3] * factor)
                    cv2.rectangle(result_image, p1, p2, (255, 255, 0), 2)
                    center_x, center_y = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
           
                    t1 = time.time()
                    self.pid_y.SetPoint = result_image.shape[1] / 2 
                    self.pid_y.update(center_x)
                    self.y_dis += self.pid_y.output
                    self.y_dis = max(200, min(self.y_dis, 800))

                    self.pid_z.SetPoint = result_image.shape[0] / 2 
                    self.pid_z.update(center_y)
                    self.z_dis += self.pid_z.output
                    self.z_dis = max(0.23, min(self.z_dis, 0.30))
                    
                    msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.kinematics_client, msg)
                    
                    t2 = time.time()
                    t = t2 - t1
                    if t < 0.02:
                        time.sleep(0.02 - t)
                        
                    if res.pulse:
                        servo_data = res.pulse
                        set_servo_position(self.joints_pub, 0.02, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, int(self.y_dis))))
                    else:
                        set_servo_position(self.joints_pub, 0.02, ((1, int(self.y_dis)), ))
                else:
                    time.sleep(0.01)
            
            self.fps.update()
            self.fps.show_fps(result_image)
            cv2.imshow("result", result_image)

            key = cv2.waitKey(1)
            if key == ord('s'):  
                self.get_logger().info("在画面窗口按下s开始选择追踪目标,然后按下空格开始追踪")
                self.tracker = None
                self.enable_select = True
            elif key == 27: 
                self.running = False

        self.init_action()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    node = KcfTrackNode('kcf_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
