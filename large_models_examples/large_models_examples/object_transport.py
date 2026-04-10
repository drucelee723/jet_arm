#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import cv2
import math
import yaml
import copy
import time
import torch
import queue
import rclpy
import threading
import numpy as np
import sdk.fps as fps
import message_filters
from sdk import common
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger, SetBool, Empty
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener, TransformException

from speech import speech
from app.common import Heart
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel, SetBox
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from kinematics.kinematics_control import set_pose_target, set_joint_value_target
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from app.utils import utils, image_process, calculate_grasp_yaw_by_depth, pick_and_place

device = 'cuda' if torch.cuda.is_available() else 'cpu'

class ObjectTransport(Node):
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],  # x
        [-1.0, 0.0, 0.0, 0.01],  # y
        [0.0, -1.0, 0.0, 0.05],  # z
        [0.0, 0.0, 0.0, 1.0]
    ]
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
        self.image_queue = queue.Queue(maxsize=2)
        self._init_parameters()
        
        self.set_above = False
        self.record_position = []
        self.lock = threading.RLock()
        self.base_gripper_height = utils.get_gripper_size(500)[1]
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.transport_finished_pub = self.create_publisher(Bool, '~/transport_finished', 1)
        timer_cb_group = ReentrantCallbackGroup()
        self.set_joint_value_target_client = self.create_client(SetJointValue, 'kinematics/set_joint_value_target', callback_group=timer_cb_group)
        self.set_joint_value_target_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_sorting_srv = self.create_service(SetBool, '~/enable_transport', self.enable_transport_srv_callback)
        self.set_pick_position_srv = self.create_service(SetBox, '~/set_pick_position', self.set_pick_position_srv_callback)
        self.set_place_position_srv = self.create_service(SetBox, '~/set_place_position', self.set_place_position_srv_callback)
        self.record_position_srv = self.create_service(SetBox, '~/record_position', self.record_position_srv_callback)
        

        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        self.data = common.get_yaml_data("/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml")  
        self.lab_data = self.data['/**']['ros__parameters']
        tf_buffer = Buffer()
        self.tf_listener = TransformListener(tf_buffer, self)
        tf_future = tf_buffer.wait_for_transform_async(
            target_frame='depth_cam_depth_optical_frame',
            source_frame='depth_cam_color_frame',
            time=rclpy.time.Time()
        )

        rclpy.spin_until_future_complete(self, tf_future)
        try:
            transform = tf_buffer.lookup_transform(
                'depth_cam_color_frame', 'depth_cam_depth_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0) )
            self.static_transform = transform  # 保存变换数据
            self.get_logger().info(f'Static transform: {self.static_transform}')
        except TransformException as e:
            self.get_logger().error(f'Failed to get static transform: {e}')

        # 提取平移和旋转
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        self.transform_matrix = common.xyz_quat_to_mat([translation.x, translation.y, translation.z], [rotation.w, rotation.x, rotation.y, rotation.z])
        self.hand2cam_tf_matrix = np.matmul(self.transform_matrix, self.hand2cam_tf_matrix)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def _init_parameters(self):
        self.heart = None
        self.enter = False
        self.start_transport = False
        self.enable_transport = False
        self.sync = None
        self.start_get_roi = False
        self.rgb_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.depth_info_sub = None
        self.white_area_center = None
        self.roi = None
        self.plane = None
        self.extristric = None
        self.corners = None
        self.intrinsic = None
        self.distortion = None
        self.action_list = []
        self.target = []
        self.start_stamp = time.time()

    def init_process(self):
        self.timer.cancel()

        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def go_home(self, interrupt=True, back=True):
        if interrupt:
            set_servo_position(self.joints_pub, 0.5, ((10, 200), ))
            time.sleep(0.5)
        
        joint_angle = [500, 520, 210, 50, 500]
        
        msg = set_joint_value_target(joint_angle)
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t, pose_r = endpoint.pose.position, endpoint.pose.orientation
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
        set_servo_position(self.joints_pub, 1, ((2, joint_angle[1]), (3, joint_angle[2]), (4, joint_angle[3]), (5, 500)))
        time.sleep(1)
        
        if back:
            set_servo_position(self.joints_pub, 1, ((1, joint_angle[0]), ))
            time.sleep(1.5)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter object transport")
        with self.lock:
            self._init_parameters()
            self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # Heartbeat package 心跳包
            if self.sync is None:
                self.rgb_sub = message_filters.Subscriber(self, Image, 'depth_cam/rgb/image_raw')
                self.depth_sub = message_filters.Subscriber(self, Image, 'depth_cam/depth/image_raw')
                self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/depth/camera_info')
                self.info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/rgb/camera_info')
                # Synchronize the timestamp. The time is allowed to have an error of 0.03 seconds. 同步时间戳, 时间允许有误差在0.03s
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    [self.rgb_sub, self.depth_sub, self.info_sub, self.depth_info_sub], 3, 0.2)
                self.sync.registerCallback(self.multi_callback)

            self.enter = True
            self.start_get_roi = True
        
        self.go_home()
        response.success = True
        response.message = "start"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "exit  object transport")
        with self.lock:
            self._init_parameters()
            if self.sync is not None:
                self.sync.disconnect(self.multi_callback)
                self.sync = None
            pick_and_place.interrupt()
        response.success = True
        response.message = "start"
        return response

    def enable_transport_srv_callback(self, request, response):
        with self.lock:
            if request.data:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'enable  object transport')
                self.enable_transport = True
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'exit  object transport')
                pick_and_place.interrupt()
                self.enable_transport = False
        response.success = True
        response.message = "start"
        return response

    def set_pick_position_srv_callback(self, request, response):
        with self.lock:
            self.action_list.append(['pick', request.box])
        response.success = True
        response.message = "start"
        return response

    def record_position_srv_callback(self, request, response):
        with self.lock:
            self.action_list.append(['record', request.label, request.box])
        response.success = True
        response.message = "start"
        return response

    def set_place_position_srv_callback(self, request, response):
        with self.lock:
            if request.label and not request.box:
                self.action_list.append(['restore', request.label])
            else:
                self.action_list.append(['place', request.offset, request.box])
        response.success = True
        response.message = "start"
        return response

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            corners = np.array(config['corners']).reshape(-1, 3)
            with self.lock:
                self.extristric = np.array(config['extristric'])
                self.white_area_center = np.array(config['white_area_pose_world'])
                self.plane = config['plane']
                self.corners = np.array(config['corners'])

        while self.intrinsic is None or self.distortion is None:
            self.get_logger().info("waiting for camera info")
            time.sleep(0.1)

        with self.lock:
            tvec = self.extristric[:1]
            rmat = self.extristric[1:]

            tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
            imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), self.intrinsic, self.distortion)
            imgpts = np.int32(imgpts).reshape(-1, 2)

            x_min = min(imgpts, key=lambda p: p[0])[0]
            x_max = max(imgpts, key=lambda p: p[0])[0]
            y_min = min(imgpts, key=lambda p: p[1])[1]
            y_max = max(imgpts, key=lambda p: p[1])[1]
            roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
            self.roi = roi

    def cal_grap_point(self, mask, x, y, box, edge_index):
        edge_vector = box[(edge_index + 1) % 4] - box[edge_index]
        edge_vector = edge_vector / np.linalg.norm(edge_vector)

        perpendicular_vector = np.array([-edge_vector[1], edge_vector[0]])

        line_length = 1000
        line_start = (x - int(perpendicular_vector[0] * line_length), y - int(perpendicular_vector[1] * line_length))
        line_end = (x + int(perpendicular_vector[0] * line_length), y + int(perpendicular_vector[1] * line_length))

        line_image = np.zeros_like(mask)
        cv2.line(line_image, line_start, line_end, 255, 1, cv2.LINE_AA)

        intersection_image = cv2.bitwise_and(mask, line_image)
        intersection_contours, _ = cv2.findContours(intersection_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areaMaxContour, area_max = common.get_area_max_contour(intersection_contours)
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            center = [rect[0][0], rect[0][1]]
            object_width = max(rect[1])
            return [center, object_width]
        else:
            return False

    def get_object_pixel_position(self, image, roi):
        """
        :param image: 完整的BGR图像
        :param roi: 目标物体的边界框，格式为 [y_min, y_max, x_min, x_max]
        :return: 抓取点信息或False
        """
        y_min, y_max, x_min, x_max = roi
        center_y = (y_min + y_max) / 2
        center_x = (x_min + x_max) / 2

        new_height = (y_max - y_min) * 1.5
        new_width = (x_max - x_min) * 1.5

        y_min = center_y - new_height/2
        y_max = center_y + new_height/2
        x_min = center_x - new_width/2 
        x_max = center_x + new_width/2


        # 确保ROI坐标是整数且在图像边界内
        y_min, y_max = int(max(0, y_min)), int(min(image.shape[0], y_max))
        x_min, x_max = int(max(0, x_min)), int(min(image.shape[1], x_max))
        
        # 1. 从主图像中裁剪ROI
        roi_image = image[y_min:y_max, x_min:x_max]

        if roi_image.size == 0:
            self.get_logger().warn("ROI is empty or out of bounds.")
            return False

        # 2. 处理ROI以创建二进制掩码
        gray_roi = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        # 使用Otsu方法进行自动阈值处理。THRESH_BINARY_INV适用于深色物体在浅色背景上。
        _, mask_roi = cv2.threshold(blurred_roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 3. 在ROI中找到最大的轮廓，假定它就是目标物体
        contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areaMaxContour, _ = common.get_area_max_contour(contours)

        if areaMaxContour is not None:
            # 4. 计算质心（在ROI坐标系中）
            M = cv2.moments(areaMaxContour)
            if M["m00"] == 0:
                return False
            cx_roi = int(M["m10"] / M["m00"])
            cy_roi = int(M["m01"] / M["m00"])

            # 将质心坐标转换回完整的图像坐标系
            cx = cx_roi + x_min
            cy = cy_roi + y_min
            
            # 创建一个与原图同样大小的完整掩码，以供cal_grap_point函数使用
            full_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            full_mask[y_min:y_max, x_min:x_max] = mask_roi

            # 5. 获取最小外接矩形（在ROI坐标系中）
            rect = cv2.minAreaRect(areaMaxContour)
            _, _, angle = rect

            # 将矩形的角点转换到完整的图像坐标系以进行绘制
            box_roi = cv2.boxPoints(rect)
            bbox_full_image = np.intp(box_roi + (x_min, y_min)) # 坐标平移
            
            cv2.drawContours(image, [bbox_full_image], -1, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)

            # 6. 使用与之前相同的逻辑计算抓取点
            edge_lengths = [np.linalg.norm(bbox_full_image[i] - bbox_full_image[(i + 1) % 4]) for i in range(4)]
            long_edge_index = np.argmax(edge_lengths)
            short_edge_index = np.argmin(edge_lengths)
            
            # 传递完整掩码和在完整图像系统中的坐标
            grasp_point1 = self.cal_grap_point(full_mask, cx, cy, bbox_full_image, long_edge_index)
            grasp_point2 = self.cal_grap_point(full_mask, cx, cy, bbox_full_image, short_edge_index)

            if not grasp_point1 and not grasp_point2:
                return False

            if grasp_point1:
                grasp_point1.append(angle)
                cv2.circle(image, (int(grasp_point1[0][0]), int(grasp_point1[0][1])), 5, (0, 0, 255), -1)
            if grasp_point2: 
                grasp_point2.append(angle - 90)
                cv2.circle(image, (int(grasp_point2[0][0]), int(grasp_point2[0][1])), 5, (255, 0, 0), -1)
            
            # 为了调试，绘制原始的ROI框
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2, 1)

            self.get_logger().info('\033[1;32m%s\033[0m' % "exit hand gesture")
            return [grasp_point1, grasp_point2]
            
        return False

    def get_grap_angle(self, bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix):
        image_height, image_width = bgr_image.shape[:2]
        x, y, object_width = object_info[0][0], object_info[0][1], object_info[1]
        w = 50
        if x + w > image_width:
            w = image_width - x
        elif x - w < 0:
            w = x
        h = 50
        if y + h > image_height:
            h = image_height - y
        elif y - h < 0:
            h = y

        angle = 0
        box = np.intp(cv2.boxPoints(((x + 25, y - 40), (w, h), angle)))
        mask = np.zeros(depth_image.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [box], 255)
        masked_depth = cv2.bitwise_and(depth_image, depth_image, mask=mask)
        min_dist = utils.find_depth_range(masked_depth, max_dist)
        fx, fy = depth_intrinsic_matrix[0], depth_intrinsic_matrix[4]

        object_width = object_width / (fx / (min_dist / 1000.0))

        gripper_angle = utils.set_gripper_size(object_width)

        return gripper_angle, min_dist

    def get_position(self, object_info, min_dist, angle, gripper_angle, depth_intrinsic_matrix):
        gripper_info = utils.get_gripper_size(gripper_angle)
        d = gripper_info[1] - self.base_gripper_height
    
        x, y = object_info[0][0], object_info[0][1]
        cx, cy = x + 25, y - 40
        position = utils.calculate_world_position(cx, cy, min_dist, self.plane, self.endpoint, self.hand2cam_tf_matrix, depth_intrinsic_matrix)
        position[-1] += d

        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
        offset = tuple(config_data['depth']['offset'])
        scale = tuple(config_data['depth']['scale'])
        for i in range(3):
            position[i] = position[i] * scale[i]
            position[i] = position[i] + offset[i]

        yaw = math.degrees(math.atan2(position[1], position[0]))

        if position[0] < 0:
            if position[1] < 0:
                yaw += 180
            else:
                yaw -= 180

        yaw += angle
        yaw = 500 + int(yaw / 240 * 1000)
        
        object_info = [position, [x, y], yaw]
        return object_info 

    def get_object_world_position(self, bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values):
        object_info_ = []
        if object_info[0]:
            gripper_angle, min_dist = self.get_grap_angle(bgr_image, depth_image, object_info[0], max_dist, depth_intrinsic_matrix)
            gripper_angle = 540
            object_info_ = self.get_position(object_info[0], min_dist, object_info[0][-1], gripper_angle, depth_intrinsic_matrix)
        
        if object_info[1]:
            gripper_angle, min_dist = self.get_grap_angle(bgr_image, depth_image, object_info[1], max_dist, depth_intrinsic_matrix)
            gripper_angle = 540
            info = self.get_position(object_info[1], min_dist, object_info[1][-1], gripper_angle, depth_intrinsic_matrix)
            if object_info_:
                if abs(object_info_[-1] - 500) > abs(info[-1] - 500):
                    object_info_ = info
            else:
                object_info_ = info
        if object_info_:
            object_info_.append(gripper_angle)
        return object_info_

    def main(self):
        while True:
            if self.enter:
                try:
                    bgr_image, depth_image, camera_info, depth_camera_info = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    continue
                with self.lock:
                    self.intrinsic = np.matrix(camera_info.k).reshape(1, -1, 3)
                    self.distortion = np.array(camera_info.d)
                    if self.start_get_roi:
                        self.get_roi()
                        self.start_get_roi = False
                max_dist = 350
                depth_image = utils.create_roi_mask(depth_image, bgr_image, self.corners, camera_info, self.extristric, max_dist, 0.08)
                sim_depth_image = (1 - np.clip(depth_image, 0, max_dist).astype(np.float64) / max_dist) * 255
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                if self.enable_transport:
                    with self.lock:
                        if self.action_list:
                            box_info = self.action_list[0]
                            box_data = box_info[-1]
                            # 将box格式从[x1, y1, x2, y2]转换为函数期望的[y1, y2, x1, x2]
                            roi_for_function = [box_data[1], box_data[3], box_data[0], box_data[2]]
                            
                            if box_info[0] == 'pick':
                                if not self.target:
                                    depth_intrinsic_matrix = depth_camera_info.k
                                    plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
                                    object_info = self.get_object_pixel_position(bgr_image, roi_for_function)
                                    if object_info:
                                        self.target = self.get_object_world_position(bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values)
                                        self.target.extend([box_info[0]])
                                        self.start_stamp = time.time()
                                else:
                                    if time.time() - self.start_stamp > 2:
                                        self.start_transport = True        
                                        self.enable_transport = False
                                    cv2.circle(bgr_image, (int(self.target[1][0]), int(self.target[1][1])), 10, (255, 0, 0), -1)
                                cv2.rectangle(bgr_image, (box_data[0], box_data[1]), (box_data[2], box_data[3]), (0, 255, 0), 2, 1)
                            elif box_info[0] == 'place':
                                if not self.target:
                                    depth_intrinsic_matrix = depth_camera_info.k
                                    plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
                                    object_info = self.get_object_pixel_position(bgr_image, roi_for_function)
                                    if object_info:
                                        self.target = self.get_object_world_position(bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values)
                                        self.target.extend([box_info[0]])
                                        self.target[0][0] += box_info[-2][0]
                                        self.target[0][1] += box_info[-2][1]
                                        if box_info[-2][0] != 0 or box_info[-2][1] != 0:
                                            self.target[0][2] = 0.015
                                            yaw_init = 500
                                            yaw_list = [] 
                                            for i in object_info:
                                                angle = i[-1]
                                                position = self.target[0]
                                                yaw = math.degrees(math.atan2(position[1], position[0]))

                                                if position[0] < 0:
                                                    if position[1] < 0:
                                                        yaw += 180
                                                    else:
                                                        yaw -= 180
                                                yaw += angle
                                                yaw = 500 + int(yaw / 240 * 1000)
                                                yaw_list.append(yaw)
                                            if len(yaw_list) == 1:
                                                self.target[2] = yaw_list[0]
                                            else:
                                                if abs(yaw_list[0] - yaw_init) > abs(yaw_list[1] - yaw_init):
                                                    self.target[2] = yaw_list[1]
                                        else:
                                            self.target[0][2] += 0.015
                                        self.start_stamp = time.time()
                                else:
                                    if time.time() - self.start_stamp > 2:
                                        self.start_transport = True        
                                        self.enable_transport = False
                                    cv2.circle(bgr_image, (int(self.target[1][0]), int(self.target[1][1])), 10, (255, 0, 0), -1)
                                cv2.rectangle(bgr_image, (box_data[0], box_data[1]), (box_data[2], box_data[3]), (0, 255, 0), 2, 1)
                            elif box_info[0] == 'record':
                                depth_intrinsic_matrix = depth_camera_info.k
                                plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
                                object_info = self.get_object_pixel_position(bgr_image, roi_for_function)
                                if object_info:
                                    self.target = self.get_object_world_position(bgr_image, depth_image, object_info, max_dist, depth_intrinsic_matrix, plane_values)
                                    self.target.extend(['restore'])
                                    cv2.circle(bgr_image, (int(self.target[1][0]), int(self.target[1][1])), 10, (255, 0, 0), -1)
                                    self.record_position.append([box_info[1], self.target])
                                    del self.action_list[0]
                                    if not self.action_list:
                                        speech.play_audio(record_finish_audio_path)
                                        self.enable_transport = False
                                        self.target = []
                                cv2.rectangle(bgr_image, (box_data[0], box_data[1]), (box_data[2], box_data[3]), (0, 255, 0), 2, 1)
                            elif box_info[0] == 'restore':
                               for i in self.record_position:
                                   if box_info[1].lower() in i[0].lower():
                                       self.target = i[1]
                                       self.start_transport = True
                                       self.enable_transport = False
                                       break
                        else:
                            msg = Bool()
                            msg.data = True
                            self.transport_finished_pub.publish(msg)
                self.fps.update()
                self.fps.show_fps(bgr_image)
                result_image = np.concatenate([bgr_image[40:440, ], depth_color_map], axis=1)
                cv2.imshow('image', result_image)
                cv2.waitKey(1)
                if not self.set_above:
                    cv2.moveWindow('image', 1920 - 640*2, 0)
                    os.system("wmctrl -r image -b add,above")
                    self.set_above = True
            else:
                time.sleep(0.1)
        cv2.destroyAllWindows()

    def transport_thread(self):
        while True:
            if self.start_transport:
                pose = []
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                p = copy.deepcopy(self.target)
                for i in range(3):
                    p[0][i] = p[0][i] * scale[i]
                    p[0][i] = p[0][i] + offset[i]
                if self.target[-1] == 'pick':
                    finish = pick_and_place.pick_without_back(p[0], 80, p[2], 550, 0.015, self.joints_pub, self.kinematics_client)
                    if not finish:
                        self.go_home()
                    else:
                        self.go_home(False)
                elif self.target[-1] == 'place':
                    finish = pick_and_place.place(p[0], 80, p[2], 400, self.joints_pub, self.kinematics_client)
                    if not finish:
                        self.go_home()
                    else:
                        self.go_home(False)
                elif self.target[-1] == 'restore':
                    p[0][-1] -= 0.015
                    finish = pick_and_place.place(p[0], 80, p[2], 400, self.joints_pub, self.kinematics_client)
                    if not finish:
                        self.go_home()
                    else:
                        self.go_home(False)
                with self.lock:
                    self.target = []
                    del self.action_list[0]
                self.enable_transport = True
                self.start_transport = False
            else:
                time.sleep(0.1)

    def multi_callback(self, ros_rgb_image, ros_depth_image, camera_info, depth_camera_info):
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8,
                               buffer=ros_rgb_image.data)
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16,
                                 buffer=ros_depth_image.data)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((bgr_image, depth_image, camera_info, depth_camera_info))

def main():
    node = ObjectTransport('object_transport')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
