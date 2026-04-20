#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import csv
from std_srvs.srv import Trigger  # 导入 Trigger 服务
import time
import cv2
import numpy as np

class NavThroughPosesClient(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('nav_through_poses_client')
        # 创建Action客户端，用于导航
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        # 订阅速度命令
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # 发布车辆速度命令
        self.car_cmd_vel_pub = self.create_publisher(Twist, '/car_cmd_vel', 10)
        # 初始化变量
        self.distance_remaining = None  # 剩余距离
        self.is_paused = False  # 标志位，用于控制是否暂停导航
        self.cycle_number = 0  # 循环次数
        self.fin_flag = False  # 完成标志
        self.stop_timer = None  # 用于存储停止计时器
        
        # 添加状态管理
        self.last_stop_time = 0  # 上次停车的时间
        self.stop_cooldown = 5.0  # 停车冷却时间（秒）
        self.last_distance = None  # 记录上一次的距离
        self.is_forward = True  # 标记当前是去程还是返程

        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().info("Error: Unable to open camera.")

    def send_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self._action_client.wait_for_server(timeout_sec=10.0)
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation result: {0}'.format(result))
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.distance_remaining > 0.0:
            self.get_logger().info(f'Distance to target: {feedback.distance_remaining:.2f} meters')
            # 更新距离和方向
            if self.last_distance is not None:
                # 如果距离在增加，说明是返程
                if feedback.distance_remaining > self.last_distance:
                    self.is_forward = False
                # 如果距离在减少，说明是去程
                else:
                    self.is_forward = True
            self.last_distance = feedback.distance_remaining
            self.distance_remaining = feedback.distance_remaining
            
            # 如果距离目标点的距离小于 1 米，发送停止命令
            if self.distance_remaining is not None and feedback.distance_remaining < 0.5 and not self.is_paused:
                self.get_logger().info('Reached the target, stopping the robot.')
                self.is_paused = True  # 设置标志位为暂停状态
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.car_cmd_vel_pub.publish(twist)

    def cmd_vel_callback(self, msg):
        if self.distance_remaining is not None and self.distance_remaining > 0.5 and not self.is_paused:
            # 否则，正常转发速度命令
            self.car_cmd_vel_pub.publish(msg)

    def detect_callback(self, label):
        current_time = time.time()
        # 检查是否在冷却时间内
        if current_time - self.last_stop_time < self.stop_cooldown:
            return

        if label in ['red', 'white'] and not self.is_paused:
            # 对于红绿灯，只在距离终点30米内才识别
            if label == 'red' and (self.distance_remaining is None or self.distance_remaining > 30.0):
                return
                
            # 记录是去程还是返程
            direction = "forward" if self.is_forward else "return"
            self.get_logger().info(f'Detected {label} in {direction} direction')
            
            self.get_logger().info(f'Stopping for 3 seconds')
            self.is_paused = True
            self.last_stop_time = current_time
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.car_cmd_vel_pub.publish(twist)
            
            # 创建定时器，3秒后恢复导航
            if self.stop_timer is not None:
                self.stop_timer.cancel()
            self.stop_timer = self.create_timer(3.0, self.resume_navigation)

    def resume_navigation(self):
        self.get_logger().info('Resuming navigation')
        self.is_paused = False
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
        
        # 发送一个小的初始速度，让导航系统重新开始规划
        twist = Twist()
        twist.linear.x = 0.3  # 设置一个小的初始速度
        twist.angular.z = 0.0
        self.car_cmd_vel_pub.publish(twist)
        
        # 短暂延迟后停止，让导航系统接管
        time.sleep(0.1)
        twist.linear.x = 0.0
        self.car_cmd_vel_pub.publish(twist)

    def ensure_odd(self, val):
        return val if val % 2 == 1 else val + 1

    def red_light_detection(self, frame):
        # 设置默认参数值
        smooth_kernel_size = 21
        gaussian_blur_val = 15
        morph_kernel_size = 5
        min_area = 100
        max_area = 3000

        # 平滑滤波
        smooth = cv2.medianBlur(frame, smooth_kernel_size)
        
        # 高斯模糊
        gaussian_blurred = cv2.GaussianBlur(smooth, (gaussian_blur_val, gaussian_blur_val), 0)
        
        # 转换到 HSV 色彩空间
        hsv = cv2.cvtColor(gaussian_blurred, cv2.COLOR_BGR2HSV)
        
        # 提取红色区域
        # 第一个红色范围
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        # 第二个红色范围
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 创建掩码
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # 合并两个掩码
        mask = mask1 + mask2
        
        # 形态学操作，侵蚀和膨胀让红灯轮廓闭合
        kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
        morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected = False
        output_image = frame.copy()
        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area < area < max_area:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                circle_area = np.pi * (radius ** 2)
                
                if min_area < circle_area < max_area:
                    cv2.circle(output_image, center, radius, (0, 255, 0), 8)
                    label = "red"
                    self.detect_callback(label)
                    detected = True
        
        # cv2.imshow('Red Light Detection', output_image)
        
        if cv2.waitKey(1) & 0xFF == 27:  # 按下 'Esc' 键退出
            return False

        return detected

    def zebra_crossing_detection(self, frame):
        # 设置默认参数值
        crop_percent = 0.3
        median_blur_val = 15
        gaussian_blur_val = 0
        canny_threshold1 = 50
        canny_threshold2 = 134
        morph_kernel_size = 1
        area_threshold = 3000
        hough_threshold = 16
        line_threshold = 10

        # 处理图像
        height = frame.shape[0]
        cropped_image = frame[int(crop_percent * height):, :]
        
        gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        
        # 应用中值滤波
        if median_blur_val > 1:
            blurred = cv2.medianBlur(gray, median_blur_val)
        else:
            blurred = gray
        
        # 应用高斯模糊
        if gaussian_blur_val > 1:
            gaussian_blurred = cv2.GaussianBlur(blurred, (gaussian_blur_val, gaussian_blur_val), 0)
        else:
            gaussian_blurred = blurred
        
        edges = cv2.Canny(gaussian_blurred, canny_threshold1, canny_threshold2)
        
        kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros_like(closed_edges)
        for contour in contours:
            if cv2.contourArea(contour) > area_threshold:
                cv2.drawContours(mask, [contour], -1, (255), thickness=cv2.FILLED)
        
        roi = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)
        
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi_edges = cv2.Canny(roi_gray, canny_threshold1, canny_threshold2)
        
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, threshold=hough_threshold, minLineLength=50, maxLineGap=10)
        
        line_image = np.copy(cropped_image)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 8)
                    label = "white"
                    self.detect_callback(label)
        
        detected = lines is not None and len(lines) > line_threshold

        # cv2.imshow('Detected Lines', line_image)
        
        if cv2.waitKey(1) & 0xFF == 27:  # 按下 'Esc' 键退出
            return False

        return detected

    def run_detection(self):
        ret, frame = self.cap.read()
        if ret:
            self.red_light_detection(frame)
            self.zebra_crossing_detection(frame)

def read_waypoints_from_csv(filename):
    waypoints = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) == 4:  # Ensure each row has x, y, z, w
                waypoints.append({
                    "x": float(row[0]),
                    "y": float(row[1]),
                    "z": float(row[2]),
                    "w": float(row[3])
                })
    return waypoints

def main(args=None):
    rclpy.init(args=args)

    client = NavThroughPosesClient()

    # 从CSV文件读取导航点
    # waypoints = read_waypoints_from_csv('/home/davinci-mini/racecar/src/racecar/scripts/ai_test.csv')
    waypoints = read_waypoints_from_csv('/home/davinci-mini/racecar/src/racecar/scripts/out_test.csv')

    # 将导航点转换为 PoseStamped 消息
    poses = []
    for point in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 0
        pose.pose.position.x = point["x"]
        pose.pose.position.y = point["y"]
        pose.pose.position.z = point["z"]
        pose.pose.orientation.w = point["w"]
        poses.append(pose)

    client.send_goal(poses)

    while rclpy.ok():
        rclpy.spin_once(client)
        client.run_detection()

    client.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()