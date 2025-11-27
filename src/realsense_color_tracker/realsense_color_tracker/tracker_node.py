import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import message_filters

import cv2
import numpy as np
import math
from collections import deque
import time

class ColorTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker_node')

        # --- 1. 参数设置 ---
        # 蓝色 (Blue) 阈值 (保持你调优后的数值)
        self.targetLower = (102, 168, 85)
        self.targetUpper = (118, 255, 255)
        
        # 平滑缓冲
        self.smooth_buffer = deque(maxlen=10)
        self.buffer_size = 64
        self.pts = deque(maxlen=self.buffer_size)

        # 相机内参 (收到第一帧 CameraInfo 后更新)
        self.intrinsics = None
        self.cv_bridge = CvBridge()

        # --- 2. ROS 2 通信设置 ---
        
        # 发布者: 3D 坐标
        self.publisher_ = self.create_publisher(PoseStamped, 'detected_object_pose', 10)
        
        # 订阅者: CameraInfo (只需要收一次)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # 订阅者: Color 和 Aligned Depth (使用消息过滤器同步)
        # 注意：这里订阅的是 aligned_depth_to_color
        color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        # 时间同步器 (容忍度设为 0.1秒)
        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("Color Tracker Node Started. Waiting for frames...")

    def camera_info_callback(self, msg):
        # 获取相机内参矩阵 K: [fx, 0, ppx, 0, fy, ppy, 0, 0, 1]
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'ppx': msg.k[2],
                'ppy': msg.k[5]
            }
            self.get_logger().info(f"Camera Intrinsics Received: {self.intrinsics}")

    def sync_callback(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        try:
            # 1. 转换 ROS 图像为 OpenCV 格式
            # color_msg -> bgr8
            color_image = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            # depth_msg -> 16UC1 (单位: 毫米)
            depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # 2. 图像处理逻辑 (移植自之前的代码)
        result = color_image.copy()
        
        # 减小模糊核 (5,5) 以保留远处细节
        blurred = cv2.GaussianBlur(color_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.targetLower, self.targetUpper)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 兼容不同 OpenCV 版本
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        
        center = None
        detected = False
        
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)

            # 面积阈值 (针对 2m+ 距离)
            if area > 100: 
                x, y, w, h = cv2.boundingRect(c)
                
                # 长宽比检查
                aspect_ratio = float(w) / h
                if 0.5 < aspect_ratio < 2.0:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        detected = True

                        # 绘制
                        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 255), 2)
                        cv2.circle(result, center, 5, (0, 0, 255), -1)
                        
                        # --- 深度计算与坐标映射 ---
                        self.process_depth_and_publish(depth_image, center, x, y, result)

        # 绘制轨迹
        self.pts.appendleft(center)
        for i in range(1, len(self.pts)):
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            thickness = int(np.sqrt(self.buffer_size / float(i + 1)) * 2.5)
            cv2.line(result, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

        # 显示画面
        cv2.imshow("ROS2 Color Tracker", result)
        cv2.waitKey(1)

    def process_depth_and_publish(self, depth_image, center, rect_x, rect_y, result_img):
        pixel_x, pixel_y = center
        height, width = depth_image.shape
        
        # 边界检查
        if not (0 <= pixel_x < width and 0 <= pixel_y < height):
            return

        # 1. 读取深度 (毫米)
        dist = depth_image[pixel_y, pixel_x]
        
        # 2. 深度容错 (如果中心点是0，搜索周围 5x5 区域)
        if dist == 0:
            dists = []
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    nx, ny = pixel_x + dx, pixel_y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        d = depth_image[ny, nx]
                        if d > 0:
                            dists.append(d)
            if len(dists) > 0:
                dist = np.mean(dists)

        # 3. 坐标转换 (2D -> 3D)
        if 0 < dist < 6000: # 6米内有效
            # 使用内参反投影
            Xtemp = dist * (pixel_x - self.intrinsics['ppx']) / self.intrinsics['fx']
            Ytemp = dist * (pixel_y - self.intrinsics['ppy']) / self.intrinsics['fy']
            Ztemp = dist

            # 4. 坐标系调整 (保持你之前的逻辑)
            # 原始代码逻辑:
            # Xtarget = Xtemp - 35
            # Ytarget = -(Ztemp*math.sin(0) + Ytemp*math.cos(0)) -> -Ytemp
            # Ztarget = Ztemp*math.cos(0) + Ytemp*math.sin(0) -> Ztemp
            
            Xtarget = Xtemp - 35
            Ytarget = -Ytemp
            Ztarget = Ztemp
            
            # 5. 平滑数据
            self.smooth_buffer.append([Xtarget, Ytarget, Ztarget])
            avg_pos = np.mean(self.smooth_buffer, axis=0)
            
            # 6. 发布 ROS 消息
            self.publish_pose(avg_pos)
            
            # 7. 在画面上显示
            txt = f"D:{int(avg_pos[2])}mm X:{int(avg_pos[0])} Y:{int(avg_pos[1])}"
            cv2.putText(result_img, txt, (rect_x, rect_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def publish_pose(self, pos):
        # pos 是 [x, y, z] 单位 mm
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_frame" # 对应 Realsense 的 TF 坐标系
        
        # 转换为米 (ROS 标准单位)
        msg.pose.position.x = pos[2] / 1000.0
        msg.pose.position.y = -pos[0] / 1000.0
        msg.pose.position.z = pos[1] / 1000.0
        
        # 默认姿态 (无旋转)
        msg.pose.orientation.w = 1.0
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()