import argparse
import time
import datetime
import math
from decimal import Decimal, ROUND_HALF_UP
from collections import deque
from timeit import default_timer as timer

import cv2
import numpy as np
import imutils
import pyrealsense2 as rs

# --------------------------
# 参数设置
# --------------------------
ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size for drawing")
args = vars(ap.parse_args())

file_name = 'data_c1/' + datetime.datetime.now().strftime("%m%d-%H%M") + 'c1.csv'

# 设置高分辨率
WIDTH = 1280
HEIGHT = 720
FPS = 30

# --- 更新后的 HSV 阈值 ---
# H: 102-118 (蓝色/青色区间)
targetLower = (102, 168, 85)   
targetUpper = (118, 255, 255) 

pts = deque(maxlen=args["buffer"])
smooth_buffer = deque(maxlen=10) # 平滑缓冲

def detect_video():
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # 1. RealSense 配置
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

    pipeline = rs.pipeline()
    print(f"[INFO] Starting RealSense at {WIDTH}x{HEIGHT}...")
    
    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"[Error] 启动失败: {e}")
        return

    # 2. 滤波器设置 (增强远距离效果)
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter() # 填孔滤波

    align_to = rs.stream.color
    align = rs.align(align_to)
    
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    theta = 0 

    with open(file_name, "a+") as f_csv:
        try:
            while True:
                # 3. 获取数据
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                
                raw_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not raw_depth_frame or not color_frame:
                    continue
                
                # 应用滤波
                filtered = spatial.process(raw_depth_frame)
                filtered = temporal.process(filtered)
                filtered = hole_filling.process(filtered)
                depth_frame = filtered.as_depth_frame() # 强制转换回 depth_frame

                frame = np.asanyarray(color_frame.get_data())
                result = frame.copy()

                # 图像处理
                # 减小模糊核 (5,5) 以保留远处的细节
                blurred = cv2.GaussianBlur(frame, (5, 5), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # 应用新的阈值
                mask = cv2.inRange(hsv, targetLower, targetUpper)
                
                # 只做膨胀，防止远处小物体被腐蚀掉
                mask = cv2.dilate(mask, None, iterations=2)

                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                
                center = None
                
                if len(cnts) > 0:
                    c = max(cnts, key=cv2.contourArea)
                    area = cv2.contourArea(c)

                    # 面积阈值保持较低 (针对 2m+ 距离)
                    if area > 100: 
                        (x, y, w, h) = cv2.boundingRect(c)
                        
                        # 长宽比检查 (过滤噪点)
                        aspect_ratio = float(w) / h
                        if 0.5 < aspect_ratio < 2.0:

                            M = cv2.moments(c)
                            if M["m00"] > 0:
                                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                                
                                # 绘制矩形和中心点
                                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 255), 2)
                                cv2.circle(result, center, 5, (0, 0, 255), -1)

                                pixel_x, pixel_y = center[0], center[1]
                                
                                if 0 <= pixel_x < WIDTH and 0 <= pixel_y < HEIGHT:
                                    # 4. 深度读取 (带容错)
                                    dist = depth_frame.get_distance(pixel_x, pixel_y) * 1000
                                    
                                    # 如果中心点是死区(0)，搜索周围 5x5 区域
                                    if dist == 0:
                                        dists = []
                                        for dy in range(-2, 3):
                                            for dx in range(-2, 3):
                                                nx, ny = pixel_x + dx, pixel_y + dy
                                                if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                                                    d = depth_frame.get_distance(nx, ny) * 1000
                                                    if d > 0: dists.append(d)
                                        if len(dists) > 0:
                                            dist = np.mean(dists)

                                    # 坐标计算
                                    if dist > 0 and dist < 6000: # 有效范围 6米内
                                        Xtemp = dist * (pixel_x - intr.ppx) / intr.fx
                                        Ytemp = dist * (pixel_y - intr.ppy) / intr.fy
                                        Ztemp = dist

                                        Xtarget = Xtemp - 35
                                        Ytarget = -(Ztemp * math.sin(theta) + Ytemp * math.cos(theta))
                                        Ztarget = Ztemp * math.cos(theta) + Ytemp * math.sin(theta)
                                        
                                        # 数据平滑
                                        smooth_buffer.append([Xtarget, Ytarget, Ztarget])
                                        avg_pos = np.mean(smooth_buffer, axis=0)
                                        
                                        def fmt(val): return str(int(val))
                                        
                                        # 屏幕显示信息
                                        txt = f"D:{fmt(avg_pos[2])}mm X:{fmt(avg_pos[0])} Y:{fmt(avg_pos[1])}"
                                        font_scale = 0.6 * (WIDTH / 640)
                                        cv2.putText(result, txt, (x, y - 10),
                                                    font, font_scale, (255, 255, 255), 2)
                                        
                                        # 写入 CSV
                                        f_csv.write(f"{fmt(avg_pos[0])},{fmt(avg_pos[1])},{fmt(avg_pos[2])},{time.time()}\n")

                # 绘制轨迹
                pts.appendleft(center)
                for i in range(1, len(pts)):
                    if pts[i - 1] is None or pts[i] is None: continue
                    thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                    cv2.line(result, pts[i - 1], pts[i], (0, 0, 255), thickness)

                cv2.namedWindow("Target Tracker", cv2.WINDOW_NORMAL)
                cv2.imshow("Target Tracker", result)
                # cv2.imshow("Mask", mask) # 调试用

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_video()