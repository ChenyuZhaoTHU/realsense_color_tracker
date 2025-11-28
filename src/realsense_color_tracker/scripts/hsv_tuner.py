import cv2
import numpy as np
import pyrealsense2 as rs

def nothing(x):
    pass

# 1. 启动 Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 2. 创建调节窗口
cv2.namedWindow("Trackbars")
# 默认值可以先设宽一点
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

try:
    while True:
        frames = pipeline.wait_for_frames()
        frame = np.asanyarray(frames.get_color_frame().get_data())
        
        # 转 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 获取滑动条的值
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])
        
        # 创建 Mask
        mask = cv2.inRange(hsv, lower_range, upper_range)
        
        # 显示只有颜色的部分
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)
        cv2.imshow("result", result)
        
        key = cv2.waitKey(1)
        if key == 27: # Esc
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()