import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib

from yoloseg import YOLOSeg

# 配置深度和彩色流
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动管道
pipeline.start(config)

# Initialize YOLOv7 object detector
model_path = "models/yolov5s-seg.onnx"
yoloseg = YOLOSeg(model_path, conf_thres=0.3, iou_thres=0.3)
cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)

try:
    while True:
        # 等待获取一帧
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 将图像转换为NumPy数组
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 将深度图像转换为灰度颜色范围
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # # 显示图像
        # cv2.imshow('RealSense', np.hstack((color_image, depth_colormap)))
        # cv2.imshow('depth_color', depth_image)
        
        boxes, scores, class_ids, masks = yoloseg(color_image)
        combined_img = yoloseg.draw_masks(color_image)
        cv2.imshow("Detected Objects", combined_img)
        
        # SHOW
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 停止管道
    pipeline.stop()
