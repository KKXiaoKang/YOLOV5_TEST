import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time
from yoloseg import YOLOSeg

# 初始化全局变量
depth_image = None
color_image = None
combined_img = None
frame_lock = threading.Lock()

# 获取图像的函数
def get_frames(pipeline):
    global depth_image, color_image
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        with frame_lock:
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

# 预处理、推理、后处理的函数
def process_frames(yoloseg):
    global color_image, combined_img
    while True:
        if color_image is None:
            continue
        with frame_lock:
            input_image = color_image.copy()
        
        # 推理
        start_time = time.time()
        boxes, scores, class_ids, masks = yoloseg(input_image)
        combined_img = yoloseg.draw_masks(input_image)
        end_time = time.time()
        inference_time = end_time - start_time
        fps = 1 / inference_time
        print(f" FPS : {fps:.4f} HZ")
        time.sleep(0.01)  # 模拟处理时间

# 推流显示的函数
def display_frames():
    global combined_img
    while True:
        if combined_img is None:
            continue
        with frame_lock:
            display_image = combined_img.copy()
        
        # 显示图像
        cv2.imshow('Detected Objects', display_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# 初始化深度和彩色流的配置
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

# 初始化YOLOSeg对象
model_path = "models/yolov5s-seg.onnx"
yoloseg = YOLOSeg(model_path, conf_thres=0.3, iou_thres=0.3)

# 启动线程
thread1 = threading.Thread(target=get_frames, args=(pipeline,))
thread2 = threading.Thread(target=process_frames, args=(yoloseg,))
thread3 = threading.Thread(target=display_frames)

# 设置守护线程，确保主线程退出时子线程也退出
thread1.daemon = True
thread2.daemon = True
thread3.daemon = True

# 启动线程
thread1.start()
thread2.start()
thread3.start()

# 等待所有线程结束
thread1.join()
thread2.join()
thread3.join()

# 停止管道
pipeline.stop()
