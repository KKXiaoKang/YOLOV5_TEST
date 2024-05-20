# #!/usr/bin/env python
# """
#     现在目标检测的结果仍为2维坐标
#     不是三维坐标，要通过特殊方法转换一下
# """
# import rospy
# import rospkg
# import numpy as np
# import cv2
# import time
# from sensor_msgs.msg import Image, CameraInfo
# from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
# from cv_bridge import CvBridge, CvBridgeError

# from geometry_msgs.msg import TransformStamped
# import tf2_ros
# import tf2_geometry_msgs

# class ARControlNode(object):
#     def __init__(self):
#         rospy.init_node('ar_control_node')
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster()

#         self.bridge = CvBridge()
        
#         self.camera_info = None
#         self.depth_image = None

#         # 订阅深度图像和相机内参
#         self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
#         self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)

#         self.publisher = rospy.Publisher('/tf2_yolov_base_result', Detection2DArray, queue_size=10)
#         self.subscription = rospy.Subscriber('/object_yolo_segment_result', Detection2DArray, self.yolo_callback)
    
#     def depth_callback(self, msg):
#         try:
#             self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
    
#     def camera_info_callback(self, msg):
#         # 如果没有值则直接更新
#         if self.camera_info == None:
#             self.camera_info = msg

#     def yolo_callback(self, msg):
#         if self.camera_info is None or self.depth_image is None:
#             return

#         try:
#             # 获取头部相机光学坐标系到头部相机坐标系的转换
#             trans = self.tf_buffer.lookup_transform("head_camera_link", "head_camera_color_optical_frame", rospy.Time())
            
#             # 遍历检测结果
#             for i, detection in enumerate(msg.detections):
#                 # 获取检测结果的边界框中心点
#                 center_x = int(detection.bbox.center.x)
#                 center_y = int(detection.bbox.center.y)
                
#                 # 获取中心点对应的深度值
#                 depth = self.depth_image[center_y, center_x]
                
#                 # 如果深度值无效，跳过该检测结果
#                 if np.isnan(depth) or depth == 0:
#                     continue

#                 # 获取相机内参
#                 fx = self.camera_info.K[0]
#                 fy = self.camera_info.K[4]
#                 cx = self.camera_info.K[2]
#                 cy = self.camera_info.K[5]

#                 # 将像素坐标转换为相机坐标系下的三维坐标
#                 x = (center_x - cx) * depth / fx
#                 y = (center_y - cy) * depth / fy
#                 z = depth

#                 # 创建TransformStamped消息
#                 tf_msg = TransformStamped()
#                 tf_msg.header = msg.header
#                 tf_msg.header.frame_id = 'head_camera_link'
#                 tf_msg.child_frame_id = f"transformed_object_{detection.results[0].id}"
#                 tf_msg.transform.translation.x = x
#                 tf_msg.transform.translation.y = y
#                 tf_msg.transform.translation.z = z
#                 tf_msg.transform.rotation.x = 0.0
#                 tf_msg.transform.rotation.y = 0.0
#                 tf_msg.transform.rotation.z = 0.0
#                 tf_msg.transform.rotation.w = 1.0

#                 self.tf_broadcaster.sendTransform(tf_msg)
            
#             # 发布
#             self.publisher.publish(msg)
            
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             rospy.logerr("Transform lookup failed: {0}".format(e))

# def main():
#     try:
#         ar_control_node = ARControlNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

# if __name__ == "__main__":
#     main()
