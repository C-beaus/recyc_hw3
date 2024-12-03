#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

class ImgSaver(Node):
    def __init__(self):
        super().__init__('img_saver_node')
        
        self.bridge = CvBridge()
        self.save_path = "/home/anuj/pointcloud/src/pointcloud_exercise/data/"
        self.end_name = "object_1"

        # Subscribers
        self.depth_img_sub = Subscriber(self, Image, "/camera/camera/aligned_depth_to_color/image_raw")
        self.color_img_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
        self.cam_info_sub = Subscriber(self, CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info")

        # Time Synchronizer
        ats = ApproximateTimeSynchronizer(
            [self.depth_img_sub, self.color_img_sub, self.cam_info_sub],
            queue_size=10,
            slop=0.1
        )
        ats.registerCallback(self.image_received)

    def image_received(self, depth_img, color_img, cam_info):
        try:
            # RGB image
            cv_image = self.bridge.imgmsg_to_cv2(color_img, "bgr8")
            cv2.imwrite(self.save_path + f"rgb_{self.end_name}.png", cv_image)

            # Depth image
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")
            np.save(self.save_path + f"depth_{self.end_name}.npy", cv_depth_image)

            # Point Cloud
            scale = 0.001
            cx = cam_info.k[2]
            cy = cam_info.k[5]
            fx = cam_info.k[0]
            fy = cam_info.k[4]
            rows, cols = cv_depth_image.shape

            c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
            valid = (cv_depth_image > 0) & (cv_depth_image <= 900)
            z = np.where(valid, scale * cv_depth_image, np.nan)
            x = np.where(valid, z * (c - cx) / fx, 0)
            y = np.where(valid, z * (r - cy) / fy, 0)

            points = np.dstack((x, y, z)).transpose(2, 0, 1).reshape(3, -1)
            np.save(self.save_path + f"pcl_{self.end_name}.npy", points)

            self.get_logger().info("Saved images and point cloud data successfully.")

            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing images: {e}")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    img_saver = ImgSaver()
    rclpy.spin(img_saver)
    img_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
