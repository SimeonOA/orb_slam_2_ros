#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict

from autolab_core import ColorImage
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class SLAMClient(Node):

    def __init__(self, dataset, fps):
        super().__init__("orbslam2_client")

        # Setup filenames.
        self.rgb_fns = sorted(glob.glob(os.path.join(dataset, "rgb/*.png")))
        self.depth_fns = sorted(glob.glob(os.path.join(dataset, "depth/*.png")))
        cam_intr_fn = os.path.join(dataset, "ost.txt")

        self.camera_info_msg = CameraInfo()
        with open(cam_intr_fn) as f:
            lines = f.readlines()
            for line in lines:
                if "width" in line:
                    self.camera_info_msg.width = int(lines[lines.index(line)+1])
                elif "height" in line:
                    self.camera_info_msg.height = int(lines[lines.index(line)+1])
                elif "camera matrix" in line:
                    self.camera_info_msg.k = np.fromstring(" ".join(lines[lines.index(line)+1:lines.index(line)+4]), sep=' ')
                elif "distortion" in line:
                    self.camera_info_msg.distortion_model = "plum_bob"
                    self.camera_info_msg.d = np.fromstring(lines[lines.index(line)+1].rstrip(), sep=' ').tolist()
                elif "rectification" in line:
                    self.camera_info_msg.r = np.fromstring(" ".join(lines[lines.index(line)+1:lines.index(line)+4]), sep=' ')
                elif "projection" in line:
                    self.camera_info_msg.p = np.fromstring(" ".join(lines[lines.index(line)+1:lines.index(line)+4]), sep=' ')

        # Set up publishers and subscriber
        self.cam_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)
        self.rgb_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_rect_raw", 10)
        self.grasp_sub = self.create_subscription(PoseStamped, "/RGBD/pose", self.print_sub, 10)

        self.create_timer(1.0 / fps, self.publish)
        self.future = rclpy.task.Future()
        self.published = 0

    def publish(self):

        if self.published >= len(self.depth_fns):
            self.future.set_result(1)
            return
        
        self.get_logger().info(f"Sending image {self.published} ({self.depth_fns[self.published]})")
        
        # Read depth image and create msg
        rgb_im = ColorImage.open(self.rgb_fns[self.published], frame="camera")
        depth_im = ColorImage.open(self.depth_fns[self.published], frame="camera")
        rgb_msg = rgb_im.rosmsg
        depth_msg = depth_im.rosmsg

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = rgb_im.frame
        self.camera_info_msg.header = header
        rgb_msg.header = header
        depth_msg.header = header

        self.cam_info_pub.publish(self.camera_info_msg)
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.published += 1

    def print_sub(self, msg):
        self.get_logger().info(f"{message_to_ordereddict(msg)}")


if __name__ == "__main__":

    # Parse args.
    parser = argparse.ArgumentParser(
        description="Run an ORB SLAM ROS client using images from the TUM dataset"
    )
    parser.add_argument(
        "--dataset",
        type=str,
        default=None,
        help="path to image files",
    )

    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    dataset = args.dataset

    # Initialize the ROS node.
    rclpy.init()
    client = SLAMClient(dataset, 20)

    rclpy.spin_until_future_complete(client, client.future)
    client.destroy_node()
    rclpy.shutdown()
    
