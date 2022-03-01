#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import sys

import numpy as np
import rclpy

from autolab_core import ColorImage
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header



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
    node = rclpy.create_node("orbslam2_client")

    # Setup filenames.
    rgb_fns = sorted(glob.glob(os.path.join(dataset, "rgb/*.png")))
    depth_fns = sorted(glob.glob(os.path.join(dataset, "depth/*.png")))
    cam_intr_fn = os.path.join(dataset, "ost.txt")

    camera_info_msg = CameraInfo()
    with open(cam_intr_fn) as f:
        lines = f.readlines()
        for line in lines:
            if "width" in line:
                camera_info_msg.width = int(lines[lines.index(line)+1])
            elif "height" in line:
                camera_info_msg.height = int(lines[lines.index(line)+1])
            elif "camera matrix" in line:
                camera_info_msg.k = np.fromstring(" ".join(lines[lines.index(line)+1:lines.index(line)+4]), sep=' ')
            elif "distortion" in line:
                camera_info_msg.distortion_model = "plum_bob"
                camera_info_msg.d = np.fromstring(lines[lines.index(line)+1].rstrip(), sep=' ').tolist()
            elif "rectification" in line:
                camera_info_msg.r = np.fromstring(" ".join(lines[lines.index(line)+1:lines.index(line)+4]), sep=' ')
            elif "projection" in line:
                camera_info_msg.p = np.fromstring(" ".join(lines[lines.index(line)+1:lines.index(line)+4]), sep=' ')

    # Set up publishers
    cam_info_pub = node.create_publisher(CameraInfo, "/gqcnn/camera_info", 10)
    rgb_pub = node.create_publisher(Image, "/camera/color/image_raw", 10)
    depth_pub = node.create_publisher(Image, "/camera/depth/image_rect_raw", 10)

    # Set up subscriber and processor
    # grasp_processor = GraspProcessor(camera_intr, vis_grasp=vis_grasp)

    grasp_sub = node.create_subscription(
        PoseStamped, "/RGBD/pose", node.get_logger().info, 10
    )

    
    i = 0
    future = rclpy.task.Future()
    def timer_callback():
        nonlocal i

        if i >= len(depth_fns):
            future.set_result(1)
            return
        
        node.get_logger().info(f"Sending image {i} ({depth_fns[i]})")
        
        # Read depth image and create msg
        rgb_im = ColorImage.open(rgb_fns[i], frame="camera")
        depth_im = ColorImage.open(depth_fns[i], frame="camera")
        rgb_msg = rgb_im.rosmsg
        depth_msg = depth_im.rosmsg

        header = Header()
        header.stamp = node.get_clock().now().to_msg()
        header.frame_id = rgb_im.frame
        camera_info_msg.header = header
        rgb_msg.header = header
        depth_msg.header = header

        cam_info_pub.publish(camera_info_msg)
        rgb_pub.publish(rgb_msg)
        depth_pub.publish(depth_msg)
        i += 1

    node.create_timer(0.05, timer_callback)
    rclpy.spin_until_future_complete(node, f)
    node.destroy_node()
    rclpy.shutdown()
    
