#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Optional
from timeit import default_timer

import numpy as np
import rclpy
from rclpy.node import Node

from autolab_core import ColorImage, RigidTransform
from sensor_msgs.msg import CameraInfo, Image
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
        self.grasp_sub = self.create_subscription(PoseStamped, "/RGBD/pose", self.add_pose, 10)

        self.create_timer(1.0 / fps, self.publish)
        self.pub_future = rclpy.task.Future()
        self.sub_future = rclpy.task.Future()
        self.published = 0
        self.poses = []
        self.proc_times = np.zeros(len(self.depth_fns))
        self.rt_times = np.zeros(len(self.depth_fns))

    def publish(self):

        if self.published >= len(self.depth_fns):
            self.pub_future.set_result(1)
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

        self.rt_times[self.published] = default_timer()
        self.cam_info_pub.publish(self.camera_info_msg)
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.published += 1

    def add_pose(self, msg):
        self.proc_times[len(self.poses)] = int(msg.header.frame_id.split(";")[-1]) / 1e6
        self.rt_times[len(self.poses)] = default_timer() - self.rt_times[len(self.poses)]
        self.poses.append(ReferenceFrame(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
                                         np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])))
        
        if len(self.poses) >= len(self.depth_fns):
            self.sub_future.set_result(1)

def draw3d_arrow(
    arrow_location: np.ndarray,
    arrow_vector: np.ndarray,
    head_length: float = 0.05,
    color: Optional[str] = None,
    name: Optional[str] = None,
    ax: Optional[Axes3D] = None,
) -> Axes3D:
    if ax is None:
        ax = plt.gca()

    ax.quiver(
        *arrow_location,
        *arrow_vector,
        arrow_length_ratio=head_length / np.linalg.norm(arrow_vector) + 1e-6,
        color=color,
    )
    if name is not None:
        ax.text(*(arrow_location + arrow_vector), name)

    return ax


class ReferenceFrame:
    def __init__(
        self,
        origin: np.ndarray,
        q: np.ndarray,
        name: Optional[str] = None,
    ) -> None:
        self.origin = origin
        rot = RigidTransform.rotation_from_quaternion(q)
        self.dx = rot[:, 0]
        self.dy = rot[:, 1]
        self.dz = rot[:, 2]
        self.name = name

    def draw3d(
        self,
        head_length: float = 0.05,
        ax: Optional[Axes3D] = None,
    ) -> Axes3D:
        if ax is None:
            ax = plt.gca()

        if self.name is not None:
            ax.text(*self.origin + 0.5, f"({self.name})")
        ax = draw3d_arrow(
            ax=ax,
            arrow_location=self.origin,
            arrow_vector=self.dx,
            head_length=head_length,
            color="tab:red",
            # name="x",
        )
        ax = draw3d_arrow(
            ax=ax,
            arrow_location=self.origin,
            arrow_vector=self.dy,
            head_length=head_length,
            color="tab:green",
            # name="y",
        )
        ax = draw3d_arrow(
            ax=ax,
            arrow_location=self.origin,
            arrow_vector=self.dz,
            head_length=head_length,
            color="tab:blue",
            # name="z",
        )
        return ax

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
    parser.add_argument(
        "--fps",
        type=int,
        default=20,
        help="fps for publishing frames"
    )

    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    dataset = args.dataset
    fps = args.fps

    # Initialize the ROS node.
    rclpy.init()
    client = SLAMClient(dataset, fps)
    rclpy.spin_until_future_complete(client, client.pub_future)
    rclpy.spin_until_future_complete(client, client.sub_future)

    fig = plt.figure()
    ax = plt.axes(projection="3d")
    client.poses[0].draw3d()
    prev_t = client.poses[0].origin
    for pose in client.poses:
        if np.linalg.norm(pose.origin - prev_t) > 1e-4:
            pose.draw3d()
            # draw3d_arrow(prev_t, pose.origin, color="gray")
            prev_t = pose.origin
    ax.set_title("Camera Motion")
    plt.tight_layout()
    plt.savefig("/output/fig.png")
    np.save("/output/proc_times.npy", client.proc_times)
    np.save("/output/rt_times.npy", client.rt_times)

    client.destroy_node()
    rclpy.shutdown()
    
