import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

launch_args = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),

    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory("orb_slam2_ros"),
            'ros', 'config', 'params_d435_rgbd.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'),

    DeclareLaunchArgument(
        'voc_file',
        default_value=os.path.join(
            get_package_share_directory("orb_slam2_ros"),
            'orb_slam2', 'Vocabulary', 'ORBvoc.txt'),
        description='Full path to vocabulary file to use'),

    DeclareLaunchArgument("compress", default_value="0"),
]

def launch_setup(context):
    compress = LaunchConfiguration("compress").perform(context)
    if compress == "1":
        img_transport_in = "compressed"
        img_transport_in_remap = "in/compressed"
        img_transport_params = [{"in.format": "png"}, {"in.png_level": 9}]
    elif compress == "2":
        img_transport_in = "h264"
        img_transport_in_remap = "in/h264"
        img_transport_params = []
    else:
        img_transport_in = "raw"
        img_transport_in_remap = "in"
        img_transport_params = []

    orb_slam2_rgbd_node = Node(
        parameters=[
            LaunchConfiguration("params_file"),
            {"voc_file": LaunchConfiguration("voc_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        package="orb_slam2_ros",
        executable="orb_slam2_ros_rgbd",
        output="screen",
        name="orb_slam2_ros_rgbd",
        remappings = [
            ('/camera/rgb/image_raw', '/camera/color/image'),
            ('/camera/depth_registered/image_raw', '/camera/depth/image_rect'),
            ('/camera/camera_info', '/camera/color/camera_info'),
        ]
    )
    
    rgb_repub_node = Node(
        package="image_transport",
        name="rgb_repub_node",
        executable="republish",
        arguments=[
            img_transport_in,  # Input
            "raw",  # Output
        ],
        remappings=[
            (img_transport_in_remap, "/camera/color/image_compressed"),
            ("out", "/camera/color/image"),
        ],
        parameters=img_transport_params
    )
    depth_repub_node = Node(
        package="image_transport",
        name="depth_repub_node",
        executable="republish",
        arguments=[
            img_transport_in,  # Output
            "raw",  # Input
        ],
        remappings=[
            (img_transport_in_remap, "/camera/depth/image_rect_compressed"),
            ("out", "/camera/depth/image_rect"),
        ],
        parameters=img_transport_params
    )

    return[orb_slam2_rgbd_node, rgb_repub_node, depth_repub_node]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
