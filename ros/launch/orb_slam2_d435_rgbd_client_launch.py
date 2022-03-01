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
    DeclareLaunchArgument("fps", default_value="20"),
    DeclareLaunchArgument("dataset")
]

def launch_setup(context):
    compress = LaunchConfiguration("compress").perform(context)
    if compress == "1":
        img_transport_out = "compressed"
        img_transport_out_remap = "out/compressed"
        img_transport_params = [{"out.format": "png"}, {"out.png_level": 9}]
    elif compress == "2":
        img_transport_out = "h264"
        img_transport_out_remap = "out/h264"
        img_transport_params = []
    else:
        img_transport_out = "raw"
        img_transport_out_remap = "out"
        img_transport_params = []

    client_args = ["--dataset", LaunchConfiguration("dataset").perform(context)]
    client_args.extend(
        ["--fps", LaunchConfiguration("fps").perform(context)]
    )

    orb_slam2_rgbd_client = Node(
        parameters=[
            LaunchConfiguration("params_file"),
            {"voc_file": LaunchConfiguration("voc_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        package="orb_slam2_ros",
        executable="client.py",
        output="screen",
        name="orb_slam2_ros_rgbd_client",
        arguments=client_args,
    )
    
    rgb_repub_node = Node(
        package="image_transport",
        name="rgb_repub_client",
        executable="republish",
        arguments=[
            "raw",  # Input
            img_transport_out,  # Output
        ],
        remappings=[
            ("in", "/camera/color/image_raw"),
            (img_transport_out_remap, "/camera/color/image_compressed"),
        ],
        parameters=img_transport_params
    )
    depth_repub_node = Node(
        package="image_transport",
        name="depth_repub_client",
        executable="republish",
        arguments=[
            "raw",  # Input
            img_transport_out,  # Output
        ],
        remappings=[
            ("in", "/camera/depth/image_rect_raw"),
            (img_transport_out_remap, "/camera/depth/image_rect_compressed"),
        ],
        parameters=img_transport_params
    )

    return[orb_slam2_rgbd_client, rgb_repub_node, depth_repub_node]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
