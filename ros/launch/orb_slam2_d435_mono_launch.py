import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

use_sim_time = LaunchConfiguration('use_sim_time')
params_file = LaunchConfiguration('params_file')
voc_file = LaunchConfiguration('voc_file')


launch_args = [
    LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'ros', 'config', 'params_d435_mono.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'orb_slam2', 'Vocabulary', 'ORBvoc.txt'),
            description='Full path to vocabulary file to use'),
    ])
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

    client_args= ["--fps", LaunchConfiguration("fps").perform(context)]

    orb_slam2_mono_node = Node(
        parameters=[
            params_file,
            {"voc_file": voc_file,
            "use_sim_time": use_sim_time},
        ],
        package="orb_slam2_ros",
        executable="orb_slam2_ros_mono",
        output="screen",
        name="orb_slam2_mono",
        arguments=[
            client_args
        ],
        remappings = [
        ('/camera/image_raw', '/camera/color/image_raw'),
        ('/camera/camera_info', '/camera/color/camera_info'),
    ]
    )
    return [orb_slam2_mono_node]


def generate_launch_description():

    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
