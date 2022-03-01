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
                'ros', 'config', 'params_d435_rgbd.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'orb_slam2', 'Vocabulary', 'ORBvoc.txt'),
            description='Full path to vocabulary file to use'),

        Node(
            parameters=[
                params_file,
                {"voc_file": voc_file,
                 "use_sim_time": use_sim_time},
            ],
            package='orb_slam2_ros',
            node_executable='orb_slam2_ros_rgbd',
            node_name='orb_slam2_rgbd',
            output='screen',
            remappings=remappings
        )
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

    client_args = (
    ["--vis_grasp"]
    if LaunchConfiguration("vis").perform(context) == "1"
    else []
    )
    client_args.extend(
        ["--num_pubs", LaunchConfiguration("num_pubs").perform(context)]
    )

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
        executable="orb_slam2_ros_rgbd",
        output="screen",
        name="orb_slam2_ros_rgbd",
        arguments=[
            client_args
        ],
        remappings = [
        ('/camera/image_raw', '/camera/color/image_raw'),
        ('/camera/camera_info', '/camera/color/camera_info'),
    ]
    )
     remappings = [
        ('/camera/rgb/image_raw', '/camera/color/image_raw'),
        ('/camera/depth_registered/image_raw', '/camera/depth/image_rect_raw'),
        ('/camera/camera_info', '/camera/color/camera_info'),
    ]

    im_repub_node = Node(
        package="image_transport",
        name="im_repub_client",
        executable="republish",
        arguments=[
            "raw",  # Input
            img_transport_out,  # Output
        ],
        remappings=[
            ("in", "/gqcnn/image_raw"),
            (img_transport_out_remap, "/gqcnn/image_compressed"),
        ],
        parameters=img_transport_params
    )
    mask_repub_node = Node(
        package="image_transport",
        name="mask_repub_client",
        executable="republish",
        arguments=[
            "raw",  # Input
            img_transport_out,  # Output
        ],
        remappings=[
            ("in", "/gqcnn/mask_raw"),
            (img_transport_out_remap, "/gqcnn/mask_compressed"),
        ],
        parameters=img_transport_params
    )

    return[orb_slam2_ros_rgbd_node, im_repub_node, mask_repub_node]





def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
