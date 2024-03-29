import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory("orb_slam3_ros2")

    use_sim_time = LaunchConfiguration('use_sim_time')
    # params_file = LaunchConfiguration('params_file')
    voc_file = LaunchConfiguration('voc_file')
    settings_file = LaunchConfiguration('settings_file')
    publish_raw = LaunchConfiguration('publish_raw')
    publish_tf = LaunchConfiguration('publish_tf')
    use_viewer = LaunchConfiguration('use_viewer')
    target_frame_id = LaunchConfiguration('target_frame_id')
    image_left_topic = LaunchConfiguration('image_left_topic')
    image_right_topic = LaunchConfiguration('image_right_topic')

    remappings = [
        # ('/odom', '/odometry/filtered'),
        # ('/odom', '/wheel/odom'),
        # ('/camera/image_raw', '/camera/infra1/image_rect_raw'),
        # ('/camera/image_raw', '/gst_image_pub'),
        # ('/camera/camera_info', '/camera/color/camera_info'),
        # ('/camera/depth/image_rect_raw', '/camera/aligned_depth_to_color/image_raw')
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'publish_raw',
            default_value='false',
            description='Publish only raw twc data'),

        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Publish Tf'),

        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam3_ros2"),
                'params', 'orb_slam', 'ORBvoc.txt'),
            description='Full path to vocabulary file to use'),

        DeclareLaunchArgument(
            'settings_file',
            default_value=os.path.join(
                package_dir,
                'params', 'orb_slam', 'stereo_D435i.yaml'),
            description='Full path to ORB-SLAM3 settings file to use'),

        DeclareLaunchArgument(
            'use_viewer',
            default_value="false",
            description="Use Viewer"
        ),

        DeclareLaunchArgument(
            'target_frame_id',
            default_value="odom",
            description="Target Tf frame id"
        ),

        DeclareLaunchArgument(
            'image_left_topic',
            default_value="/camera/infra1/image_rect_raw",
            description="camera topic"
        ),

        DeclareLaunchArgument(
            'image_right_topic',
            default_value="/camera/infra2/image_rect_raw",
            description="camera topic"
        ),

        Node(
            parameters=[
              {"voc_file": voc_file,
               "publish_raw": publish_raw,
               "publish_tf": publish_tf,
               "use_sim_time": use_sim_time,
               "settings_file": settings_file,
               "use_viewer": use_viewer,
               "target_frame_id": target_frame_id,
               "image_left_topic": image_left_topic,
               "image_right_topic": image_right_topic},
            ],
            package='orb_slam3_ros2',
            executable='orb_slam_stereo',
            name='orb_slam_stereo',
            output='screen',
            # prefix=['xterm -e gdb -ex run --args'],
            remappings=remappings
        )
    ])
