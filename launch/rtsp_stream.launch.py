from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_pipeline',
            executable='rtsp_stream',
            name='rtsp_camera_node',
            output='screen',
            parameters=[{
                'rtsp_url': 'rtsp://172.20.10.1:8554/live'
            }]
        )
    ])
