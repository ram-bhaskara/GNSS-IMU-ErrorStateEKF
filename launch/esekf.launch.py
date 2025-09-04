from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss_imu_esekf',
            executable='esekf_node', 
            name='esekf_node',
            output='screen',
            parameters=['$(find-pkg-share gnss_imu_esekf)/config/params.yaml']
        )
    ])