from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cocel_driver'),
        'config',
        'cocel_driver_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='cocel_driver',
            executable='cocel_driver_node',
            namespace='cocel_driver_node',
            output='screen',
            parameters=[config],
        ),
    ])
