import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    repub_config = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 
        'config',
        'hap_driver_config.yaml')
    
    livox_repub_node = Node(
        package='livox_ros_driver2',
        executable='livox_repub_node',
        name='livox_repub_node',
        output='screen',
        parameters=[repub_config],
        #prefix=['xterm -e gdb -ex run --args'],
        )
    return LaunchDescription([
        livox_repub_node,
    ])