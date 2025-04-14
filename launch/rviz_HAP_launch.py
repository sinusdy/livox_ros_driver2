import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config')
rviz_config_path = os.path.join(config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(config_path, 'HAP_config.json')
livox_config_path = os.path.join(config_path, 'hap_driver_config.yaml')
################### user configure parameters for ros2 end #####################

def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[livox_config_path, {"user_config_path": user_config_path}]
        )

    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

    return LaunchDescription([
        livox_driver,
        livox_rviz,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
