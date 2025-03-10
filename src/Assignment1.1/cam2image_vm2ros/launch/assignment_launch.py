import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the absolute path of cam2image.yaml from the package
    cam2image_config = os.path.join(
        get_package_share_directory('cam2image_vm2ros'),  # Ensure package name is correct
        'config',
        'cam2image.yaml'
    )

    return LaunchDescription([
        Node(
            package='ball_detector',
            executable='ball_detector',
            name='ball_detector_node',
            output='screen'
        ),
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[cam2image_config],
            output='screen'
        ),
        Node(
            package='relbot_setpoint_generator',
            executable='setpoint_generator',
            name='setpoint_generator_node',
            output='screen'
        ),
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator_node',
            output='screen'
        )
    ])
