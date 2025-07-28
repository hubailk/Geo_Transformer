from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geo_transformer',
            executable='geo_transformer_node',
            name='geo_transformer',
            output='screen'
        ),
        Node(
            package='geo_transformer',
            executable='test_client',
            name='test_client',
            output='screen'
        )
    ])
