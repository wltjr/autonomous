
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    autonomy_node = Node(package='autonomous',
                               executable='autonomy_node',
                               name='autonomy_node',
                               parameters=[],
    )

    return LaunchDescription([
        autonomy_node,
    ])
