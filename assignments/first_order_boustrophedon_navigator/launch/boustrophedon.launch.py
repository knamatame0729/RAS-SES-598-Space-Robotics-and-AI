from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('first_order_boustrophedon_navigator'),
        'config',
        'boustrophedon_params.yaml'
    )
    
    return LaunchDescription([
        # Start the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # Start our boustrophedon controller
        Node(
            package='first_order_boustrophedon_navigator',
            executable='boustrophedon_controller',
            name='lawnmower_controller',
            parameters=[config],
            output='screen'
        )
    ]) 