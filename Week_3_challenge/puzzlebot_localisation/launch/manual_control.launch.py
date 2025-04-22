from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de Odometría
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_odometry',
            name='puzzlebot_odometry',
            output='screen'
        ),

        # Nodo de Controlador de Poses
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_pose_controller',
            name='puzzlebot_pose_controller',
            output='screen'
        ),
    ])
