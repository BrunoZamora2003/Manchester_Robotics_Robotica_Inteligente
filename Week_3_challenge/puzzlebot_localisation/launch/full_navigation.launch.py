from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Nodo de Odometría
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_odometry',
            name='puzzlebot_odometry',
            output='screen'
        ),

        # Nodo de Controlador
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_pose_controller',
            name='puzzlebot_pose_controller',
            output='screen'
        ),

        # Nodo de Generador de Rutas
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_path_generator',
            name='puzzlebot_path_generator',
            output='screen'
        ),

        # Ejecutar RViz2 con configuración específica
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                'share', 'puzzlebot_localisation', 'config', 'puzzlebot_visualization.rviz'
            )],
            output='screen'
        )
    ])
