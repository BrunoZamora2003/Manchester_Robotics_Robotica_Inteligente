#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess                # ← conservada la importación original
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al directorio share de tu paquete
    pkg_share = get_package_share_directory('puzzlebot_localisation')

    # Ruta al archivo RViz
    rviz_config = os.path.join(pkg_share, 'config', 'puzzlebot_visualization.rviz')

    return LaunchDescription([

        # 2) Nodo de Odometría
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_odometry',
            name='puzzlebot_odometry',
            output='screen'
        ),

        # 3) Nodo de Detección de Color (semafórico)
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_color_detection',
            name='color_detection_node',
            output='screen'
        ),

        # 4) Nodo de Control (PID + semáforo)
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_controller',
            name='puzzlebot_controller',
            output='screen'
        ),

        # 5) Nodo Generador de Rutas (waypoints.yaml)
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_path_generator',
            name='puzzlebot_path_generator',
            output='screen'
        ),

    ])
