import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os
import signal
import numpy as np
from ament_index_python.packages import get_package_share_directory

class PuzzlebotPathGenerator(Node):
    def __init__(self):
        super().__init__('puzzlebot_path_generator')

        # Cargar waypoints desde el archivo
        self.waypoints = self.load_waypoints()
        self.current_idx = 0

        # Umbral para considerar que llegó
        self.goal_threshold = 0.05  # [m]

        # Estado de posición actual (opcional para validación extra)
        self.x = 0.0
        self.y = 0.0

        # Publicador
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Timer para publicación y verificación de cambio de objetivo
        self.publish_timer = self.create_timer(1.0, self.publish_goal)  # Cada 1s

        self.get_logger().info("Nodo de Generador de Rutas Iniciado.")

    def load_waypoints(self):
        """Carga waypoints desde archivo waypoints.yaml"""
        try:
            package_share_directory = get_package_share_directory('puzzlebot_localisation')
            yaml_path = os.path.join(package_share_directory, 'config', 'waypoints.yaml')

            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            self.get_logger().info(f"Waypoints cargados exitosamente: {data['waypoints']}")
            return data['waypoints']
        except Exception as e:
            self.get_logger().error(f"Error al cargar waypoints: {e}")
            return []

    def publish_goal(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info("Todos los waypoints completados.")
            return

        goal_x, goal_y = self.waypoints[self.current_idx]

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'odom'  # Importante: todo debe estar en el mismo frame

        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0

        # Orientación vacía (no la necesitamos aquí)
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Publicando objetivo {self.current_idx+1}: ({goal_x}, {goal_y})")

        # Cambiar de waypoint manualmente: asumir que 1 segundo después ya llegó
        self.current_idx += 1

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPathGenerator()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit triggered. Shutting down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
