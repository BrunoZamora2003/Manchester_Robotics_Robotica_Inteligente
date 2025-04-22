import rclpy
import numpy as np
import signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy import qos

class ErrorCalculator(Node):
    def __init__(self):
        super().__init__('error_calculator')

        # Parámetros reales
        self.distancia_real = 3.0  # metros
        self.angulo_real = 90.0    # grados

        # Suscripción a odometría
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)

        self.get_logger().info("Nodo de Cálculo de Errores iniciado.")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Calcular distancia
        distancia_medida = np.hypot(x, y)

        # Calcular ángulo (yaw)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        angulo_medido = np.degrees(theta)

        # Calcular errores
        error_distancia = self.distancia_real - distancia_medida
        error_angulo = self.angulo_real - angulo_medido

        error_distancia_pct = (error_distancia / self.distancia_real) * 100
        error_angulo_pct = (error_angulo / self.angulo_real) * 100

        # Mostrar resultados
        self.get_logger().info(f"Distancia medida: {distancia_medida:.3f} m | Error: {error_distancia:.3f} m ({error_distancia_pct:.2f}%)")
        self.get_logger().info(f"Ángulo medido: {angulo_medido:.2f}° | Error: {error_angulo:.2f}° ({error_angulo_pct:.2f}%)")

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = ErrorCalculator()
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
