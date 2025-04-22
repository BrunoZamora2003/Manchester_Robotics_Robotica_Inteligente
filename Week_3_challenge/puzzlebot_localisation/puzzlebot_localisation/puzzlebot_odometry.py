import rclpy
import numpy as np
import signal
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import transforms3d

class PuzzlebotOdometry(Node):
    def __init__(self):
        super().__init__('puzzlebot_odometry')

        # Estado del robot
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Parámetros físicos
        self._l = 0.18  # distancia entre ruedas [m]
        self._r = 0.05  # radio de rueda [m]
        self._sample_time = 0.01
        self.rate = 200.0

        # Estado temporal
        self.first = True
        self.start_time = 0.0
        self.last_time = 0.0

        # Velocidades
        self.wr = 0.0
        self.wl = 0.0
        self.V = 0.0
        self.Omega = 0.0

        # Mensaje Odometry
        self.odom_msg = Odometry()

        # Subscripciones
        self.sub_encR = self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, qos.qos_profile_sensor_data)

        # Publicador
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.run)

        self.get_logger().info("Nodo de Odometría iniciado.")

    def encR_callback(self, msg):
        self.wr = msg.data

    def encL_callback(self, msg):
        self.wl = msg.data

    def run(self):
        if self.first:
            self.start_time = self.get_clock().now()
            self.last_time = self.start_time
            self.first = False
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        if dt > self._sample_time:
            # Cinemática diferencial
            v_r = self._r * self.wr
            v_l = self._r * self.wl

            self.V = 0.5 * (v_r + v_l)
            self.Omega = (1.0 / self._l) * (v_r - v_l)

            # Integración para pose
            self.Th += self.Omega * dt
            self.Th = self.wrap_to_pi(self.Th)

            delta_x = self.V * dt * np.cos(self.Th)
            delta_y = self.V * dt * np.sin(self.Th)

            self.X += delta_x
            self.Y += delta_y

            self.last_time = current_time

            # Publicar odometría
            self.publish_odometry(current_time)

    def publish_odometry(self, timestamp):
        q = transforms3d.euler.euler2quat(0, 0, self.Th)

        self.odom_msg.header.stamp = timestamp.to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_footprint'

        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = q[1]
        self.odom_msg.pose.pose.orientation.y = q[2]
        self.odom_msg.pose.pose.orientation.z = q[3]
        self.odom_msg.pose.pose.orientation.w = q[0]

        self.odom_msg.twist.twist.linear.x = self.V
        self.odom_msg.twist.twist.angular.z = self.Omega

        self.odom_pub.publish(self.odom_msg)

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotOdometry()
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
