#!/usr/bin/env python3
import rclpy
import numpy as np
import signal
from rclpy.node import Node
from rclpy import qos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # —————————————————————
     # 1) PARÁMETROS PID AJUSTADOS
        self.Kp_d, self.Ki_d, self.Kd_d = 2.0, 0.005, 0.10   # distancia
        self.Kp_th, self.Ki_th, self.Kd_th = 2.0, 0.01, 0.005 # ángulo

        # Integrales y errores previos
        self.e_d_int   = 0.0
        self.e_th_int  = 0.0
        self.prev_e_d  = 0.0
        self.prev_e_th = 0.0

        # Límites de velocidad
        self.max_v     = 0.25   # m/s
        self.max_w     = 0.30   # rad/s
        self.min_v     = 0.05   # zona muerta lineal
        self.min_w     = 0.05   # zona muerta angular

        # Pose actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Objetivo (waypoint)
        self.gx = None
        self.gy = None

        # —————————————————————
        # 2) LÓGICA DE SEMÁFORO SIMPLIFICADA
        # 🔴→ frena, 🟡→ velocidad fija, 🟢→ v_cmd
        self.color_state      = None
        self.state_start_time = None

        # Parámetros de rampa para verde
        self.ramp_duration = 1.0    # s para aceleración verde
        self.slow_factor   = 0.1    # velocidad mínima en verde (10%)

        # Velocidad fija en amarillo (15% de v_cmd)
        self.yellow_factor = 0.15

        # —————————————————————
        # 3) SUSCRIPCIONES Y PUBLICADORES
        self.sub_odom = self.create_subscription(
            Odometry, 'corrected_odom', self.odom_cb,
            qos.qos_profile_sensor_data)
        self.sub_goal = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_cb,
            qos.qos_profile_sensor_data)
        self.sub_color = self.create_subscription(
            String, 'detected_color', self.color_cb, 10)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        # nuevos publishers de error
        self.pub_error_distance = self.create_publisher(Float32, 'error_distance', 10)
        self.pub_error_angle    = self.create_publisher(Float32, 'error_angle', 10)

        # Control loop a 20 Hz
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Controller iniciado → semáforo simplificado activo.')

    # —————————————————————————————————————————
    def odom_cb(self, msg: Odometry):
        """Actualiza la pose del robot."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.theta = np.arctan2(siny, cosy)

    # —————————————————————————————————————————
    def goal_cb(self, msg: PoseStamped):
        """Actualiza el waypoint objetivo y resetea el PID."""
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        self.e_d_int   = 0.0
        self.e_th_int  = 0.0
        self.prev_e_d  = 0.0
        self.prev_e_th = 0.0

    # —————————————————————————————————————————
    def color_cb(self, msg: String):
        """Recibe el color detectado y actualiza estado."""
        prev = self.color_state
        self.color_state = None if msg.data == 'none' else msg.data
        if self.color_state != prev:
            self.get_logger().info(f'🔔 Color cambiado a: {self.color_state}')
            self.state_start_time = self.get_clock().now()

    # —————————————————————————————————————————
    def control_loop(self):
        """Bucle de control: PID + semáforo + publicación de errores."""
        # 1) Si no hay objetivo, frena
        if self.gx is None:
            self.pub_cmd.publish(Twist())
            # publicar error cero
            err_d = Float32(data=0.0)
            err_th = Float32(data=0.0)
            self.pub_error_distance.publish(err_d)
            self.pub_error_angle.publish(err_th)
            return

        # 2) Cálculo de errores
        ex, ey = self.gx - self.x, self.gy - self.y
        e_d     = np.hypot(ex, ey)
        theta_d = np.arctan2(ey, ex)
        e_th    = (theta_d - self.theta + np.pi) % (2*np.pi) - np.pi
        dt      = 0.05

        # Publicar errores
        err_d_msg  = Float32()
        err_d_msg.data = float(e_d)
        self.pub_error_distance.publish(err_d_msg)
        err_th_msg = Float32()
        err_th_msg.data = float(e_th)
        self.pub_error_angle.publish(err_th_msg)

        # 3) PID ángulo → w_cmd
        self.e_th_int += e_th * dt
        de_th = (e_th - self.prev_e_th) / dt
        w_cmd = (self.Kp_th*e_th
               + self.Ki_th*self.e_th_int
               + self.Kd_th*de_th)
        self.prev_e_th = e_th

        # 4) PID distancia → v_cmd
        self.e_d_int += e_d * dt
        self.e_d_int = np.clip(self.e_d_int, -0.5, 0.5)
        de_d = (e_d - self.prev_e_d) / dt
        v_cmd = (self.Kp_d*e_d
               + self.Ki_d*self.e_d_int
               + self.Kd_d*de_d)
        self.prev_e_d = e_d

        # 5) Ajustes por orientación y zonas muertas
        v_cmd *= np.exp(-5 * abs(e_th))
        if abs(e_th) > np.deg2rad(10):
            v_cmd = 0.0
        if abs(w_cmd) < self.min_w and abs(e_th) > np.deg2rad(2):
            w_cmd = np.sign(w_cmd) * self.min_w

        # 6) Lógica de semáforo simplificada
        if   self.color_state == 'red':
            v_after = 0.0
        elif self.color_state == 'yellow':
            v_after = 0.15
        else:  # green o None
            v_after = v_cmd

        # 7) Saturaciones finales
        if abs(v_after) < self.min_v:
            v_after = 0.0
        v_after = float(np.clip(v_after, 0.0, self.max_v))
        w_after = float(np.clip(w_cmd, -self.max_w, self.max_w))

        # 8) Publicar cmd_vel
        twist = Twist()
        twist.linear.x  = v_after
        twist.angular.z = w_after
        self.pub_cmd.publish(twist)

    # —————————————————————————————————————————
    def stop_handler(self, sig, frame):
        """Detiene el nodo limpiamente con Ctrl+C."""
        self.get_logger().info('Controller detenido.')
        raise SystemExit

def main():
    rclpy.init()
    node = Controller()
    signal.signal(signal.SIGINT, node.stop_handler)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
