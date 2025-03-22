#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion,
                               Twist, TwistWithCovariance, Vector3)
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32MultiArray

def quaternion_from_yaw(yaw):
    """Genera un cuaternión (solo en z) a partir de un ángulo yaw (en radianes)."""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('ackerman_odo')
        
        # Parámetros (ajustar según tu robot)
        # Radio de la rueda (en metros)
        self.wheel_radius = (6.5 / 2) / 100.0  # 6.5 cm / 2
        # Distancia entre ejes (wheelbase) (en metros)
        self.wheelbase_length = 195 / 1000.0  # 195 mm

        # Estado del robot: x, y y theta (orientación en radianes)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Tiempo del último callback (para integración)
        self.last_time = self.get_clock().now()

        # Publicador de odometría (sin covarianza)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Suscripción a los datos del Arduino
        self.create_subscription(
            Float32MultiArray,
            '/arduino_data',
            self.arduino_callback,
            10
        )

    def arduino_callback(self, msg: Float32MultiArray):
        """
        Se espera que msg.data contenga:
         - Índice 0: ángulo del servo en grados (85° = sin giro)
         - Índice 1: velocidad de la rueda izquierda (en rpm)
         - Índice 3: velocidad de la rueda derecha (en rpm, se invierte el signo)
        """
        data = msg.data
        if len(data) < 4:
            self.get_logger().error("Datos insuficientes recibidos desde Arduino.")
            return

        # Extraer datos
        servo_angle_deg = data[0]
        left_rpm = data[2]
        right_rpm = -data[4]  # se invierte el signo

        # Convertir el ángulo del servo a ángulo de giro efectivo (en radianes)
        # Se define que 85° corresponde a 0 radianes de giro.
        effective_steering_angle = math.radians(servo_angle_deg - 85.0)

        # Convertir rpm a m/s:
        # velocidad (m/s) = rpm * (2*pi/60) * wheel_radius
        v_left = left_rpm * (2 * math.pi / 60.0) * self.wheel_radius
        v_right = right_rpm * (2 * math.pi / 60.0) * self.wheel_radius

        # Velocidad lineal del vehículo (promedio de las dos ruedas)
        v = (v_left + v_right) / 2.0

        # Calcular velocidad angular:
        # Si no hay giro (ángulo cerca de 0), w = 0; de lo contrario, se estima el radio de giro.
        if abs(effective_steering_angle) < 1e-6:
            w = 0.0
        else:
            # Modelo simple: radio de giro = wheelbase / tan(ángulo efectivo)
            turn_radius = self.wheelbase_length / math.tan(effective_steering_angle)
            w = v / turn_radius

        # Calcular el tiempo transcurrido
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Integrar (método de Euler) para actualizar la posición y orientación
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Construir el mensaje Odometry
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose
        odom.pose.pose = Pose()
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = quaternion_from_yaw(self.theta)
        # Como no es necesario la covarianza, se dejan en 0.
        # Twist
        odom.twist.twist = Twist()
        odom.twist.twist.linear = Vector3(x=v, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=w)

        # Publicar el mensaje
        self.odom_pub.publish(odom)

        # Imprimir datos para depuración
        self.get_logger().info(
            f"Servo: {servo_angle_deg:.2f}°, Effective Angle: {effective_steering_angle:.3f} rad, "
            f"v_left: {v_left:.3f} m/s, v_right: {v_right:.3f} m/s, "
            f"v: {v:.3f} m/s, w: {w:.3f} rad/s, Pose: ({self.x:.3f}, {self.y:.3f}, {self.theta:.3f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
