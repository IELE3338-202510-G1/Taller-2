#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class arduino_writer(Node):
    def __init__(self):
        super().__init__('arduino_writer')
        # Suscriptor al tópico /ackerman_cmdVel
        self.subscription = self.create_subscription(
            Twist,
            '/ackerman_cmdVel',
            self.listener_callback,
            10
        )
        # Configuración del puerto serial
        self.serial_port = "/dev/ttyACM0"
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1, rtscts=True)
            self.get_logger().info(f"Puerto serie {self.serial_port} abierto a {self.baud_rate} baudios")
            
        except Exception as e:
            self.get_logger().error(f"Error al abrir el puerto serie: {e}")
            return

    def listener_callback(self, msg):
        vel_motor1 = msg.linear.x
        vel_motor2 = msg.linear.y
        servo_angle = msg.angular.z
        output = f"{vel_motor1:.2f},{-vel_motor2:.2f},{servo_angle:.2f}\n"
        try:
            self.ser.write(output.encode('utf-8'))
            self.ser.flush()  # Asegúrate que se envía
            self.get_logger().info(f"Enviado: {output.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error al enviar datos: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = arduino_writer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
