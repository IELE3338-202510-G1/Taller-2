#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class arduino_reader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        # Publicador al tópico /arduino_data
        self.publisher_ = self.create_publisher(String, '/arduino_data', 10)

        # Configuración del puerto serial
        self.serial_port = "/dev/ttyACM0"
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Puerto serie {self.serial_port} abierto a {self.baud_rate} baudios")
            
        except Exception as e:
            self.get_logger().error(f"Error al abrir el puerto serie: {e}")
            return

        # Timer para leer cada 50ms
        self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f"Recibido: {line}")
        except Exception as e:
            self.get_logger().error(f"Error al leer puerto serie: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = arduino_reader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
