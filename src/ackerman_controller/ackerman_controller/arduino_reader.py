#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        # Publicador al tópico /arduino_data como Float32MultiArray
        self.publisher_ = self.create_publisher(Float32MultiArray, '/arduino_data', 10)

        # Configuración del puerto serial
        self.serial_port = "/dev/ttyACM0"
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Puerto serie {self.serial_port} abierto a {self.baud_rate} baudios")
            
        except Exception as e:
            self.get_logger().error(f"Error al abrir el puerto serie: {e}")
            return

        # Timer para leer cada 10ms
        self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                # Convertir la línea a lista de floats
                float_values = [float(val) for val in line.split(',')]
                msg = Float32MultiArray()
                msg.data = float_values
                self.publisher_.publish(msg)
                self.get_logger().info(f"Recibido: {float_values}")
        except ValueError as ve:
            self.get_logger().error(f"Error de conversión: {ve}")
        except Exception as e:
            self.get_logger().error(f"Error al leer puerto serie: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
