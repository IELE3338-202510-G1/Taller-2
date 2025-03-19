#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Configuración del teclado
settings = termios.tcgetattr(sys.stdin)

# Mensaje de instrucciones
msg = """
Control del Ackerman con el teclado
-------------------------------------
W: Avanzar    S: Retroceder
A/D: Disminuir/Incrementar ángulo de las llantas
ESPACIO: Detener

Q/E: Aumentar/Disminuir velocidad lineal

CTRL+C para salir
"""

# Diccionario de controles de movimiento (solo W y S para avanzar/retroceder)
moveBindings = {
    'w': 1,  # Adelante
    's': -1,  # Atrás
}

# Diccionario de ajuste: q/e para velocidad, z/c para servo
speedBindings = {
    'q': (2, 0),   # Aumentar velocidad lineal
    'e': (-2, 0),  # Disminuir velocidad lineal
    'd': (0, 1),   # Incrementar ángulo del servo (gira más a la izquierda)
    'a': (0, -1)   # Disminuir ángulo del servo (gira más a la derecha)
}

def getKey():
    """ Captura la tecla presionada sin necesidad de presionar Enter """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)  # Espera por una tecla durante 0.1s
    key = sys.stdin.read(1) if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0] else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class ackerman_teleop(Node):
    """
    Nodo de ROS 2 para el control del robot Ackerman mediante teclado.
    Publica un mensaje Twist en el tópico "/ackerman_cmdVel" donde:
      - twist.linear.x y twist.linear.y son la velocidad de las ruedas traseras (iguales).
      - twist.angular.z es el ángulo del servo.
    """
    
    def __init__(self):
        super().__init__('ackerman_teleop')
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/ackerman_cmdVel", 10)

        print(msg)

        # Parámetros de movimiento
        self.speed = 70.0            # Velocidad lineal inicial
        self.servo_angle = 90.0      # Ángulo inicial del servo (entre 60 y 120)
        self.x = 0                 # Dirección de movimiento (1, -1 o 0)
        
        self.last_key_pressed = None  # Para detectar liberación de tecla

        # Configurar un temporizador para ejecutar la función teleop periódicamente
        self.timer_ = self.create_timer(0.01, self.teleop)

    def teleop(self):
        """ Captura la entrada del teclado y publica los comandos de velocidad solo cuando cambian. """
        key = getKey()

        if key in moveBindings:
            # Solo se actualiza el movimiento de las ruedas
            self.x = moveBindings[key]
            self.publish_velocity()
        
        elif key in speedBindings:
            self.speed += speedBindings[key][0]
            self.servo_angle += speedBindings[key][1]
            
            if self.servo_angle > 120:
                self.servo_angle = 120.0
            elif self.servo_angle < 60:
                self.servo_angle = 60.0
            print(f"Velocidad lineal: {self.speed:.2f}, Ángulo del servo: {self.servo_angle:.2f}")
            self.publish_velocity()
        elif key == '\x03':  # Ctrl+C para salir
            self.destroy_node()
            rclpy.shutdown()
            return
        elif key == '' and self.last_key_pressed is not None:  
            # Solo detiene el robot si se suelta una tecla
            self.x , self.th = 0, 0
            self.publish_velocity()

        # Actualizar la última tecla procesada
        self.last_key_pressed = key if key else None

    def publish_velocity(self):
        """ Publica el mensaje de velocidad en el tópico. """
        twist = Twist()
        twist.linear.x = self.x * self.speed  # Velocidad de la rueda izquierda
        twist.linear.y = self.x * self.speed  # Velocidad de la rueda derecha
        twist.angular.z = self.servo_angle      # Ángulo del servo
        self.cmd_vel_pub_.publish(twist)

def main():
    """ Función principal que inicializa el nodo y lo mantiene en ejecución. """
    rclpy.init()
    node = ackerman_teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()