#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Configuración del teclado
settings = termios.tcgetattr(sys.stdin)

# Mensaje de instrucciones
msg = """
Control del TurtleBot con el teclado
-------------------------------------
W: Avanzar    S: Retroceder
A: Girar izq  D: Girar der
ESPACIO: Detener

Q/E: Aumentar/Disminuir velocidad lineal
Z/C: Aumentar/Disminuir velocidad angular

CTRL+C para salir
"""

# Diccionario de controles de movimiento
moveBindings = {
    'w': (1, 0),  # Adelante
    's': (-1, 0),  # Atrás
    'a': (0, 1),  # Girar izquierda
    'd': (0, -1),  # Girar derecha
}

# Diccionario de ajuste de velocidades
speedBindings = {
    'q': (0.1, 0),  # Aumentar velocidad lineal
    'e': (-0.1, 0),  # Disminuir velocidad lineal
    'z': (0, 0.1),  # Aumentar velocidad angular
    'c': (0, -0.1)   # Disminuir velocidad angular
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
    Nodo de ROS 2 para el control del TurtleBot mediante teclado.
    Captura las teclas presionadas y publica los comandos de velocidad en el tópico correspondiente.
    """
    
    def __init__(self):
        super().__init__('ackerman_teleop')
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)

        print(msg)

        self.speed = 0.7  # Velocidad lineal inicial
        self.turn = 0.7   # Velocidad angular inicial
        self.x = 0  # Dirección lineal
        self.th = 0  # Dirección angular
        self.last_key_pressed = None  # Última tecla procesada

        # Configurar un temporizador para ejecutar la función teleop periódicamente
        self.timer_ = self.create_timer(0.01, self.teleop)

    def teleop(self):
        """ Captura la entrada del teclado y publica los comandos de velocidad solo cuando cambian. """
        key = getKey()

        # Solo actualiza si la tecla presionada cambió
        if key in moveBindings:
            self.x , self.th = moveBindings[key]
            self.publish_velocity()
        elif key in speedBindings:
            self.speed += speedBindings[key][0]
            self.turn += speedBindings[key][1]
            print(f"Velocidad lineal: {self.speed:.2f}, Velocidad angular: {self.turn:.2f}")
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
        twist.linear.x = self.x * self.speed
        twist.angular.z = self.th * self.turn
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