# Lab2
Reporte de Laboratorio 2-Diseño de Sistemas Roboticos.

## Introducción a ROS.

ROS (Robot Operating System) es un marco de trabajo de código abierto para el desarrollo de software de robots. Proporciona una infraestructura de sistemas distribuidos para ayudar en el desarrollo, ejecución y gestión de aplicaciones robóticas.

### Caracteristicas principales de ROS

1. Arquitectura modular: ROS está diseñado con un enfoque modular, lo que significa que está compuesto por un conjunto de paquetes independientes que pueden ser utilizados, modificados y compartidos de forma flexible.
2. Comunicación entre procesos: Utiliza un sistema de mensajería para permitir la comunicación entre los diferentes componentes de un sistema robótico, incluso si están ejecutándose en diferentes computadoras.
3. Herramientas de desarrollo: Proporciona una amplia variedad de herramientas para el desarrollo de software, incluyendo herramientas de visualización, simulación, depuración y análisis.
4. Soporte multiplataforma: ROS es compatible con múltiples sistemas operativos, incluyendo Linux, macOS y Windows.

### Cómo utilizar ROS con Python:

ROS admite varios lenguajes de programación, incluyendo C++, Python y otros. Aquí hay algunos conceptos básicos sobre cómo utilizar ROS con Python:

1. Instalación de ROS: Primero, necesitas instalar ROS en tu sistema. Puedes seguir las instrucciones de instalación en el sitio web oficial de ROS para la distribución de Linux que estés utilizando. Tutorial seguido en clase para la instalacion de ROS: [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. Crear un paquete: En ROS, el desarrollo de software se organiza en paquetes. Puedes crear un nuevo paquete utilizando el comando catkin_create_pkg y especificando sus dependencias. Tutorial seguido en clase para la instalacion de ROS: [Creating a ROS Package](https://wiki.ros.org/ROS/Tutorials/CreatingPackage)
3. Escribir nodos: Los nodos son procesos individuales que realizan tareas específicas dentro de un sistema robótico. Puedes escribir nodos en Python utilizando la biblioteca **'rospy'**.
4. Publicación y suscripción de mensajes: ROS utiliza un sistema de mensajería para permitir que los nodos se comuniquen entre sí. Puedes publicar y suscribirte a mensajes utilizando los métodos proporcionados por **'rospy'**.
5. Ejecución de nodos: Una vez que hayas escrito tus nodos, puedes ejecutarlos utilizando el comando **'rosrun'** o lanzarlos dentro de un archivo de lanzamiento (launch file) utilizando **'roslaunch'**.
6. Herramientas de ROS: Además de escribir tu propio código, puedes utilizar las numerosas herramientas proporcionadas por ROS, como Rviz para la visualización, Gazebo para la simulación y RViz para la depuración.

Con ROS y Python, se puede desarrollar una amplia variedad de aplicaciones robóticas, desde sistemas de control simples hasta sistemas complejos de percepción y planificación.

## Problemas a resolver

**Basic**
- Crear un paquete llamado Practicas_lab de ros con dependencias rospy, roscpp y std_msgs
- Colocar los archivos listener.py y talker.py
- Compilar el paquete.
- Ejecutar el talker y listener.

**Medium**  
- Crear un control por teclado para turtlesim
- Dibujar un cuadrado y un triángulo equilátero con turtlesim (Sin controlador)

**Advanced**
- Control de posición para turtlesim (P)
- Control de posición para turtlesim (PI)
- Control de posición para turtlesim (PID)
- Comparar el desempeño de cada uno de los controladores, mediante el uso de Plot Juggler o alguna otra herramienta de graficación.
- Reportar en markdown.

## Codigos y soluciones

Para el primer problema se pueden encontrar los codigos listener.py y talker.py en la carpeta practica_lab2 >> src, a continuacion mostraremos una imagen de la ejecucion y funcionamiento de ambos programas en conjunto: 
![BasicRos](https://github.com/andre261220/Lab2/assets/132303647/86ca3d26-3359-41ee-86b1-4dbf8e5c0ec2)

Para los problemas de nivel medio se realizaron los siguientes codigos (los cuales tambien se encuentran en la carpeta carpeta practica_lab2 >> src).

Los codigos que se mostraran a continuacion son para dibujar un triangulo equilatero y un cuadrado utilizando turtlesim.

```python
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi

# Función para dibujar un cuadrado
def dibujar_cuadrado(pub, rate):
    for _ in range(4):
        # Avanzar hacia adelante
        pub.publish(Twist(linear=Vector3(x=2.0), angular=Vector3(z=0.0)))
        rate.sleep()
        # Girar 90 grados a la izquierda
        pub.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=pi/2)))
        rate.sleep()

# Función para dibujar un triángulo equilátero
def dibujar_triangulo(pub, rate):
    for _ in range(3):
        # Avanzar hacia adelante
        pub.publish(Twist(linear=Vector3(x=2.0), angular=Vector3(z=0.0)))
        rate.sleep()
        # Girar 120 grados a la izquierda
        pub.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=2*pi/3)))
        rate.sleep()

if __name__ == '__main__':
    try:
        # Inicialización del nodo ROS
        rospy.init_node('turtlesim_shapes')
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(0.5)  # Velocidad de dibujo

        # Dibujar un cuadrado
        dibujar_cuadrado(pub, rate)

        # Esperar un momento antes de dibujar el siguiente
        rospy.sleep(1)

        # Reiniciar la posición del robot
        pub.publish(Twist())
        rospy.sleep(1)

        # Dibujar un triángulo equilátero
        dibujar_triangulo(pub, rate)

        # Mantener el programa ejecutándose
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

```python
#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import math


class TeleopTurtleSim:
    def __init__(self):
        rospy.init_node('tri', anonymous=False)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)
        self.twist = Twist()
        self.rate = rospy.Rate(1)  # Reducido a 1Hz para facilitar la visualización
        self.key = None
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def teleopLoop(self):
        rospy.loginfo("Presiona Ctrl-C para salir y observa cómo la tortuga forma un triángulo equilátero.")

        for _ in range(4):
            # Avanzar hacia adelante
            self.twist.linear.x = 1.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
            self.rate.sleep()

            # Detenerse
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
            self.rate.sleep()

            # Girar 120 grados hacia la izquierda
            self.twist.linear.x = 0.0
            self.twist.angular.z = math.radians(120)  # Convertir 120 grados a radianes
            self.pub.publish(self.twist)
            self.rate.sleep()

            # Detenerse
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
            self.rate.sleep()

        rospy.loginfo("Triángulo equilátero completado. Saliendo del bucle.")


if __name__ == '__main__':
    teleop_turtlesim = TeleopTurtleSim()
    try:
        teleop_turtlesim.teleopLoop()
    except rospy.ROSInterruptException:
        pass
```

Estos códigos utilizan rospy para comunicarse con el nodo de turtlesim y controlar el movimiento de la tortuga. Define dos funciones, dibujar_cuadrado y dibujar_triangulo, que utilizan comandos de movimiento para dibujar un cuadrado y un triángulo equilátero respectivamente.

Cada función envía comandos de movimiento a la tortuga utilizando el publicador pub, que controla el movimiento lineal y angular. La velocidad de dibujo está configurada por rate, que define la frecuencia a la que se envían los comandos de movimiento.

Simulacion:

![TrianguloTurtlesim](https://github.com/andre261220/Lab2/assets/132303647/ef1b5c0b-e92b-44e1-813e-1104c768ebf4)
![cuboTurltesim](https://github.com/andre261220/Lab2/assets/132303647/a569225d-8a98-4982-bb88-437ff49f49ae)

Igualmente se genero un control por teclado para el turtlesim, mostaremos a continuación el código que realizamos para realizar el problema dando una breve explicacion de su funcionamiento y su simulacion.

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

KEYS= ["w", "a", "s", "d"]

class TeleopTurtleSim:
    def __init__(self):
        rospy.init_node('little_turle', anonymous=False)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.rate = rospy.Rate(10)  # 10Hz
        self.key = None
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def teleopLoop(self):
        rospy.loginfo("Usa las teclas de flecha para controlar el turtle. Presiona Ctrl-C para salir.")

        while not rospy.is_shutdown():
            self.getKey()
            rospy.loginfo("Tecla presionada: {}".format(self.key))  # Mensaje de depuración
            if self.key == '\x03':  # Ctrl-C
                break
            self.processKey()
            self.pub.publish(self.twist)
            self.rate.sleep()

    def processKey(self):
        if self.key == 'w':
            self.twist.linear.x = 1.0
            self.twist.angular.z = 0.0
        elif self.key == 's':
            self.twist.linear.x = -1.0
            self.twist.angular.z = 0.0
        elif self.key == 'a':
            self.twist.linear.x = 0.0
            self.twist.angular.z = 1.0
        elif self.key == 'd':
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.0
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0


if __name__ == '__main__':
    teleop_turtlesim = TeleopTurtleSim()
    try:
        teleop_turtlesim.teleopLoop()
    except rospy.ROSInterruptException:
        pass
```
Este código utiliza rospy para comunicarse con el nodo de turtlesim y controlar el movimiento de la tortuga. Permite controlar la tortuga utilizando las teclas 'w' para avanzar, 'a' para girar a la izquierda, 'd' para girar a la derecha, 's' para detenerse y 'x' para salir del programa.

![ControlTortuga](https://github.com/andre261220/Lab2/assets/132303647/bb868152-3d73-46ce-99b1-79141b976442)

Finalmente realizamos un código para poder tener una solucion al nivel avanzado de introducción a ROS, contruyendo un controlador PID para turtlesim.

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians, sqrt

class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pid')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables para el controlador PID en cada eje
        self.Kp_x = 1
        self.Kp_y = 1
        self.Kp_theta = 1

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        while not rospy.is_shutdown():
            # Calcular los errores de posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            error_theta = desired_theta - self.current_theta
            
            # Calcular las velocidades lineales y angular del movimiento
            vel_x = self.Kp_x * error_x
            vel_y = self.Kp_y * error_y
            vel_theta = self.Kp_theta * error_theta
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            twist_msg.angular.z = vel_theta
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual y los errores en la terminal
            rospy.loginfo("Posición actual: x=%f, y=%f, theta=%f", self.current_x, self.current_y, self.current_theta)
            rospy.loginfo("Errores: ex=%f, ey=%f, etheta=%f", error_x, error_y, error_theta)
            
            # Verificar si se alcanza la posición deseada
            if sqrt(error_x**2 + error_y**2) < 0.1 and abs(error_theta) < radians(5):
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def get_desired_pose_from_user(self):
        print("Ingrese la posición y orientación deseadas:")
        desired_x = float(input("Coordenada x: "))
        desired_y = float(input("Coordenada y: "))
        desired_theta = radians(float(input("Orientación (en grados): ")))
        return desired_x, desired_y, desired_theta

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la pose deseada del usuario
            desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()

            # Mover la tortuga primero en x
            self.move_turtle_to_desired_pose(desired_x, self.current_y, self.current_theta)
            
            # Mover la tortuga luego en y
            self.move_turtle_to_desired_pose(desired_x, desired_y, self.current_theta)

            # Mover la tortuga finalmente en theta
            self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```

Este código crea una clase TurtlesimPositionController que controla la posición de turtlesim utilizando una interfaz de publicación y suscripción de ROS. La función move_to_position(x, y) se encarga de mover la tortuga a la posición especificada (x, y).

El controlador suscribe la posición actual de la tortuga desde el tópico /turtle1/pose y publica comandos de velocidad en el tópico /turtle1/cmd_vel para mover la tortuga hacia la posición objetivo.

La velocidad lineal está determinada por la distancia a la posición objetivo, mientras que la velocidad angular está determinada por el ángulo de rotación necesario para orientar la tortuga hacia la posición objetivo.

![PIDtortuga](https://github.com/andre261220/Lab2/assets/132303647/47f89957-56ee-412e-be18-d77517ebccaa)

Los codigos .py de cada problema se pueden encontrar en la carpeta practica_lab2 adjuntada en este repositorio.
