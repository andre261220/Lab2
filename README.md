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

Para los problemas de nivel medio se realizaron los siguientes codigos (los cuales tambien se encuentran en la carpeta carpeta practica_lab2 >> src):

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
