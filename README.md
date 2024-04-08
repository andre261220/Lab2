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
