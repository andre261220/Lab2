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
