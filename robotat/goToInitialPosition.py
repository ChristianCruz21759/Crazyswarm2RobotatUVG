#!/usr/bin/env python3

# GO TO INITIAL POSITION
# Este script obtiene la posición de un Crazyflie usando ROS2,
# y mueve el dron a su posición inicial, realizando una secuencia de despegue, vuelo y aterrizaje.

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from crazyflie_interfaces.msg import Status

from crazyflie_py import Crazyswarm
import numpy as np

# Parámetros de vuelo
DEFAULT_CF_NUMBER = 1  # Número del Crazyflie
Z = 0.3  # Altura de vuelo en metros
OFFSET = [0.0, 0.0, 0.0]  # Offset adicional a la posición objetivo
TAKEOFF_DURATION = 3.0  # Duración del despegue en segundos
HOVER_DURATION = 3.0    # Tiempo de espera en la posición objetivo en segundos

def main():
    swarm = Crazyswarm() # Inicializar Crazyswarm y ROS2
    timeHelper = swarm.timeHelper
    node = swarm.allcfs

    # Declarar y obtener parámetros
    node.declare_parameter("cf_number", DEFAULT_CF_NUMBER)
    node.declare_parameter("offset", OFFSET)

    node.cf_number = node.get_parameter("cf_number").value
    node.offset = node.get_parameter("offset").value

    # Variables para almacenar posiciones
    node.cf_position = None
    node.battery_voltage = None

    def poses_callback(msg):
        for named_pose in msg.poses:
            if named_pose.name == f'cf{node.cf_number}':
                node.cf_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])

    def status_callback(msg):
        node.battery_voltage = msg.battery_voltage

    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    # Suscribirse al tópico de poses
    node.create_subscription(NamedPoseArray, '/poses', poses_callback, qos_profile)

    # Suscribirse al tópico de status
    node.create_subscription(Status, f'cf{node.cf_number}/status', status_callback, 10)

    cf = node.crazyfliesByName[f'cf{node.cf_number}'] # Obtener el Crazyflie por su nombre

    print(f'Batería del cf{node.cf_number}: {node.battery_voltage:.2f} V')
    if node.battery_voltage <= 3.5:
        print('Nivel crítico de batería. El vuelo no es seguro, cargar batería manualmente')
        node.destroy_node()
        rclpy.shutdown()
        return
    elif node.battery_voltage <= 3.7:
        print('Nivel bajo de batería. Enviar a estación de carga')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Esperar hasta recibir la posición
    while rclpy.ok() and node.cf_position is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'Posición del cf{node.cf_number} [x: {node.cf_position[0]:.3f} y: {node.cf_position[1]:.3f} z: {node.cf_position[2]:.3f}]')

    # Calcular posición objetivo - posición inicial
    goal = np.array(cf.initialPosition)
    goal[2] = Z # Altura fija
    goal = goal + np.array(node.offset)
    print(f'Posicion objetivo [x: {goal[0]:.3f} y: {goal[1]:.3f} z: {goal[2]:.3f}]')

    distance = np.linalg.norm(goal - node.cf_position)
    velocity = 0.2 # m/s (velocidad baja para precisión)
    goto_duration = max(distance/velocity, 1.0)

    # Secuencia de vuelo
    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    cf.goTo(goal, yaw=0.0, duration=goto_duration)
    timeHelper.sleep(goto_duration + HOVER_DURATION)

    cf.land(targetHeight=0.02, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
