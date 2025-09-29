#!/usr/bin/env python3

# GO TO ORIGIN
# Este script obtiene la posición de un Crazyflie usando ROS2,
# y mueve el dron al origen (0,0), realizando una secuencia de despegue, vuelo y aterrizaje.

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

from crazyflie_py import Crazyswarm
import numpy as np

# Parámetros de vuelo
DEFAULT_CF_NUMBER = 8  # Número del Crazyflie
Z = 0.3  # Altura de vuelo en metros
OFFSET = np.array([0.0, 0.0, 0.0])  # Offset adicional a la posición objetivo
TAKEOFF_DURATION = 3.0  # Duración del despegue en segundos
HOVER_DURATION = 2.0    # Tiempo de espera en la posición objetivo en segundos

def main():
    swarm = Crazyswarm() # Inicializar Crazyswarm y ROS2
    timeHelper = swarm.timeHelper
    node = swarm.allcfs

    # Declarar y obtener parámetros
    node.declare_parameter("cf_number", DEFAULT_CF_NUMBER)
    node.cf_number = node.get_parameter("cf_number").value

    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    # Suscribirse al tópico de poses
    node.create_subscription(
        NamedPoseArray,
        '/poses',
        poses_callback,
        qos_profile
    )

    node.cf_position = None

    def poses_callback(msg):
        for named_pose in msg.poses:
            if named_pose.name == f'cf{node.cf_number}':
                node.cf_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])

    cf = node.crazyfliesByName[f'cf{node.cf_number}'] # Obtener el Crazyflie por su nombre

    # Esperar hasta recibir la posición
    while rclpy.ok() and node.cf_position is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'Posición del cf{node.cf_number} [x: {node.cf_position[0]:.3f} y: {node.cf_position[1]:.3f} z: {node.cf_position[2]:.3f}]')

    # Calcular posición objetivo - origen (0,0)
    goal = np.array([0.0, 0.0, Z]) + OFFSET

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
