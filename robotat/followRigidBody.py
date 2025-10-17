#!/usr/bin/env python3

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from crazyflie_interfaces.msg import Status

from crazyflie_py import Crazyswarm
import numpy as np

# Parámetros de vuelo
DEFAULT_CF_NUMBER = 8  # Número del Crazyflie
DEFAULT_RB_NAME = 'RigidBody75' # Nombre del cuerpo rigido
Z = 0.3  # Altura de vuelo en metros
OFFSET = [0.0, 0.0, 0.0]  # Offset adicional a la posición objetivo
TAKEOFF_DURATION = 3.0  # Duración del despegue en segundos
HOVER_DURATION = 0.0    # Tiempo de espera en la posición objetivo en segundos


def main():
    N_CYCLES = 10
    velocity = 0.3

    # rclpy.init()
    # Usar el nodo de Crazyswarm para la suscripción
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    node = swarm.allcfs

    # Declarar y obtener parámetros
    node.declare_parameter("cf_number", DEFAULT_CF_NUMBER)
    node.declare_parameter("rigid_body_name", DEFAULT_RB_NAME)
    node.declare_parameter("offset", OFFSET)

    node.cf_number = node.get_parameter("cf_number").value
    node.rigid_body_name = node.get_parameter("rigid_body_name").value
    node.offset = node.get_parameter("offset").value

    # Variables para almacenar posiciones
    cf_position = None
    rb_position = None
    node.battery_voltage = None

    # Callback para la suscripción
    def poses_callback(msg):
        nonlocal cf_position, rb_position
        for named_pose in msg.poses:
            if named_pose.name == f'cf{node.cf_number}':
                cf_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])
            if named_pose.name == node.rigid_body_name:
                rb_position = np.array([
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

    # Suscribirse al topico de status
    node.create_subscription(Status, f'{cf}/status', status_callback, 10)

    cf = node.crazyfliesByName[f'cf{node.cf_number}']

    while rclpy.ok() and node.battery_voltage is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    
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

    # ---- Despegue ----
    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    # ---- Bucle de seguimiento ----
    for i in range(N_CYCLES):
        # Esperar hasta recibir ambas posiciones
        cf_position = None
        rb_position = None
        while rclpy.ok() and (cf_position is None or rb_position is None):
            rclpy.spin_once(node, timeout_sec=0.1)

        goal = np.array(rb_position)
        goal[2] = max(Z, goal[2])  # Seguir altura de cuerpo rigido o altura minima
        goal = goal + np.array(node.offset)

        distance = np.linalg.norm(goal - cf_position)
        goto_duration = max(distance / velocity, 1.0)

        print(f'Ciclo {i+1}/{N_CYCLES} -> Moviendo hacia [x: {goal[0]:.3f} y: {goal[1]:.3f} z: {goal[2]:.3f}]')

        cf.goTo(goal, yaw=0.0, duration=goto_duration + HOVER_DURATION)
        timeHelper.sleep(goto_duration + HOVER_DURATION)

    # ---- Aterrizaje ----
    cf.land(targetHeight=0.02, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    # Finalizar ROS
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
