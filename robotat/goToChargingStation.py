#!/usr/bin/env python3

# GO TO CHARGING STATION
# Este script obtiene la posición de un Crazyflie y un cuerpo rígido (RigidBody) usando ROS2,
# y mueve el dron a la posición del cuerpo rígido, realizando una secuencia de despegue, vuelo y aterrizaje.

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import Status
from rclpy.qos import QoSProfile, ReliabilityPolicy
from crazyflie_py import Crazyswarm
import numpy as np

# Parámetros de vuelo
DEFAULT_CF_NUMBER = 1  # Número del Crazyflie
DEFAULT_CB_NUMBER = 1  # Número de la estación de carga
Z = 0.3  # Altura de vuelo en metros
OFFSET = [0.0, -0.5, 0.0]  # Offset adicional a la posición objetivo
TAKEOFF_DURATION = 3.0  # Duración del despegue en segundos
HOVER_DURATION = 3.0    # Tiempo de espera en la posición objetivo en segundos

def main():
    # Inicializar Crazyswarm y ROS2
    swarm = Crazyswarm() 
    timeHelper = swarm.timeHelper
    node = swarm.allcfs

    # Declarar y obtener parámetros
    node.declare_parameter("cf_number", DEFAULT_CF_NUMBER)
    node.declare_parameter("charging_station_number", DEFAULT_CB_NUMBER)
    node.declare_parameter("offset", OFFSET)

    node.cf_number = node.get_parameter("cf_number").value
    node.charging_station_number = node.get_parameter("charging_station_number").value
    node.offset = node.get_parameter("offset").value

    # Obtener el Crazyflie por su nombre
    cf = node.crazyfliesByName[f'cf{node.cf_number}']

    # Variables para almacenar posiciones y estado
    node.cf_position = None
    node.charging_station_position = None
    node.battery_voltage = None

    # --- Callbacks ---
    def poses_callback(msg):
        for named_pose in msg.poses:
            if named_pose.name == f'cf{node.cf_number}':
                node.cf_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])
            if named_pose.name == f'ChgBase{node.charging_station_number}':
                node.charging_station_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])

    def status_callback(msg):
        node.battery_voltage = msg.battery_voltage

     # --- QoS Profile ---
    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    # Suscribirse al tópico de poses
    node.create_subscription(NamedPoseArray, '/poses', poses_callback, qos_profile)

    # Suscribirse al topico de status
    node.create_subscription(Status, f'{cf}/status', status_callback, 10)

    # --- Esperar datos de bateria ---
    while rclpy.ok() and node.battery_voltage is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'Batería del cf{node.cf_number}: {node.battery_voltage} V')
    if node.battery_voltage <= 3.5:
        print('Nivel crítico de batería. El vuelo no es seguro, cargar batería manualmente')
        node.destroy_node()
        rclpy.shutdown()
        return
    elif node.battery_voltage <= 3.65:
        print('Nivel bajo de batería. Enviar a estación de carga')
        node.destroy_node()
        rclpy.shutdown()
        return

    # --- Esperar datos de posicion ---
    while rclpy.ok() and (node.cf_position is None or node.charging_station_position is None):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Mostrar posiciones y definir objetivo
    goal = np.array(node.charging_station_position)
    goal[2] = Z # Altura fija
    goal = goal + np.array(node.offset)
    print(f'cf{node.cf_number}: posición inicial [x={node.cf_position[0]:.2f}, y={node.cf_position[1]:.2f}, z={node.cf_position[2]:.2f}] -> objetivo [x={goal[0]:.2f}, y={goal[1]:.2f}, z={goal[2]:.2f}]')

    distance = np.linalg.norm(goal - node.cf_position)
    velocity = 0.2
    goto_duration = max(distance/velocity, 1.0)

    # --- Secuencia de vuelo ---
    print("Iniciando secuencia de vuelo...")

    # Despegue
    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    # Ir a la posicion objetivo
    cf.goTo(goal, yaw=0.0, duration=goto_duration)
    timeHelper.sleep(goto_duration + HOVER_DURATION)

    # Aterrizaje
    cf.land(targetHeight=0.02, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    print("Secuencia finalizada. Drones aterrizados.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
