#!/usr/bin/env python3

# HOVER MULTIPLE
# Este script obtiene la posición de varios Crazyflie usando ROS2,
# y mueve los drones al origen (0,0), realizando una secuencia de despegue, vuelo y aterrizaje.

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import Status
from rclpy.qos import QoSProfile, ReliabilityPolicy

from crazyflie_py import Crazyswarm
import numpy as np
import yaml
import os

# Parámetros de vuelo generales
Z = 0.4  # Altura de vuelo en metros
TAKEOFF_DURATION = 3.0  # Duración del despegue en segundos
HOVER_DURATION = 3.0    # Tiempo de espera en la posición objetivo en segundos

# Ruta del archivo YAML
YAML_PATH = os.path.join(
        # get_package_share_directory('robotat'),
        '/home/cruz/ros2_ws/src/robotat/'
        'config',
        'crazyflies_robotat.yaml')


def load_drone_config(yaml_path):
    """Carga la configuración YAML y devuelve una lista de drones habilitados con su offset."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    drones_enabled = []
    for drone in data.get('drones', []):
        if drone.get('enabled', False):
            drones_enabled.append({
                'cf_number': drone['cf_number'],
                'offset': drone.get('offset', [0.0, 0.0, 0.0])
            })
    return drones_enabled


def main():
    # Cargar configuración de drones
    drones_config = load_drone_config(YAML_PATH)
    CF_NUMBERS = [d['cf_number'] for d in drones_config]
    OFFSETS = {d['cf_number']: np.array(d['offset']) for d in drones_config}

    if not CF_NUMBERS:
        print("No hay drones habilitados en el archivo YAML. Abortando misión.")
        return

    print(f"Drones habilitados: {CF_NUMBERS}")

    # Inicializar Crazyswarm y ROS2
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    node = swarm.allcfs

    # Variables para almacenar posiciones y voltajes
    node.cf_positions = {cf_number: np.array([0.0, 0.0, 0.0]) for cf_number in CF_NUMBERS}
    node.battery_voltages = {cf_number: None for cf_number in CF_NUMBERS}

    # --- Callbacks ---
    def poses_callback(msg):
        for named_pose in msg.poses:
            for cf_number in CF_NUMBERS:
                if named_pose.name == f'cf{cf_number}':
                    node.cf_positions[cf_number] = np.array([
                        named_pose.pose.position.x,
                        named_pose.pose.position.y,
                        named_pose.pose.position.z
                    ])

    def status_callback(msg, cf_number):
        node.battery_voltages[cf_number] = msg.battery_voltage

    # --- QoS Profile ---
    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    # Suscribirse a los tópicos
    node.create_subscription(NamedPoseArray, '/poses', poses_callback, qos_profile)
    for cf_number in CF_NUMBERS:
        node.create_subscription(
            Status,
            f'cf{cf_number}/status',
            lambda msg, cf_number=cf_number: status_callback(msg, cf_number),
            qos_profile
        )

    # --- Esperar datos ---
    print("Esperando datos de voltaje...")
    while rclpy.ok() and not all(v is not None for v in node.battery_voltages.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Comprobar baterías
    for cf_number in CF_NUMBERS:
        voltage = node.battery_voltages[cf_number]
        print(f'Batería del cf{cf_number}: {voltage:.2f} V')
        if voltage <= 3.5:
            print(f'Nivel crítico de batería en cf{cf_number}. Abortando.')
            node.destroy_node()
            rclpy.shutdown()
            return
        elif voltage <= 3.7:
            print(f'Nivel bajo de batería en cf{cf_number}. Recomendado cargar antes del vuelo.')
            node.destroy_node()
            rclpy.shutdown()
            return

    print("Esperando posiciones de los drones...")
    while rclpy.ok() and not all(p is not None for p in node.cf_positions.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Obtener objetos Crazyflie
    cfs = [node.crazyfliesByName[f'cf{cf_number}'] for cf_number in CF_NUMBERS]

    # Mostrar posiciones y definir objetivos
    goals = {}
    for cf_number in CF_NUMBERS:
        pos = node.cf_positions[cf_number]
        offset = OFFSETS[cf_number]
        # goal = np.array([pos[0], pos[1], Z]) + offset
        goal = np.array([pos[0], pos[1], Z])
        goals[cf_number] = goal
        print(f'cf{cf_number}: posición inicial [x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}] > objetivo [x={goal[0]:.2f}, y={goal[1]:.2f}, z={goal[2]:.2f}]')

    # --- Secuencia de vuelo ---
    print("Iniciando secuencia de vuelo...")

    # Despegue
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    # Ir a la posición objetivo
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.goTo(goals[cf_number], yaw=0.0, duration= 1.0 + HOVER_DURATION)
    timeHelper.sleep(1.0 + HOVER_DURATION)

    # Aterrizaje
    for cf in cfs:
        cf.land(targetHeight=0.025, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    print("Secuencia finalizada. Drones aterrizados.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
