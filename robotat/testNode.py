#!/usr/bin/env python3

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import Status
from rclpy.qos import QoSProfile, ReliabilityPolicy
from crazyflie_py import Crazyswarm
import numpy as np
import yaml
import os

# Drones 1 10 5 6

# Parámetros de vuelo generales
Z = 0.6  # Altura de vuelo en metros
TAKEOFF_DURATION = 3.0  # Duración del despegue en segundos
HOVER_DURATION = 3.0    # Tiempo de espera en la posición objetivo en segundos

# Ruta del archivo YAML
YAML_PATH = os.path.join(
        '/home/cruz/ros2_ws/src/robotat/'
        'config',
        'crazyflies_robotat.yaml')

# Funcion para cargar la configuracion de cada dron
def load_drone_config(yaml_path):
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
        print("No hay drones habilitados en el archivo YAML.")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"Drones habilitados: {CF_NUMBERS}")

    # Inicializar Crazyswarm y ROS2
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    node = swarm.allcfs

    # Obtener objetos Crazyflie por nombre
    cfs = [node.crazyfliesByName[f'cf{cf_number}'] for cf_number in CF_NUMBERS]

    # Variables para almacenar posiciones y voltajes
    node.cf_positions = {cf_number: np.array([0.0, 0.0, 0.0]) for cf_number in CF_NUMBERS}
    node.battery_voltages = {cf_number: None for cf_number in CF_NUMBERS}

    positions_stage1 = [[0.0, 0.5, Z], [0.5, 0.0, Z], [0.0, -0.5, Z], [-0.5, 0.0, Z]]
    positions_stage2 = [[0.5, 0.0, Z], [0.0, -0.5, Z], [-0.5, 0.0, Z], [0.0, 0.5, Z]]
    positions_stage3 = [[0.0, -0.5, Z], [-0.5, 0.0, Z], [0.0, 0.5, Z], [0.5, 0.0, Z]]
    positions_stage4 = [[-0.5, 0.0, Z], [0.0, 0.5, Z], [0.5, 0.0, Z], [0.0, -0.5, Z]]

    positions_stage6 = [[0.0, 0.0, Z + 0.3], [0.5, 0.0, Z], [0.0, 0.0, Z-0.3], [-0.5, 0.0, Z]]
    positions_stage7 = [[0.0, -0.5, Z], [0.5, 0.0, Z], [0.5, 0.0, Z], [-0.5, 0.0, Z]]


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

    # --- Esperar datos de bateria ---
    while rclpy.ok() and not all(v is not None for v in node.battery_voltages.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    for cf_number in CF_NUMBERS:
        voltage = node.battery_voltages[cf_number]
        print(f'Batería del cf{cf_number}: {voltage:.2f} V')
        if voltage <= 3.5:
            print(f'Nivel crítico de batería en cf{cf_number}. Abortando.')
            node.destroy_node()
            rclpy.shutdown()
            return
        elif voltage <= 3.6:
            print(f'Nivel bajo de batería en cf{cf_number}. Recomendado cargar antes del vuelo.')
            node.destroy_node()
            rclpy.shutdown()
            return

    # --- Esperar datos de posicion ---
    while rclpy.ok() and not all(p is not None for p in node.cf_positions.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    # --- Secuencia de vuelo ---
    print("Iniciando secuencia de vuelo...")

    # Despegue
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    goToDuration = 2.0                              

    # Metas Stage1
    goals = {}
    i = 0
    for cf_number in CF_NUMBERS:
        goal = np.array(positions_stage1[i])
        # goal = np.array([pos[0], pos[1], Z])
        goals[cf_number] = goal
        print(f'{cf_number}: {goal}')
        i += 1

    # Ir a la posición objetivo
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.goTo(goals[cf_number], yaw=0.0, duration= goToDuration)
    timeHelper.sleep(goToDuration)

    # Metas Stage2
    goals = {}
    i = 0
    for cf_number in CF_NUMBERS:
        goal = np.array(positions_stage2[i])
        # goal = np.array([pos[0], pos[1], Z])
        goals[cf_number] = goal
        print(f'{cf_number}: {goal}')
        i += 1

    # Ir a la posición objetivo
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.goTo(goals[cf_number], yaw=0.0, duration= goToDuration)
    timeHelper.sleep(goToDuration)

    # Metas Stage3
    goals = {}
    i = 0
    for cf_number in CF_NUMBERS:
        goal = np.array(positions_stage3[i])
        # goal = np.array([pos[0], pos[1], Z])
        goals[cf_number] = goal
        print(f'{cf_number}: {goal}')
        i += 1

    # Ir a la posición objetivo
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.goTo(goals[cf_number], yaw=0.0, duration= goToDuration)
    timeHelper.sleep(goToDuration)

    # Metas Stage4
    goals = {}
    i = 0
    for cf_number in CF_NUMBERS:
        goal = np.array(positions_stage4[i])
        # goal = np.array([pos[0], pos[1], Z])
        goals[cf_number] = goal
        print(f'{cf_number}: {goal}')
        i += 1

    # Ir a la posición objetivo
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.goTo(goals[cf_number], yaw=0.0, duration= goToDuration)
    timeHelper.sleep(goToDuration)

    # Metas Stage5 (1)
    goals = {}
    i = 0
    for cf_number in CF_NUMBERS:
        goal = np.array(positions_stage1[i])
        # goal = np.array([pos[0], pos[1], Z])
        goals[cf_number] = goal
        print(f'{cf_number}: {goal}')
        i += 1

    # Ir a la posición objetivo
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.goTo(goals[cf_number], yaw=0.0, duration= goToDuration)
    timeHelper.sleep(goToDuration)

    # # Metas Stage6
    # goals = {}
    # i = 0
    # for cf_number in CF_NUMBERS:
    #     goal = np.array(positions_stage6[i])
    #     # goal = np.array([pos[0], pos[1], Z])
    #     goals[cf_number] = goal
    #     print(f'{cf_number}: {goal}')
    #     i += 1

    # # Ir a la posición objetivo
    # for cf, cf_number in zip(cfs, CF_NUMBERS):
    #     cf.goTo(goals[cf_number], yaw=0.0, duration= goToDuration)
    # timeHelper.sleep(goToDuration)

    # # Metas Stage7
    # goals = {}
    # i = 0
    # for cf_number in CF_NUMBERS:
    #     goal = np.array(positions_stage7[i])
    #     # goal = np.array([pos[0], pos[1], Z])
    #     goals[cf_number] = goal
    #     print(f'{cf_number}: {goal}')
    #     i += 1

    # # Ir a la posición objetivo
    # for cf, cf_number in zip(cfs, CF_NUMBERS):
    #     cf.goTo(goals[cf_number], yaw=0.0, duration= goToDuration)
    # timeHelper.sleep(goToDuration)


    # Aterrizaje
    for cf in cfs:
        cf.land(targetHeight=0.025, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    print("Secuencia finalizada. Drones aterrizados.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
