#!/usr/bin/env python3

# FOLLOW RIGID BODY
# Este script obtiene la posición de un Crazyflie y un cuerpo rígido (RigidBody) usando ROS2,
# y mueve el dron a la posición del cuerpo rígido durante un numero de ciclos, realizando una secuencia de despegue, vuelo y aterrizaje.

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import Status
from rclpy.qos import QoSProfile, ReliabilityPolicy
from crazyflie_py import Crazyswarm
import numpy as np
import yaml
import os

# Parámetros de vuelo
# DEFAULT_CF_NUMBER = 1  # Número del Crazyflie
DEFAULT_RB_NAME = 'RigidBody69' # Nombre del cuerpo rigido
Z = 0.3  # Altura de vuelo en metros
# OFFSET = [0.3, -0.3, 0.0]  # Offset adicional a la posición objetivo
TAKEOFF_DURATION = 3.0  # Duración del despegue en segundos
HOVER_DURATION = 0.0    # Tiempo de espera en la posición objetivo en segundos
N_CYCLES = 30

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

    # Declarar y obtener parámetros
    # node.declare_parameter("cf_number", DEFAULT_CF_NUMBER)
    node.declare_parameter("rigid_body_name", DEFAULT_RB_NAME)
    # node.declare_parameter("offset", OFFSET)

    # node.cf_number = node.get_parameter("cf_number").value
    node.rigid_body_name = node.get_parameter("rigid_body_name").value
    # node.offset = node.get_parameter("offset").value

    # Obtener objetos Crazyflie por nombre
    cfs = [node.crazyfliesByName[f'cf{cf_number}'] for cf_number in CF_NUMBERS]

    # Variables para almacenar posiciones y estado
    rb_position = None
    node.cf_positions = {cf_number: np.array([0.0, 0.0, 0.0]) for cf_number in CF_NUMBERS}
    node.battery_voltages = {cf_number: None for cf_number in CF_NUMBERS}

    # --- Callbacks ---
    def poses_callback(msg):
        nonlocal rb_position
        for named_pose in msg.poses:
            if named_pose.name == node.rigid_body_name:
                rb_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])
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

    # Suscribirse al tópico de poses
    node.create_subscription(NamedPoseArray, '/poses', poses_callback, qos_profile)

    # Suscribirse al topico de status
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

    # --- Secuencia de vuelo ---
    print("Iniciando secuencia de vuelo...")

    # Despegue
    for cf, cf_number in zip(cfs, CF_NUMBERS):
        cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    # ---- Bucle de seguimiento ----
    for i in range(N_CYCLES):

        # --- Esperar datos de posicion --- 
        node.cf_positions = {cf_number: None for cf_number in CF_NUMBERS}
        rb_position = None

        # --- Esperar datos de posicion ---
        while rclpy.ok() and (any(p is None for p in node.cf_positions.values()) or rb_position is None):
            rclpy.spin_once(node, timeout_sec=0.1)

        # Mostrar posiciones y definir objetivos
        goals = {}
        goto_durations = {}
        for cf, cf_number in zip(cfs, CF_NUMBERS):
            # pos = node.cf_positions[cf_number]
            # cf__pos = node.cf_positions[cf_number]
            pos = rb_position
            print(pos)
            offset = OFFSETS[cf_number]
            goal = np.array([pos[0], pos[1], max(pos[2], Z)]) + offset
            # goal = np.array([pos[0], pos[1], Z])

            distance = np.linalg.norm(goal - node.cf_positions[cf_number])
            velocity = 0.3
            goto_durations[cf_number] = max(distance / velocity, 1.0)

            goals[cf_number] = goal
            # print(f'cf{cf_number}: posición inicial [x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}] > objetivo [x={goal[0]:.2f}, y={goal[1]:.2f}, z={goal[2]:.2f}]')

            print(f'Ciclo {i+1}/{N_CYCLES} cf{cf_number} -> Moviendo hacia [x: {goal[0]:.3f} y: {goal[1]:.3f} z: {goal[2]:.3f}]')
            
        # Ir a la posición objetivo
        for cf, cf_number in zip(cfs, CF_NUMBERS):
            cf.goTo(goals[cf_number], yaw=0.0, duration= goto_durations[cf_number] + HOVER_DURATION)
        timeHelper.sleep(1.0 + HOVER_DURATION)

    # Aterrizaje
    for cf in cfs:
        cf.land(targetHeight=0.025, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + 3.0)

    print("Secuencia finalizada. Drones aterrizados.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
