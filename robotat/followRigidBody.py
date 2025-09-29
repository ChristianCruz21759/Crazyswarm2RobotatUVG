#!/usr/bin/env python3

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

from crazyflie_py import Crazyswarm
import numpy as np

# Parámetros de vuelo
DEFAULT_CF_NUMBER = 7
DEFAULT_RB_NAME = 'RigidBody71'
Z = 0.3
OFFSET = np.array([0.0, 0.0, 0.0])
TAKEOFF_DURATION = 3.0
HOVER_DURATION = 3.0

def main():
    N_CYCLES = 10
    velocity = 0.3
    cf_number = DEFAULT_CF_NUMBER
    rb_name = DEFAULT_RB_NAME

    # rclpy.init()
    # Usar el nodo de Crazyswarm para la suscripción
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    node = swarm.allcfs
    cf = node.crazyfliesByName[f'cf{cf_number}']

    # Variables para almacenar posiciones
    cf_position = None
    rb_position = None

    # Callback para la suscripción
    def poses_callback(msg):
        nonlocal cf_position, rb_position
        for named_pose in msg.poses:
            if named_pose.name == f'cf{cf_number}':
                cf_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])
            if named_pose.name == rb_name:
                rb_position = np.array([
                    named_pose.pose.position.x,
                    named_pose.pose.position.y,
                    named_pose.pose.position.z
                ])

    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    # Suscribirse al tópico de poses
    subscription = node.create_subscription(
        NamedPoseArray,
        '/poses',
        poses_callback,
        qos_profile
    )

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

        goal = np.array(rb_position) + OFFSET
        goal[2] = Z  # Mantener altura constante

        distance = np.linalg.norm(goal - cf_position)
        goto_duration = max(distance / velocity, 0.5)

        print(f'Ciclo {i+1}/{N_CYCLES} -> Moviendo hacia [x: {goal[0]:.3f} y: {goal[1]:.3f} z: {goal[2]:.3f}]')

        cf.goTo(goal, yaw=0.0, duration=goto_duration)
        timeHelper.sleep(goto_duration + 1.0)

    # ---- Aterrizaje ----
    cf.land(targetHeight=0.02, duration=TAKEOFF_DURATION + Z)
    timeHelper.sleep(TAKEOFF_DURATION + Z)

    # Finalizar ROS
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
