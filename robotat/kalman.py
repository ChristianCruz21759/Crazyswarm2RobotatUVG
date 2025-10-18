#!/usr/bin/env python3

# LECTURA DE FILTRO DE KALMAN
# Script para verificar el funcionamiento del filtro de kalman versus el sistema de captura, para que este codigo funcione debe activarse el firmware logger en el archivo crazyflies.yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import csv
import matplotlib.pyplot as plt
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric  # Agrega esta importación
from rclpy.timer import Timer

CF_NAME = 'cf10'  # Nombre del Crazyflie
READ_FREQ_HZ = 50.0   # Frecuencia de lectura en Hz

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        self.object_name = CF_NAME 

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Suscripción a /poses
        self.subscription_pose = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.listener_callback_pose,
            qos_profile
        )
        # Suscripción a /kalman_state_estimate usando LogDataGeneric
        self.subscription_kalman = self.create_subscription(
            LogDataGeneric,
            f'/{CF_NAME}/kalman_state_estimate',
            self.listener_callback_kalman,
            qos_profile
        )

        self.pose_data = []      # Solo una lista para el objeto
        self.kalman_data = []

        self.csv_file = 'object_pose_vs_kalman.csv'
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            header = [f'x_{self.object_name}_pose', f'y_{self.object_name}_pose', f'z_{self.object_name}_pose',
                      'x_kalman', 'y_kalman', 'z_kalman']
            writer.writerow(header)

        self.latest_pose = (None, None, None)
        self.latest_kalman = (None, None, None)

        self.get_logger().info(f'Suscrito a /poses y /kalman_state_estimate, escribiendo a {self.csv_file}')

        # Timer para lectura periódica
        self.timer = self.create_timer(1.0 / READ_FREQ_HZ, self.periodic_save_row)

    def listener_callback_pose(self, msg):
        for named_pose in msg.poses:
            if named_pose.name == self.object_name:
                x = named_pose.pose.position.x
                y = named_pose.pose.position.y
                z = named_pose.pose.position.z
                self.latest_pose = (x, y, z)
                self.get_logger().info(f'MoCap: x={x:.2f}, y={y:.2f}, z={z:.2f}')

    def listener_callback_kalman(self, msg):
        if len(msg.values) >= 3:
            x = msg.values[0]
            y = msg.values[1]
            z = msg.values[2]
            self.latest_kalman = (x, y, z)
            self.get_logger().info(f'Kalman: x={x:.2f}, y={y:.2f}, z={z:.2f}')
        else:
            self.get_logger().warn('El mensaje de kalman_state_estimate no tiene suficientes valores (se esperaban al menos 3)')

    def periodic_save_row(self):
        if self.latest_pose != (None, None, None) and self.latest_kalman != (None, None, None):
            row = list(self.latest_pose) + list(self.latest_kalman)
            self.pose_data.append(self.latest_pose)
            self.kalman_data.append(self.latest_kalman)
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row)
            # Imprime en tiempo real
            print(f"Pose: x={self.latest_pose[0]:.3f}, y={self.latest_pose[1]:.3f}, z={self.latest_pose[2]:.3f} | "
                  f"Kalman: x={self.latest_kalman[0]:.3f}, y={self.latest_kalman[1]:.3f}, z={self.latest_kalman[2]:.3f}")

    def plot_data(self):
        if not self.pose_data or not self.kalman_data:
            print("No hay datos para graficar.")
            return

        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        t_pose = range(len(self.pose_data))
        t_kalman = range(len(self.kalman_data))

        # Graficar datos de pose
        if self.pose_data:
            x_vals, y_vals, z_vals = zip(*self.pose_data)
            axs[0].plot(t_pose, x_vals, label=f'{self.object_name} pose')
            axs[1].plot(t_pose, y_vals, label=f'{self.object_name} pose')
            axs[2].plot(t_pose, z_vals, label=f'{self.object_name} pose')

        # Graficar datos de kalman
        if self.kalman_data:
            xk, yk, zk = zip(*self.kalman_data)
            axs[0].plot(t_kalman, xk, label='kalman', linestyle='-')
            axs[1].plot(t_kalman, yk, label='kalman', linestyle='-')
            axs[2].plot(t_kalman, zk, label='kalman', linestyle='-')

        axs[0].set_ylabel('X')
        axs[1].set_ylabel('Y')
        axs[2].set_ylabel('Z')
        axs[2].set_xlabel('Muestra')
        for ax in axs:
            ax.legend()
            ax.grid(True)
        plt.suptitle('Comparación Pose vs Kalman')
        plt.tight_layout()
        plt.show()

def main():
    rclpy.init()
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
