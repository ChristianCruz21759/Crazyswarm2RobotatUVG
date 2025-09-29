import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import csv
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from motion_capture_tracking_interfaces.msg import NamedPoseArray

# Lista de nombres de objetos a rastrear
OBJECT_NAMES = ['cf5']  # Modifica según tus objetos

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        self.object_names = OBJECT_NAMES

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.listener_callback,
            qos_profile
        )
        self.pose_data = {name: [] for name in self.object_names}

        # Archivo CSV de salida
        self.csv_file = 'objects_pose.csv'
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Encabezado: x_objeto1, y_objeto1, z_objeto1, x_objeto2, y_objeto2, z_objeto2, ...
            header = []
            for name in self.object_names:
                header += [f'x_{name}', f'y_{name}', f'z_{name}']
            writer.writerow(header)

        self.get_logger().info(f'Suscrito a /poses y escribiendo a {self.csv_file} para los objetos: {self.object_names}')

    def listener_callback(self, msg):
        # Diccionario para almacenar la posición actual de cada objeto
        current_positions = {name: (None, None, None) for name in self.object_names}

        for named_pose in msg.poses:
            if named_pose.name in self.object_names:
                x = named_pose.pose.position.x
                y = named_pose.pose.position.y
                z = named_pose.pose.position.z
                current_positions[named_pose.name] = (x, y, z)
                self.pose_data[named_pose.name].append((x, y, z))
                self.get_logger().info(f'Pose de {named_pose.name}: x={x:.2f}, y={y:.2f}, z={z:.2f}')

        # Solo guarda si al menos un objeto fue encontrado en el mensaje
        if any(pos != (None, None, None) for pos in current_positions.values()):
            row = []
            for name in self.object_names:
                row += list(current_positions[name])
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row)

    def plot_data(self):
        if not any(self.pose_data.values()):
            print("No hay datos para graficar.")
            return

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for name, poses in self.pose_data.items():
            if poses:  # Asegurarse de que la lista no esté vacía
                x_vals, y_vals, z_vals = zip(*poses)
                ax.plot(x_vals, y_vals, z_vals, marker='o', label=name)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Trayectorias 3D')
        ax.legend()
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
