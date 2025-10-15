from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta del archivo YAML (ajústala según tu estructura de paquete)
    config_path = os.path.join(
        get_package_share_directory('robotat'),
        'config',
        'crazyflies_robotat.yaml'
    )

    # Leer configuración YAML
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    drones = config.get('drones', [])
    nodes = []

    for drone in drones:
        if not drone.get('enabled', False):
            continue  # Saltar los drones deshabilitados

        cf_number = drone['cf_number']
        # offset = drone.get('offset', [0.0, 0.0, 0.0])

        nodes.append(
            Node(
                package='robotat',
                executable='hover',
                name=f"hover{cf_number}",
                output='screen',
                parameters=[{
                    'cf_number': cf_number,
                    # 'offset': offset
                }],
            )
        )

    return LaunchDescription(nodes)
