from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    drones = [4, 5]  # Lista de cf_number que quieras lanzar
    offsets = [[0.3, -0.3, 0.0], [0.6, -0.6, 0.0]]

    i = 0
    nodes = []
    for cf in drones:
        nodes.append(
            Node(
                package="robotat",
                executable="followRigidBody",
                name=f"folow-rb-{cf}",       # nombre único por nodo
                output="screen",
                parameters=[{"cf_number": cf, "offset": offsets[i]}]
            )
        )
        i += 1

    return LaunchDescription(nodes)
