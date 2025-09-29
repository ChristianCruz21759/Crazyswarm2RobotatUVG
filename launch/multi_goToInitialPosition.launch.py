from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    drones = [4, 5, 9]  # Lista de cf_number que quieras lanzar

    nodes = []
    for cf in drones:
        nodes.append(
            Node(
                package="robotat",
                executable="goToInitialPosition",
                name=f"initial{cf}",       # nombre único por nodo
                output="screen",
                parameters=[{"cf_number": cf}]
            )
        )

    return LaunchDescription(nodes)
