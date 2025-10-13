from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    drones = [5, 7, 8, 9, 10]  # Lista de cf_number
    offsets = [
        [-0.5, 0.5, 0.0],
        [0.5, 0.0, 0.0],
        [-0.5, 0.0, 0.0],
        [0.0, 0.5, 0.0],
        [0.0, -0.5, 0.0],
    ]

    nodes = []
    for i, cf in enumerate(drones):
        nodes.append(
            Node(
                package="robotat",
                executable="goToRigidBody",
                name=f"goTorb{cf}",
                output="screen",
                parameters=[{
                    "cf_number": cf,
                    "offset": offsets[i],
                }],
            )
        )

    return LaunchDescription(nodes)
