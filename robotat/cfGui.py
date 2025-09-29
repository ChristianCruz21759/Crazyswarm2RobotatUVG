#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status
from geometry_msgs.msg import Point

from nicegui import ui
import threading
import asyncio
import time

NUM_DRONES = 10  # <-- Cambia este número según la cantidad de drones
TIMEOUT_SECONDS = 1.0

class GuiNode(Node):
    def __init__(self, num_drones):
        super().__init__('multi_cf_node')

        # Diccionarios para almacenar datos por dron
        self.positions = {}
        self.battery_voltages = {}
        self.pm_states = {}
        self.latency = {}
        self.last_status_time = {}

        for i in range(1, num_drones + 1):
            drone_id = f'cf{i}'
            self.reset_drone_data(drone_id)
            self.last_status_time[drone_id] = time.time()

            self.create_subscription(PoseStamped, f'{drone_id}/pose', self.make_pose_callback(drone_id), 10)
            self.create_subscription(Status, f'{drone_id}/status', self.make_status_callback(drone_id), 10)
            
        # Temporizador para verificar timeouts
        self.create_timer(1.0, self.check_status_timeout)

    def reset_drone_data(self, drone_id):
        """Reinicia valores iniciales del dron."""
        self.positions[drone_id] = Point()
        self.battery_voltages[drone_id] = 0.0
        self.pm_states[drone_id] = 5  # 5 = no conectado
        self.latency[drone_id] = 0

    def make_pose_callback(self, drone_id):
        def callback(msg):
            self.positions[drone_id] = msg.pose.position
        return callback

    def make_status_callback(self, drone_id):
        def callback(msg):
            self.battery_voltages[drone_id] = msg.battery_voltage
            self.pm_states[drone_id] = msg.pm_state
            self.latency[drone_id] = msg.latency_unicast
            self.last_status_time[drone_id] = time.time()
        return callback

    def check_status_timeout(self):
        """Verifica si ha pasado demasiado tiempo sin recibir status."""
        now = time.time()
        for drone_id in self.last_status_time:
            if now - self.last_status_time[drone_id] > TIMEOUT_SECONDS:
                # self.get_logger().warn(f"{drone_id}: sin datos de /status por más de {TIMEOUT_SECONDS} s, restaurando valores iniciales.")
                self.reset_drone_data(drone_id)

def battery_status(voltage, pm_state):
    if voltage >= 3.7:
        color = 'green'
    elif voltage >= 3.5:
        color = 'orange'
    else:
        color = 'red'

    status_map = {
        0: 'En funcionamiento',
        1: 'Cargando',
        2: 'Cargada',
        3: 'Baja batería',
        4: 'Apagado',
        5: 'Crazyflie no conectado'
    }
    return status_map.get(pm_state, 'Desconocido'), color

def start_ros_spin(node):
    rclpy.spin(node)

@ui.page('/')
async def main_page():
    ui.label('Estado de Flota Crazyflie - Robotat').classes('text-2xl font-bold mb-4')

    drone_widgets = {}

    with ui.row().classes('w-full gap-4 flex-wrap justify-start'):
        for i in range(1, NUM_DRONES + 1):
            drone_id = f'cf{i}'
            with ui.card().classes('w-72 p-4 shadow-md'):
                ui.label(f'Dron {i} ({drone_id})').classes('text-lg font-semibold mb-1')
                ui.label('📍 Posición (XYZ)').classes('text-sm text-gray-500')
                pos_label = ui.label('X: 0.00 | Y: 0.00 | Z: 0.00').classes('text-sm mb-1')

                ui.label('🔋 Batería').classes('text-sm text-gray-500 mt-2')
                volt_label = ui.label('Voltaje: 0.00 V').classes('text-sm')
                status_label = ui.label('Estado: Desconocido').classes('text-sm')
                progress = ui.linear_progress(value=0.0, show_value=False, size='8px').classes('w-full mt-1')
                latency_label = ui.label('Latencia: 0 ms').classes('text-sm mb-1')

                drone_widgets[drone_id] = {
                    'pos_label': pos_label,
                    'volt_label': volt_label,
                    'status_label': status_label,
                    'progress': progress,
                    'latency_label': latency_label
                }

    map_chart = ui.echart({
        'title': {'text': 'Mapa 2D de Drones'},
        'xAxis': {'min': -1, 'max': 1, 'name': 'X (m)'},
        'yAxis': {'min': -1.5, 'max': 1.5, 'name': 'Y (m)'},
        'series': [{
            'type': 'scatter',
            'symbolSize': 14,
            'data': [],
            'label': {'show': True, 'formatter': '{b}'}
        }],
        'tooltip': {'trigger': 'item'},
    }).classes('w-full max-w-xl h-[500px] mt-6 mx-auto shadow-md')

    async def update():
        while True:
            scatter_data = []
            for i in range(1, NUM_DRONES + 1):
                drone_id = f'cf{i}'
                pos = ros_node.positions[drone_id]
                voltage = ros_node.battery_voltages[drone_id]
                pm_state = ros_node.pm_states[drone_id]
                latency = ros_node.latency[drone_id]

                status, color = battery_status(voltage, pm_state)
                progress_value = max(0.0, min((voltage - 3.0) / (4.23 - 3.0), 1.0))

                widgets = drone_widgets[drone_id]
                widgets['pos_label'].text = f'X: {pos.x:.2f} | Y: {pos.y:.2f} | Z: {pos.z:.2f}'
                widgets['volt_label'].text = f'Voltaje: {voltage:.2f} V'
                widgets['status_label'].text = f'Estado: {status}'
                widgets['status_label'].style(f'color: {color}')
                widgets['progress'].value = progress_value
                widgets['progress'].props(f'color={color}')
                widgets['latency_label'].text = f'Latencia: {latency} ms'

                scatter_data.append({
                    'value': [pos.x, pos.y],
                    'name': drone_id
                })

            map_chart.options['series'][0]['data'] = scatter_data
            map_chart.update()
            await asyncio.sleep(0.5)

    asyncio.create_task(update())

if __name__ in {"__main__", "__mp_main__"}:
    rclpy.init()
    ros_node = GuiNode(NUM_DRONES)
    ros_thread = threading.Thread(target=start_ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    ui.run(title='Crazyflie Dashboard', port=8082)
