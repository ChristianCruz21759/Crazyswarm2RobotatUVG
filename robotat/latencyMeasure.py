#!/usr/bin/env python3

# LATENCY MEASURE
# Este script mide la ultima latencia enviada por cada dron del enjambre y guarda el historico en un archivo csv, los drones van del cf1 a cf10

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import Status
import csv
import os
import time

NUM_DRONES = 10
CSV_FILE = '10 drones 2 antenas 2.csv'
TIMEOUT_SECONDS = 2.0
READ_INTERVAL = 0.1     # segundos entre lecturas
TOTAL_DURATION = 30.0   # segundos de registro

class LatencyLogger(Node):
    def __init__(self, num_drones):
        super().__init__('latency_logger')

        self.start_time = time.time()
        self.num_drones = num_drones
        self.drone_ids = [f'cf{i}' for i in range(1, num_drones + 1)]
        self.latency = {drone_id: 0 for drone_id in self.drone_ids}
        self.last_status_time = {drone_id: time.time() for drone_id in self.drone_ids}
        self.latency_records = {drone_id: [] for drone_id in self.drone_ids}  # guardar todas las lecturas

        self.get_logger().info("Iniciando medicion...")

        # Crear encabezado del CSV si no existe
        if not os.path.exists(CSV_FILE):
            with open(CSV_FILE, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.drone_ids)

        # Suscribirse a todos los drones
        for drone_id in self.drone_ids:
            self.create_subscription(Status, f'{drone_id}/status', self.make_status_callback(drone_id), 10)

        # Temporizadores
        self.create_timer(READ_INTERVAL, self.save_latencies_to_csv)
        self.create_timer(READ_INTERVAL, self.check_status_timeout)

    def make_status_callback(self, drone_id):
        def callback(msg):
            self.latency[drone_id] = msg.latency_unicast
            self.last_status_time[drone_id] = time.time()
        return callback

    def check_status_timeout(self):
        now = time.time()
        for drone_id in self.drone_ids:
            if now - self.last_status_time[drone_id] > TIMEOUT_SECONDS:
                self.latency[drone_id] = 0

    def save_latencies_to_csv(self):
        elapsed = time.time() - self.start_time
        if elapsed >= TOTAL_DURATION:
            self.get_logger().info("Tiempo de medición finalizado.")

            # Calcular promedios
            print("\n--- Promedio de latencias ---")
            for drone_id in self.drone_ids:
                values = self.latency_records[drone_id]
                if values:
                    avg = sum(values) / len(values)
                else:
                    avg = 0
                print(f"{drone_id}: {avg:.2f} ms")

            # Cerrar nodo y rclpy
            self.destroy_node()
            rclpy.shutdown()
            return

        # Guardar fila en CSV y en memoria
        row = [self.latency[drone_id] for drone_id in self.drone_ids]
        with open(CSV_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

        # Guardar en la lista para promedios
        for drone_id in self.drone_ids:
            self.latency_records[drone_id].append(self.latency[drone_id])

def main():
    rclpy.init()
    node = LatencyLogger(NUM_DRONES)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
