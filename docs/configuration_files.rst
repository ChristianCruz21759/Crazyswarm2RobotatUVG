Archivos de configuración
===============================

crazyflies.yaml
=====================
El archivo crazyflies.yaml contiene la configuración específica para los drones Crazyflie utilizados en el sistema. Este archivo define parámetros como 
la dirección del radio, la potencia de transmisión y otros ajustes relacionados con la comunicación y el control de los drones. En este proyecto se 
utilizan diferentes numeros de antena y canales de radio para evitar interferencias entre los drones, y el mismo tipo de drone (cf21_single_marker) 
para todos ellos.

.. code-block:: yaml
    robots:
        cf0:
            enabled: false                      # Habilitar o deshabilitar este drone
            uri: radio://0/80/2M/E7E7E7E7EA     # Dirección del radio
            initial_position: [0.0, 0.0, 0.0]   # Posición inicial (x, y, z)
            type: cf21_single_marker            # Tipo de robot

El tipo de robot también se define en crazyflies.yaml y hace referencia a una configuración específica, para este proyecto se utiliza el tipo 
cf21_single_marker, usando librigidbodytracker para el tracking de los drones en lugar de usar el treacker específico del fabricante, en nuestro caso Motive.

.. code-block:: yaml
    robots_types:
        cf21_single_marker:
            motion_capture: 
                tracking: "librigidbodytracker" # one of "vendor", "librigidbodytracker"
                marker: cf_single_marker
                dynamics: default
            big_quad: false
            battery:
                voltage_warning: 3.8  # V
                voltage_critical: 3.7 # V

En este archivo tambien se definen configuraciones globales para todos los drones, como la frecuencia de actualización de la posicion y estado, 
topicos personalizados, tipo de controlador y estimador, etc.

motion_capture.yaml
=====================

server.yaml
=====================

