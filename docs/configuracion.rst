Configuracion
===============================

crazyflies.yaml
-----------------------------
El archivo ``crazyflies.yaml`` contiene la configuración específica para los drones Crazyflie utilizados en el sistema. Este archivo define parámetros como la dirección del radio, la potencia de transmisión y otros ajustes relacionados con la comunicación y el control de los drones. En este proyecto se utilizan diferentes numeros de antena y canales de radio para evitar interferencias entre los drones, y el mismo tipo de drone (cf21_single_marker) para todos ellos.

.. code-block:: yaml

    robots:
        cf0:
            enabled: false                      # Habilitar o deshabilitar este drone
            uri: radio://0/80/2M/E7E7E7E7EA     # Dirección del radio
            initial_position: [0.0, 0.0, 0.0]   # Posición inicial (x, y, z)
            type: cf21_single_marker            # Tipo de robot

El tipo de robot también se define en ``crazyflies.yaml`` y hace referencia a una configuración específica, para este proyecto se utiliza el tipo cf21_single_marker, el cual se encuentra definido en ``motion_capture.yaml``, usando **librigidbodytracker** para el tracking de los drones en lugar de usar el treacker específico del fabricante, en nuestro caso Motive.

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

En este archivo tambien se definen configuraciones globales para todos los drones, como la frecuencia de actualización de la posicion y estado, topicos personalizados, tipo de controlador y estimador, etc.

motion_capture.yaml
-----------------------------

El archivo ``motion_capture.yaml`` contiene la configuración del sistema de captura de movimiento utilizado para rastrear la posición y orientación de los drones en el espacio. Este archivo define parámetros como la frecuencia de actualización, el tipo de sistema de captura de movimiento y otros ajustes relacionados con la integración del sistema de captura de movimiento con Crazyswarm2. Para este proyecto se cuenta con las siguientes configuraciones:

- **Tipo:** optitrack_closed_source, esta configuracion representa al sistema de captura de movimiento OptiTrack, utilizando las versiones mas recientes del software Motive.
- **Hostname:** esta configuracion define la direccion IP del computador donde se esta ejecutando el software de captura de movimiento, segun la infraestructura del Robotat esta direccion es la direccion del router ubicado en el laboratorio.

.. code-block:: yaml

    ros__parameters:
        # one of "optitrack", "optitrack_closed_source", "vicon", "qualisys", "nokov", "vrpn", "motionanalysis"
        type: "optitrack_closed_source"
        # Specify the hostname or IP of the computer running the motion capture software
        hostname: "192.168.50.200"
        # mode: "motionCapture"
        topics:
        poses:
            qos:
            mode: "sensor"
            deadline: 100.0 # Hz

En este mismo archivo se definen las configuraciones de los marcadores utilizados para el tracking de los drones. En este proyecto se utiliza un marcador por dron, el cual esta definido como cf_single_marker, y cuenta con un offset en z para considerar la altura del marcador respecto al centro del dron.

.. code-block:: yaml

        marker_configurations:
            cf_single_marker:   
                offset: [0.0, 0.0, 0.0]
                points:
                    p0: [0.0, 0.0, 0.023]

Por ultimo se definen las configuraciones de dinamica del dron, las cuales son utilizadas por el controlador para limitar la velocidad y aceleracion del dron. En este proyecto se utiliza la configuracion default, la cual define limites de velocidad y aceleracion adecuados para los drones Crazyflie 2.1.

.. code-block:: yaml

    dynamics_configurations:
      default:
        max_velocity: [1.5, 1.5, 1.0]         
        max_angular_velocity: [7, 7, 5]       
        max_roll: 0.7                        # ≈ 40°
        max_pitch: 0.7                        # ≈ 40°
        max_fitness_score: 0.001 

server.yaml
-----------------------------

