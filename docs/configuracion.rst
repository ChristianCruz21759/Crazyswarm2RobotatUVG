Configuracion
===============================

crazyflies.yaml
-----------------------------
El archivo ``crazyflies.yaml`` contiene la configuración específica para los drones Crazyflie utilizados en el sistema. Este archivo define parámetros como la dirección del radio, posiciones iniciales y tipo de robot. En este proyecto se utilizan diferentes numeros de antena y canales de radio para evitar interferencias entre los drones, y el mismo tipo de drone (cf21_single_marker) para todos ellos.

.. code-block:: yaml

    robots:
        cf0:
            enabled: false                      # Habilitar o deshabilitar este drone
            uri: radio://0/80/2M/E7E7E7E7EA     # URI "radio:// Numero de Crayradio // Canal / Velocidad de datos / Dirección del Crazyflie"
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

En el archivo ``server.yaml`` se definen configuraciones para las alertas del Crazyflie Server y configuraciones relacionadas con la simulacion. En este caso solo se modifica la frecuencia de las alertas para el sistema de captura de movimiento y la comunicacion con los drones.

.. code-block:: yaml

  ros__parameters:
    warnings:
      frequency: 10.0 # report/run checks once per second
      motion_capture:
        warning_if_rate_outside: [80.0, 120.0]
      communication:
        max_unicast_latency: 30.0 # ms
        min_unicast_ack_rate: 0.9
        min_unicast_receive_rate: 0.9 # requires status topic to be enabled
        min_broadcast_receive_rate: 0.9 # requires status topic to be enabled
        publish_stats: false

Modulos Crazyradio PA
---------------------

Para la comunicacion entre el computador y los drones Crazyflie se utilizan modulos Crazyradio PA, los cuales permiten una comunicacion estable y de largo alcance. Estos modulos se conectan al computador por medio de un puerto USB y se configuran en el archivo ``crazyflies.yaml``. Para poder utilizar los Crazyradio PA es necesario seguir los pasos de instalacion y configuracion descritos en la documentacion oficial de Bitcraze, disponibles en el siguiente enlace: `Crazyradio PA Installation and USB Permissions <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/>`_.

Ademas de seguir estos pasos es necesario actualizar el firmware de la Crazyradio PA al incluido en Crazyswarm2, el cual se encuentra en la carpeta ``/ros2_ws/src/crazyswarm2/prebuilt``. Los pasos para actualizar el firmware se encuentran en la documentacion oficial de Bitcraze, disponibles en el siguiente enlace: `Updating the Crazyradio PA firmware <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/crazyradio/>`_.