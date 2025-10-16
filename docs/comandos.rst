Comandos
========

Crazyswarm2 es un paquete de ROS2, por lo cual, todas las herramientas y comandos se ejecutan por medio de la terminal. A continuación se describen los principales comandos utilizados para ejecutar Crazyswarm2.

Crazyflie Server
----------------

El comando ``ros2 launch crazyflie launch.py backend:=cflib`` inicia el servidor de Crazyswarm2, que es responsable de gestionar la comunicación con los drones Crazyflie. Este comando debe ejecutarse antes de iniciar cualquier rutina de vuelo o comando relacionado con los drones. En este caso se utiliza el backend ``cflib``, que es el backend de python que presento mejores resultados en el proyecto.

.. image:: img/NodosCrazyswarm2.png
    :width: 600px
    :align: center
    :alt: Nodos de Crazyswarm2

Rutinas individuales 
--------------------
Para ejecutar las rutinas individuales, se utiliza el comando ``ros2 run robotat <nombre_rutina>``. Cada rutina es un nodo independiente de ROS2 que puede ser ejecutado por separado. Para incluir un parametro al momento de ejecutar el nodo, se utiliza la siguiente sintaxis: ``ros2 run robotat <nombre_rutina> --ros-args -p <parametro1>:=<valor1> -p <parametro2>:=<valor2>``. Los parametros disponibles para cada rutina se describen en la documentación de las rutinas.

Rutinas de enjambre
-------------------

Para ejecutar las rutinas de enjambre, se utiliza el comando ``ros2 launch robotat <nombre_rutina_enjambre>.launch.py``. Cada rutina de enjambre es un archivo launch que inicia multiples nodos de rutina individual, uno por cada dron en el enjambre. Los parametros para cada nodo se definen en el archivo ``crazyflies_robotat.yaml``.

cfGui.py
--------

La interfaz grafica no se inicia como un nodo de ROS2, sino que se ejecuta por separado. Para iniciar la interfaz grafica, se utiliza el comando ``python3 cfGui.py`` desde la carpeta ``robotat`` o desde VsCode. Esta interfaz permite monitorear el estado de los drones y visualizar la posición de los drones en tiempo real.

