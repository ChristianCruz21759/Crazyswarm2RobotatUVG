Rutinas de vuelo individuales
===============================

Las rutinas de vuelo individuales son secuencias predefinidas que un solo dron puede ejecutar de manera autónoma. Estas rutinas permiten realizar tareas específicas, como despegar, aterrizar, mantener una posición fija, moverse a una posición inicial, entre otras.

Cada rutina fue desarrollada como un nodo independiente de ROS2, lo que permite leer los topicos de estado y posicion relacionadas a cada Crazyflie, y obtener parametros al momento de ejecutar cada nodo. Dentro de los parametros definidos se incluyen el numero de Crazyflie (cf_number), offsets (offset), nombre de cuerpo rigido (rigid_body_name), entre otros. 

Todas las rutinas cuentan con una estructura similar, que incluye la inicialización del nodo, la suscripción a los topicos necesarios, la configuración de los parametrosy la implementación de la lógica de la rutina. La logica incluye la obtencion del nivel de bateria para verificar que el vuelo se pueda realizar, la obtencion de la posicion actual del dron y meta, calculo de distancia y tiempo adecuado para el movimiento, finalizando con envio de comandos de control para ejecutar la rutina deseada.

A continuación se describen las principales rutinas de vuelo individuales desarrolladas:


hover
-----

La rutina ``hover`` permite que el dron mantenga una posición fija en el espacio. Al ejecutar este nodo, el dron despega y se posiciona en una altura predeterminada en el mismo lugar donde se encuentra, manteniéndose estable en esa posición durante un tiempo. Esta rutina es útil para verificar el funcionamiento del dron y Crazyswarm2.

goToInitialPosition
-------------------

La rutina ``goToInitialPosition`` permite que el dron se desplace a su posicion inicial definida en ``crazyflies.yaml``. Al ejecutar este nodo, el dron despega y vuela hacia su posicion inicial. Esta rutina se utilizo para reiniciar la flota de drones pues estos deben encontrarse en su posicion inicial para iniciar el **Crazyflie Server**.

goToOrigin
----------

La rutina ``goToOrigin`` permite que el dron se desplace a la posicion [0,0,0]. Esta rutina es util para centrar la posicion del dron en el espacio de vuelo.

goToRigidBody
-------------

La rutina ``goToRigidBody`` permite que el dron se desplace a la posicion de un cuerpo rigido especifico. Al ejecutar este nodo, el dron vuela hacia la posicion del cuerpo rigido definido en los parametros. Esta rutina es util para posicionar el dron en una ubicacion especifica.

goToChargingStation
-------------------

La rutina ``goToChargingStation`` permite que el dron se desplace a la posicion de una estacion de carga. Al ejecutar este nodo, el dron vuela hacia la posicion de la estacion de carga. Esta rutina es util para llevar el dron a una ubicacion donde pueda recargar su bateria.

followRigidBody
---------------

La rutina ``followRigidBody`` permite que el dron siga a un cuerpo rigido especifico. Al ejecutar este nodo, el dron ajusta su posicion en tiempo real para mantenerse cerca del cuerpo rigido definido en los parametros. Esta rutina es util para realizar seguimientos de objetos en movimiento.

Otros nodos
-------------

- **graph.** Este nodo se utiliza para obtener la posicion de objetos en el espacio de vuelo y graficar su trayectoria durante el tiempo que el nodo estuvo activo, guardando los datos en un archivo csv.
- **kalman.** Este nodo compara la posicion medida por el sistema de captura de movimiento y la posicion estimada por el filtro de Kalman de un dron individual, graficando ambas posiciones para analizar la precision del filtro.
- **latencyMeasure.** Este nodo mide la latencia entre Crazyswarm2 y los Crazyflies, guardando los datos en un archivo csv para su posterior analisis.