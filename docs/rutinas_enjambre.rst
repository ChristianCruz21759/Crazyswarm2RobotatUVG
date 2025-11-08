Rutinas de vuelo en enjambre
===============================

Se modificaron las rutinas individuales para poder controlar mas de un solo dron, esto por medio de ciclos que envien los mismos comandos a los drones seleccionados. Cada dron cuenta con diferentes parametros, los cuales se definen en el archivo ``crazyflies_robotat.yaml``. Las rutinas desarrolladas fueron funciones simples para verificar el correcto funcionamiento de Crazyswarm2, dejando espacio a implementar rutinas mas complejas en el futuro.

multi_hover
-----------

La rutina ``multi_hover`` permite que todos los drones en el enjambre mantengan una posición fija en el espacio. Al ejecutar este nodo, cada dron despega y se posiciona en una altura predeterminada en el mismo lugar donde se encuentra, manteniéndose estable en esa posición durante un tiempo.

multi_goToInitialPosition
-------------------------
La rutina ``multi_goToInitialPosition`` permite que todos los drones en el enjambre se desplacen a su posicion inicial definida en ``crazyflies.yaml``. Al ejecutar este nodo, cada dron despega y vuela hacia su posicion inicial. Esta rutina se utilizo para reiniciar la flota de drones pues estos deben encontrarse en su posicion inicial para iniciar el **Crazyflie Server**.

multi_goToRigidBody
-------------------

La rutina ``multi_goToRigidBody`` permite que todos los drones en el enjambre se desplacen a la posicion de un cuerpo rigido especifico. Al ejecutar este nodo, cada dron vuela hacia la posicion del cuerpo rigido definido con un offset definido en los parametros.

multi_followRigidBody
---------------------

La rutina ``multi_followRigidBody`` permite que todos los drones en el enjambre sigan a un cuerpo rigido especifico. Al ejecutar este nodo, cada dron ajusta su posicion en tiempo real para mantenerse cerca del cuerpo rigido definido con un offset definido en los parametros.
