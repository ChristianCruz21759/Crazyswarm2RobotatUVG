# Crazyswarm2RobotatUVG

## Implementación de un enjambre de drones Crazyflie 2.1 mediante Crazyswarm2 y ROS2 dentro del ecosistema Robotat  

### Trabajo de graduación por Christian Cruz

En este proyecto se desarrolla la implementación de un enjambre de drones Crazyflie 2.1 en el ecosistema robótico Robotat en la Universidad del Valle de Guatemala (UVG) mediante Crazyswarm2 y ROS2. Crazyswarm2 es un paquete de ROS2 creado para controlar los agentes robóticos de Bitcraze, es un proyecto de código abierto y disponible en el siguiente [GitHub](https://github.com/IMRCLab/crazyswarm2). El siguiente artículo describe el proyecto original de Crazyswarm y es la base de todo el proyecto.

```bibtex
@inproceedings{crazyswarm,
  author    = {James A. Preiss* and
               Wolfgang  H\"onig* and
               Gaurav S. Sukhatme and
               Nora Ayanian},
  title     = {Crazyswarm: {A} large nano-quadcopter swarm},
  booktitle = {{IEEE} International Conference on Robotics and Automation ({ICRA})},
  pages     = {3299--3304},
  publisher = {{IEEE}},
  year      = {2017},
  url       = {https://doi.org/10.1109/ICRA.2017.7989376},
  doi       = {10.1109/ICRA.2017.7989376},
  note      = {Software available at \url{https://github.com/USC-ACTLab/crazyswarm}},
}
```

Se logro implementar un enjambre de 10 Crazyflies 2.1, utilizando el sistema de captura de movimiento Optitrack en el ecosistema Robotat, para esta implementacion se utilizo un marcador individual por dron demostrando facilidad de uso respecto a un cuerpo rigido. Ademas de implementar las rutinas de ejemplo incluidas en Crazyswarm2 se desarrollaron rutinas personalizadas las cuales se encuentran en este repositorio. Ademas de las rutinas se incluye una interfaz grafica para monitorear el estado de todos los drones dentro del enjambre y documentacion para la correcta configuracion en el laboratorio de robotica en la UVG.

![Enjambre de drones](img/EnjambreCrazyflies.jpg)

