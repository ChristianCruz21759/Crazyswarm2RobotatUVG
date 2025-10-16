.. robotat documentation master file, created by
   sphinx-quickstart on Mon Oct 13 15:20:27 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Documentacion Crazyswarm2RobotatUVG
================================================

En este proyecto se desarrolla la implementacion de un enjambre de drones Crazyflie 2.1 en el ecosistema robotico Robotat en la Universidad del Valle de Guatemala mediante Crazyswarm2 y ROS2. Crazyswarm2 es una paquete de ROS2 creado para controlar los agentes roboticos de Bitcraze, es un proyecto de codigo abierto y disponible en el siguiente `github <https://github.com/IMRCLab/crazyswarm2>`_. El siguiente articulo describe el proyecto original de Crazyswarm y es la base de todo el proyecto.

.. code-block:: bibtex

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

En esta documentacion se describen los pasos para la configuracion de Crazyswarm2 en nuestro entorno, asi como las rutinas de vuelo individuales y de enjambre desarrolladas para el proyecto. Ademas, se incluyen recomendaciones de uso y consideraciones importantes para el correcto funcionamiento del sistema.

.. toctree::
   :maxdepth: 2
   :caption: Contenido:
   
   configuracion
   comandos
   rutinas_individuales
   rutinas_enjambre

