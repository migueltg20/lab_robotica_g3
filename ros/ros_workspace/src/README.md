# Proyecto TurtleBot3 con Algoritmo VFF y Detección de Obstáculos

En este README encontrarán todo lo necesario para probar la eficacia de nuestro algoritmo VFF junto con la detección de obstáculos para hacer que el TurtleBot3 alcance puntos en el espacio según se le vaya indicando mientras que evita de forma reactiva obstáculos simples.

## Instrucciones de Lanzamiento

Para lanzar el proyecto, dentro de la carpeta `catkin_ws`, sigan estos pasos:

1. Lanzar el entorno de simulación:
    ```bash
    roslaunch challenge_evaluation robotics_challenge_reactive.launch
    ```

2. Lanzar los nodos creados:
    ```bash
    roslaunch autonomous_navigation VFF.launch
    ```

En caso de que no les funcione el último `.launch`, ejecuten los siguientes comandos:

```bash
rosrun autonomous_navigation obstacles_detection.py
rosrun autonomous_navigation robot_controller_A.py
```

Una vez lanzada la simulación y con los nodos funcionando, deberán añadir en RVIZ el topic "visualization_marker" que publica "MarkerArray" para visualizar los vectores del algoritmo VFF. Por último, usando "2D Nav Goal", añadan el punto al que deberá ir el robot para comprobar el funcionamiento.