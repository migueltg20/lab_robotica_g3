# Proyecto TurtleBot3 con Algoritmo VFF y Detección de Obstáculos

En este README encontrarán todo lo necesario para probar la eficacia de nuestro algoritmo VFF junto con la detección de obstáculos para hacer que el TurtleBot3 alcance puntos en el espacio según se le vaya indicando mientras que evita de forma reactiva obstáculos simples.

## Instrucciones de Lanzamiento

Para lanzar el proyecto, dentro de la carpeta `catkin_ws`, sigan estos pasos:

1. Lanzar el entorno de simulación:
    ```bash
    roslaunch challenge_evaluation robotics_challenge_reactive.launch
    ```


Una vez lanzada la simulación y con los nodos funcionando, deberán añadir en RVIZ el topic "visualization_marker" que publica "MarkerArray" para visualizar los vectores del algoritmo VFF.
