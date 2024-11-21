# Simulación de Apertura y Servicio de Vino con Robots

## Descripción

Este proyecto tiene como objetivo programar y simular dos brazos manipuladores, uno que coge una botella de vino y lo sirve en un vaso y otro que coge el vaso ya servido y lo deposita para entregarselo a una persona. Para ello, se ha modelado una botella de vino y un vaso de manera simplificada, asignando a cada objeto un manipulador robótico. Se ha logrado un resultado sadisfactorio, aunque surgieron algunos problemas, los cuales se detallan a continuación.

## Desarrollo y Problemas

### Modelado de Objetos y Robots

La botella de vino y el vaso fueron modelados de manera básica mediante la unión de cilindros y conos, así como la diferencia con otros cilindros más pequeños para los huecos.  En la simulación se utilizaron dos robots ABB IRB 120, colocados uno frente a otro: uno para manipular la botella y el otro para sujetar el vaso. Para el segundo robot se creó una nueva estación vinculada a él para controlarlo.

### Herramientas y Funciones Utilizadas

1. **Grippers:** Se cambió la herramienta original de los robots, el soldador, por la herramienta "ABB smart grippers", la variante de 'servo fingers', perteneciente a la librería predeterminada de Robotstudio. Esta herramienta con forma de 'pinzas' tiene dos 'dedos' que son controlados por unos servo motores, controlando la distancia entre los dos 'dedos', abriendose y cerrandose con el objetivo de coger objetos. En su máxima amplitud la 'pinza' se abre 5 cm. Al gripper de cada robot se le han añadido una posición abierta (máxima amplitud) y una cerrada con el tamaño del objeto en la zona elegida para el agarre.
 
2. **Acciones de Agarre y Desagarre:** En la configuración de la simulación, dentro del apartado de 'manejo de eventos' se han creados dos eventos de coger y solar objeto, accionados por el valor 0 o 1 de una señal digital, creada por nosotros, llamada 'Gripper'. Cada evento tiene dos acciones: el gripper adopta la posición de abierto o cerrado y las acciones de "attach" y "detach" del simulador. La acción de 'attach' permite que la herramienta conecte con el objeto más cercano, botella o vaso en este caso, y se muevan juntos hasta que se ejecute 'detach'. 

### Problemas Encontrados

1. **Referencias de Puntos de Trabajo:** Uno de los desafíos fue la referencia de puntos de trabajo al sistema de la botella. Esto ocasionó que, si la botella cambiaba de posición, no se podía retornar al mismo punto debido a la pérdida de orientación respecto al sistema de coordenadas del mundo. Este problema fue solucionado añadiendo puntos con el sistema de referencia global donde se devuelven los objetos depués de ser movidos. La mayoría de puntos estan referidos a los objetos con el objetivo de ser capaz de funcionar aunque se cambie la posición inicial de los objetos.

2. **Movimiento conciso de los objetos:** Al principio, utilizando 'joint movement' el robot movía los objetos con giros indeseados, considerando que el objeto es una botella abierta. Para una mayor adecuación a la situación, se han modificado algunos movimiento a tipo 'linear' para un moviento más suave. Además, hemos considerado el choque de los objetos, especificamente hemos girando la botella un poco antes de bajar a servir en el vaso.

3. **Modelado del Tapón con Rosca:** Aunque era posible modelar un tapón de rosca funcional, se consideró una tarea compleja que añadiría poco valor al resultado final de la simulación. Se decidió simplificar este aspecto y hacer que uno de los manipuladores sujetara la botella mientras el otro robot sostenía el vaso.
   
4. **Acción simultanea:** Se han encontrado problemas a la hora de ejecutar las trayectorias de ambos robots a la vez. Aunque teníamos varias ideas para la sincronización de los movimientos, unas más simples y otras más complejas, finalmente ha sido por falta de tiempo para abundar en el tema que finalmete se acciona un robot y después el otro.

## Fuentes

Se consultaron las siguientes fuentes de referencia durante el desarrollo de este proyecto:

- [Video de YouTube - Tutorial de Simulación Parte 1](https://www.youtube.com/watch?v=7g5ViG5wBQk&t=637s)
- [Video de YouTube - Tutorial de Simulación Parte 2](https://www.youtube.com/watch?v=gLQQtWKNmyE&list=PLdG9DWheQeVIuLbnW-xQPJGr8pew8jWOQ&index=2)
- [Video de YouTube - Tutorial de Simulación Parte 3](https://www.youtube.com/watch?v=EjHA3RFqsAg)

---

Este repositorio proporciona los archivos y configuraciones necesarios para la simulación del servicio de vino utilizando robots ABB IRB 120. 
