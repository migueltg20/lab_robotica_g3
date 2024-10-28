# Simulación de Apertura y Servicio de Vino con Robots

## Descripción

Este proyecto tiene como objetivo simular la apertura de una botella de vino y el servicio del mismo en un vaso, empleando dos robots manipuladores. Para ello, se ha modelado una botella y un vaso de manera simplificada, asignando a cada objeto un manipulador robótico. Durante el desarrollo se lograron avances significativos, aunque surgieron algunos problemas, los cuales se detallan a continuación.

## Desarrollo y Problemas

### Modelado de Objetos y Robots

La botella de vino y el vaso fueron modelados de manera básica mediante cilindros y conos para simular el proceso de servir el vino. En la simulación se utilizaron dos robots ABB IRB 120, colocados frente a frente: uno para manipular la botella y el otro para sujetar el vaso.

### Herramientas y Funciones Utilizadas

1. **Grippers:** Se cambió la herramienta original de los robots por "grippers" (pinzas) para permitir el agarre de los objetos.
2. **Acciones de Agarre y Desagarre:** En la configuración de la simulación se programaron acciones de "attach" (agarre) y "detach" (desagarre) para la botella y el vaso, activadas mediante señales digitales. Estas señales permitieron determinar el momento en que un objeto era sujetado o soltado.

### Problemas Encontrados

1. **Referencias de Puntos de Trabajo:** Uno de los desafíos fue la referencia de puntos de trabajo al sistema de la botella. Esto ocasionó que, si la botella cambiaba de posición, no se podía retornar al mismo punto debido a la pérdida de orientación respecto al sistema de coordenadas del mundo. Este problema fue solucionado ajustando el sistema de referencia de los puntos afectados.

2. **Modelado del Tapón con Rosca:** Aunque era posible modelar un tapón de rosca funcional, se consideró una tarea compleja que añadiría poco valor al resultado final de la simulación. Se decidió simplificar este aspecto y hacer que uno de los manipuladores sujetara la botella mientras el otro robot sostenía el vaso, con ambos movimientos sincronizados para simular el servicio de vino.

## Fuentes

Se consultaron las siguientes fuentes de referencia durante el desarrollo de este proyecto:

- [Video de YouTube - Tutorial de Simulación Parte 1](https://www.youtube.com/watch?v=7g5ViG5wBQk&t=637s)
- [Video de YouTube - Tutorial de Simulación Parte 2](https://www.youtube.com/watch?v=gLQQtWKNmyE&list=PLdG9DWheQeVIuLbnW-xQPJGr8pew8jWOQ&index=2)
- [Video de YouTube - Tutorial de Simulación Parte 3](https://www.youtube.com/watch?v=EjHA3RFqsAg)

---

Este repositorio proporciona los archivos y configuraciones necesarios para la simulación del servicio de vino utilizando robots ABB IRB 120. 
