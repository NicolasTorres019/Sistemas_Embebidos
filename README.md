# Sistemas_Embebidos 
Nicolás Torres Muñoz - 20201005046, Oscar Poblador Parra -  20211005116 
## Roboto recoge basura
### Introducción
El presente proyecto consiste en el diseño y desarrollo de un carrito recolector de basura autónomo con enfoque IoT, utilizando microcontroladores como la Raspberry Pi Pico 2 W y el ESP32-CAM. Su objetivo principal es simular un sistema de recolección inteligente capaz de identificar, recoger, clasificar y transportar residuos, representados en este caso por LEDs de distintos colores, hacia puntos de acopio designados. El proyecto se plantea en dos etapas: en la primera, el carrito funciona de manera independiente, empleando sensores, procesamiento de imágenes y un electroimán para realizar la separación y apilado de los objetos; en la segunda, se busca implementar la colaboración entre dos carritos, los cuales podrán comunicarse entre sí y coordinarse para manipular objetos de mayor volumen mediante cajones motorizados. De esta manera, se integra la robótica móvil con tecnologías IoT, proponiendo una solución innovadora y didáctica que combina autonomía, clasificación inteligente y trabajo colaborativo.

### Plantamiento del problema
La recolección y clasificación de residuos es un proceso que normalmente requiere intervención humana, lo cual limita la eficiencia y puede implicar riesgos y costos elevados. Los sistemas tradicionales carecen de autonomía y de capacidades inteligentes para optimizar estas tareas. Por ello, surge la necesidad de desarrollar soluciones basadas en robótica e IoT que permitan automatizar la identificación, recolección y separación de residuos. En este proyecto se plantea el diseño de un carrito recolector autónomo, capaz de detectar y clasificar desechos representados por LEDs de diferentes colores, y que en una segunda etapa pueda coordinarse con otro carrito para manipular objetos de mayor tamaño, demostrando el potencial de la colaboración robótica en escenarios de recolección inteligente.

### Objetivo General

Diseñar y desarrollar un prototipo de carrito recolector de basura autónomo con integración de tecnologías IoT, capaz de identificar, clasificar y transportar objetos representados por LEDs de diferentes colores, y que en una segunda etapa logre coordinarse con otro carrito para la manipulación colaborativa de objetos de mayor tamaño.

### Objetivos Específicos

1. Implementar el sistema electrónico del carrito integrando los microcontroladores, sensores, actuadores, motores y módulos de control de potencia, necesarios para garantizar su desplazamiento, recolección y manipulación de objetos.

2. Establecer la comunicación IoT entre microcontroladores (ESP32-CAM, Raspberry Pi Pico 2 W, entre otros) para el monitoreo, control remoto y transmisión de datos en tiempo real.

3. Desarrollar el sistema de detección e identificación de objetos mediante procesamiento de imágenes, asociando los LEDs de diferentes colores a tipos de residuos específicos para su correcta clasificación.

4.Realizar la fase de pruebas y validación del prototipo, evaluando la autonomía del carrito, la precisión en la identificación de los objetos y la eficiencia en la recolección y descarga de los mismos.

5.Diseñar e implementar el mecanismo de acople y coordinación con otro carrito, mediante comunicación inalámbrica y sincronización de movimientos, para permitir la manipulación colaborativa de objetos de mayor tamaño.

### Materiales:
*     2 Motores DC                $ 12.000
*     1 Puente H L298N            $  9.800
*     3 Servomotores              $ 50.300
*     2 Llantas de caucho         $ 11.000
*     ESP32 Cam                   $ 42.000
*     Raspberry Pi Pico W         $ 40.000
*     Tornillos                   $ 10.000
*     Me Arm                      $ 40.000
*     Rueda Loca                  $  5.500
*     Control remoto IR  TX,RX    $  6.500
*     Sensor de proximidad        $  6.000
*     Electroimán                 $ 25.000
*     2 Imánes                    $ 10.000
*     Pilas 18650                 $ 11.000
*     TOTAL                       $278.800

### METODOLOGÍA 

La metodología propuesta para el desarrollo del proyecto se fundamenta en el uso de programación de microcontroladores mediante lenguajes como Python y MicroPython, lo que permitirá un control flexible y eficiente de los distintos módulos del sistema. Se plantea la implementación de una arquitectura distribuida, en la cual un MCU principal se encargue del control del carrito, gestionando motores, sensores y actuadores, mientras que un MCU secundario procese la información proveniente de la cámara para la identificación de objetos. Esta división de tareas permitirá  facilitar el desempeño del sistema, garantizando la correcta integración de hardware y software, y facilitando el desarrollo modular de las funciones de navegación, recolección, clasificación e interacción IoT entre los dispositivos.

### Resultados esperados 

Se espera que el proyecto permita la construcción de un carrito recolector autónomo plenamente funcional, capaz de desplazarse, identificar y clasificar objetos representados por LEDs de distintos colores, garantizando su recolección y descarga en los puntos asignados. El prototipo debe demostrar un manejo adecuado de sistemas embebidos al integrar y controlar de manera eficiente microcontroladores, sensores y actuadores, así como el uso de tecnologías IoT para la comunicación, el monitoreo y la coordinación entre dispositivos. En una segunda etapa, se prevé que el sistema logre establecer comunicación colaborativa entre dos carritos, permitiendo el acople y la manipulación conjunta de objetos de mayor tamaño, lo que validará la escalabilidad y versatilidad de la solución propuesta.

