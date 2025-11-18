<!-- Resumen general del proyecto y cómo correr Root, Child y Dashboard. -->
FireMesh-DX es un sistema distribuido de detección de incendios basado en una red de nodos ESP32 utilizando la librería PainlessMesh. El objetivo principal del proyecto es demostrar cómo se aplican en un entorno real los conceptos de sistemas distribuidos, tales como comunicación distribuida, sincronización de relojes, consistencia de datos, replicación y tolerancia a fallos. El sistema incluye sensores físicos de humo y flama, sincronización de tiempo mediante un protocolo NTP implementado desde cero, tolerancia a fallos por pérdida de conexión, consistencia eventual y un dashboard web en tiempo real conectado a Firebase.

Arquitectura General
El sistema está compuesto por dos tipos de nodos: Root Node y Child Nodes.
El Root Node es el coordinador principal de la red. Se encarga de manejar la comunicación en la malla, ejecutar tareas concurrentes usando FreeRTOS, responder solicitudes de sincronización de tiempo y enviar datos a Firebase Real-Time Database.
Los Child Nodes contienen sensores de humo MQ-2 y sensores de flama KY-026. Estos nodos envían datos al Root Node, ajustan su reloj mediante el protocolo NTP y almacenan datos localmente cuando pierden conectividad.

Root Node
El Root Node tiene las siguientes responsabilidades:

Coordinar la red PainlessMesh.

Ejecutar varias tareas FreeRTOS como manejo de mensajes, reconexión automática y envío de datos a Firebase.

Actuar como servidor de tiempo mediante un protocolo NTP propio, implementado sin librerías externas.

Manejar tolerancia a fallos detectando desconexiones y restableciendo enlaces.

Recibir datos de los Child Nodes, ordenarlos mediante timestamps y enviarlos a Firebase.

Child Nodes
Los Child Nodes realizan estas funciones:

Capturar datos de los sensores MQ-2 (humo) y KY-026 (flama).

Sincronizar su tiempo con el Root Node para mantener coherencia temporal.

Enviar datos por broadcast a la red.

Al perder conexión, almacenar hasta 20 lecturas en una estructura FIFO y retransmitirlas en orden al reconectar.

Implementar tolerancia a fallos local mediante reintentos de envío y reconstrucción de la malla.

Sincronización de Tiempo
El sistema implementa un protocolo NTP propio basado en el algoritmo de Cristian. El Root Node es la autoridad del tiempo. Los Child Nodes solicitan la hora, reciben un timestamp, calculan el retardo y ajustan su reloj local. El objetivo es mantener un margen máximo de error de aproximadamente un segundo.

Modelo de Consistencia
El sistema utiliza consistencia eventual. Los Child Nodes almacenan datos temporalmente si pierden conexión y los reenvían en orden al reconectar. El Root Node respeta los timestamps al escribir en Firebase, garantizando que el historial quede en orden incluso si los datos no llegan en tiempo real. La consistencia fuerte no es necesaria para este tipo de sistema real-time suave.

Tolerancia a Fallos
El sistema incluye varios mecanismos de tolerancia a fallos:

Heartbeats de la red Mesh para detectar desconexiones.

Reconexión automática tanto en Root como en Child Nodes.

Almacenamiento temporal local mediante FIFO.

Redundancia funcional, ya que múltiples nodos pueden cubrir la misma zona.
El objetivo es que ante fallas parciales, el sistema siga operando sin intervención humana.

Comunicación y Replicación
La comunicación entre nodos se hace mediante broadcast dentro de la red PainlessMesh. El Root Node replica todos los datos hacia Firebase Real-Time Database, donde se almacenan de manera estructurada. Firebase permite sincronización en tiempo real con el dashboard.

Dashboard Web
El dashboard se desarrolla en React o Angular. Se conecta directamente a Firebase usando WebSockets. El dashboard muestra en tiempo real el estado de los nodos, sus valores sensoriales y alertas de incendio cuando los sensores detectan valores fuera de rango. También refleja las reconexiones y la llegada de datos antiguos.

Flujo General del Sistema
Un Child Node toma una lectura.
El Child Node solicita sincronización de tiempo si es necesario.
El Child Node envía la lectura con timestamp.
El Root Node recibe el dato y lo ordena por tiempo.
El Root Node envía la información a Firebase.
El dashboard recibe los cambios en tiempo real.
Si hay desconexión, el Child Node guarda datos y los envía más tarde.

Etapas de Evaluación
Primera etapa: implementación de la red Mesh y sincronización de tiempo.
Segunda etapa: tolerancia a fallos, FIFO y reconexión automática.
Entrega final: integración completa con Firebase y dashboard web funcional.

Conclusiones
FireMesh-DX demuestra que la teoría de sistemas distribuidos puede implementarse en un entorno físico real. El sistema combina sincronización de relojes, comunicación distribuida, consistencia eventual, tolerancia a fallos y replicación en tiempo real. El resultado final es un sistema IoT robusto, resiliente y capaz de operar aun con fallas parciales sin perder datos.
