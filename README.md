# Proyecto: invernadero automatizado.

Este proyecto implementa un sistema de monitoreo de temperatura, humedad y luz utilizando un ESP32, sensores DHT11 y LDR, una pantalla LCD para visualizar los datos y un sistema de alarmas basado en LEDs y un buzzer. Se emplea FreeRTOS para gestionar múltiples tareas concurrentes, asegurando una lectura eficiente de los sensores y una respuesta rápida ante valores fuera de rango. Además, se integra un RTC DS1307 que proporciona la hora, fecha y día, y dos botones de interrupción para la interacción del usuario.
