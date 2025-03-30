# Proyecto de invernadero.

Este código implementa un sistema basado en FreeRTOS para leer sensores de temperatura, humedad y luz, mostrar los datos en un LCD y activar alarmas si los valores están fuera de los rangos establecidos.

## Librerías utilizadas.

```cpp
/**
 * @file main.ino
 * @brief Código para leer sensores y manejar alarmas con FreeRTOS en ESP32
 * 
 * Este programa lee la temperatura, humedad y luz, mostrando los datos en un LCD
 * y activando alarmas en caso de valores fuera de rango.
 * 
 * @author Karol Tatiana Palechor Valencia
 * @date 2025-03-26
 * @version 1.0.0
 */

#include <Arduino.h>
#include "DHT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <LiquidCrystal.h>
#include <RTClib.h>
```

## Configuración de hardware.
Definimos los pines usados para sensores, LEDs y botones.

```cpp
/// Definición de pines y configuración del hardware
#define LED_BLUE 27 ///< LED azul para indicar estado
#define LED_RED 26 ///< LED rojo para indicar alerta
#define DHTPIN 4 ///< Pin del sensor DHT11
#define LDRPIN 34 ///< Pin del sensor de luz (LDR)
#define BUZZER_PIN 25 ///< Pin del buzzer
#define DHTTYPE DHT11 ///< Tipo de sensor DHT

/// Definición de pines para botones de control
const int button = 13; ///< Botón de incremento de contador
const int button2 = 14; ///< Botón adicional para futuras funciones
volatile int contador = 0; ///< Contador global
```

## Definición de estructuras.
Estructura para manejar las colas de FreeRTOS y los dispositivos.

```cpp
/**
 * @struct TaskParams
 * @brief Estructura para pasar parámetros a las tareas FreeRTOS
 * 
 * Contiene colas para comunicación entre tareas y punteros a los sensores y pantalla LCD.
 */
struct TaskParams {
    QueueHandle_t queueTemp; ///< Cola para temperatura
    QueueHandle_t queueHumidity; ///< Cola para humedad
    QueueHandle_t queueLight; ///< Cola para luz
    DHT *dht; ///< Puntero al sensor DHT
    RTC_DS1307 *rtc; ///< Puntero al reloj RTC
    LiquidCrystal *lcd; ///< Puntero a la pantalla LCD
};
```

Estructura para manejar la estabilidad de las lecturas.

```cpp
/**
 * @struct StabilityData
 * @brief Estructura para manejar estabilidad en las lecturas
 * 
 * Permite verificar si las lecturas se mantienen estables antes de realizar cambios en las alarmas.
 */
struct StabilityData {
    unsigned long lastStableTime; ///< Último tiempo de estabilidad registrado
    const unsigned long stableThreshold; ///< Umbral de estabilidad en milisegundos
    const unsigned long lightSleepDuration; ///< Duración del modo de bajo consumo
    bool isStable; ///< Indicador de estabilidad

    /**
     * @brief Constructor por defecto
     */
    StabilityData() : lastStableTime(0), stableThreshold(30000), lightSleepDuration(20000), isStable(true) {}
};
```

## Declaración de funciones.
Delcaración de funciones para la lectura de temperatura, humedad y luz, 
```cpp
void TaskReadTemp(void *pvParameters);
void TaskReadHumidity(void *pvParameters);
void TaskReadLight(void *pvParameters);
void TaskDisplayData(void *pvParameters);
void IRAM_ATTR buttonISR();
```

## Configuración inicial del sistema.

```cpp
/**
 * @brief Configuración inicial del sistema
 * Se inicializan los sensores, la pantalla LCD, las colas de mensajes y las tareas de FreeRTOS.
 */
void setup() {
    Serial.begin(115200);
    Wire.begin(33, 32);
    
    auto *rtc = new RTC_DS1307();
    rtc->begin();
    if (!rtc->isrunning()) {
        rtc->adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    auto *dht = new DHT(DHTPIN, DHTTYPE);
    dht->begin();
    
    auto *lcd = new LiquidCrystal(23, 22, 21, 19, 18, 5);
    lcd->begin(16, 2);
    
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    pinMode(button, INPUT_PULLUP);
    pinMode(button2, INPUT_PULLUP);
    
    attachInterrupt(button, buttonISR, FALLING);
    attachInterrupt(button2, buttonISR, FALLING);
    
    QueueHandle_t queueTemp = xQueueCreate(5, sizeof(float));
    QueueHandle_t queueHumidity = xQueueCreate(5, sizeof(float));
    QueueHandle_t queueLight = xQueueCreate(5, sizeof(int));
    
    if (queueTemp && queueHumidity && queueLight) {
        auto *params = new TaskParams{queueTemp, queueHumidity, queueLight, dht, rtc, lcd};
        xTaskCreate(TaskReadTemp, "ReadTemp", 2048, params, 2, NULL);
        xTaskCreate(TaskReadHumidity, "ReadHumidity", 2048, params, 2, NULL);
        xTaskCreate(TaskReadLight, "ReadLight", 2048, params, 1, NULL);
        xTaskCreate(TaskDisplayData, "DisplayData", 4096, params, 1, NULL);
    }
}
```

## Bucle principal.
El bucle principal se deja vacío ya que las tareas se ejecutan en FreeRTOS.

```cpp
/**
 * @brief Bucle principal vacío, ya que se usa FreeRTOS
 * 
 * El control del flujo del programa se delega a las tareas creadas en FreeRTOS.
 */
void loop() {}
```

## Tareas de FreeRTOS.
Función de FreeRTOS para leer la temperatura y enviarla a la cola de mensajes.

```cpp
/**
 * @brief Tarea que lee la temperatura y la almacena en la cola
 * @param pvParameters Puntero a parámetros de la tarea
 *
 * Se obtiene la lectura del sensor DHT11 y se envía a la cola para ser procesada.
 */
void TaskReadTemp(void *pvParameters) {
    auto *params = static_cast<TaskParams *>(pvParameters);
    for (;;) {
        float temp = params->dht->readTemperature();
        if (!isnan(temp)) {
            xQueueSend(params->queueTemp, &temp, portMAX_DELAY);
        }
        if (temp < 24 || temp > 35) {
                digitalWrite(LED_RED, HIGH);
                tone(BUZZER_PIN, 2000);
                vTaskDelay(pdMS_TO_TICKS(700));
                digitalWrite(LED_RED, LOW);
                noTone(BUZZER_PIN);
                vTaskDelay(pdMS_TO_TICKS(400));
        }
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

void TaskReadHumidity(void *pvParameters) {
    auto *params = static_cast<TaskParams *>(pvParameters);
    for (;;) {
        float humedad = params->dht->readHumidity();
        if (!isnan(humedad)) {
            xQueueSend(params->queueHumidity, &humedad, portMAX_DELAY);
        }
        if (humedad > 70) {
                digitalWrite(LED_RED, HIGH);
                tone(BUZZER_PIN, 2000);
                vTaskDelay(pdMS_TO_TICKS(700));
                digitalWrite(LED_RED, LOW);
                noTone(BUZZER_PIN);
                vTaskDelay(pdMS_TO_TICKS(400));
        }
        vTaskDelay(pdMS_TO_TICKS(3200));
    }
}

void TaskReadLight(void *pvParameters) {
    auto *params = static_cast<TaskParams *>(pvParameters);
    for (;;) {
        int luz = analogRead(LDRPIN);
        xQueueSend(params->queueLight, &luz, portMAX_DELAY);
        if (luz > 500) {
                digitalWrite(LED_BLUE, HIGH);
                tone(BUZZER_PIN, 2000);
                vTaskDelay(pdMS_TO_TICKS(700));
                digitalWrite(LED_BLUE, LOW);
                noTone(BUZZER_PIN);
                vTaskDelay(pdMS_TO_TICKS(400));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskDisplayData(void *pvParameters) {
    auto *params = static_cast<TaskParams *>(pvParameters);
    StabilityData stabilityData; // Crear instancia de la estructura dentro de la tarea

    for (;;) {
        float temp, humedad;
        int luz;
        bool alarma = false;

        if (xQueueReceive(params->queueTemp, &temp, pdMS_TO_TICKS(100)) &&
            xQueueReceive(params->queueHumidity, &humedad, pdMS_TO_TICKS(100)) &&
            xQueueReceive(params->queueLight, &luz, pdMS_TO_TICKS(100))) {

            if (temp < 24 || temp > 35 || humedad > 70 || luz > 500) {
                alarma = true;
                stabilityData.isStable = false; 
                stabilityData.lastStableTime = millis(); 
            } else {
                if (millis() - stabilityData.lastStableTime >= stabilityData.stableThreshold) {
                    stabilityData.isStable = true; 
                }
            }

            int crc = ((int)temp) ^ ((int)humedad) ^ luz;
            DateTime now = params->rtc->now();

            Serial.printf("$ %02d:%02d:%02d, %lu, %.2f, %.2f, %d, %s, %d $\n",
                          now.hour(), now.minute(), now.second(), 
                          xTaskGetTickCount() / configTICK_RATE_HZ,  
                          temp, humedad, luz, alarma ? "ON" : "OFF", crc);

            params->lcd->clear();
            params->lcd->setCursor(0, 0);
            params->lcd->printf("T:%.1fC H:%d%%", temp, (int)humedad);
            params->lcd->setCursor(0, 1);
            params->lcd->printf("L:%d Alar:%s", luz, alarma ? "ON" : "OFF");

            if (stabilityData.isStable) {
                Serial.println("Condiciones normales por 30 seg, entrando en Light Sleep...");
                delay(100); 
                esp_sleep_enable_timer_wakeup(stabilityData.lightSleepDuration * 1000);
                esp_light_sleep_start(); 
                Serial.println("Despertando del Light Sleep...");
                stabilityData.lastStableTime = millis(); 
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```

## Interrupciones.
Manejador de interrupción para los botones.

```cpp
/**
 * @brief Interrupción del botón
 * 
 * Maneja el incremento del contador cuando se detecta una pulsación válida del botón.
 */
void IRAM_ATTR buttonISR() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    
    if (interruptTime - lastInterruptTime > 200) {
        contador++;
        Serial.print("Contador: ");
        Serial.println(contador);
    }
    lastInterruptTime = interruptTime;
}
```
