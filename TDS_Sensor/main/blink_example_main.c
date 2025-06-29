/*
 * Proyecto: Lectura de Sensores DS18B20 y PH-4502C  con ESP32 utilizando FreeRTOS
 * Autor: Morel Penco Lautaro
 * Fecha: 29 de Agosto de 2024
 *
 * Descripción:
 * Este programa implementa la lectura de un sensor de temperatura DS18B20 y un sensor de temperatura y humedad AM2320
 * conectados a un microcontrolador ESP32. Se utilizan tareas de FreeRTOS para gestionar las lecturas de cada sensor 
 * de forma independiente y una tarea adicional para el parpadeo de un LED. Los tiempos de sensado de cada sensor son
 * configurables.
 *
 * Configuraciones:
 * - GPIO 2: Pin de control del LED.
 * - GPIO 4: Pin de datos del sensor DS18B20.
 * - GPIO 21: Pin SDA del bus I2C.
 * - GPIO 22: Pin SCL del bus I2C.
 * - Frecuencia I2C: 100 kHz.
 * - Dirección I2C del sensor AM2320: 0x5C.
 */

#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp32/rom/ets_sys.h"  // Incluye ets_delay_us()
#include "esp_timer.h"
#include "ONEWire.h"
#include "DS18B20.h"
#include "driver/adc.h"  // Asegúrate de incluir esto para el manejo del ADC

// Pines y configuración del I2C
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

// Pines y configuración del DS18B20
#define DS18B20_PIN                 4

// Configuración del LED
#define BLINK_GPIO                  2

// Dirección del sensor AM2320
#define AM2320_SENSOR_ADDR          0x5C 

// Pines y configuración del sensor de pH
#define PH_SENSOR_PIN			 	5  				// GPIO5 -> ADC2_CH0
#define TDS_SENSOR_PIN 				ADC1_CHANNEL_1  // GPIO21 -> ADC1_CHANNEL_1
#define VREF 						3300      		// 3.3V * 100 (sin decimales, para evitar el uso de float)
#define SCOUNT 						30      		// Número de muestras para el filtro
#define NUM_SAMPLES 				10
// Variables de estado
static const char *TAG = "SENSOR_APP";
static uint8_t s_led_state = 0;

// Variables configurables para los tiempos de sensado
uint32_t ds18b20_read_interval_us 	= 5000000; // 5 segundos
uint32_t TDS_read_interval_us		= 5000000;


// DS18B20
_sOWHandle ow0;
_sDS18B20Handle ds18b20_0;
int16_t tempDS18B20;
// Parámetro de calibración
uint32_t calibration = 7000;


// Variables configurables para los tiempos de sensado
uint32_t ph_read_interval_us = 5000000; // 5 segundos

// Buffer de lectura del sensor

uint8_t buf[NUM_SAMPLES];
uint16_t temp;

//Variables TDS
uint16_t analogBuffer[SCOUNT];    // Almacenar lecturas ADC
uint16_t analogBufferTemp[SCOUNT];
uint16_t analogBufferIndex = 0, copyIndex = 0;
uint16_t averageVoltage = 0, tdsValue = 0;  


int getMedianNum(uint16_t bArray[], int iFilterLen);



/*********************************** FUNCIONES PARA EL DS18B20 ****************************************/

/**
 * @brief Configura el pin de datos como entrada.
 */
void OneWireSetPinInput(void) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
}

/**
 * @brief Configura el pin de datos como salida.
 */
void OneWireSetPinOutput(void) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
}

/**
 * @brief Lee el estado del pin de datos.
 */
uint8_t OneWireReadPin(void) {
    return gpio_get_level(DS18B20_PIN);
}

/**
 * @brief Escribe un nivel lógico en el pin de datos.
 */
void OneWireWritePin(uint8_t value) {
    gpio_set_level(DS18B20_PIN, value);
}

/**
 * @brief Genera un delay bloqueante en microsegundos.
 */
static inline int delayBlocking(int time_us) {
    ets_delay_us(time_us);
    return 1;
}

/*********************************** TAREA DE PARPADEO DEL LED ****************************************/

/**
 * @brief Parpadea el LED configurado en el pin BLINK_GPIO.
 */
static void blink_led(void) {
    gpio_set_level(BLINK_GPIO, s_led_state);
}

/**
 * @brief Configura el pin del LED como salida.
 */
static void configure_led(void) {
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

/**
 * @brief Tarea FreeRTOS para controlar el parpadeo del LED.
 */
void led_blink_task(void *pvParameter) {
    while (1) {
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/*********************************** TAREA DE TEMPERATURA DEL DS18B20 ****************************************/

/**
 * @brief Tarea FreeRTOS para manejar la lectura de temperatura del sensor DS18B20.
 */
/**
 * @brief Tarea FreeRTOS para manejar la lectura de temperatura del sensor DS18B20.
 */
void ds18b20_task(void *pvParameter) {
    // Configuración del DS18B20
    ow0.DELAYus = &delayBlocking;
    ow0.SETPinInput = &OneWireSetPinInput;
    ow0.SETPinOutput = &OneWireSetPinOutput;
    ow0.ReadPinBit = &OneWireReadPin;
    ow0.WritePinBit = &OneWireWritePin;

    ds18b20_0.OW = &ow0;
    DS18B20_Init(&ds18b20_0, NULL);

    uint32_t lastReadTime = esp_timer_get_time();
    uint32_t currentTime_us;
    const TickType_t delayAfterTempRead = pdMS_TO_TICKS(3000); // 3 segundos de espera
    bool measurementActive = false;

    while (1) {
        currentTime_us = esp_timer_get_time();

        // Si la bandera está activa, realizar la medición
        if (measurementActive) {
            DS18B20_Task(&ds18b20_0, currentTime_us);

            // Verificar el estado y leer la temperatura si está lista
            _eDS18B20Status state = DS18B20_Status(&ds18b20_0);
            if (state == DS18B20_ST_TEMPOK) {
                tempDS18B20 = DS18B20_ReadLastTemp(&ds18b20_0);
                ESP_LOGI(TAG, "DS18B20 Temperature: %d.%04d C", tempDS18B20 / 16, (tempDS18B20 % 16) * 625);

                // Desactivar la bandera y esperar 3 segundos antes de la siguiente medición
                measurementActive = false;
                vTaskDelay(delayAfterTempRead);
                lastReadTime = esp_timer_get_time();
            } else if (state == DS18B20_ST_TEMPCRCERROR) {
                ESP_LOGE(TAG, "DS18B20 CRC Error reading temperature!");
            }

            // Ceder el procesador sin una demora fija
            taskYIELD();
        } else {
            // Verificar si ha pasado el intervalo configurado para iniciar una nueva medición
            if ((currentTime_us - lastReadTime) >= ds18b20_read_interval_us) {
                if (DS18B20_Status(&ds18b20_0) == DS18B20_ST_IDLE) {
                    DS18B20_StartReadTemp(&ds18b20_0);
                    ESP_LOGI(TAG, "DS18B20 START READ TEMP");
                    measurementActive = true;
                } else {
                    ESP_LOGW(TAG, "DS18B20 no está en estado IDLE. Estado actual: %d", DS18B20_Status(&ds18b20_0));
                }
            }

            // Si no es tiempo de iniciar una nueva medición, esperar un poco antes de continuar
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
/*****************************************TAREAS TDS SENSOR************************************************************/
// Función para leer y procesar el sensor TDS
void readTdsSensorTask(void *pvParameter) {
    static TickType_t lastSampleTime = 0;
    static TickType_t lastPrintTime = 0;
    
    while(1) {
        TickType_t currentTime = xTaskGetTickCount();
        
        // Leer ADC cada 40ms
        if (currentTime - lastSampleTime > pdMS_TO_TICKS(40)) {
            lastSampleTime = currentTime;
            analogBuffer[analogBufferIndex] = adc1_get_raw(TDS_SENSOR_PIN);  // Leer el sensor
            analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;  // Recorrer el índice

            // Filtro de mediana
            for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
                analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
            }
            averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 4095;  // Convertir ADC a voltaje (sin decimales)
        }

        // Imprimir cada 800ms
        if (currentTime - lastPrintTime > pdMS_TO_TICKS(800)) {
            lastPrintTime = currentTime;

            // Utilizar la temperatura medida por el DS18B20 para la compensación de TDS
            int compensationCoefficient = 100 + 2 * (tempDS18B20 / 10 - 25);  // Compensación de temperatura (sin decimales)
            int compensationVoltage = averageVoltage / compensationCoefficient;  // Ajuste por temperatura
            tdsValue = (13342 * compensationVoltage * compensationVoltage * compensationVoltage - 
                        25586 * compensationVoltage * compensationVoltage + 
                        85739 * compensationVoltage) / 1000;  // Cálculo de TDS (entero)

            ESP_LOGI(TAG, "TDS Value: %lu ppm", (unsigned long)tdsValue);  // Mostrar solo TDS
        }
        
        vTaskDelay(pdMS_TO_TICKS(3000));  // Esperar 3 segundos antes de la siguiente lectura
    }
}

// Función de filtro de mediana
int getMedianNum(uint16_t bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (int i = 0; i < iFilterLen; i++) {
        bTab[i] = bArray[i];
    }
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;

    return bTemp;
}

// Función para configurar el ADC
void configure_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TDS_SENSOR_PIN, ADC_ATTEN_DB_0);  // Configuración de TDS
}

/*********************************** MAIN ****************************************/

void app_main(void) {
    // Inicializar el I2C y el LED
    configure_led();
    // Configurar el ADC para leer el sensor de pH
    configure_adc();

    // Crear las tareas
    xTaskCreate(&led_blink_task, "LED Blink Task", 2048, NULL, 5, NULL);
    xTaskCreate(&ds18b20_task, "DS18B20 Task", 4096, NULL, 5, NULL);
    xTaskCreate(&readTdsSensorTask, "Read TDS Sensor Task", 2048, NULL, 5, NULL);

    // El bucle principal se deja vacío, ya que las tareas FreeRTOS manejan todo
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Esperar para evitar que el bucle se ejecute continuamente
    }
}

/**
 * @brief Ajusta el intervalo de lectura del sensor DS18B20.
 */
void set_ds18b20_read_interval(uint32_t interval_sec) {
    ds18b20_read_interval_us = interval_sec * 1000000;
}
/**
 * @brief Ajusta el intervalo de lectura del sensor TDS.
 */
void set_tds_read_interval(uint32_t interval_sec) {
    TDS_read_interval_us = interval_sec * 1000000;  // Convertir segundos a microsegundos
}