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
uint32_t ds18b20_read_interval_us = 5000000; // 5 segundos
uint32_t am2320_read_interval_ms  = 2000;     // 2 segundos

// Buffer para recibir datos del AM2320
uint8_t _buf[8];

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
uint16_t averageVoltage = 0, tdsValue = 0, temperature = 25;  // Temperatura predeterminada 25°C


int getMedianNum(uint16_t bArray[], int iFilterLen);

/*********************************** FUNCIONES PARA EL AM2320 ****************************************/

/**
 * @brief Función para inicializar el bus I2C en modo maestro.
 * 
 * @return esp_err_t ESP_OK si la configuración es exitosa, de lo contrario un error de ESP-IDF.
 */
static esp_err_t set_i2c(void) {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    return ESP_OK;
}

/**
 * @brief Función para despertar el sensor AM2320 antes de realizar una lectura.
 */
static void wake_sensor() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    ets_delay_us(800);  // Retraso de 800 microsegundos para despertar el sensor
}

/**
 * @brief Función para calcular el CRC16 de los datos recibidos del sensor AM2320.
 */
static uint16_t crc16(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Función para leer los registros del sensor AM2320.
 */
static esp_err_t read_registers(uint8_t reg, uint8_t len) {
    esp_err_t ret;

    // Despertar el sensor
    wake_sensor();

    // Comando de lectura
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x03, true); // Función de lectura (0x03)
    i2c_master_write_byte(cmd, reg, true);  // Dirección del registro inicial
    i2c_master_write_byte(cmd, len, true);  // Número de bytes a leer
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }
    ets_delay_us(1500);  // Retraso de 1.5 ms según las especificaciones del sensor

    // Leer los datos del sensor
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, _buf, len + 4, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    // Verificar el CRC
    uint16_t received_crc = (_buf[len + 3] << 8) | _buf[len + 2];
    if (crc16(_buf, len + 2) != received_crc) {
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

/**
 * @brief Función para obtener los valores de temperatura y humedad del sensor AM2320.
 */
static esp_err_t get_am2320_data(float *temperature, float *humidity) {
    esp_err_t ret = read_registers(0x00, 4); // Leer 4 bytes desde el registro 0x00
    if (ret != ESP_OK) {
        return ret;
    }

    // Convertir la humedad y la temperatura
    uint16_t hum_raw = (_buf[2] << 8) | _buf[3];
    uint16_t temp_raw = (_buf[4] << 8) | _buf[5];

    *humidity = hum_raw / 10.0;
    *temperature = ((temp_raw & 0x7FFF) / 10.0) * ((temp_raw & 0x8000) ? -1 : 1);

    return ESP_OK;
}

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

            int compensationCoefficient = 100 + 2 * (temperature - 25);  // Compensación de temperatura (sin decimales)
            int compensationVoltage = averageVoltage / compensationCoefficient;  // Ajuste por temperatura
            tdsValue = (13342 * compensationVoltage * compensationVoltage * compensationVoltage - 
                        25586 * compensationVoltage * compensationVoltage + 
                        85739 * compensationVoltage) / 1000;  // Cálculo de TDS (entero)

            ESP_LOGI(TAG, "TDS Value: %lu ppm", (unsigned long)tdsValue);  // Usar %lu para unsigned long
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Ceder tarea por 50ms
	}
}
// Función de filtro de mediana
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
/*****************************************TAREAS PH-4502C**************************************************************/
// Función para leer el sensor de pH
void readPHTask(void *parameter) {
    uint32_t avgValue = 0, pHVol = 0, phValue = 0;


    while (true) {
        // Leer el sensor de pH múltiples veces y almacenar en un buffer
        for (int i = 0; i < NUM_SAMPLES; i++) {
            buf[i] = adc1_get_raw(PH_SENSOR_PIN);  // Lee el valor del ADC y lo almacena en uint32_t
            vTaskDelay(pdMS_TO_TICKS(30));  // Pausa entre lecturas para estabilizar
        }

        // Ordenar los valores leídos
        for (int i = 0; i < NUM_SAMPLES - 1; i++) {
            for (int j = i + 1; j < NUM_SAMPLES; j++) {
                if (buf[i] > buf[j]) {
                    temp = buf[i];
                    buf[i] = buf[j];
                    buf[j] = temp;
                }
            }
        }

        // Calcular pHVol usando enteros, multiplicando por un factor de escala para evitar decimales
        pHVol = (avgValue * 3300) / (4095 * 6);  // Multiplicamos por 3300 (3.3V) para trabajar con enteros

        // Calcular el valor de pH
        phValue = (pHVol * 570) / 1000;  // Ajuste para trabajar con uint32_t

        // Aplicar la calibración al valor de pH
        phValue = (phValue + calibration);  // Calibración: al ser uint32_t, no hay decimales

        // Imprimir el valor de pH
        ESP_LOGI(TAG, "pH sensor value: %lu", (unsigned long)phValue);  // Utilizando %lu para uint32_t


        vTaskDelay(pdMS_TO_TICKS(500));  // Esperar 500 ms antes de la siguiente lectura
    }
}


// Función para configurar el ADC
void configure_adc() {
    // Configurar ADC si es necesario, por ejemplo, para TDS y pH
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TDS_SENSOR_PIN, ADC_ATTEN_DB_0);  // Configuración de TDS
    // Agregar configuración para otros sensores si es necesario
}
/**
 * @brief Ajusta el intervalo de lectura del sensor de pH.
 */
void set_ph_read_interval(uint32_t interval_sec) {
    ph_read_interval_us = interval_sec * 1000000;
}
/*********************************** TAREA DE TEMPERATURA Y HUMEDAD DEL AM2320 ****************************************/

/**
 * @brief Tarea FreeRTOS para manejar la lectura de temperatura y humedad del sensor AM2320.
 */
void am2320_task(void *pvParameter) {
    float temperature, humidity;

    while (1) {
        esp_err_t ret = get_am2320_data(&temperature, &humidity);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "AM2320 Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
        } else {
            ESP_LOGE(TAG, "AM2320 Failed to read sensor data: %s", esp_err_to_name(ret));
        }
        vTaskDelay(am2320_read_interval_ms / portTICK_PERIOD_MS);
    }
}

/*********************************** MAIN ****************************************/

void app_main(void) {
    // Inicializar el I2C y el LED
    ESP_ERROR_CHECK(set_i2c());
    configure_led();
    // Configurar el ADC para leer el sensor de pH
    configure_adc();

    // Crear las tareas
    xTaskCreate(&led_blink_task, "LED Blink Task", 2048, NULL, 5, NULL);
    xTaskCreate(&ds18b20_task, "DS18B20 Task", 4096, NULL, 5, NULL);
    xTaskCreate(&am2320_task, "AM2320 Task", 4096, NULL, 5, NULL);
    xTaskCreate(&readPHTask, "Read PH Task", 2048, NULL, 5, NULL);
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
 * @brief Ajusta el intervalo de lectura del sensor AM2320.
 */
void set_am2320_read_interval(uint32_t interval_ms) {
    am2320_read_interval_ms = interval_ms;
}
