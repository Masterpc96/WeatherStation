#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <wifi_config.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "i2c/i2c.h"
#include "bmp280/bmp280.h"



const uint8_t i2c_bus = 0;
const uint8_t scl_pin = 5;
const uint8_t sda_pin = 4;

homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);

static void temperature_sensor_task(void *pvParameters)
{
    bmp280_params_t  params;
    float temperature_value, humidity_value, pressure_value;

    bmp280_init_default_params(&params);

    bmp280_t bmp280_dev;
    bmp280_dev.i2c_dev.bus = i2c_bus;
    bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;

    while (1) {
        while (!bmp280_init(&bmp280_dev, &params)) {
            printf("BMP280 initialization failed\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
        printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

        while(1) {

            if (!bmp280_read_float(&bmp280_dev, &temperature_value, &pressure_value, &humidity_value)) {
                break;
            }

            temperature.value.float_value = temperature_value;
            humidity.value.float_value = humidity_value;

            homekit_characteristic_notify(&temperature, HOMEKIT_FLOAT(temperature_value));
            homekit_characteristic_notify(&humidity, HOMEKIT_FLOAT(humidity_value));

            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
    }
}

void on_wifi_event(wifi_config_event_t event) {
    if (event == WIFI_CONFIG_CONNECTED) {
        printf("Connected to WiFi\n");
    } else if (event == WIFI_CONFIG_DISCONNECTED) {
        printf("Disconnected from WiFi\n");
    }
}



void temperature_sensor_identify(homekit_value_t _value) {
    printf("Temperature sensor identify\n");
}


void temperature_sensor_init() {
    xTaskCreate(temperature_sensor_task, "Temperatore Sensor", 256, NULL, 2, NULL);
}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "0012345"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MyTemperatureSensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, temperature_sensor_identify),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            &temperature,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"),
            &humidity,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "683-35-159"
};

void user_init(void) {
    uart_set_baud(0, 115200);

    i2c_init(i2c_bus, scl_pin, sda_pin, I2C_FREQ_400K);

    wifi_config_init2("Apple Home 0x90", NULL, on_wifi_event);
    temperature_sensor_init();
    homekit_server_init(&config);



}

