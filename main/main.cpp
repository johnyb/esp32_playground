/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "bmp180.h"

#include "u8g2.h"

#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"

static const char* tag = "MAIN";

TaskHandle_t hReadSensor = NULL;
TaskHandle_t hUpdateDisplay = NULL;

u8g2_t u8g2;

double temperature = 0;

/**
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    ESP_LOGD(tag, "I2C init");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_4;
    conf.scl_io_num = GPIO_NUM_15;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 50000;
    i2c_param_config(I2C_NUM_0, &conf);
    ESP_ERROR_CHECK( i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0) );
}

void updateDisplay() {
    ESP_LOGD(tag, "Starting display update loop");
    char str[50];
    sprintf(str, "Temp: %.2f °C", temperature);

    u8g2_ClearBuffer(&u8g2);

    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tf);
    u8g2_DrawUTF8(&u8g2, 2, 17, str);
    u8g2_SendBuffer(&u8g2);

    vTaskDelete(NULL);
}

void readSensorTimer(TimerHandle_t pxTimer) {
    vTaskResume(hReadSensor);
}

void readSensor() {
    ESP_LOGD(tag, "Starting I2C read loop");
    fflush(stdout);
    esp_light_sleep_start();
    BMP180 *sensor = new BMP180();
    int32_t pressure = 0;

    while (1) {
        pressure = sensor->readPressure();
        temperature = sensor->readTemperature();
        ESP_LOGI(tag, "\n");

        ESP_LOGI(tag, "Pressure: %d hPa", pressure);
        ESP_LOGI(tag, "Temperature: %.2f °C", temperature);
        fflush(stdout);
        esp_light_sleep_start();
        TimerHandle_t readTimer = xTimerCreate("read_sensor", pdMS_TO_TICKS(1000), pdFALSE, (void*)1, &readSensorTimer);
        if (readTimer != NULL) {
            ESP_LOGI(tag, "Starting timer");
            if (xTimerStart(readTimer, 0) != pdPASS) {
                ESP_LOGE(tag, "Error");
            }
        } else {
            ESP_LOGE(tag, "Could not register timer");
        }
        ESP_LOGI(tag, "\n");

        xTaskCreate((TaskFunction_t)updateDisplay, "update display", 1024 * 2, NULL, 10, &hUpdateDisplay);

        vTaskSuspend(hReadSensor);
    }

    delete sensor;
    vTaskDelete(NULL);
}

extern "C" {
    void app_main();
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(tag, "Hello world!");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(tag, "This is ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision %d, %dMB %s flash",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
            chip_info.revision,
            spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external"
    );

    i2c_master_init();

    esp_sleep_enable_timer_wakeup(5000);
    esp_light_sleep_start();
    u8g2_esp32_hal_t u8g2_esp32_hal;
    //setup already done, not specifying sda and scl skips setup
    u8g2_esp32_hal.sda = U8G2_ESP32_HAL_UNDEFINED;
    u8g2_esp32_hal.scl = U8G2_ESP32_HAL_UNDEFINED;
    u8g2_esp32_hal.reset = GPIO_NUM_16;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1306_i2c_128x64_vcomh0_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    u8g2_InitDisplay(&u8g2);

    u8g2_SetPowerSave(&u8g2, 0);

    esp_light_sleep_start();

    xTaskCreate((TaskFunction_t)readSensor, "read sensor", 1024 * 2, NULL, 10, &hReadSensor);
}
