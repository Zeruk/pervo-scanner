/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_event.h"

// #include "uln2003.h"
#include "ydlidar.h"
#include "sdCard.h"
#include "./server/network.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

#define CONFIG_FREERTOS_HZ 100
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_240 y
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 240

static const char *TAG = "Main";

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
           CHIP_NAME,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    net_initialise_all();

    // check if stepper motor working
    // StepperControl.init();
    // while(1) {
    //     printf("step %f, ", StepperControl.position);
    //     fflush(stdout);
    //     StepperControl.direction = 1;
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);
    //     printf("step %f, ", StepperControl.position);
    //     fflush(stdout);
    //     StepperControl.direction = 0;
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);
    //     printf("step %f, ", StepperControl.position);
    //     fflush(stdout);
    //     StepperControl.direction = -1;
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);

    // }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Initialize YDLIiDAR");
    // YdlidarController.init();
    // YdlidarController.start();

    ESP_LOGI(TAG, "Initialize SDCard");
    SDCard.init();
    while (SDCard.state < 1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        SDCard.init();
    }

    YdlidarController.fileWriteFunction = SDCard.writeFile;
    SDCard.newFile("/sdcard/scan1.xyz");
    while (SDCard.state != 2)
    {
        ESP_LOGE(TAG, "Error creating file");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        SDCard.newFile("/sdcard/scan1.xyz");
    }
    ESP_LOGI(TAG, "Start file write");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // YdlidarController.stop();

    SDCard.closeFile();
    // SDCard.unmountCard();
    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    };

    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
