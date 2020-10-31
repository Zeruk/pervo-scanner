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
#include "uln2003.h"
#include "ydlidar.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

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
    //////////////////////////////////////////////////////
    // check if ydlidar working
    YdlidarController.init();
    while (1)
    {
        // printf("Set PWM to %d, ", 255);
        // fflush(stdout);
        // YdlidarController.changePWM(255);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // printf("Set PWM to %d, ", 190);
        // fflush(stdout);
        // YdlidarController.changePWM(190);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // printf("Set PWM to %d, ", 128);
        // fflush(stdout);
        // YdlidarController.changePWM(128);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // printf("Set PWM to %d, ", 64);
        // fflush(stdout);
        // YdlidarController.changePWM(64);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        printf("Set PWM to %d, ", 128);
        fflush(stdout);
        YdlidarController.changePWM(128);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    

    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
