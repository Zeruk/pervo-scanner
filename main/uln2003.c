// uln2003.c

#include "uln2003.h"

#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* TAG = "example";


static void periodic_timer_callback(void* arg);
esp_timer_handle_t periodic_timer;
struct stepperControl StepperControl;

const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"
};


void init() {
    StepperControl.state=0;

    // GPIO init
    gpio_pad_select_gpio(STEPPER_P1);
    gpio_pad_select_gpio(STEPPER_P2);
    gpio_pad_select_gpio(STEPPER_P3);
    gpio_pad_select_gpio(STEPPER_P4);

    gpio_set_direction(STEPPER_P1, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEPPER_P2, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEPPER_P3, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEPPER_P4, GPIO_MODE_OUTPUT);
    
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, STEPPER_TIMER_PERIOD));
}


void step(int dir) {
    // full step mode
    switch(StepperControl.state) {
        case 0:
            gpio_set_level(STEPPER_P1, 1);
            gpio_set_level(STEPPER_P2, 1);
            gpio_set_level(STEPPER_P3, 0);
            gpio_set_level(STEPPER_P4, 0);
            break;
        case 1:
            gpio_set_level(STEPPER_P1, 0);
            gpio_set_level(STEPPER_P2, 1);
            gpio_set_level(STEPPER_P3, 1);
            gpio_set_level(STEPPER_P4, 0);
            break;
        case 2:
            gpio_set_level(STEPPER_P1, 0);
            gpio_set_level(STEPPER_P2, 0);
            gpio_set_level(STEPPER_P3, 1);
            gpio_set_level(STEPPER_P4, 1);
            break;
        case 3:
        default:
            gpio_set_level(STEPPER_P1, 1);
            gpio_set_level(STEPPER_P2, 0);
            gpio_set_level(STEPPER_P3, 0);
            gpio_set_level(STEPPER_P4, 1);
            break;
    }
    StepperControl.state += dir;

    
    if (StepperControl.state == 4) 
        StepperControl.state = 0;
    else if (StepperControl.state == -1) 
        StepperControl.state = 3;
}

static void periodic_timer_callback(void* arg)
{
    // int64_t time_since_boot = esp_timer_get_time();
    // ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us ", time_since_boot);
    // ESP_LOGI(TAG, "Periodic timer called, dir: %d, state: %d ", StepperControl.direction, StepperControl.state);
    if(StepperControl.direction != 0) step(StepperControl.direction);
}




struct stepperControl StepperControl = {
    .state = 0,
    .direction = 0,
    .position = 0.0,
    .init = init,
    .step = step,
};
