#include "ydlidar.h"
#include "angles.h"
#include <stdio.h>
// #include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"


void init();

void changePWM(uint8_t pwm);


static const char* TAG = "uart_YDLIDAR";
struct ydlidarController YdlidarController = {
    .pwm_val = 0,
    .init = init,
    .changePWM = changePWM,
};
static const int RX_BUF_SIZE = 1024;

// double min_angle = angles::from_degrees(-180.f);
// double max_angle = angles::from_degrees(180.f);


// https://github.com/YDLIDAR/sdk/blob/master/include/ydlidar_protocol.h
// https://github.com/YDLIDAR/sdk/blob/master/src/CYdLidar.cpp#L210
struct my_node_info {
  uint8_t    sync_flag;  //sync flag
  uint16_t   sync_quality; //!信号质量
  uint16_t   angle_q6_checkbit; //!测距点角度
  uint16_t   distance_q2; //! 当前测距点距离
  uint64_t   stamp; //! 时间戳
  uint8_t    scan_frequence; //! 特定版本此值才有效,无效值是0
  uint8_t    debug_info[12];
  uint8_t    index;
};

struct my_node_info nodebuffer[100];

static void rx_task(void *arg)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    uint8_t countStructs = 0;
    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(TAG, "Read %d bytes'", rxBytes);
            // ESP_LOG_BUFFER_HEXDUMP(TAG, data, rxBytes, ESP_LOG_INFO);
            // COPY to struct
            countStructs = (int)(rxBytes / sizeof(struct my_node_info));
            memcpy(nodebuffer, data, countStructs * sizeof(struct my_node_info));
            // ESP_LOGI(TAG, "Readed point: %d %d %d %d ll %d %d", 
            //     nodebuffer[1].sync_flag, 
            //     nodebuffer[1].sync_quality, 
            //     nodebuffer[1].angle_q6_checkbit, 
            //     nodebuffer[1].distance_q2, 
            //     /*nodebuffer[1].stamp,*/ 
            //     nodebuffer[1].scan_frequence, 
            //     nodebuffer[1].index);
            for (uint16_t i = 0; i < countStructs; i++)
            {
                intensity = (float)nodebuffer[i].sync_quality;
                range = (float)(nodebuffer[i].distance_q2 / 4000.f);

                angle = (float)((nodebuffer[i].angle_q6_checkbit >> 1) / 64.0f);
                angle = angles::from_degrees(angle);
                angle = angles::normalize_angle(angle);

                // if (angle >= min_angle &&
                //         angle <= max_angle) {
                    printf("%f %f %f \n", intensity, angle, range);
                    // can push into resulting array
                // }
            }
            
            
        }
    }
    free(data);
};

void configurePWM() {
    //// from mcpwm_servo_control

    //1. mcpwm gpio initialization
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, YDLIDAR_PWM);

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    // 20ms - full period
    // 10ms - half (50%)
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 10000);
};

void configureUART() {
    //// from uart_async_rxtxtasks

    const uart_config_t uart_config = {
        .baud_rate = YDLIDAR_UART_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, YDLIDAR_TXD, YDLIDAR_DATA, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Listen on UART
    xTaskCreate(rx_task, "ydlidar_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
};

void init() { 
    ///////// CONFIGURE PWM /////////
    configurePWM();

    ////////// CONFIGURE UART ///////////
    configureUART();
}

void changePWM(uint8_t pwm) {
    YdlidarController.pwm_val=pwm;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (int)(20000 * pwm / 255));
}


// struct ydlidarController YdlidarController = {
//     .pwm_val = 0,
//     .init = init,
//     .changePWM = changePWM,
// };
