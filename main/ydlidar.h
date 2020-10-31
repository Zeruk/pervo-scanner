// uln2003.h

#ifndef YDLIDAR_H_
#define YDLIDAR_H_

#include <stdint.h>

#define YDLIDAR_PWM 14
#define YDLIDAR_DATA 13
#define YDLIDAR_UART_SPEED 115200
#define YDLIDAR_BUF_SIZE (1024)

// dummy for uart init
#define YDLIDAR_TXD  19

struct ydlidar {
    uint8_t pwm_val;

    void (*init)(void);
    // void init();
    void (*changePWM)(uint8_t pwm);
    // void step(int dir=1);
};

extern struct ydlidar Ydlidar;

#endif