// uln2003.h

#ifndef ULN2003_H_
#define ULN2003_H_

#include <stdint.h>

#define STEPPER_P1 32
#define STEPPER_P2 33
#define STEPPER_P3 25
#define STEPPER_P4 26

#define STEPPER_TIMER_PERIOD 982*2
// 1000000 / 63.68395 / 64 / 15 * 60 = 981.4
// 1000000us / reduction / steps_per_rot * rot_per_min * sec_in_min

struct stepperControl {
    int8_t state;
    int8_t direction;
    float position;

    void (*init)(void);
    // void init();
    void (*step)(int dir);
    // void step(int dir=1);
};

extern struct stepperControl StepperControl;

#endif