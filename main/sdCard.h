// sdCard.h

#ifndef SCARD_H_
#define SCARD_H_

#include <stdint.h>

#define SDCARD_PIN_MISO 2
#define SDCARD_PIN_MOSI 15
#define SDCARD_PIN_CLK  14
#define SDCARD_PIN_CS   13

struct sdcard {
    int8_t state; // 0=initial
    // 1 = initialized
    // 2 = writing file

    // card info
    sdmmc_card_t* card;
    FILE* file;

    int8_t direction;
    float position;

    void (*init)(void);
    void (*newFile)(char* name);
    void (*write)(char* buffer);
    void (*closeFile)(void);
};

extern struct sdcard SDCard;

#endif