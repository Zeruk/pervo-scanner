// sdCard.h

#ifndef SCARD_H_
#define SCARD_H_

#include "sdmmc_cmd.h"

// #define SDCARD_PIN_MISO 36
// #define SDCARD_PIN_CLK  39
// #define SDCARD_PIN_MOSI 34
// #define SDCARD_PIN_CS   35
#define SDCARD_PIN_MISO 21
#define SDCARD_PIN_SCK  19
#define SDCARD_PIN_MOSI 18
#define SDCARD_PIN_CS   5

#define SDMMC_MAX_EVT_WAIT_DELAY_MS 5000

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
    void (*writeFile)(char* buffer);
    void (*closeFile)(void);
    void (*unmountCard)(void);
};

extern struct sdcard SDCard;

#endif