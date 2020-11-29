#include "sdCard.h"


#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"


static const char *TAG = "SDCard";
FILE* file;

void initSDCard();
void newFile(char* name);
void writeFile(char* buffer);
void closeFile();
void unmountCard();


struct sdcard SDCard = {
    .state=0,
    // 0=initial
    // 1 = initialized
    // 2 = writing file

    .init = initSDCard,
    .newFile = newFile,
    .writeFile = writeFile,
    .closeFile = closeFile,
    .unmountCard = unmountCard,
};


void initSDCard(){
    // set GPIO direction
    ESP_LOGI(TAG, "Set GPIO direction");
    // gpio_set_direction(SDCARD_PIN_MISO, GPIO_MODE_OUTPUT);
    // gpio_set_direction(SDCARD_PIN_MOSI, GPIO_MODE_OUTPUT);
    // gpio_set_direction(SDCARD_PIN_CLK, GPIO_MODE_OUTPUT);
    // gpio_set_direction(SDCARD_PIN_CS, GPIO_MODE_OUTPUT);

    ///// from sd_card_example
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 19000 ;

    host.command_timeout_ms = 5000;
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = SDCARD_PIN_MISO;
    slot_config.gpio_mosi = SDCARD_PIN_MOSI;
    slot_config.gpio_sck  = SDCARD_PIN_SCK;
    slot_config.gpio_cs   = SDCARD_PIN_CS;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.

    gpio_set_pull_mode(SDCARD_PIN_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SDCARD_PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SDCARD_PIN_SCK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SDCARD_PIN_CS, GPIO_PULLUP_ONLY);

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &(SDCard.card));

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    SDCard.state = 1;

    
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, SDCard.card);
    ESP_LOGI(TAG, "Mounted SD card!");

};
void newFile(char* name){
    
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file %s", name);
    // SDCard.file = fopen(name, "w");
    // if (SDCard.file == NULL) {
    //     ESP_LOGE(TAG, "Failed to open file for writing");
    //     // return -1;
    // }
    file = fopen(name, "w");
    // SDCard.file = &f;
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    SDCard.state = 2;
    ESP_LOGI(TAG, "File opening success");
    
};
void writeFile(char* buffer){
    // ESP_LOGI(TAG, "File write %s", buffer);
    if(SDCard.state != 2) return;
    fprintf(file, "%s!\n", buffer);
};
void closeFile(){
    if(SDCard.state != 2) return;
    ESP_LOGI(TAG, "Close file");
    // fclose(SDCard.file);
    fflush(file);
    fclose(file);
    SDCard.state = 1;
};
void unmountCard() {
    if(SDCard.state != 1) return;
    ESP_LOGI(TAG, "Try to unmount sdcard");
    esp_vfs_fat_sdmmc_unmount();
    ESP_LOGI(TAG, "Card unmounted");
    SDCard.state = 0;
}