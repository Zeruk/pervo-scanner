#include <sdCard.h>



static const char *TAG = "SDCard";

void init();
void newFile(char* name);
void write(char* buffer);
void closeFile();

struct sdcard SDCard = {
    .state=0,
    // 0=initial
    // 1 = initialized
    // 2 = writing file

    .init = init,
    .newFile = newFile,
    .write = write,
    .closeFile = newFile,
};


void init(){
    ///// from sd_card_example
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.


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

    
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, SDCard.card);

};
void newFile(char* name){
    
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    SDCard.file = fopen(name, "w");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return -1;
    }
};
void write(char* buffer){
    fprintf(SDCard.file, "Hello %s!\n", SDCard.card->cid.name);
    ESP_LOGI(TAG, "File written");
};
void closeFile(){
    ESP_LOGI(TAG, "Close file");
    fclose(SDCard.file);
};