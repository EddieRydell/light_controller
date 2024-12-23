#include "sd_card_file_system.h"

#include <string.h>

#include "errno.h"
#include "dirent.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#define DEFAULT_MAX_FILES 5
#define DEFAULT_ALLOCATION_UNIT_SIZE (16 * 1024)

#define SD_CS_PIN 22
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19
#define SD_SCK_PIN 18

static const char* TAG = "sd card";

// Function to list the contents of a directory on the sd card specified by path
int32_t list_directory(const char* path) {
    ESP_LOGI(TAG, "Listing directory contents of %s", path);
    DIR* dir = opendir(path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory: %s", strerror(errno));
        return 0;
    }
    struct dirent* entry;
    int32_t num_directories = 0;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "Found file: %s", entry->d_name);
        num_directories++;
    }
    closedir(dir);
    return num_directories;
}

// Populates directory_names with all the directories inside a path
// Make sure directory_names is malloced with the right amount of entries
void get_directory_names(const char* path, char** directory_names) {
    ESP_LOGI(TAG, "Getting directory contents of %s", path);
    DIR* dir = opendir(path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory: %s", strerror(errno));
    }
    struct dirent* entry;
    uint32_t index = 0;
    while ((entry = readdir(dir)) != NULL) {
        directory_names[index++] = strdup(entry->d_name);
    }
    closedir(dir);
}

// Initialize SD card to read files from it
/* NOTE TO ECEN 330 STUDENTS 
 * If you are using this code as a guide on Lab 7, there are a few changes you need to make:
    1. This function assumes that the SPI system is already initialized (by lcd.c). This means that to use it for 
    your own SD card, you will need to go into lcd.c and change the these lines in the spi_master_init function:
        spi_bus_config_t buscfg = {
            .mosi_io_num = GPIO_MOSI,
            .miso_io_num = -1,        <---- This line needs to be changed to 19 (or whatever pin you're using for MISO on your SD card) instead of -1
            .sclk_io_num = GPIO_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
            .flags = 0
        };
    The spi_master_init function was originally meant to only initialize the SPI system for the LCD, so it only needed
    to send data out using MOSI. For it to work with an SD card, we need to set the MISO pin so it can also receive data.

    2. Some SD cards can't handle high frequency SPI clock speeds. If this is the case for your SD card, you'll probably
    get an error that looks like this when calling this function:
        E (607) sdmmc_sd: sdmmc_check_scr: send_scr returned 0x109
    To fix this, you'll need to add this line:
        host.max_freq_khz = 5000;
    Put this right after initializing the host on line 89 in this file. This should reduce the frequency to something
    that your SD card can handle.
*/
void init_sd_card() {
    ESP_LOGI(TAG, "Initializing SD card");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = DEFAULT_MAX_FILES,
            .allocation_unit_size = DEFAULT_ALLOCATION_UNIT_SIZE // DEFAULT_ALLOCATION_UNIT_SIZE
    };

    esp_err_t ret;
    sdmmc_card_t* card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
            .mosi_io_num = SD_MOSI_PIN,
            .miso_io_num = SD_MISO_PIN,
            .sclk_io_num = SD_SCK_PIN,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
    };


    ret = spi_bus_initialize(host.slot, &bus_cfg, (spi_dma_chan_t)SDSPI_DEFAULT_DMA);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus already initialized. Keeping previous SPI configuration");
    }
    else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = host.slot;
    ESP_LOGI(TAG, "Success initializing SD card");

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    list_directory(MOUNT_POINT);
}

