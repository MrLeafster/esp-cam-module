#include <stdio.h>

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "esp_log.h"

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"




#include "camera_handler.h"




#define TAG "MAIN"

#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13
#define MOUNT_POINT "/sdcard"

void setup_sd()
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    cam_timestamp_packet new_packet;
    strcpy(new_packet.timestamp, "");
    //xQueueSendFromISR(cam_timestamp_handle, ( void * ) &new_packet, NULL);
    camera_handler_save_buffer();
}

void gpio_setup()
{
    gpio_set_direction(12, GPIO_MODE_INPUT);
    gpio_set_intr_type(12, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(12, GPIO_INTR_ANYEDGE);

    gpio_isr_handler_add(12, gpio_isr_handler, NULL);
}

void app_main(void)
{
    setup_sd();
    camera_handler_init();
    gpio_setup();

    // DEMO

    cam_timestamp_packet new_packet;

    vTaskDelay(15000 / portTICK_PERIOD_MS);
    snprintf(
        new_packet.timestamp, 
        sizeof(new_packet.timestamp), 
        "crash_%d", 
        camera_handler_get_next_index()
    );
    camera_handler_save_buffer(&new_packet);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    snprintf(
        new_packet.timestamp, 
        sizeof(new_packet.timestamp), 
        "crash_%d", 
        camera_handler_get_next_index()
    );
    camera_handler_save_buffer(&new_packet);

    vTaskDelay(portMAX_DELAY);
}