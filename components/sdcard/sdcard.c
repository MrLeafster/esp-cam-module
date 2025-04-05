/*******************************************************************************/
/*                                  INCLUDES                                   */
/*******************************************************************************/

/**
 * @file sdcard_handler.c
 * @brief Implements initialization and filesystem mounting for an SD card using SPI on the ESP32.
 */

 #include "sdcard.h"

 #include <stdio.h>
 #include <string.h>
 #include <sys/unistd.h>
 #include <sys/stat.h>
 
 #include "esp_vfs_fat.h"
 #include "sdmmc_cmd.h"
 
 #include "esp_log.h"
 
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 
 /*******************************************************************************/
 /*                                   MACROS                                    */
 /*******************************************************************************/
 
 /*******************************************************************************/
 /*                                 DATA TYPES                                  */
 /*******************************************************************************/
 
 /*******************************************************************************/
 /*                         PRIVATE FUNCTION PROTOTYPES                         */
 /*******************************************************************************/
 
 /*******************************************************************************/
 /*                          STATIC DATA & CONSTANTS                            */
 /*******************************************************************************/
 
 /*******************************************************************************/
 /*                                 GLOBAL DATA                                 */
 /*******************************************************************************/
 
 /*******************************************************************************/
 /*                              PUBLIC FUNCTIONS                               */
 /*******************************************************************************/
 
 /**
  * @brief Initializes the SPI bus and mounts the FAT filesystem on the SD card.
  *
  * This function sets up the SPI interface using the pins defined in @ref sdcard.h,
  * initializes the SD card driver, and mounts the card to the file system at
  * the mount point defined by @ref SDCARD_MOUNT_POINT.
  *
  * It logs relevant status and error messages using the ESP-IDF logging system.
  *
  * If the initialization or mounting fails, the function logs an error and exits early.
  */
 void sdcard_init()
 {
     esp_err_t ret;
 
     // Configure FAT filesystem mount options
     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
         .format_if_mount_failed = false,
         .max_files = 5,
         .allocation_unit_size = 16 * 1024
     };
 
     sdmmc_card_t *card;
     const char mount_point[] = SDCARD_MOUNT_POINT;
     ESP_LOGI(SDCARD_TAG, "Initializing SD card");
 
     // SPI host config
     ESP_LOGI(SDCARD_TAG, "Using SPI peripheral");
 
     sdmmc_host_t host = SDSPI_HOST_DEFAULT();
     spi_bus_config_t bus_cfg = {
         .mosi_io_num = SDCARD_MOSI,
         .miso_io_num = SDCARD_MISO,
         .sclk_io_num = SDCARD_SCLK,
         .quadwp_io_num = -1,
         .quadhd_io_num = -1,
         .max_transfer_sz = 4000,
     };
 
     // Initialize the SPI bus
     ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
     if (ret != ESP_OK) {
         ESP_LOGE(SDCARD_TAG, "Failed to initialize bus.");
         return;
     }
 
     // Configure SPI device slot for SD card
     sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
     slot_config.gpio_cs = SDCARD_CS;
     slot_config.host_id = host.slot;
 
     ESP_LOGI(SDCARD_TAG, "Mounting filesystem");
 
     // Mount the filesystem
     ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
 
     if (ret != ESP_OK) {
         if (ret == ESP_FAIL) {
             ESP_LOGE(SDCARD_TAG, "Failed to mount filesystem. "
                      "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
         } else {
             ESP_LOGE(SDCARD_TAG, "Failed to initialize the card (%s). "
                      "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
         }
         return;
     }
 
     ESP_LOGI(SDCARD_TAG, "Filesystem mounted");
 
     // Print SD card information
     sdmmc_card_print_info(stdout, card);
 }
 
 /*******************************************************************************/
 /*                             PRIVATE FUNCTIONS                               */
 /*******************************************************************************/
 
 /*******************************************************************************/
 /*                             INTERRUPT HANDLERS                              */
 /*******************************************************************************/
 