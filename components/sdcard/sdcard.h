#ifndef SDCARD_HEADER_FILE_H
#define SDCARD_HEADER_FILE_H

/**
 * @file sdcard_handler.h
 * @brief Interface for initializing and managing SD card communication via SPI on ESP32.
 */

/*******************************************************************************/
/*                                  INCLUDES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                                   MACROS                                    */
/*******************************************************************************/

/**
 * @brief Logging tag for SD card related messages.
 */
#define SDCARD_TAG         "SDCARD"

/**
 * @brief SPI MISO (Master In Slave Out) GPIO pin for SD card.
 */
#define SDCARD_MISO        (2)

/**
 * @brief SPI MOSI (Master Out Slave In) GPIO pin for SD card.
 */
#define SDCARD_MOSI        (15)

/**
 * @brief SPI clock (SCLK) GPIO pin for SD card.
 */
#define SDCARD_SCLK        (14)

/**
 * @brief Chip Select (CS) GPIO pin for SD card.
 */
#define SDCARD_CS          (13)

/**
 * @brief Mount point path for the SD card on the filesystem.
 */
#define SDCARD_MOUNT_POINT "/sdcard"

/*******************************************************************************/
/*                                 DATA TYPES                                  */
/*******************************************************************************/

/*******************************************************************************/
/*                             PUBLIC DEFINITIONS                              */
/*******************************************************************************/

/*******************************************************************************/
/*                          PUBLIC FUNCTION PROTOTYPES                         */
/*******************************************************************************/

/**
 * @brief Initializes the SD card using SPI and mounts the filesystem.
 *
 * This function configures the SPI bus and mounts the SD card to the defined
 * mount point (see @ref SDCARD_MOUNT_POINT). If the initialization fails,
 * appropriate logs will be printed using the @ref SDCARD_TAG.
 */
void sdcard_init();

#endif /* SDCARD_HEADER_FILE_H */
