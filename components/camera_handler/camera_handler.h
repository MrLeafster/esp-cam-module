#ifndef HEADER_FILE_H
#define HEADER_FILE_H

/**
 * @file camera_handler.h
 * @brief Camera handler interface for ESP32-based circular video recording system.
 */

/*******************************************************************************/
/*                                  INCLUDES                                   */
/*******************************************************************************/

#include "esp_camera.h"

/*******************************************************************************/
/*                                   MACROS                                    */
/*******************************************************************************/

/**
 * @brief Logging tag for camera handler messages.
 */
#define CAMERA_HANDLER_TAG "CAM_HANDLER"

/**
 * @brief Default frame size for camera capture.
 */
#define CAMERA_HANDLER_FRAME_SIZE        FRAMESIZE_VGA

/**
 * @brief JPEG compression quality (lower is better quality).
 */
#define CAMERA_HANDLER_FRAME_QUALITY     (12)

/**
 * @brief Number of frame buffers to allocate.
 */
#define CAMERA_HANDLER_FRAME_BUFFER_SIZE (2)

/**
 * @brief Target frame rate for video recording in Hz.
 */
#define CAMERA_HANDLER_FRAMERATE_HZ     (20)

/**
 * @brief Length of recorded video in seconds.
 */
#define CAMERA_HANDLER_VIDEO_DURATION_S (10)

/**
 * @brief Path where video files will be saved on SD card.
 */
#define CAMERA_HANDLER_SAVE_PATH "/sdcard"

/**
 * @brief Camera GPIO pin assignments.
 */
#define PWDN_GPIO_NUM     32 /**< Power-down pin. */
#define RESET_GPIO_NUM    -1 /**< Reset pin (not used). */
#define XCLK_GPIO_NUM      0 /**< External clock pin. */
#define SIOD_GPIO_NUM     26 /**< I2C data pin. */
#define SIOC_GPIO_NUM     27 /**< I2C clock pin. */

#define Y9_GPIO_NUM       35 /**< Data pin Y9. */
#define Y8_GPIO_NUM       34 /**< Data pin Y8. */
#define Y7_GPIO_NUM       39 /**< Data pin Y7. */
#define Y6_GPIO_NUM       36 /**< Data pin Y6. */
#define Y5_GPIO_NUM       21 /**< Data pin Y5. */
#define Y4_GPIO_NUM       19 /**< Data pin Y4. */
#define Y3_GPIO_NUM       18 /**< Data pin Y3. */
#define Y2_GPIO_NUM        5 /**< Data pin Y2. */

#define VSYNC_GPIO_NUM    25 /**< Vertical sync pin. */
#define HREF_GPIO_NUM     23 /**< Horizontal reference pin. */
#define PCLK_GPIO_NUM     22 /**< Pixel clock pin. */

/*******************************************************************************/
/*                                 DATA TYPES                                  */
/*******************************************************************************/

/**
 * @brief Packet structure used to send timestamps that trigger video saving.
 */
typedef struct 
{
    char timestamp[30 + 1]; /**< Null-terminated timestamp string (max 30 chars). */
} cam_timestamp_packet;

/*******************************************************************************/
/*                          PUBLIC FUNCTION PROTOTYPES                         */
/*******************************************************************************/

/**
 * @brief Initializes the camera module and starts the circular video recording task.
 */
void camera_handler_init();

/**
 * @brief Sends a timestamp packet to the recording task, triggering the last N seconds to be saved.
 *
 * @param new_packet Pointer to a timestamp packet structure.
 */
void camera_handler_save_buffer(cam_timestamp_packet *new_packet);

/**
 * @brief Returns the next available index for naming saved crash files.
 *
 * @return Index number for the next video file.
 */
uint16_t camera_handler_get_next_index();

/**
 * @brief Immediately records a short video and saves it directly to the SD card.
 */
void camera_handler_direct_record();

#endif /* HEADER_FILE_H */
