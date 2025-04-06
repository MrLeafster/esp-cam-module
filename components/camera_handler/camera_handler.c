/*******************************************************************************/
/*                                  INCLUDES                                   */
/*******************************************************************************/

#include "camera_handler.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <regex.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

/*******************************************************************************/
/*                                   MACROS                                     */
/*******************************************************************************/

/** @brief Stack size for the camera handler task. */
#define CAMERA_HANDLER_TASK_STACK_SIZE (50000)

/** @brief Task priority for the camera handler task. */
#define CAMERA_HANDLER_TASK_PRIORITY   (10)

/** @brief Name of the camera handler task. */
#define CAMERA_HANDLER_TASK_NAME       "CAM_HANDLER_task"

/*******************************************************************************/
/*                                 DATA TYPES                                  */
/*******************************************************************************/

/**
 * @brief Structure to hold JPEG frame data.
 */
typedef struct {
    uint8_t *jpeg;  /**< Pointer to the JPEG buffer */
    size_t length;  /**< Length of the JPEG buffer */
} jpeg_frame_t;

/** @brief Handle for camera timestamp queue. */
QueueHandle_t cam_timestamp_handle;

/*******************************************************************************/
/*                         PRIVATE FUNCTION PROTOTYPES                         */
/*******************************************************************************/

/**
 * @brief Get the next available crash index based on existing files in the folder.
 *
 * @param folder_path Path to the folder to scan.
 * @return The next index to use.
 */
uint16_t _get_current_crash_index(const char *folder_path);

/**
 * @brief Task function for recording circular video buffer.
 *
 * @param pvParameters Parameters passed to the task (unused).
 */
void _record_circular_camera_video_task(void *pvParameters);

/**
 * @brief 
 * This is a interupt handler for the GPIO pin used to
 * trigger the recording process of the camera
 */
static void IRAM_ATTR _camera_trigger_gpio_isr_handler(void* arg);

/**
 * @brief
 * Initializator for the GPIO pin used to
 * trigger the recording process of the camera
 */
void _camera_trigger_gpio_init();

/*******************************************************************************/
/*                          STATIC DATA & CONSTANTS                            */
/*******************************************************************************/

/** @brief Next recording index for naming crash files. */
static uint16_t next_recording_index = 0;

/*******************************************************************************/
/*                              PUBLIC FUNCTIONS                               */
/*******************************************************************************/

void camera_handler_init()
{
    camera_config_t config;
    // Camera configuration setup (GPIO, pixel format, resolution etc.)
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    config.frame_size = CAMERA_HANDLER_FRAME_SIZE;
    config.jpeg_quality = CAMERA_HANDLER_FRAME_QUALITY;
    config.fb_count = CAMERA_HANDLER_FRAME_BUFFER_SIZE;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(CAMERA_HANDLER_TAG, "Couldn't init camera");
        return;
    }

    // Camera sensor tweaks
    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_special_effect(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 0);
    s->set_gainceiling(s, (gainceiling_t)0);
    s->set_bpc(s, 0);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);

    next_recording_index = _get_current_crash_index(CAMERA_HANDLER_SAVE_PATH);

    xTaskCreate(
        _record_circular_camera_video_task,
        CAMERA_HANDLER_TASK_NAME,
        CAMERA_HANDLER_TASK_STACK_SIZE,
        NULL,
        CAMERA_HANDLER_TASK_PRIORITY,
        NULL
    );

    _camera_trigger_gpio_init();
}

void camera_handler_save_buffer(cam_timestamp_packet *new_packet)
{
    xQueueSend(cam_timestamp_handle, (void *)new_packet, portMAX_DELAY);
}

uint16_t camera_handler_get_next_index()
{
    return next_recording_index++;
}

void camera_handler_direct_record()
{
    ESP_LOGI(CAMERA_HANDLER_TAG, "Starting to record!");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    jpeg_frame_t frame_buffer[CAMERA_HANDLER_FRAMERATE_HZ * CAMERA_HANDLER_VIDEO_DURATION_S];

    for(int cnt = 0; cnt < CAMERA_HANDLER_FRAMERATE_HZ * CAMERA_HANDLER_VIDEO_DURATION_S; cnt++)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;

        uint8_t *jpg_buf = (uint8_t *)malloc(fb->len);
        size_t jpg_len = fb->len;
        memcpy(jpg_buf, fb->buf, jpg_len);

        frame_buffer[cnt].jpeg = jpg_buf;
        frame_buffer[cnt].length = jpg_len;

        esp_camera_fb_return(fb);

        vTaskDelayUntil(&xLastWakeTime, 1000.f / CAMERA_HANDLER_FRAMERATE_HZ / portTICK_PERIOD_MS);
    }

    ESP_LOGI(CAMERA_HANDLER_TAG, "Writing to SD");

    FILE* f = fopen("/sdcard/video.mjpeg", "wb");
    for (int i = 0; i < CAMERA_HANDLER_FRAMERATE_HZ * CAMERA_HANDLER_VIDEO_DURATION_S; i++) {
        fwrite(frame_buffer[i].jpeg, 1, frame_buffer[i].length, f);
        free(frame_buffer[i].jpeg);
    }
    fclose(f);

    ESP_LOGI(CAMERA_HANDLER_TAG, "Finished recording!");
}

/*******************************************************************************/
/*                             PRIVATE FUNCTIONS                               */
/*******************************************************************************/

uint16_t _get_current_crash_index(const char *folder_path)
{
    DIR *dir = opendir(folder_path);
    if (!dir) {
        perror("opendir failed");
        return 0;
    }

    struct dirent *entry;
    uint16_t max_index = 0;
    regex_t regex;
    regcomp(&regex, "^crash_([0-9]+)\\.mjpeg$", REG_EXTENDED);

    while ((entry = readdir(dir)) != NULL) {
        regmatch_t matches[2];
        if (regexec(&regex, entry->d_name, 2, matches, 0) == 0) {
            char num_str[16] = {0};
            int len = matches[1].rm_eo - matches[1].rm_so;
            strncpy(num_str, entry->d_name + matches[1].rm_so, len);
            int num = atoi(num_str);
            if (num > max_index) {
                max_index = num;
            }
        }
    }

    closedir(dir);
    regfree(&regex);
    return max_index + 1;
}

void _record_circular_camera_video_task(void *pvParameters)
{
    cam_timestamp_handle = xQueueCreate(2, sizeof(cam_timestamp_packet));

    uint16_t frame_buffer_size = CAMERA_HANDLER_FRAMERATE_HZ * CAMERA_HANDLER_VIDEO_DURATION_S;
    jpeg_frame_t frame_buffer[frame_buffer_size];

    uint16_t frame_buffer_read_ind = 0;
    uint16_t frame_buffer_write_ind = 0;

    while (1)
    {
        ESP_LOGI(CAMERA_HANDLER_TAG, "Starting to query data!");

        TickType_t xLastWakeTime = xTaskGetTickCount();

        cam_timestamp_packet received_packet;
        BaseType_t queue_receive_status = xQueueReceive(cam_timestamp_handle, &received_packet, 2 / portTICK_PERIOD_MS);

        while (queue_receive_status != pdPASS)
        {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) continue;

            uint8_t *jpg_buf = (uint8_t *)malloc(fb->len);
            size_t jpg_len = fb->len;
            memcpy(jpg_buf, fb->buf, jpg_len);

            frame_buffer[frame_buffer_write_ind].jpeg = jpg_buf;
            frame_buffer[frame_buffer_write_ind].length = jpg_len;

            frame_buffer_write_ind = (frame_buffer_write_ind + 1) % (frame_buffer_size);
            if (frame_buffer_write_ind == frame_buffer_read_ind) {
                free(frame_buffer[frame_buffer_read_ind].jpeg);
                frame_buffer[frame_buffer_read_ind].jpeg = NULL;
                frame_buffer_read_ind = (frame_buffer_read_ind + 1) % (frame_buffer_size);
            }

            esp_camera_fb_return(fb);

            queue_receive_status = xQueueReceive(cam_timestamp_handle, &received_packet, 2 / portTICK_PERIOD_MS);

            vTaskDelayUntil(&xLastWakeTime, 1000.f / CAMERA_HANDLER_FRAMERATE_HZ / portTICK_PERIOD_MS);
        }

        ESP_LOGI(CAMERA_HANDLER_TAG, "Stop querying, writing to SD now!");

        char full_path[128];
        const char *dir_path = CAMERA_HANDLER_SAVE_PATH;
        snprintf(full_path, sizeof(full_path), "%s/%s.mjpeg", dir_path, received_packet.timestamp);

        ESP_LOGI(CAMERA_HANDLER_TAG, "%s", full_path);

        FILE *f = fopen(full_path, "wb");
        for (int i = frame_buffer_read_ind; i != frame_buffer_write_ind; i = (i + 1) % (frame_buffer_size)) {
            fwrite(frame_buffer[i].jpeg, 1, frame_buffer[i].length, f);
            //free(frame_buffer[i].jpeg);
        }
        fclose(f);

        ESP_LOGI(CAMERA_HANDLER_TAG, "Finished recording!");
    }
}

void _camera_trigger_gpio_init()
{
    gpio_set_direction(CAMERA_HANDLER_GPIO_TRIGGER_PIN_NUM, GPIO_MODE_INPUT);
    gpio_set_intr_type(CAMERA_HANDLER_GPIO_TRIGGER_PIN_NUM, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(CAMERA_HANDLER_GPIO_TRIGGER_PIN_NUM, GPIO_INTR_ANYEDGE);

    gpio_isr_handler_add(CAMERA_HANDLER_GPIO_TRIGGER_PIN_NUM, _camera_trigger_gpio_isr_handler, NULL);
}

/*******************************************************************************/
/*                             INTERRUPT HANDLERS                              */
/*******************************************************************************/

static void _camera_trigger_gpio_isr_handler(void* arg)
{
    cam_timestamp_packet new_packet;
    snprintf(
        new_packet.timestamp, 
        sizeof(new_packet.timestamp), 
        "crash_%d", 
        camera_handler_get_next_index()
    );
    camera_handler_save_buffer(&new_packet);
}