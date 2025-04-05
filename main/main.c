#include <stdio.h>

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

#include "esp_camera.h"
#include "esp_log.h"

#include "esp_heap_caps.h"
#include "esp_timer.h"

#include <dirent.h>
#include <regex.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#define TAG "MAIN"

// Pin definitions for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1

#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13
#define MOUNT_POINT "/sdcard"

#define CAM_FRAMERATE_HZ 20
#define CAM_VIDEO_DURATION_S 10

typedef struct {
    uint8_t *jpeg;
    size_t length;
} jpeg_frame_t;

QueueHandle_t cam_timestamp_handle;
typedef struct {
    char timestamp[30 + 1];
} cam_timestamp_packet;

uint16_t next_recording_index = 0;

#define FOLDER_PATH "/sdcard"
#define FILENAME_PREFIX "crash_"
#define FILENAME_SUFFIX ".mjpeg"

uint16_t get_next_crash_index(const char *folder_path) {
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

void setup_camera()
{   
    camera_config_t config;
 
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

    config.frame_size = FRAMESIZE_VGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 12; //10-63 lower number means higher quality
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Couldn't init camera");
        return;
    }

    // Camera quality adjustments
    sensor_t * s = esp_camera_sensor_get();

    // BRIGHTNESS (-2 to 2)
    s->set_brightness(s, 0);
    // CONTRAST (-2 to 2)
    s->set_contrast(s, 0);
    // SATURATION (-2 to 2)
    s->set_saturation(s, 0);
    // SPECIAL EFFECTS (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_special_effect(s, 0);
    // WHITE BALANCE (0 = Disable , 1 = Enable)
    s->set_whitebal(s, 1);
    // AWB GAIN (0 = Disable , 1 = Enable)
    s->set_awb_gain(s, 1);
    // WB MODES (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_wb_mode(s, 0);
    // EXPOSURE CONTROLS (0 = Disable , 1 = Enable)
    s->set_exposure_ctrl(s, 1);
    // AEC2 (0 = Disable , 1 = Enable)
    s->set_aec2(s, 0);
    // AE LEVELS (-2 to 2)
    s->set_ae_level(s, 0);
    // AEC VALUES (0 to 1200)
    s->set_aec_value(s, 300);
    // GAIN CONTROLS (0 = Disable , 1 = Enable)
    s->set_gain_ctrl(s, 1);
    // AGC GAIN (0 to 30)
    s->set_agc_gain(s, 0);
    // GAIN CEILING (0 to 6)
    s->set_gainceiling(s, (gainceiling_t)0);
    // BPC (0 = Disable , 1 = Enable)
    s->set_bpc(s, 0);
    // WPC (0 = Disable , 1 = Enable)
    s->set_wpc(s, 1);
    // RAW GMA (0 = Disable , 1 = Enable)
    s->set_raw_gma(s, 1);
    // LENC (0 = Disable , 1 = Enable)
    s->set_lenc(s, 1);
    // HORIZ MIRROR (0 = Disable , 1 = Enable)
    s->set_hmirror(s, 0);
    // VERT FLIP (0 = Disable , 1 = Enable)
    s->set_vflip(s, 0);
    // DCW (0 = Disable , 1 = Enable)
    s->set_dcw(s, 1);
    // COLOR BAR PATTERN (0 = Disable , 1 = Enable)
    s->set_colorbar(s, 0);

    next_recording_index = get_next_crash_index(MOUNT_POINT);
}

void record_camera_to_sd_basic()
{
    ESP_LOGI(TAG, "Starting to record!");

    TickType_t xLastWakeTime = xTaskGetTickCount();

    jpeg_frame_t frame_buffer[CAM_FRAMERATE_HZ * CAM_VIDEO_DURATION_S];
    
    for(int cnt = 0; cnt < CAM_FRAMERATE_HZ * CAM_VIDEO_DURATION_S; cnt++)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;

        uint8_t *jpg_buf = (uint8_t *)malloc(fb->len);
        size_t jpg_len = fb->len;
        memcpy(jpg_buf, fb->buf, jpg_len);

        frame_buffer[cnt].jpeg = jpg_buf;
        frame_buffer[cnt].length = jpg_len;

        esp_camera_fb_return(fb);
        
        vTaskDelayUntil( &xLastWakeTime, 1000.f / CAM_FRAMERATE_HZ / portTICK_PERIOD_MS );
    }

    ESP_LOGI(TAG, "Writing to sd");

    FILE* f = fopen("/sdcard/video.mjpeg", "wb");
    for (int i = 0; i < CAM_FRAMERATE_HZ * CAM_VIDEO_DURATION_S; i++) {
        fwrite(frame_buffer[i].jpeg, 1, frame_buffer[i].length, f);
        free(frame_buffer[i].jpeg);
    }
    fclose(f);

    ESP_LOGI(TAG, "Finished recording!");
}

void record_circular_camera_video( void * pvParameters )
{   
    cam_timestamp_handle = xQueueCreate(2, sizeof(cam_timestamp_packet));

    jpeg_frame_t frame_buffer[CAM_FRAMERATE_HZ * CAM_VIDEO_DURATION_S];

    while(1)
    {
        ESP_LOGI(TAG, "Starting to query data!");

        uint16_t frame_buffer_read_ind = 0;
        uint16_t frame_buffer_write_ind = 0;

        TickType_t xLastWakeTime = xTaskGetTickCount();

        cam_timestamp_packet received_packet;
        BaseType_t queue_receive_status = xQueueReceive(cam_timestamp_handle, &received_packet, 2 / portTICK_PERIOD_MS);
        
        while(queue_receive_status != pdPASS)
        {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) continue;

            uint8_t *jpg_buf = (uint8_t *)malloc(fb->len);
            size_t jpg_len = fb->len;
            memcpy(jpg_buf, fb->buf, jpg_len);

            frame_buffer[frame_buffer_write_ind].jpeg = jpg_buf;
            frame_buffer[frame_buffer_write_ind].length = jpg_len;

            frame_buffer_write_ind = (frame_buffer_write_ind + 1) % (CAM_FRAMERATE_HZ * CAM_VIDEO_DURATION_S);
            if(frame_buffer_write_ind == frame_buffer_read_ind){
                free(frame_buffer[frame_buffer_read_ind].jpeg);
                frame_buffer[frame_buffer_read_ind].jpeg = NULL;
                frame_buffer_read_ind = (frame_buffer_read_ind + 1) % (CAM_FRAMERATE_HZ * CAM_VIDEO_DURATION_S);
            }

            esp_camera_fb_return(fb);

            queue_receive_status = xQueueReceive(cam_timestamp_handle, &received_packet, 2 / portTICK_PERIOD_MS);
            
            vTaskDelayUntil( &xLastWakeTime, 1000.f / CAM_FRAMERATE_HZ / portTICK_PERIOD_MS );
        }

        ESP_LOGI(TAG, "Stop querying, writing to SD now!");

        char full_path[128];
        const char *dir_path = "/sdcard";

        snprintf(full_path, sizeof(full_path), "%s/%s.mjpeg", dir_path, received_packet.timestamp);

        ESP_LOGI(TAG, "%s", full_path);

        FILE* f = fopen(full_path, "wb");
        for (int i = frame_buffer_read_ind; i != frame_buffer_write_ind; i = (i + 1) % (CAM_FRAMERATE_HZ * CAM_VIDEO_DURATION_S)) {
            fwrite(frame_buffer[i].jpeg, 1, frame_buffer[i].length, f);
            free(frame_buffer[i].jpeg);
        }
        fclose(f);

        ESP_LOGI(TAG, "Finished recording!");
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    cam_timestamp_packet new_packet;
    strcpy(new_packet.timestamp, "");
    xQueueSendFromISR(cam_timestamp_handle, ( void * ) &new_packet, NULL);
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
    setup_camera();
    gpio_setup();

    xTaskCreate(
        record_circular_camera_video, 
        "temp_sens_task", 
        50000, 
        NULL, 
        10, 
        NULL
    );


    cam_timestamp_packet new_packet;

    vTaskDelay(15000 / portTICK_PERIOD_MS);
    snprintf(
        new_packet.timestamp, 
        sizeof(new_packet.timestamp), 
        "crash_%d", 
        next_recording_index
    );
    next_recording_index += 1;
    xQueueSend(cam_timestamp_handle, ( void * ) &new_packet, portMAX_DELAY);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    snprintf(
        new_packet.timestamp, 
        sizeof(new_packet.timestamp), 
        "crash_%d", 
        next_recording_index
    );
    next_recording_index += 1;
    xQueueSend(cam_timestamp_handle, ( void * ) &new_packet, portMAX_DELAY);
    

    vTaskDelay(portMAX_DELAY);
}