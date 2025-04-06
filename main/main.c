#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera_handler.h"
#include "sdcard.h"

#define TAG "MAIN"
#define USE_DEMO

void demo_triggers()
{
    cam_timestamp_packet new_packet;

    vTaskDelay(15000 / portTICK_PERIOD_MS);
    snprintf(
        new_packet.timestamp, 
        sizeof(new_packet.timestamp), 
        "crash_%d", 
        camera_handler_get_next_index()
    );
    camera_handler_save_buffer(&new_packet);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
    snprintf(
        new_packet.timestamp, 
        sizeof(new_packet.timestamp), 
        "crash_%d", 
        camera_handler_get_next_index()
    );
    camera_handler_save_buffer(&new_packet);
}

void app_main(void)
{
    sdcard_init();
    camera_handler_init();

#ifdef USE_DEMO
    demo_triggers();
#endif

    vTaskDelay(portMAX_DELAY);
}