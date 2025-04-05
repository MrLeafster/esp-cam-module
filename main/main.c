#include <stdio.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera_handler.h"
#include "sdcard.h"

#define TAG "MAIN"

static void IRAM_ATTR gpio_isr_handler(void* arg)
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

void gpio_setup()
{
    gpio_set_direction(12, GPIO_MODE_INPUT);
    gpio_set_intr_type(12, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(12, GPIO_INTR_ANYEDGE);

    gpio_isr_handler_add(12, gpio_isr_handler, NULL);
}

void app_main(void)
{
    sdcard_init();
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