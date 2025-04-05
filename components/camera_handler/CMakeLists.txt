idf_component_register(SRCS "camera_handler.c" "camera_handler.c"
                    INCLUDE_DIRS "."
                    REQUIRES "esp32-camera" "log" "freertos" "driver"
                    )
