idf_component_register(SRCS "sdcard.c"
                    INCLUDE_DIRS "."
                    REQUIRES "sdmmc" "log" "freertos" "fatfs"
                    )
