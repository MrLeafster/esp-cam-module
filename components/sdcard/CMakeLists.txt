idf_component_register(SRCS "sdcard.c" "sdcard.c"
                    INCLUDE_DIRS "."
                    REQUIRES "fatfs" "log" "freertos" "sdmmc"
                    )
