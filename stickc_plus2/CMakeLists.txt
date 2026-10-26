idf_component_register(SRCS "main.c" "adpcm.c" "smp_ota.c" "display.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver bt nvs_flash app_update esp_partition mbedtls
                                   esp_driver_i2s esp_driver_gpio esp_driver_uart esp_timer
                                   esp_lcd esp_driver_spi)
