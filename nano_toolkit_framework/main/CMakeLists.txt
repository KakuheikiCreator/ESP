idf_component_register(SRCS "main.c"
                    REQUIRES driver efuse nvs_flash esp_timer esp_adc lwip vfs esp_wifi bt esp_event esp_netif esp_eth esp_phy freertos fatfs json mbedtls ntfw_com ntfw_crypto ntfw_io ntfw_ble ntfw_driver
                    INCLUDE_DIRS ".")
