set(srcs "."
         "ntfw_ble_fmwk.c"
         "ntfw_ble_msg.c")

idf_component_register(SRCS "${srcs}"
                    REQUIRES driver efuse esp_timer esp_adc lwip vfs esp_wifi bt esp_event esp_netif esp_eth esp_phy freertos fatfs json mbedtls ntfw_com ntfw_crypto ntfw_io
                    INCLUDE_DIRS "." "include")
