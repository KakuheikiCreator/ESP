set(srcs "."
         "ntfw_io_file_util.c"
         "ntfw_io_gpio_util.c"
         "ntfw_io_i2c_master.c"
         "ntfw_io_touchpad_fmwk.c")

idf_component_register(SRCS "${srcs}"
                    REQUIRES driver efuse esp_timer esp_adc lwip vfs esp_wifi esp_event esp_netif esp_eth esp_phy freertos fatfs json mbedtls ntfw_com ntfw_crypto ntfw_io
                    INCLUDE_DIRS "." "include")
