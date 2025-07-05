set(srcs "."
         "ntfw_io_file_util.c"
         "ntfw_io_gpio_util.c"
         "ntfw_io_i2c_mst.c"
         "ntfw_io_touchpad_fmwk.c")

idf_component_register(SRCS "${srcs}"
                       REQUIRES driver efuse esp_timer esp_event esp_phy esp_adc esp_driver_i2c esp_driver_sdmmc esp_netif esp_wifi esp_eth freertos vfs fatfs json lwip mbedtls ntfw_com ntfw_crypto ntfw_io
                       INCLUDE_DIRS "." "include")
