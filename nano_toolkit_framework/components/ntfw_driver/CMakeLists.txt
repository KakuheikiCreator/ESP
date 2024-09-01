set(srcs "."
         "ntfw_drv_adxl345.c"
         "ntfw_drv_lis3dh.c"
         "ntfw_drv_mpu-6050.c"
         "ntfw_drv_rx8900.c"
         "ntfw_drv_st7032i.c")

idf_component_register(SRCS "${srcs}"
                    REQUIRES driver efuse esp_timer esp_adc lwip vfs esp_wifi esp_event esp_netif esp_eth esp_phy freertos fatfs json mbedtls ntfw_com ntfw_crypto ntfw_io
                    INCLUDE_DIRS "." "include")

