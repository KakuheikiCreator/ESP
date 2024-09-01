# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c         # list the source files of this component
    INCLUDE_DIRS        # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES  driver efuse nvs_flash esp_timer esp_adc lwip vfs esp_wifi bt esp_event esp_netif esp_eth esp_phy freertos fatfs json mbedtls ntfw_com ntfw_crypto ntfw_io ntfw_ble ntfw_driver # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
)
