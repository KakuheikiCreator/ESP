set(srcs "."
         "ntfw_cryptography.c")

idf_component_register(SRCS "${srcs}"
                    REQUIRES driver efuse esp_timer lwip vfs esp_wifi esp_event esp_netif esp_eth esp_phy mbedtls ntfw_com ntfw_crypto
                    INCLUDE_DIRS "." "include")
