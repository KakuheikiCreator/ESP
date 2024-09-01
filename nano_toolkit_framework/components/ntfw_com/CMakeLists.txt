set(srcs "."
         "ntfw_com_data_model.c"
         "ntfw_com_date_time.c"
         "ntfw_com_debug_util.c"
         "ntfw_com_mem_alloc.c"
         "ntfw_com_value_util.c")

idf_component_register(SRCS "${srcs}"
                    REQUIRES driver efuse esp_timer lwip vfs esp_wifi esp_event esp_netif esp_eth esp_phy
                    INCLUDE_DIRS "." "include")
