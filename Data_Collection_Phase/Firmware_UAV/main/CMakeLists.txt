idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash log dc_protocol drv_gps_neo6m drv_ble_adv)
