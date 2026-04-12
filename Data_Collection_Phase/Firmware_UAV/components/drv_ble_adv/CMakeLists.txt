idf_component_register(SRCS "drv_ble_adv.c"
                    INCLUDE_DIRS "include"
                    REQUIRES freertos log bt dc_protocol drv_gps_neo6m)
