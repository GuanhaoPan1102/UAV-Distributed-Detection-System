idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES drv_espnow_rx app_dashboard dc_protocol_rc nvs_flash)
