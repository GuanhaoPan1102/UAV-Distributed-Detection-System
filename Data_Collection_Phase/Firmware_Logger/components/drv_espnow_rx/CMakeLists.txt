idf_component_register(SRCS "drv_espnow_rx.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_event dc_protocol freertos log)