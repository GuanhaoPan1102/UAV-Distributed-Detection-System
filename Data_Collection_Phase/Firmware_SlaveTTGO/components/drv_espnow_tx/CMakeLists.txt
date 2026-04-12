idf_component_register(SRCS "drv_espnow_tx.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_event dc_protocol log)