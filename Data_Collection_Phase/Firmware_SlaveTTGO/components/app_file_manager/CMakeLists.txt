idf_component_register(SRCS "app_file_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES fatfs dc_protocol drv_espnow_tx esp_timer driver)