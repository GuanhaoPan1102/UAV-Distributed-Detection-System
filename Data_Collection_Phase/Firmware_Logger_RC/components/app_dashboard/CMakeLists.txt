idf_component_register(SRCS "app_dashboard.c"
                       INCLUDE_DIRS "include"
                       REQUIRES dc_protocol_rc freertos esp_timer log)