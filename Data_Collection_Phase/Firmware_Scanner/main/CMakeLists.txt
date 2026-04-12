idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash log dc_protocol drv_ble_scan drv_uart_tx)