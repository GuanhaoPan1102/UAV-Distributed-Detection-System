idf_component_register(SRCS "drv_uart_tx.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver dc_protocol log)