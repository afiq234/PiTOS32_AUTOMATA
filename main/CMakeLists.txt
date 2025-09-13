idf_component_register(SRCS "stepper_motor_encoder.c" "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_driver_spi esp_driver_gpio esp_driver_mcpwm esp_driver_uart esp_driver_rmt
                       INCLUDE_DIRS "")
