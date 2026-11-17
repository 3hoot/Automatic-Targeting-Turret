idf_component_register(
    SRCS 
        "framed_serial.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        freertos driver esp_tinyusb esp_driver_uart esp_driver_gpio esp_timer
)