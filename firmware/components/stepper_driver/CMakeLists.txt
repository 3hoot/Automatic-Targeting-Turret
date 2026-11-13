idf_component_register(
    SRCS 
        "stepper_driver.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_driver_rmt esp_driver_gpio esp_timer
)