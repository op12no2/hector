# the same hector.c as the Pi build; its I/O section has the ESP_PLATFORM side
idf_component_register(SRCS "../../hector.c"
                       PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_driver_usb_serial_jtag)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
