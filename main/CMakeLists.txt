idf_component_register(SRCS "gatts_demo.c" "dht11.c"
                    PRIV_REQUIRES bt nvs_flash
                    PRIV_REQUIRES esp_driver_gpio
                    INCLUDE_DIRS ".")
