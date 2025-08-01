idf_component_register(SRCS "main.cpp"
                    PRIV_REQUIRES ezmodbus spi_flash freertos log driver esp_wifi nvs_flash esp_event esp_netif
                    INCLUDE_DIRS "../include") 