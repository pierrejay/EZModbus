idf_component_register(SRCS "main.cpp"
                    PRIV_REQUIRES ezmodbus spi_flash freertos log driver esp_wifi esp_event esp_netif
                    INCLUDE_DIRS "../include") 