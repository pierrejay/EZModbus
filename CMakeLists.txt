# CMakeLists.txt for EZModbus component

set(SRCS
    "src/apps/ModbusClient.cpp"
    "src/apps/ModbusServer.cpp"
    "src/drivers/ModbusHAL_TCP.cpp"
    "src/drivers/ModbusHAL_UART.cpp"
    "src/interfaces/ModbusRTU.cpp"
    "src/interfaces/ModbusTCP.cpp"
)

set(INCLUDE_DIRS "src")

# IDF 6 no longer exposes UART/GPIO through driver. Keep driver for consumers
# that rely on its other transitive dependencies in IDF 5.
idf_component_register(SRCS "${SRCS}"
                       INCLUDE_DIRS "${INCLUDE_DIRS}"
                       REQUIRES driver esp_driver_uart esp_driver_gpio freertos lwip esp_timer esp_netif esp_rom
                       )
