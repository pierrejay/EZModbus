/**
 * @file ModbusHAL_UART.cpp
 * @brief Multi-platform HAL wrapper implementation selector
 */

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
    #include "ModbusHAL_UART_ESP32.inl"
#elif defined(STM32_HAL)
    #include "ModbusHAL_UART_STM32.inl"
#elif defined(PICO_SDK)
    #include "ModbusHAL_UART_Pico.inl"
#endif