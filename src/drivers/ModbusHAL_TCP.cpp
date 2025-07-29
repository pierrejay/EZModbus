/**
 * @file ModbusHAL_TCP.cpp
 * @brief Multi-platform HAL wrapper implementation selector
 */

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
    #include "ModbusHAL_TCP_ESP32.inl"
#elif defined(PICO_SDK)
    #include "ModbusHAL_TCP_Pico_CH9120.inl"
#endif