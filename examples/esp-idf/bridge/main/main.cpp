/**
 * @file main.cpp
 * @brief Example using EZModbus for a Modbus Bridge application
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "WiFiSTA.hpp"
#include "EZModbus.h"

// ===================================================================================
// EZMODBUS DEBUG CONFIGURATION (optional if debug is not enabled)
// ===================================================================================

#ifdef EZMODBUS_DEBUG
    // Debug print function for EZModbus - outputs to ESP32 console
    int ESP32_LogPrint(const char* msg, size_t len) {
        return fwrite(msg, 1, len, stdout);
    }

    // Automatically register debug function
    static Modbus::Debug::PrintFunctionSetter func(ESP32_LogPrint);
#endif

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char *TAG_APP  = "BRIDGE_EX";


// ===================================================================================
// EZMODBUS ALIASES
// ===================================================================================

// Just for convenience
using UART          = ModbusHAL::UART;
using UARTConfig    = ModbusHAL::UART::Config;
using TCP           = ModbusHAL::TCP;
using ModbusRTU     = ModbusInterface::RTU;
using ModbusTCP     = ModbusInterface::TCP;
using ModbusBridge  = Modbus::Bridge;


// ===================================================================================
// WI-FI CONFIGURATION
// ===================================================================================

// Connection settings
#define WIFI_SSID     "My-WiFi-Network"
#define WIFI_PASSWORD "mypassword1234"
#define MAX_RETRY     10

// Wi-Fi instance
WiFiSTA wifi(WIFI_SSID, WIFI_PASSWORD, MAX_RETRY);


// ===================================================================================
// UART & TCP CONFIGURATION
// ===================================================================================

// UART configuration 
// (side of the bridge connected to RTU slave devices)
UARTConfig uartCfg = {
    .uartNum = UART_NUM_1,
    .baud    = 9600,
    .config  = UART::CONFIG_8N1,
    .rxPin   = GPIO_NUM_16,
    .txPin   = GPIO_NUM_17,
    .dePin   = GPIO_NUM_5
};

// TCP configuration 
// (side of the bridge connected to TCP clients)
constexpr uint16_t MODBUS_TCP_PORT = 502;  // Server local port


// ===================================================================================
// MODBUS CONFIGURATION & INSTANCES
// ===================================================================================

// UART & TCP communication
UART uart(uartCfg);
TCP tcpServer(MODBUS_TCP_PORT);

// Modbus RTU & TCP interfaces
ModbusRTU  rtuInterface(uart, Modbus::CLIENT);
ModbusTCP  tcpInterface(tcpServer, Modbus::SERVER);

// Modbus Bridge application
ModbusBridge bridge(rtuInterface, tcpInterface);


// ===================================================================================
// MAIN
// ===================================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_APP, "Starting Modbus Bridge RTU<->TCP (ESP-IDF)");

    // ===================================================================================
    // WIFI INIT
    // ===================================================================================

    ESP_LOGI(TAG_APP, "Connecting to WiFi...");
    esp_err_t wifi_result = wifi.begin(15000);  // 15s timeout
    if (wifi_result != ESP_OK) {
        ESP_LOGE(TAG_APP, "WiFi connection failed");
        return;
    }
    ESP_LOGI(TAG_APP, "WiFi connected successfully");


    // ===================================================================================
    // MODBUS INIT
    // ===================================================================================

    // Initialize UART & TCP drivers
    esp_err_t uartInitResult = uart.begin();
    if (uartInitResult != ESP_OK) {
        ESP_LOGE(TAG_APP, "UART driver init failed: %s", esp_err_to_name(uartInitResult));
        return;
    }
    bool tcpInitResult = tcpServer.begin();
    if (!tcpInitResult) {
        ESP_LOGE(TAG_APP, "TCP driver init failed");
        return;
    }

    // Initialize Modbus Bridge
    ModbusBridge::Result bridgeInitResult = bridge.begin();
    if (bridgeInitResult != ModbusBridge::SUCCESS) {
        ESP_LOGE(TAG_APP, "Bridge initialization failed: %s", ModbusBridge::toString(bridgeInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Bridge initialized");


    // ===================================================================================
    // APP LAUNCH
    // ===================================================================================

    // Main loop (nothing to do here, bridge runs internally)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 