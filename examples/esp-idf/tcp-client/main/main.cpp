/**
 * @file main.cpp
 * @brief Example using EZModbus for a Modbus TCP Client application
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

    int Modbus::Debug::printLog(const char* msg, size_t len) {
        return ESP32_LogPrint(msg, len);
    }
#endif

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char *TAG_APP   = "MODBUS_TCP_EX";
static const char *TAG_TASK  = "CLIENT_TASK";


// ===================================================================================
// EZMODBUS ALIASES
// ===================================================================================

// Just for convenience
using TCP          = ModbusHAL::TCP;
using ModbusTCP    = ModbusInterface::TCP;
using ModbusClient = Modbus::Client;
using ModbusFrame  = Modbus::Frame;


// ===================================================================================
// WI-FI CONFIGURATION
// ===================================================================================

// Connection settings
#define WIFI_SSID      "My-WiFi-Network"   // Your SSID here
#define WIFI_PASSWORD  "mypassword1234"    // Your password here
#define MAX_RETRY      10

// Wi-Fi instance
WiFiSTA wifi(WIFI_SSID, WIFI_PASSWORD, MAX_RETRY);


// ===================================================================================
// TCP CONFIGURATION
// ===================================================================================

#define MODBUS_SERVER_IP   "192.168.1.24"  // Remote Server IP address
#define MODBUS_SERVER_PORT 502             // Remote Server TCP port


// ===================================================================================
// MODBUS CONFIGURATION & INSTANCES
// ===================================================================================

#define THERMOSTAT_SLAVE_ID 1  // Slave ID of the remote slave (Modbus TCP -> value doesn't matter)

// TCP communication
TCP tcp(MODBUS_SERVER_IP, MODBUS_SERVER_PORT);

// Modbus TCP interface
ModbusTCP interface(tcp, Modbus::CLIENT);

// Modbus Client application
ModbusClient client(interface);


// ===================================================================================
// EXAMPLE CONFIGURATION
// ===================================================================================

// Register map for our example thermostat
namespace RegAddr {
    constexpr uint16_t REG_TEMP_REGULATION_ENABLE = 100;      // Coil
    constexpr uint16_t REG_ALARM_START            = 200;      // Discrete Inputs (200-209)
    constexpr uint16_t REG_CURRENT_TEMPERATURE    = 300;      // Input Register (°C × 10)
    constexpr uint16_t REG_CURRENT_HUMIDITY       = 301;      // Input Register (% × 10)
    constexpr uint16_t REG_TEMPERATURE_SETPOINT   = 400;      // Holding Register (°C × 10)
    constexpr uint16_t REG_HUMIDITY_SETPOINT      = 401;      // Holding Register (% × 10)
}

// Example functions prototypes
static void read_temperature_sync(ModbusClient& client);
static void read_alarms_async(ModbusClient& client);
static void read_setpoints_sync(ModbusClient& client);
static void write_setpoints_async(ModbusClient& client);
static void clientTask(void* arg);


// ===================================================================================
// MAIN
// ===================================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_APP, "Starting Modbus TCP Client Example (ESP-IDF)");

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
    
    // Initialize TCP driver
    bool tcpInitResult = tcp.begin();
    if (!tcpInitResult) {
        ESP_LOGE(TAG_APP, "TCP driver init failed");
        return;
    }

    // Initialize Modbus Client
    ModbusClient::Result clientInitResult = client.begin();
    if (clientInitResult != Modbus::Client::SUCCESS) {
        ESP_LOGE(TAG_APP, "Failed to initialize Modbus Client: %s", Modbus::Client::toString(clientInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Modbus TCP Client initialized");


    // ===================================================================================
    // APP LAUNCH
    // ===================================================================================

    // Launch Modbus Client task
    xTaskCreate(clientTask, "modbusClientTask", 4096, &client, 5, NULL);

    // Main loop (nothing to do here)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 


// ===================================================================================
// EXAMPLE FUNCTIONS
// ===================================================================================

// Client task: executes sample requests
static void clientTask(void* arg)
{
    while (true) {
        ESP_LOGI(TAG_TASK, "========= EZModbus TCP Client Examples =========");
        read_temperature_sync(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        read_alarms_async(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        read_setpoints_sync(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        write_setpoints_async(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG_TASK, "========= Cycle complete – restarting in 10 s =========");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Read temperature (synchronous) - Using HELPER API
static void read_temperature_sync(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Reading current temperature (using helper API)...");

    uint16_t tempRaw;
    Modbus::ExceptionCode excep;
    auto result = client.read(THERMOSTAT_SLAVE_ID, Modbus::INPUT_REGISTER, 
                             RegAddr::REG_CURRENT_TEMPERATURE, 1, &tempRaw, &excep);
    
    if (result == ModbusClient::SUCCESS) {
        float temp = tempRaw / 10.0f;
        ESP_LOGI(TAG_TASK, "Temperature: %.1f°C", temp);
    } else if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
        ESP_LOGE(TAG_TASK, "Modbus exception: %s", Modbus::toString(excep));
    } else {
        ESP_LOGE(TAG_TASK, "Failed to read temperature: %s", ModbusClient::toString(result));
    }
}

// Read alarms (asynchronous using tracker)
static void read_alarms_async(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Reading alarms (async)...");

    ModbusFrame req = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_DISCRETE_INPUTS,
        .slaveId    = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_ALARM_START,
        .regCount   = 10
    };

    ModbusFrame resp;
    ModbusClient::Result tracker;
    auto result = client.sendRequest(req, resp, &tracker);
    if (result != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Failed to start alarm read: %s", ModbusClient::toString(result));
        return;
    }

    TickType_t start = xTaskGetTickCount();
    while (tracker == ModbusClient::NODATA) {
        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > 2000) {
            ESP_LOGW(TAG_TASK, "Alarm response timeout");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (tracker == ModbusClient::ERR_EXCEPTION_RESPONSE) {
        ESP_LOGE(TAG_TASK, "Modbus exception: %s", Modbus::toString(resp.exceptionCode));
        return;
    } else if (tracker != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Alarm read failed: %s", ModbusClient::toString(tracker));
        return;
    }

    for (size_t i = 0; i < resp.regCount; ++i) {
        ESP_LOGI(TAG_TASK, "Alarm %d: %s", (int)i, resp.getCoil(i) ? "ACTIVE" : "inactive");
    }
}

// Read setpoints (synchronous)
static void read_setpoints_sync(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Reading setpoints...");

    ModbusFrame req = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_TEMPERATURE_SETPOINT,
        .regCount   = 2
    };

    ModbusFrame resp;
    auto result = client.sendRequest(req, resp);
    if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
        ESP_LOGE(TAG_TASK, "Modbus exception: %s", Modbus::toString(resp.exceptionCode));
        return;
    } else if (result != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Failed to read setpoints: %s", ModbusClient::toString(result));
        return;
    }
    if (resp.regCount < 2) {
        ESP_LOGW(TAG_TASK, "Invalid response length");
        return;
    }

    float tempSet = resp.getRegister(0) / 10.0f;
    float humSet  = resp.getRegister(1) / 10.0f;
    ESP_LOGI(TAG_TASK, "Temp setpoint: %.1f°C | Hum setpoint: %.1f%%", tempSet, humSet);
}

// Write setpoints (asynchronous using callback)
static void write_setpoints_async(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Writing setpoints (async)...");

    ModbusFrame req = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::WRITE_MULTIPLE_REGISTERS,
        .slaveId    = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_TEMPERATURE_SETPOINT,
        .regCount   = 2,
        .data       = Modbus::packRegisters({225, 450})
    };

    static uint32_t updates = 0;
    static auto cb = [](ModbusClient::Result res, const ModbusFrame *resp, void *ctx) {
        if (res == ModbusClient::SUCCESS) {
            ESP_LOGI(TAG_TASK, "Callback: write SUCCESS!");
        } else if (res == ModbusClient::ERR_EXCEPTION_RESPONSE && resp) {
            ESP_LOGE(TAG_TASK, "Callback: Modbus exception %s", Modbus::toString(resp->exceptionCode));
        } else {
            ESP_LOGE(TAG_TASK, "Callback: write FAILED (%s)", ModbusClient::toString(res));
        }
        updates++;
        (void)ctx;
    };

    auto result = client.sendRequest(req, cb, nullptr);
    if (result != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Failed to queue write request: %s", ModbusClient::toString(result));
    }
}