/**
 * @file main.cpp
 * @brief Example using EZModbus for a Modbus TCP Server application
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "WiFiSTA.hpp"
#include "EZModbus.h"

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char *TAG_APP  = "TCP_SERVER_EX";
static const char *TAG_DATA = "DATA";


// ===================================================================================
// EZMODBUS ALIASES
// ===================================================================================

// Just for convenience
using TCP          = ModbusHAL::TCP;
using ModbusTCP    = ModbusInterface::TCP;
using ModbusServer = Modbus::Server;
using ModbusWord   = Modbus::Word;


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

#define MODBUS_PORT    502  // Server local port


// ===================================================================================
// MODBUS CONFIGURATION & INSTANCES
// ===================================================================================

// TCP communication instance
TCP tcpServer(MODBUS_PORT);

// Modbus TCP interface instance
ModbusTCP interfaceTCP(tcpServer, Modbus::SERVER);

// Modbus Server storage & application instance
Modbus::StaticWordStore<200> wordStore; // Static storage for Modbus words
ModbusServer server(interfaceTCP, wordStore);


// ===================================================================================
// EXAMPLE CONFIGURATION
// ===================================================================================

// Register map for our example Modbus Server
namespace RegAddr {
    constexpr uint16_t SENSOR_TEMP     = 100; // Direct pointer
    constexpr uint16_t SENSOR_HUMIDITY = 101; // Direct pointer
    constexpr uint16_t CONFIG_SETPOINT = 200; // Single-reg handlers
    constexpr uint16_t CONFIG_MODE     = 201;
    constexpr uint16_t STATS_BLOCK     = 300; // 3 regs handler
    constexpr uint16_t FLOAT_VALUE     = 310; // 2 regs handler
    constexpr uint16_t TIMESTAMP       = 320; // 2 regs handler
    constexpr uint16_t STATUS_FLAGS    = 400; // Coil pointer
    constexpr uint16_t ALARM_FLAGS     = 500; // Discrete input pointer
}

// Demo data container to store values
struct DemoData {
    volatile uint16_t sensorTemp     = 250;  // 25.0 °C ×10
    volatile uint16_t sensorHumidity = 600;  // 60.0 % ×10
    uint16_t configSetpoint = 220;           // 22.0 °C ×10
    uint16_t configMode     = 1;             // 0=manual,1=auto,2=debug
    struct { uint16_t min = 180; uint16_t max = 320; uint16_t avg = 250; } tempStats;
    float    floatValue = 3.14159f;          // Example float
    uint32_t timestamp  = 1672531200;        // UNIX epoch
    volatile uint16_t statusFlags = 0b0101;  // Example bits
    volatile uint16_t alarmFlags  = 0b0010;
    uint32_t updateCounter = 0;
} demo;

// Example functions prototypes
static void logWordAccess(const char* op, const ModbusWord& word, const uint16_t* vals, size_t count);
static void setupWords();
static void simulationTask(void* arg);

// Server Word handlers prototypes
static Modbus::ExceptionCode readSetpoint(const ModbusWord& word, uint16_t* outVals, void* ctx);
static Modbus::ExceptionCode writeSetpoint(const uint16_t* writeVals, const ModbusWord& word, void* ctx);
static Modbus::ExceptionCode readConfigMode(const ModbusWord& word, uint16_t* outVals, void* ctx);
static Modbus::ExceptionCode writeConfigMode(const uint16_t* writeVals, const ModbusWord& word, void* ctx);


// ===================================================================================
// MAIN
// ===================================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_APP, "Starting Modbus TCP Server (ESP-IDF)");

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
    bool tcpInitResult = tcpServer.begin();
    if (!tcpInitResult) {
        ESP_LOGE(TAG_APP, "Failed to initialize TCP HAL");
        return;
    }

    // Register Modbus Words on the server
    setupWords();

    // Initialize Modbus Server
    ModbusServer::Result serverInitResult = server.begin();
    if (serverInitResult != ModbusServer::SUCCESS) {
        ESP_LOGE(TAG_APP, "Modbus server initialization failed: %s", ModbusServer::toString(serverInitResult));
        return;
    }

    // ===================================================================================
    // APP LAUNCH
    // ===================================================================================

    // Launch simulation task
    xTaskCreate(simulationTask, "demoSim", 4096, nullptr, 5, nullptr);
    ESP_LOGI(TAG_APP, "Server ready on port %d", MODBUS_PORT);


    // Main loop (nothing to do here)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 


// ===================================================================================
// EXAMPLE HANDLERS & FUNCTIONS
// ===================================================================================

// Helper to log Word accesses
static void logWordAccess(const char* op, const ModbusWord& word, const uint16_t* vals, size_t count)
{
    if (!vals) return;
    ESP_LOGI(TAG_DATA, "[%s] Addr %u (%u regs) => %u...", op, word.startAddr, word.nbRegs, vals[0]);
}

// Read Setpoint handler
static Modbus::ExceptionCode readSetpoint(const ModbusWord& word, uint16_t* outVals, void*)
{
    outVals[0] = demo.configSetpoint;
    logWordAccess("READ", word, outVals, 1);
    return Modbus::NULL_EXCEPTION;
}

// Write Setpoint handler
static Modbus::ExceptionCode writeSetpoint(const uint16_t* writeVals, const ModbusWord& word, void*)
{
    uint16_t val = writeVals[0];
    if (val < 100 || val > 400) {
        return Modbus::ILLEGAL_DATA_VALUE;
    }
    demo.configSetpoint = val;
    logWordAccess("WRITE", word, writeVals, 1);
    return Modbus::NULL_EXCEPTION;
}

// Read Config Mode handler
static Modbus::ExceptionCode readConfigMode(const ModbusWord& word, uint16_t* outVals, void*)
{
    outVals[0] = demo.configMode;
    logWordAccess("READ", word, outVals, 1);
    return Modbus::NULL_EXCEPTION;
}

// Write Config Mode handler
static Modbus::ExceptionCode writeConfigMode(const uint16_t* writeVals, const ModbusWord& word, void*)
{
    uint16_t mode = writeVals[0];
    if (mode > 2) {
        return Modbus::ILLEGAL_DATA_VALUE;
    }
    demo.configMode = mode;
    logWordAccess("WRITE", word, writeVals, 1);
    return Modbus::NULL_EXCEPTION;
}

// Function to register all Words on the server
static void setupWords()
{
    // Method 1 : direct pointer access (single registers)
    server.addWord({Modbus::INPUT_REGISTER,  RegAddr::SENSOR_TEMP,     1, &demo.sensorTemp});
    server.addWord({Modbus::INPUT_REGISTER,  RegAddr::SENSOR_HUMIDITY, 1, &demo.sensorHumidity});
    server.addWord({Modbus::COIL,            RegAddr::STATUS_FLAGS,    1, &demo.statusFlags});
    server.addWord({Modbus::DISCRETE_INPUT,  RegAddr::ALARM_FLAGS,     1, &demo.alarmFlags});

    // Method 2 : single-register handlers
    server.addWord({Modbus::HOLDING_REGISTER, RegAddr::CONFIG_SETPOINT, 1, nullptr,  readSetpoint,  writeSetpoint});
    server.addWord({Modbus::HOLDING_REGISTER, RegAddr::CONFIG_MODE,     1, nullptr,  readConfigMode, writeConfigMode});

    // Method 3 : multi-register handlers

    // Block read handler
    server.addWord({
        Modbus::INPUT_REGISTER, // Type
        RegAddr::STATS_BLOCK,   // Start address
        3,                      // Number of registers
        nullptr,                // Value pointer
        // Read handler
        [](const ModbusWord& w, uint16_t* out, void*) {
            out[0] = demo.tempStats.min;
            out[1] = demo.tempStats.max;
            out[2] = demo.tempStats.avg;
            logWordAccess("READ", w, out, 3);
            return Modbus::NULL_EXCEPTION;
        }
    });

    // Float read/write handler
    server.addWord({
        Modbus::HOLDING_REGISTER, // Type
        RegAddr::FLOAT_VALUE,     // Start address
        2,                        // Number of registers
        nullptr,                  // Value pointer
        // Read handler
        [](const ModbusWord& w, uint16_t* out, void*) {
            ModbusCodec::floatToRegisters(demo.floatValue, out);
            logWordAccess("READ", w, out, 2);
            return Modbus::NULL_EXCEPTION;
        },
        // Write handler
        [](const uint16_t* v, const ModbusWord& w, void*) {
            demo.floatValue = ModbusCodec::registersToFloat(v);
            logWordAccess("WRITE", w, v, 2);
            return Modbus::NULL_EXCEPTION;
        }
    });

    // Timestamp read handler
    server.addWord({
        Modbus::INPUT_REGISTER, // Type
        RegAddr::TIMESTAMP,     // Start address
        2,                      // Number of registers
        nullptr,                // Value pointer
        // Read handler
        [](const ModbusWord& w, uint16_t* out, void*) {
            out[0] = (demo.timestamp >> 16) & 0xFFFF;
            out[1] =  demo.timestamp        & 0xFFFF;
            logWordAccess("READ", w, out, 2);
            return Modbus::NULL_EXCEPTION;
        }
    });

    ESP_LOGI(TAG_APP, "All Words registered");
}

// Simulation task: simulates change of values for demo purposes
static void simulationTask(void*)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        demo.updateCounter++;

        // Update demo values
        int16_t dT = (esp_random() % 41) - 20; // ±2.0 °C ×10
        demo.sensorTemp = std::clamp<uint16_t>(demo.sensorTemp + dT, 150, 350);
        demo.sensorHumidity = std::clamp<uint16_t>(demo.sensorHumidity + ((esp_random() % 101) - 50), 200, 900);

        demo.tempStats.min = std::min((uint16_t)demo.tempStats.min, (uint16_t)demo.sensorTemp);
        demo.tempStats.max = std::max((uint16_t)demo.tempStats.max, (uint16_t)demo.sensorTemp);
        demo.tempStats.avg = (demo.tempStats.min + demo.tempStats.max) / 2;

        demo.timestamp = xTaskGetTickCount() / configTICK_RATE_HZ;

        if (demo.sensorTemp > 300 || demo.sensorTemp < 200) demo.alarmFlags |= 0x0001; else demo.alarmFlags &= ~0x0001;

        ESP_LOGI(TAG_DATA, "Upd %lu | Temp %.1f°C | Hum %.1f%%", demo.updateCounter, demo.sensorTemp / 10.0f, demo.sensorHumidity / 10.0f);
    }
}