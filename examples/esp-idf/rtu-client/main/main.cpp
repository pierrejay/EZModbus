/**
 * @file main.cpp
 * @brief Example using EZModbus for a Modbus RTU Client application
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "EZModbus.h"

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char *TAG_APP  = "MODBUS_RTU_EX";
static const char *TAG_TASK = "CLIENT_TASK";


// ===================================================================================
// EZMODBUS ALIASES
// ===================================================================================

// Just for convenience
using UART          = ModbusHAL::UART;
using UARTConfig    = ModbusHAL::UART::Config;
using ModbusRTU     = ModbusInterface::RTU;
using ModbusClient  = Modbus::Client;
using ModbusFrame   = Modbus::Frame;


// ===================================================================================
// UART CONFIGURATION
// ===================================================================================

UARTConfig uartCfg = {
    .uartNum = UART_NUM_1,
    .baud    = 9600,
    .config  = UART::CONFIG_8N1,
    .rxPin   = GPIO_NUM_16,
    .txPin   = GPIO_NUM_17,
    .dePin   = GPIO_NUM_5
};


// ===================================================================================
// MODBUS CONFIGURATION & INSTANCES
// ===================================================================================

// Client configuration
#define THERMOSTAT_SLAVE_ID 1   // Slave ID of the remote slave (thermostat)

// UART communication
UART uart(uartCfg);

// Modbus RTU interface
ModbusRTU interface(uart, Modbus::CLIENT);

// Modbus Client application
ModbusClient client(interface);


// ===================================================================================
// EXAMPLE CONFIGURATION
// ===================================================================================

// Register map for our example thermostat
namespace RegAddr {
    constexpr uint16_t REG_TEMP_REGULATION_ENABLE = 100; // Coil
    constexpr uint16_t REG_ALARM_START            = 200; // Discrete Inputs (200-209)
    constexpr uint16_t REG_CURRENT_TEMPERATURE    = 300; // Input Register (°C × 10)
    constexpr uint16_t REG_CURRENT_HUMIDITY       = 301; // Input Register (% × 10)
    constexpr uint16_t REG_TEMPERATURE_SETPOINT   = 400; // Holding Register (°C × 10)
    constexpr uint16_t REG_HUMIDITY_SETPOINT      = 401; // Holding Register (% × 10)
}

// Example functions prototypes
static void readTemperature_Simple(ModbusClient& client);
static void readSetpoints_Sync(ModbusClient& client);
static void readAlarms_Async(ModbusClient& client);
static void writeSetpoints_Callback(ModbusClient& client);
static void clientTask(void* arg);


// ===================================================================================
// MAIN
// ===================================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_APP, "Starting Modbus RTU Client Example (ESP-IDF)");

    // ===================================================================================
    // MODBUS INIT
    // ===================================================================================

    // Initialize UART
    esp_err_t uartInitResult = uart.begin();
    if (uartInitResult != ESP_OK) {
        ESP_LOGE(TAG_APP, "Failed to initialize UART HAL: %s", esp_err_to_name(uartInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "UART initialized (baud %lu)", uartCfg.baud);


    // Initialize Modbus Client
    ModbusClient::Result clientInitResult = client.begin();
    if (clientInitResult != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_APP, "Failed to initialize Modbus Client: %s", ModbusClient::toString(clientInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Modbus RTU Client initialized");


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
        ESP_LOGI(TAG_TASK, "========== Starting EZModbus Examples ==========");

        // Example 1: Simple read
        ESP_LOGI(TAG_TASK, "");
        ESP_LOGI(TAG_TASK, "****** EXAMPLE 1: Simple Read ******");
        readTemperature_Simple(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Example 2: Synchronous read using raw frame
        ESP_LOGI(TAG_TASK, "");
        ESP_LOGI(TAG_TASK, "****** EXAMPLE 2: Synchronous Read ******");
        readSetpoints_Sync(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Example 3: Asynchronous read
        ESP_LOGI(TAG_TASK, "");
        ESP_LOGI(TAG_TASK, "****** EXAMPLE 3: Asynchronous Read ******");
        readAlarms_Async(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Example 4: Asynchronous write with callback
        ESP_LOGI(TAG_TASK, "");
        ESP_LOGI(TAG_TASK, "****** EXAMPLE 4: Asynchronous Write with Callback ******");
        writeSetpoints_Callback(client);
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG_TASK, "");
        ESP_LOGI(TAG_TASK, "========== All Examples Completed ==========");
        ESP_LOGI(TAG_TASK, "Waiting 10 seconds before running again...");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/**
 * Example 1: Synchronous read of current temperature using simple read/write method
 */
static void readTemperature_Simple(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Reading current temperature...");

    uint16_t tempRaw;
    Modbus::ExceptionCode excep;

    // Read one input register synchronously
    auto result = client.read(THERMOSTAT_SLAVE_ID, Modbus::INPUT_REGISTER,
                              RegAddr::REG_CURRENT_TEMPERATURE, 1, &tempRaw, &excep);

    // Check outcome
    if (result != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Communication error: %s", ModbusClient::toString(result));
        return;
    } else if (excep) {
        ESP_LOGE(TAG_TASK, "Modbus exception: %s", Modbus::toString(excep));
        return;
    } else {
        float temp = tempRaw / 10.0f;
        ESP_LOGI(TAG_TASK, "Temperature: %.1f°C", temp);
    }
}

/**
 * Example 2: Synchronous read of setpoints using raw frame
 */
static void readSetpoints_Sync(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Reading temperature and humidity setpoints...");

    // Create frame to read multiple holding registers
    ModbusFrame request = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_TEMPERATURE_SETPOINT,
        .regCount   = 2,  // Read both temperature and humidity setpoints
        .data       = {}
    };

    // Send request and wait for response
    // (tracker not provided -> waits until response received or timeout)
    ModbusFrame response;
    auto result = client.sendRequest(request, response);

    // Check if the request was successful
    if (result != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Failed to start setpoint read: %s", ModbusClient::toString(result));
        return;
    }

    // Check if the response has an exception
    if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
        ESP_LOGE(TAG_TASK, "Modbus exception reading setpoints: %s", Modbus::toString(response.exceptionCode));
        return;
    }

    // Check if the response has the correct number of registers
    if (response.regCount < 2) {
        ESP_LOGW(TAG_TASK, "Invalid response format");
        return;
    }

    // Get the temperature and humidity setpoints from the response
    float tempSetpoint = response.getRegister(0) / 10.0f;
    float humSetpoint = response.getRegister(1) / 10.0f;

    // Print the read setpoints
    ESP_LOGI(TAG_TASK, "Temperature setpoint: %.1f°C", tempSetpoint);
    ESP_LOGI(TAG_TASK, "Humidity setpoint: %.1f%%", humSetpoint);
}

/**
 * Example 3: Asynchronous read of alarm status
 */
static void readAlarms_Async(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Reading alarm status...");

    // Create frame to read multiple discrete inputs
    ModbusFrame request = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_DISCRETE_INPUTS,
        .slaveId    = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_ALARM_START,
        .regCount   = 10,  // Read 10 alarms
        .data       = {}
    };

    // Create frame for response and status tracker
    ModbusFrame response;
    ModbusClient::Result tracker;

    // Send request asynchronously
    // (tracker provided -> returns immediately after transfer started)
    auto result = client.sendRequest(request, response, &tracker);

    if (result != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Failed to start alarm read: %s", ModbusClient::toString(result));
        return;
    }

    ESP_LOGI(TAG_TASK, "Alarm read request sent. Waiting for completion...");

    // Wait for the request to complete
    // (usually this is done in another task/function)
    while (tracker == ModbusClient::NODATA) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Check if the request was successful
    if (tracker != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Failed to start alarm read: %s", ModbusClient::toString(tracker));
        return;
    } else if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
        ESP_LOGE(TAG_TASK, "Modbus exception reading alarms: %s", Modbus::toString(response.exceptionCode));
        return;
    }

    // Print all alarm states
    ESP_LOGI(TAG_TASK, "Alarm read complete!");
    for (size_t i = 0; i < response.regCount; ++i) {
        ESP_LOGI(TAG_TASK, "Alarm %d: %s", (int)i, response.getCoil(i) ? "ACTIVE" : "inactive");
    }
}

/**
 * Example 4: Asynchronous write of setpoints using the callback API
 */
static void writeSetpoints_Callback(ModbusClient& client)
{
    ESP_LOGI(TAG_TASK, "Writing temperature and humidity setpoints (callback mode)...");

    // Two variables that we want to update from the callback
    static volatile uint32_t totalUpdates = 0;
    static volatile uint32_t lastUpdateTime = 0;

    // Build frame to write both setpoints (22.5 °C & 45 % RH)
    ModbusFrame request = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::WRITE_MULTIPLE_REGISTERS,
        .slaveId    = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_TEMPERATURE_SETPOINT,
        .regCount   = 2,
        .data       = Modbus::packRegisters({225, 450})
    };

    // Simple context shared with the callback
    struct CbCtx {
        volatile uint32_t& nb = totalUpdates;
        volatile uint32_t& time = lastUpdateTime;
    } ctx;

    // Static, non-capturing lambda -> decays to a function pointer
    static auto cb = [](ModbusClient::Result res, const Modbus::Frame* resp, void* ctx) {
        auto* c = static_cast<CbCtx*>(ctx);

        if (res == ModbusClient::SUCCESS && resp && resp->exceptionCode == Modbus::NULL_EXCEPTION) {
            ESP_LOGI(TAG_TASK, "Callback: write SUCCESS!");
        } else {
            ESP_LOGE(TAG_TASK, "Callback: write FAILED (%s)", ModbusClient::toString(res));
        }

        if (c) {
            c->nb++;
            c->time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
    };

    // Launch request (returns immediately)
    auto result = client.sendRequest(request, cb, &ctx);
    if (result != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_TASK, "Failed to queue write request: %s", ModbusClient::toString(result));
        return;
    }

    // Fire & forget: no need to wait for the request to complete!
    // The callback will handle everything in the background
}
