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
        ESP_LOGI(TAG_TASK, "========= EZModbus RTU Client Examples =========");
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
