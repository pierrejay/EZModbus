/**
 * @file main.cpp
 * @brief Example using EZModbus for a Modbus RTU Client <-> Server Loopback application
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "EZModbus.h"

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char *TAG_APP = "MODBUS_LOOPBACK_APP";
static const char *TAG_CLIENT_TASK = "CLIENT_TASK";


// ===================================================================================
// EZMODBUS ALIASES
// ===================================================================================

// Just for convenience
using UART          = ModbusHAL::UART;
using UARTConfig    = ModbusHAL::UART::Config;
using ModbusRTU     = ModbusInterface::RTU;
using ModbusClient  = Modbus::Client;
using ModbusServer  = Modbus::Server;
using ModbusWord    = Modbus::Word;


// ===================================================================================
// UART CONFIGURATION
// ===================================================================================

// UART Server configuration
UARTConfig uartServerConfig = {
    .uartNum = UART_NUM_1,
    .baud = 115200,
    .config = UART::CONFIG_8N1,
    .rxPin = GPIO_NUM_44,
    .txPin = GPIO_NUM_7,
    .dePin = GPIO_NUM_NC
};

// UART Client configuration
UARTConfig uartClientConfig = {
    .uartNum = UART_NUM_2,
    .baud = 115200,
    .config = UART::CONFIG_8N1,
    .rxPin = GPIO_NUM_6,
    .txPin = GPIO_NUM_43,
    .dePin = GPIO_NUM_NC
};


// ===================================================================================
// MODBUS CONFIGURATION & INSTANCES
// ===================================================================================

// Common Modbus RTU cfg
#define SERVER_SLAVE_ID  1 // Server slave ID
#define NUM_WORDS        1 // Number of registers to read/write

// UART communication instances
UART uartServer(uartServerConfig);
UART uartClient(uartClientConfig);

// Modbus interfaces instances
ModbusRTU rtuServer(uartServer, Modbus::SERVER);
ModbusRTU rtuClient(uartClient, Modbus::CLIENT);

// Modbus Server application
Modbus::StaticWordStore<NUM_WORDS> store; // Static storage for Modbus words
ModbusServer modbusServer(rtuServer, store, SERVER_SLAVE_ID);

// Modbus Client application
ModbusClient modbusClient(rtuClient);


// ===================================================================================
// EXAMPLE CONFIGURATION
// ===================================================================================

#define TARGET_REGISTER         100
#define CLIENT_POLL_INTERVAL_MS 2000

// Server variable (value exposed by the server for this example)
volatile uint16_t counter = 1000;

// Client Task: sends regular requests (Read & Write) to the Modbus Server
void clientTask(void* arg);


// ===================================================================================
// MAIN
// ===================================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_APP, "Starting Modbus RTU Client <-> Server Loopback Test (Strict Aliases)");

    // ===================================================================================
    // MODBUS INIT
    // ===================================================================================

    // Initialize UART on both interfaces
    esp_err_t uartServerInitResult = uartServer.begin();
    if (uartServerInitResult != ESP_OK) {
        ESP_LOGE(TAG_APP, "Error initializing Server UART HAL: %s", esp_err_to_name(uartServerInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Server UART HAL (UART) initialized.");
    esp_err_t uartClientInitResult = uartClient.begin();
    if (uartClientInitResult != ESP_OK) {
        ESP_LOGE(TAG_APP, "Error initializing Client UART HAL: %s", esp_err_to_name(uartClientInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Client UART HAL (UART) initialized.");

    // Declare Words to be exposed by the server
    ModbusWord regDesc = {
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = TARGET_REGISTER,
        .nbRegs = 1,
        .value = &counter
    };
    ModbusServer::Result addWordResult = modbusServer.addWord(regDesc);
    if (addWordResult != ModbusServer::SUCCESS) {
        ESP_LOGE(TAG_APP, "Error adding word to server: %s", ModbusServer::toString(addWordResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Register %u added to server with initial value %lu.", TARGET_REGISTER, (unsigned long)counter);

    // Initialize Server & Client
    ModbusServer::Result serverInitResult = modbusServer.begin();
    if (serverInitResult != ModbusServer::SUCCESS) {
        ESP_LOGE(TAG_APP, "Error initializing Modbus Server: %s", ModbusServer::toString(serverInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Modbus Server initialized (Slave ID: %d).", SERVER_SLAVE_ID);
    ModbusClient::Result clientInitResult = modbusClient.begin();
    if (clientInitResult != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_APP, "Error initializing Modbus Client: %s", ModbusClient::toString(clientInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Modbus Client initialized.");

    // ===================================================================================
    // APP LAUNCH
    // ===================================================================================

    // Launch the Client Task
    xTaskCreate(clientTask, "modbusClientTask", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG_APP, "Setup complete. Client will send periodic requests.");
    ESP_LOGI(TAG_APP, "Initial server register value (app_main): %lu", (unsigned long)counter);

    // Main loop (nothing to do here, just log the server register value every 5 seconds to signal we're alive)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG_APP, "Current server register value (app_main loop): %lu", (unsigned long)counter);
    }
}

/**
 * Client Task: Demonstrates both simple API and Frame-based API
 * - READ: Uses simple helper method
 * - WRITE: Uses raw Frame API
 */
void clientTask(void* pvParameters) {
    ESP_LOGI(TAG_CLIENT_TASK, "Client Modbus task started.");
    uint16_t valueToWrite = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CLIENT_POLL_INTERVAL_MS));

        if (!modbusClient.isReady()) {
            ESP_LOGW(TAG_CLIENT_TASK, "Modbus client not ready.");
            continue;
        }

        // READ using helper method (simple)
        uint16_t readVal;
        Modbus::ExceptionCode excep;

        ESP_LOGI(TAG_CLIENT_TASK, "Sending READ request for register %u...", TARGET_REGISTER);
        ModbusClient::Result readResult = modbusClient.read(
            SERVER_SLAVE_ID,
            Modbus::HOLDING_REGISTER,
            TARGET_REGISTER,
            1,
            &readVal,
            &excep
        );

        // Error sending request: abort
        if (readResult != ModbusClient::SUCCESS) {
            ESP_LOGE(TAG_CLIENT_TASK, "Communication error (READ): %s", ModbusClient::toString(readResult));
            continue;
        }

        // Response received with Modbus exception: abort
        if (excep != Modbus::NULL_EXCEPTION) {
            ESP_LOGE(TAG_CLIENT_TASK, "Modbus Exception on READ: %s (0x%02X)",
                     Modbus::toString(excep), excep);
            continue;
        }

        // Response received with data: display register value
        ESP_LOGI(TAG_CLIENT_TASK, "READ response: Register %u = %u", TARGET_REGISTER, readVal);

        // WRITE using raw Frame API
        Modbus::Frame writeRequest = {
            .type = Modbus::REQUEST,
            .fc = Modbus::WRITE_REGISTER,
            .slaveId = SERVER_SLAVE_ID,
            .regAddress = TARGET_REGISTER,
            .regCount = 1
        };
        valueToWrite = readVal + 1;
        writeRequest.setRegisters({valueToWrite}); // Set Frame data with value to write

        // Create response placeholder & send the request (synchronous method)
        Modbus::Frame writeResponse;
        ESP_LOGI(TAG_CLIENT_TASK, "Sending WRITE request for register %u with value %u...", TARGET_REGISTER, valueToWrite);
        ModbusClient::Result writeResult = modbusClient.sendRequest(writeRequest, writeResponse);

        // Error sending request: abort
        if (writeResult != ModbusClient::SUCCESS) {
            ESP_LOGE(TAG_CLIENT_TASK, "Error on sendRequest (WRITE): %s", ModbusClient::toString(writeResult));
            continue;
        }

        // Response received with Modbus exception: abort
        if (writeResponse.exceptionCode != Modbus::NULL_EXCEPTION) {
            ESP_LOGE(TAG_CLIENT_TASK, "Modbus Exception on WRITE: %s (0x%02X)",
                     Modbus::toString(writeResponse.exceptionCode), writeResponse.exceptionCode);
            continue;
        }

        // Response received with success: display echo
        ESP_LOGI(TAG_CLIENT_TASK, "WRITE response: Success (Echo Addr: %u, Val: %u)",
                 writeResponse.regAddress, writeResponse.getRegister(0));
    }
}