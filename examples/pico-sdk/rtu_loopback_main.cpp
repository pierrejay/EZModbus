/**
 * @file main.cpp
 * @brief EZModbus RTU Client <-> Server Loopback Test for RPi Pico-SDK (RP2040 & RP2350)
 * 
 * Wiring:
 * - RP2040:
 *      GP16 (UART0 TX) → GP5 (UART1 RX)
 *      GP17 (UART0 RX) → GP4 (UART1 TX)
 * - RP2350:
 *      GP0 (UART0 TX) → GP8 (UART1 RX)
 *      GP1 (UART0 RX) → GP9 (UART1 TX)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "EZModbus.h"

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char *TAG_APP = "MODBUS_LOOPBACK_APP";
static const char *TAG_CLIENT_TASK = "CLIENT_TASK";
static const char *TAG_EVENT_TASK = "EVENT_TASK";


// ===================================================================================
// EZMODBUS ALIASES
// ===================================================================================

// Just for convenience
using UART          = ModbusHAL::UART;
using UARTConfig    = ModbusHAL::UART::Config;
using ModbusRTU     = ModbusInterface::RTU;
using ModbusClient  = Modbus::Client;
using ModbusServer  = Modbus::Server;
using ModbusFrame   = Modbus::Frame;
using ModbusWord    = Modbus::Word;


// ===================================================================================
// UART CONFIGURATION
// ===================================================================================

// === PLATFORM-SPECIFIC UART PINS ===
// Define UART pins based on chip (RP2040 or RP2350)
// Uses the literal flag defined by Pico SDK in CMakeLists.txt
constexpr std::string_view board = PICO_BOARD;

// RP2040 (Pico): TX0 (GP17) <> (GP4) RX1  - RX0 (GP16) <> (GP5) TX1
// RP2350 (Pico2): TX0 (GP1) <> (GP9) RX1 - RX0 (GP0) <> (GP8) TX1
// Pico & Pico2 W: TX0 (GP12) <> (GP9) RX1 - RX0 (GP13) <> (GP8) TX1 
constexpr uint8_t TX0_PIN = (board == "pico") ? 17 :
                            (board == "pico2") ? 1 :
                            (board == "pico_w" || board == "pico2_w") ? 12 : UINT8_MAX;
constexpr uint8_t RX0_PIN = (board == "pico") ? 16 :
                            (board == "pico2") ? 0 :
                            (board == "pico_w" || board == "pico2_w") ? 13 : UINT8_MAX;
constexpr uint8_t TX1_PIN = (board == "pico") ? 5 :
                            (board == "pico2" || board == "pico_w" || board == "pico2_w") ? 8 : UINT8_MAX;
constexpr uint8_t RX1_PIN = (board == "pico") ? 4 :
                            (board == "pico2" || board == "pico_w" || board == "pico2_w") ? 9 : UINT8_MAX;

                            
UARTConfig uartServerCfg = {
    .uart    = uart0,
    .baud    = 921600,
    .config  = UART::CONFIG_8N1,
    .rxPin   = RX0_PIN,
    .txPin   = TX0_PIN,
    .dePin   = -1
};

UARTConfig uartClientCfg = {
    .uart    = uart1,
    .baud    = 921600,
    .config  = UART::CONFIG_8N1,
    .rxPin   = RX1_PIN,
    .txPin   = TX1_PIN,
    .dePin   = -1
};

// ===================================================================================
// MODBUS CONFIGURATION & INSTANCES
// ===================================================================================

// Common Modbus RTU cfg
constexpr uint8_t SERVER_SLAVE_ID = 1;
constexpr uint16_t TARGET_REGISTER = 100;
constexpr size_t NUM_WORDS = 1;   // Number of registers to read/write
constexpr uint32_t CLIENT_POLL_INTERVAL_MS = 2000;
volatile uint16_t counter = 1000; // Shared server variable

// Server instances
UART uartServer(uartServerCfg);
ModbusRTU rtuServer(uartServer, Modbus::SERVER);
Modbus::StaticWordStore<NUM_WORDS> store;
ModbusServer modbusServer(rtuServer, store, SERVER_SLAVE_ID);

// Client instances
UART uartClient(uartClientCfg);
ModbusRTU rtuClient(uartClient, Modbus::CLIENT);
ModbusClient modbusClient(rtuClient);


// ===================================================================================
// MAIN
// ===================================================================================

// Tasks forward declarations
static void initTask(void* param);   // Main scheduler task
static void clientTask(void* param); // Launched by initTask
static void eventTask(void* param); // Event Task: logs events to the console (conditionally compiled)

// Main loop
int main() {
    stdio_init_all();
    
    // Create init task that will handle all initialization and monitoring
    xTaskCreate(initTask, "Init", 2048, NULL, 3, NULL);
    
    vTaskStartScheduler();
    
    // Never reached
    printf("FATAL: Scheduler returned!\n");
    while (1) sleep_ms(1000);
}


// ===================================================================================
// MAIN APP TASKS
// ===================================================================================

// Initialization task: sets up UART, Modbus Server/Client, and starts client task
void initTask(void* param) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n=== %s ===\n", TAG_APP);
    printf("Starting Modbus RTU Client <-> Server Loopback Test\n");
    printf("Check wiring in main.cpp file\n\n");

    // Initialize Server UART HAL
    auto uartServerInitResult = uartServer.begin();
    if (uartServerInitResult != UART::SUCCESS) {
        printf("[%s] Error initializing Server UART HAL: %s\n", TAG_APP, UART::toString(uartServerInitResult));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] Server UART HAL (UART0) initialized.\n", TAG_APP);

    // Initialize Client UART HAL
    auto uartClientInitResult = uartClient.begin();
    if (uartClientInitResult != UART::SUCCESS) {
        printf("[%s] Error initializing Client UART HAL: %s\n", TAG_APP, UART::toString(uartClientInitResult));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] Client UART HAL (UART1) initialized.\n", TAG_APP);

    // Add register to server
    ModbusWord regDesc = {
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = TARGET_REGISTER,
        .nbRegs = 1,
        .value = &counter
    };
    ModbusServer::Result addWordResult = modbusServer.addWord(regDesc);
    if (addWordResult != ModbusServer::SUCCESS) {
        printf("[%s] Error adding word to server: %s\n", TAG_APP, ModbusServer::toString(addWordResult));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] Register %u added to server with initial value %u.\n", TAG_APP, TARGET_REGISTER, (unsigned int)counter);

    // Initialize Modbus Server
    ModbusServer::Result serverInitResult = modbusServer.begin();
    if (serverInitResult != ModbusServer::SUCCESS) {
        printf("[%s] Error initializing Modbus Server: %s\n", TAG_APP, ModbusServer::toString(serverInitResult));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] Modbus Server initialized (Slave ID: %d).\n", TAG_APP, SERVER_SLAVE_ID);

    // Initialize Modbus Client
    ModbusClient::Result clientInitResult = modbusClient.begin();
    if (clientInitResult != ModbusClient::SUCCESS) {
        printf("[%s] Error initializing Modbus Client: %s\n", TAG_APP, ModbusClient::toString(clientInitResult));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] Modbus Client initialized.\n", TAG_APP);

    // Start client task
    xTaskCreate(clientTask, "modbusClientTask", 2048, NULL, 2, NULL);
    printf("[%s] Client task started.\n", TAG_APP);

    printf("[%s] Setup complete. Client will send periodic requests.\n", TAG_APP);
    printf("[%s] Initial server register value: %u\n", TAG_APP, (unsigned int)counter);

    // Launch the Event Task (only if EZMODBUS_EVENTBUS is enabled & EZMODBUS_DEBUG is disabled)
    #if defined(EZMODBUS_EVENTBUS) && !defined(EZMODBUS_DEBUG)
        xTaskCreate(eventTask, "modbusEventTask", 4096, NULL, 5, NULL);
        printf("[%s] Event task started.\n", TAG_APP);
    #endif

    // Main monitoring loop
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        printf("\r\n");
        printf("[%s] Current server register value (main loop): %u\n", TAG_APP, (unsigned int)counter);
    }
}

// Client task: periodically reads and writes to the server
void clientTask(void* param) {
    printf("[%s] Client Modbus task started.\n", TAG_CLIENT_TASK);
    uint16_t valueToWrite = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CLIENT_POLL_INTERVAL_MS));

        if (!modbusClient.isReady()) {
            printf("[%s] Modbus client not ready.\n", TAG_CLIENT_TASK);
            continue;
        }

        Modbus::Frame readRequest = {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_HOLDING_REGISTERS,
            .slaveId = SERVER_SLAVE_ID,
            .regAddress = TARGET_REGISTER,
            .regCount = 1
        };

        Modbus::Frame readResponse;
        printf("\r\n");
        printf("[%s] Sending READ request for register %u...\n", TAG_CLIENT_TASK, TARGET_REGISTER);
        ModbusClient::Result readResult = modbusClient.sendRequest(readRequest, readResponse);

        if (readResult == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on READ: %s (0x%02X)\n", TAG_CLIENT_TASK,
                   Modbus::toString(readResponse.exceptionCode), readResponse.exceptionCode);
            continue;
        } else if (readResult != ModbusClient::SUCCESS) {
            printf("[%s] Error on sendRequest (READ): %s\n", TAG_CLIENT_TASK, ModbusClient::toString(readResult));
            continue;
        }

        uint16_t receivedValue = readResponse.getRegister(0);
        printf("[%s] READ response: Register %u = %u\n", TAG_CLIENT_TASK, TARGET_REGISTER, receivedValue);

        valueToWrite = receivedValue + 1;
        Modbus::Frame writeRequest = {
            .type = Modbus::REQUEST,
            .fc = Modbus::WRITE_REGISTER,
            .slaveId = SERVER_SLAVE_ID,
            .regAddress = TARGET_REGISTER,
            .regCount = 1,
            .data = Modbus::packRegisters({valueToWrite})
        };

        Modbus::Frame writeResponse;
        printf("[%s] Sending WRITE request for register %u with value %u...\n", TAG_CLIENT_TASK, TARGET_REGISTER, valueToWrite);
        ModbusClient::Result writeResult = modbusClient.sendRequest(writeRequest, writeResponse);

        if (writeResult == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on WRITE: %s (0x%02X)\n", TAG_CLIENT_TASK,
                   Modbus::toString(writeResponse.exceptionCode), writeResponse.exceptionCode);
            continue;
        } else if (writeResult != ModbusClient::SUCCESS) {
            printf("[%s] Error on sendRequest (WRITE): %s\n", TAG_CLIENT_TASK, ModbusClient::toString(writeResult));
            continue;
        }

        printf("[%s] WRITE response: Success (Echo Addr: %u, Val: %u)\n", TAG_CLIENT_TASK,
               writeResponse.regAddress, writeResponse.getRegister(0));
    }
}

#if defined(EZMODBUS_EVENTBUS) && !defined(EZMODBUS_DEBUG)
// Event Task: logs events to the console (conditionally compiled)
void eventTask(void* param) {
    constexpr uint32_t EVT_POP_WAIT_MS = 100; // Wait time for event bus pop
    // Main loop: pop events from the event bus and log them
    while (1) {
        // Pop an event from the event bus
        Modbus::EventBus::Record evt;
        if (Modbus::EventBus::pop(evt, EVT_POP_WAIT_MS)) {
            // Format timestamp 
            float timestamp = (float)(evt.timestampUs / 1000000.0f);

            // Log the event
            printf("\r\n");
            if (evt.eventType == Modbus::EVENT_REQUEST) {
                // Request event - show function code, address, count
                printf("[%s] [%3f][%s:%u] REQUEST: %s addr=%u count=%u\n", 
                TAG_EVENT_TASK,
                timestamp, evt.fileName, evt.lineNo,
                Modbus::toString(evt.requestInfo.fc), evt.requestInfo.regAddress, evt.requestInfo.regCount);
            } else if (evt.desc && *evt.desc != '\0') {
                // Error event with description
                printf("[%s] [%3f][%s:%u] ERROR: %s (%s)\n", 
                TAG_EVENT_TASK,
                timestamp, evt.fileName, evt.lineNo, evt.resultStr, evt.desc);
            } else {
                // Error event without description
                printf("[%s] [%3f][%s:%u] ERROR: %s\n", 
                TAG_EVENT_TASK,
                timestamp, evt.fileName, evt.lineNo, evt.resultStr);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif // EZMODBUS_EVENTBUS && !EZMODBUS_DEBUG