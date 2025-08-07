/**
 * @file tcp_server_main.cpp
 * @brief EZModbus TCP Server using CH9120 Ethernet driver for RPi Pico-SDK (RP2040 & RP2350)
 * 
 * Hardware:
 * - CH9120 Ethernet module connected to UART1 with Waveshare pinout:
 *      GP20 (UART1 TX) → CH9120 RX
 *      GP21 (UART1 RX) → CH9120 TX  
 *      GP18 → CH9120 CFG pin
 *      GP19 → CH9120 RES pin
 *      GP17 → CH9120 STATUS pin
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"
// Include EZModbus (should work now with proper CMake dependency management)
#include "EZModbus.h"

// ===================================================================================
// EZMODBUS DEBUG CONFIGURATION (optional if debug is not enabled)
// ===================================================================================

#ifdef EZMODBUS_DEBUG
    // Debug print function for EZModbus - outputs to Pico stdio
    int Pico_LogPrint(const char* msg, size_t len) {
        return printf("%.*s", (int)len, msg);
    }

    // Automatically register debug function
    static Modbus::Debug::PrintFunctionSetter func(Pico_LogPrint);
#endif

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char *TAG_APP = "MODBUS_TCP_SERVER";
static const char *TAG_EVENT_TASK = "EVENT_TASK";

// ===================================================================================
// SERVER CONFIGURATION
// ===================================================================================

// Server parameters
constexpr uint8_t SERVER_SLAVE_ID = 1;              // Slave ID for this server
constexpr uint16_t MODBUS_PORT = 5020;              // Non-privileged port (avoid sudo)
constexpr uint16_t TARGET_REGISTER = 100;           // Start address of register bank
constexpr size_t NUM_WORDS = 10;                    // Number of registers (100-109)

// Register bank with initial values
volatile uint16_t registers[NUM_WORDS] = {1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009};


// ===================================================================================
// EZMODBUS CONFIG & INSTANCES
// ===================================================================================

// Type alias for convenience
using TCP = ModbusHAL::TCP;
using ModbusTCP = ModbusInterface::TCP;
using ModbusServer = Modbus::Server;
using ModbusFrame = Modbus::Frame;
using ModbusWord = Modbus::Word;

// Hardware configuration for CH9120 (Waveshare pinout)
const CH9120Driver::HardwareConfig hwConfig = {
    .uart = uart1,
    .baudrate = 115200,
    .txPin = 20,        // UART1 TX
    .rxPin = 21,        // UART1 RX
    .cfgPin = 18,       // Config pin
    .resPin = 19,       // Reset pin
    .statusPin = 17     // Status pin
};

// Network configuration for CH9120 - server mode
const CH9120Driver::NetworkConfig netConfig = {
    .localIp = {192, 168, 0, 124},               // CH9120's IP (server)
    .gateway = {192, 168, 0, 1},                 // Gateway
    .subnetMask = {255, 255, 255, 0},            // Subnet mask
    .useDhcp = true,                             // Use DHCP
    .targetIp = {192, 168, 0, 1},                // Dummy value (required, no 0.0.0.0 allowed)
    .localPort = MODBUS_PORT,                    // Modbus TCP port
    .targetPort = 80,                            // Dummy value (required, no 0 allowed)
    .mode = CH9120Driver::Mode::TCP_SERVER       // Auto-detects server mode
};

// Unified API: constructor configures both hardware AND network mode
TCP tcpHal(hwConfig, netConfig);
ModbusTCP tcpInterface(tcpHal, Modbus::SERVER);
Modbus::StaticWordStore<NUM_WORDS> store;
ModbusServer modbusServer(tcpInterface, store, SERVER_SLAVE_ID);

// ===================================================================================
// MAIN
// ===================================================================================

// Tasks forward declarations
static void initTask(void* param);        // Main initialization task
static void monitorTask(void* param);     // Monitor task
static void eventTask(void* param);       // Event Task: logs events to the console (conditionally compiled)

// Main loop
int main() {
    stdio_init_all();
    
    // Wait for USB serial
    sleep_ms(2000);
    
    // Create init task that will handle all initialization
    xTaskCreate(initTask, "Init", 4096, NULL, 3, NULL);
    
    vTaskStartScheduler();
    
    // Never reached
    printf("FATAL: Scheduler returned!\n");
    while (1) sleep_ms(1000);
}

// ===================================================================================
// MAIN APP TASKS
// ===================================================================================

// Initialization task: sets up CH9120, TCP HAL, and Modbus Server
void initTask(void* param) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n=== %s ===\n", TAG_APP);
    printf("Starting Modbus TCP Server using CH9120 Ethernet\n");
    printf("Hardware: CH9120 on UART1 (Waveshare pinout)\n");
    printf("Server will listen on port %d\n\n", MODBUS_PORT);

    // Initialize TCP HAL (unified API: configuration was done in constructor)
    printf("[%s] Starting TCP HAL (unified API)...\n", TAG_APP);
    if (!tcpHal.begin()) {
        printf("[%s] Error starting TCP HAL\n", TAG_APP);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] TCP HAL started successfully (server mode on port %d).\n", TAG_APP, MODBUS_PORT);

    // Add registers to server
    printf("[%s] Adding %zu registers starting from address %u...\n", TAG_APP, NUM_WORDS, TARGET_REGISTER);
    for (size_t i = 0; i < NUM_WORDS; i++) {
        ModbusWord regDesc = {
            .type = Modbus::HOLDING_REGISTER,
            .startAddr = static_cast<uint16_t>(TARGET_REGISTER + i),
            .nbRegs = 1,
            .value = const_cast<uint16_t*>(&registers[i])
        };
        ModbusServer::Result addWordResult = modbusServer.addWord(regDesc);
        if (addWordResult != ModbusServer::SUCCESS) {
            printf("[%s] Error adding register %u: %s\n", TAG_APP, 
                   static_cast<unsigned int>(TARGET_REGISTER + i), ModbusServer::toString(addWordResult));
            while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    printf("[%s] Registers added (values: %u-%u).\n", TAG_APP, 
           static_cast<unsigned int>(registers[0]), static_cast<unsigned int>(registers[NUM_WORDS-1]));

    // Initialize Modbus Server
    printf("[%s] Initializing Modbus Server...\n", TAG_APP);
    ModbusServer::Result serverInitResult = modbusServer.begin();
    if (serverInitResult != ModbusServer::SUCCESS) {
        printf("[%s] Error initializing Modbus Server: %s\n", TAG_APP, ModbusServer::toString(serverInitResult));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] Modbus Server initialized (Slave ID: %d).\n", TAG_APP, SERVER_SLAVE_ID);

    // Start monitor task
    xTaskCreate(monitorTask, "Monitor", 2048, NULL, 2, NULL);
    printf("[%s] Monitor task started.\n", TAG_APP);

    // Launch the Event Task (only if EZMODBUS_EVENTBUS is enabled & EZMODBUS_DEBUG is disabled)
    #if defined(EZMODBUS_EVENTBUS) && !defined(EZMODBUS_DEBUG)
        xTaskCreate(eventTask, "modbusEventTask", 4096, NULL, 5, NULL);
        printf("[%s] Event task started.\n", TAG_APP);
    #endif

    printf("\n[%s] === SERVER READY ===\n", TAG_APP);
    printf("[%s] Waiting for Modbus TCP clients on port %d...\n", TAG_APP, MODBUS_PORT);
    printf("[%s] Test with: mbpoll -t 4 -r %u -c %zu <CH9120_IP>\n", TAG_APP, TARGET_REGISTER, NUM_WORDS);
    printf("[%s] Or from Python: client.read_holding_registers(%u, %zu)\n\n", TAG_APP, TARGET_REGISTER, NUM_WORDS);

    // Initialization complete, delete this task
    vTaskDelete(nullptr);
}

// Monitor task: displays server status and updates register values
void monitorTask(void* param) {
    printf("[%s] Monitor task started.\n", TAG_APP);
    uint32_t updateCounter = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Update some register values to show activity
        updateCounter++;
        registers[0] = 1000 + updateCounter;           // Counter register
        registers[1] = (updateCounter % 2) ? 0xFFFF : 0x0000; // Toggle register
        registers[2] = updateCounter * 10;             // Scaled counter
        
        printf("\r\n");
        printf("[%s] === STATUS UPDATE %lu ===\n", TAG_APP, updateCounter);
        printf("[%s] CH9120 State: %s\n", TAG_APP, 
               tcpHal.isClientConnected() ? "CONNECTED" : "DISCONNECTED");
        printf("[%s] TCP HAL: %s\n", TAG_APP, 
               tcpHal.isServerRunning() ? "RUNNING" : "STOPPED");
        printf("[%s] Active connections: %zu\n", TAG_APP, tcpHal.getActiveSocketCount());
        
        // Display current register values
        printf("[%s] Registers [%u-%u]: ", TAG_APP, TARGET_REGISTER, TARGET_REGISTER + NUM_WORDS - 1);
        for (size_t i = 0; i < NUM_WORDS; i++) {
            printf("%u ", static_cast<unsigned int>(registers[i]));
            if (i == 4) printf("\n[%s]                      ", TAG_APP); // Line break for readability
        }
        printf("\n");
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
            if (evt.eventType == Modbus::EventBus::EVT_REQUEST) {
                // Request event - show function code, address, count
                printf("[%s] [%3f][%s:%u] REQUEST: %s addr=%u count=%u\n", 
                TAG_EVENT_TASK,
                timestamp, evt.fileName, evt.lineNo,
                Modbus::toString(evt.requestInfo.fc), evt.requestInfo.regAddress, evt.requestInfo.regCount);
            } else {
                // Error event
                printf("[%s] [%3f][%s:%u] ERROR: %s\n", 
                TAG_EVENT_TASK,
                timestamp, evt.fileName, evt.lineNo, evt.resultStr);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif // EZMODBUS_EVENTBUS && !EZMODBUS_DEBUG