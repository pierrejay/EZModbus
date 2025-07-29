/**
 * @file tcp_client_main.cpp
 * @brief EZModbus TCP Client using CH9120 Ethernet driver for RPi Pico-SDK (RP2040 & RP2350)
 * 
 * This example demonstrates:
 * - Single register read/write operations
 * - Multiple register read/write operations
 * - Verification of written values
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
#include "EZModbus.h"

// ===================================================================================
// LOG TAGS
// ===================================================================================

static const char* TAG_APP = "MODBUS_TCP_CLIENT";

// ===================================================================================
// CONFIGURATION
// ===================================================================================

// Server configuration - Mac running the Python test server
constexpr const char* SERVER_IP = "192.168.0.234";  // IP of the Mac running test server
constexpr uint16_t SERVER_PORT = 5020;              // Non-privileged port (avoid sudo)
constexpr uint8_t SERVER_SLAVE_ID = 1;              // Slave ID on the server

// Test registers
constexpr uint16_t SINGLE_REGISTER_ADDR = 10;       // Address for single read/write
constexpr uint16_t MULTI_REGISTER_START = 10;       // Start address for multiple operations
constexpr uint16_t MULTI_REGISTER_COUNT = 10;       // Number of registers (10-19)
constexpr uint32_t REQ_DELAY_MS = 1;                // Yield to scheduler after each request for proper resource cleanup


// ===================================================================================
// EZMODBUS CONFIG & INSTANCES
// ===================================================================================

// Type alias for convenience
using TCP = ModbusHAL::TCP;
using ModbusTCP = ModbusInterface::TCP;
using ModbusClient = Modbus::Client;

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

// Network configuration for CH9120 - client mode
const CH9120Driver::NetworkConfig netConfig = {
    .localIp = {192, 168, 0, 101},       // CH9120's IP
    .gateway = {192, 168, 0, 1},         // Gateway
    .subnetMask = {255, 255, 255, 0},    // Subnet mask
    .useDhcp = true,                     // Use DHCP
    .targetIp = {192, 168, 0, 234},      // Server IP
    .localPort = 8000,                   // Local port
    .targetPort = SERVER_PORT,           // Server port
    .mode = CH9120Driver::Mode::TCP_CLIENT  // Auto-detects client mode
};

// Unified API: constructor configures both hardware AND network mode
TCP tcpHal(hwConfig, netConfig);
ModbusTCP tcpInterface(tcpHal, Modbus::CLIENT);
ModbusClient modbusClient(tcpInterface);

// ===================================================================================
// CLIENT TASK
// ===================================================================================

void clientTask(void* param) {
    // Wait a bit for hardware to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    printf("\n=== Modbus TCP Client Test ===\n");
    printf("Target server: %s:%d (Slave ID: %d)\n", SERVER_IP, SERVER_PORT, SERVER_SLAVE_ID);
    
    // Initialize TCP HAL (unified API: configuration was done in constructor)
    printf("\n[%s] Starting TCP HAL (unified API)...\n", TAG_APP);
    if (!tcpHal.begin()) {
        printf("[%s] Error starting TCP HAL\n", TAG_APP);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] TCP HAL started successfully (client mode to %s:%d).\n", TAG_APP, SERVER_IP, SERVER_PORT);
    
    // Initialize Modbus client
    printf("[%s] Initializing Modbus client...\n", TAG_APP);
    auto initResult = modbusClient.begin();
    if (initResult != ModbusClient::SUCCESS) {
        printf("[%s] Error initializing Modbus client: %s\n", TAG_APP, 
               ModbusClient::toString(initResult));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[%s] Modbus client initialized.\n", TAG_APP);
    
    // Wait for connection
    printf("\n[%s] Waiting for connection to server...\n", TAG_APP);
    while (!tcpHal.isClientConnected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("[%s] Connected to server!\n", TAG_APP);
    
    // Main test loop
    while (1) {
        printf("\n[%s] === Starting test sequence ===\n", TAG_APP);
        
        // ===================================================================================
        // SINGLE REGISTER OPERATIONS
        // ===================================================================================
        
        printf("\n[%s] --- Single Register Operations (Address %d) ---\n", TAG_APP, SINGLE_REGISTER_ADDR);
        
        // 1. Read single register (Using HELPER API)
        printf("[%s] Reading register %u (using helper API)...\n", TAG_APP, SINGLE_REGISTER_ADDR);
        uint16_t originalValue = 0;
        Modbus::ExceptionCode excep;
        auto result = modbusClient.read(SERVER_SLAVE_ID, Modbus::HOLDING_REGISTER, 
                                       SINGLE_REGISTER_ADDR, 1, &originalValue, &excep);
        
        if (result == ModbusClient::SUCCESS) {
            printf("[%s] Read single register: value = %u (0x%04X)\n", TAG_APP, originalValue, originalValue);
        } else if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on READ: %s (0x%02X)\n", TAG_APP,
                   Modbus::toString(excep), excep);
        } else {
            printf("[%s] Failed to read single register: %s\n", TAG_APP, ModbusClient::toString(result));
        }

        // Yield to the scheduler for proper request cleanup
        vTaskDelay(pdMS_TO_TICKS(REQ_DELAY_MS));
        
        // 2. Write single register
        uint16_t newValue = originalValue + 100;
        printf("[%s] Writing new value: %u (0x%04X)\n", TAG_APP, newValue, newValue);
        
        Modbus::Frame writeRequest = {
            .type = Modbus::REQUEST,
            .fc = Modbus::WRITE_REGISTER,
            .slaveId = SERVER_SLAVE_ID,
            .regAddress = SINGLE_REGISTER_ADDR,
            .regCount = 1
        };
        writeRequest.setRegisters({newValue});
        
        Modbus::Frame writeResponse;
        result = modbusClient.sendRequest(writeRequest, writeResponse);
        
        if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on WRITE: %s (0x%02X)\n", TAG_APP,
                   Modbus::toString(writeResponse.exceptionCode), writeResponse.exceptionCode);
        } else if (result == ModbusClient::SUCCESS) {
            printf("[%s] Write single register: SUCCESS (Echo: %u)\n", TAG_APP, writeResponse.getRegister(0));
        } else {
            printf("[%s] Failed to write single register: %s\n", TAG_APP, ModbusClient::toString(result));
        }
        
        // Yield to the scheduler for proper request cleanup
        vTaskDelay(pdMS_TO_TICKS(REQ_DELAY_MS));

        // 3. Read back to verify (fresh request)
        Modbus::Frame readbackRequest;
        readbackRequest.type = Modbus::REQUEST;
        readbackRequest.slaveId = SERVER_SLAVE_ID;
        readbackRequest.fc = Modbus::READ_HOLDING_REGISTERS;
        readbackRequest.regAddress = SINGLE_REGISTER_ADDR;
        readbackRequest.regCount = 1;
        Modbus::Frame readbackResponse;
        result = modbusClient.sendRequest(readbackRequest, readbackResponse);
        if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on READBACK: %s (0x%02X)\n", TAG_APP,
                   Modbus::toString(readbackResponse.exceptionCode), readbackResponse.exceptionCode);
        } else if (result == ModbusClient::SUCCESS) {
            uint16_t readbackValue = readbackResponse.getRegister(0);
            printf("[%s] Readback single register: value = %u (0x%04X) - %s\n", 
                   TAG_APP, readbackValue, readbackValue,
                   (readbackValue == newValue) ? "VERIFIED" : "MISMATCH!");
        } else {
            printf("[%s] Failed to readback single register: %s\n", TAG_APP, ModbusClient::toString(result));
        }
        
        // Yield to the scheduler for proper request cleanup
        vTaskDelay(pdMS_TO_TICKS(REQ_DELAY_MS));

        // ===================================================================================
        // MULTIPLE REGISTER OPERATIONS
        // ===================================================================================
        
        printf("\n[%s] --- Multiple Register Operations (Address %d-%d) ---\n", 
               TAG_APP, MULTI_REGISTER_START, MULTI_REGISTER_START + MULTI_REGISTER_COUNT - 1);
        
        // 1. Read multiple registers
        Modbus::Frame multiReadRequest = {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_HOLDING_REGISTERS,
            .slaveId = SERVER_SLAVE_ID,
            .regAddress = MULTI_REGISTER_START,
            .regCount = MULTI_REGISTER_COUNT
        };
        
        Modbus::Frame multiReadResponse;
        printf("[%s] Sending READ request for %u registers starting at %u...\n", 
               TAG_APP, MULTI_REGISTER_COUNT, MULTI_REGISTER_START);
        result = modbusClient.sendRequest(multiReadRequest, multiReadResponse);
        
        std::vector<uint16_t> originalValues;
        if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on MULTI-READ: %s (0x%02X)\n", TAG_APP,
                   Modbus::toString(multiReadResponse.exceptionCode), multiReadResponse.exceptionCode);
        } else if (result == ModbusClient::SUCCESS) {
            // Extract all register values from response
            for (uint16_t i = 0; i < multiReadResponse.regCount; i++) {
                originalValues.push_back(multiReadResponse.getRegister(i));
            }
            printf("[%s] Read %zu registers: ", TAG_APP, originalValues.size());
            for (auto val : originalValues) {
                printf("%u ", val);
            }
            printf("\n");
        } else {
            printf("[%s] Failed to read multiple registers: %s\n", TAG_APP, ModbusClient::toString(result));
        }

        // Yield to the scheduler for proper request cleanup
        vTaskDelay(pdMS_TO_TICKS(REQ_DELAY_MS));
        
        // 2. Write multiple registers (increment each by 10)
        std::vector<uint16_t> newValues;
        for (size_t i = 0; i < MULTI_REGISTER_COUNT; i++) {
            uint16_t val = (i < originalValues.size()) ? originalValues[i] + 10 : 1000 + i;
            newValues.push_back(val);
        }
        
        printf("[%s] Writing new values: ", TAG_APP);
        for (auto val : newValues) {
            printf("%u ", val);
        }
        printf("\n");
        
        Modbus::Frame multiWriteRequest = {
            .type = Modbus::REQUEST,
            .fc = Modbus::WRITE_MULTIPLE_REGISTERS,
            .slaveId = SERVER_SLAVE_ID,
            .regAddress = MULTI_REGISTER_START,
            .regCount = MULTI_REGISTER_COUNT
        };
        multiWriteRequest.setRegisters(newValues);
        
        Modbus::Frame multiWriteResponse;
        result = modbusClient.sendRequest(multiWriteRequest, multiWriteResponse);
        
        if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on MULTI-WRITE: %s (0x%02X)\n", TAG_APP,
                   Modbus::toString(multiWriteResponse.exceptionCode), multiWriteResponse.exceptionCode);
        } else if (result == ModbusClient::SUCCESS) {
            printf("[%s] Write multiple registers: SUCCESS (Addr: %u, Count: %u)\n", 
                   TAG_APP, multiWriteResponse.regAddress, multiWriteResponse.regCount);
        } else {
            printf("[%s] Failed to write multiple registers: %s\n", TAG_APP, ModbusClient::toString(result));
        }

        // Yield to the scheduler for proper request cleanup
        vTaskDelay(pdMS_TO_TICKS(REQ_DELAY_MS));
        
        // 3. Read back to verify
        result = modbusClient.sendRequest(multiReadRequest, multiReadResponse);
        if (result == ModbusClient::ERR_EXCEPTION_RESPONSE) {
            printf("[%s] Modbus Exception on MULTI-READBACK: %s (0x%02X)\n", TAG_APP,
                   Modbus::toString(multiReadResponse.exceptionCode), multiReadResponse.exceptionCode);
        } else if (result == ModbusClient::SUCCESS) {
            printf("[%s] Readback %u registers: ", TAG_APP, multiReadResponse.regCount);
            bool allMatch = true;
            for (uint16_t i = 0; i < multiReadResponse.regCount; i++) {
                uint16_t readbackValue = multiReadResponse.getRegister(i);
                printf("%u ", readbackValue);
                if (i < newValues.size() && readbackValue != newValues[i]) {
                    allMatch = false;
                }
            }
            printf("- %s\n", allMatch ? "ALL VERIFIED" : "MISMATCH!");
        } else {
            printf("[%s] Failed to readback multiple registers: %s\n", TAG_APP, ModbusClient::toString(result));
        }
        
        // Wait before next test cycle
        printf("\n[%s] Test sequence complete. Waiting 6 seconds...\n", TAG_APP);
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}

// ===================================================================================
// MAIN
// ===================================================================================

int main() {
    stdio_init_all();
    
    // Wait for USB serial
    sleep_ms(2000);
    
    printf("\n");
    printf("=====================================\n");
    printf("EZModbus TCP Client Example\n");
    printf("=====================================\n");
    printf("Hardware: CH9120 on UART1\n");
    printf("Target: %s:%d\n", SERVER_IP, SERVER_PORT);
    printf("=====================================\n\n");
    
    // Create client task
    xTaskCreate(clientTask, "ClientTask", 4096, NULL, 5, NULL);
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    while (1) {
        tight_loop_contents();
    }
    
    return 0;
}