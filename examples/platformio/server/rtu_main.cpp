/**
 * @file rtu_main.cpp
 * @brief EZModbus RTU Server demonstration showing all Word registration methods
 * @brief Illustrates: direct pointer access, single handlers, multi-register handlers
 */

#include <Arduino.h>
#include "EZModbus.h"

// EZModbus debug configuration (optional if debug is not enabled)
#ifdef EZMODBUS_DEBUG
    // Debug print function for EZModbus - outputs to Arduino Serial
    int Arduino_LogPrint(const char* msg, size_t len) {
        return Serial.write(msg, len);
    }

    // Automatically register debug function
    static Modbus::Debug::PrintFunctionSetter func(Arduino_LogPrint);
#endif

// Server configuration
#define DEMO_SLAVE_ID 1

// Register map demonstrating different Word registration methods
namespace RegAddr {
    // Method 1: Direct pointer access (single registers only)
    constexpr uint16_t SENSOR_TEMP = 100;     // Direct pointer to volatile variable
    constexpr uint16_t SENSOR_HUMIDITY = 101; // Direct pointer to volatile variable
    
    // Method 2: Single register with handler (for validation/logging)
    constexpr uint16_t CONFIG_SETPOINT = 200; // Single reg with handler
    constexpr uint16_t CONFIG_MODE = 201;     // Single reg with handler
    
    // Method 3: Multi-register handlers (atomic operations)
    constexpr uint16_t STATS_BLOCK = 300;     // Multi-reg: min/max/avg (3 registers)
    constexpr uint16_t FLOAT_VALUE = 310;     // Multi-reg: IEEE 754 float (2 registers) 
    constexpr uint16_t TIMESTAMP = 320;       // Multi-reg: Unix timestamp (2 registers)
    
    // Method 4: Mixed coils and discrete inputs
    constexpr uint16_t STATUS_FLAGS = 400;    // Coil with handler for logging
    constexpr uint16_t ALARM_FLAGS = 500;     // Discrete input with direct pointer
}

// Aliases for convenience
using UART = ModbusHAL::UART;
using UARTConfig = ModbusHAL::UART::Config;
using ModbusRTU = ModbusInterface::RTU;
using ModbusServer = Modbus::Server;

// UART Configuration
UARTConfig uartConfig = {
    .serial = Serial2,
    .baud = 9600,
    .config = SERIAL_8N1,
    .rxPin = 16,
    .txPin = 17,
    .dePin = 5  // DE/RE pin for RS485 communication
};

// UART port for Modbus RTU
UART uart(uartConfig);

// Modbus RTU interface
ModbusRTU interface(uart, Modbus::SERVER);

// WordStore & ModbusServer instance
Modbus::StaticWordStore<200> wordStore;  // Static allocation on stack for example
ModbusServer server(interface, wordStore, DEMO_SLAVE_ID);

// Variables demonstrating different Word storage methods
struct {
    // Method 1: Direct pointer variables (volatile uint16_t*)
    volatile uint16_t sensorTemp = 250;      // 25.0°C
    volatile uint16_t sensorHumidity = 600;  // 60.0%
    
    // Method 2: Single register handler variables
    uint16_t configSetpoint = 220;           // 22.0°C (internal storage)
    uint16_t configMode = 1;                 // Auto mode
    
    // Method 3: Multi-register handler variables
    struct {
        uint16_t min = 180;      // 18.0°C minimum
        uint16_t max = 320;      // 32.0°C maximum  
        uint16_t avg = 250;      // 25.0°C average
    } tempStats;
    
    float floatValue = 3.14159f;             // IEEE 754 float
    uint32_t timestamp = 1672531200;         // Unix timestamp
    
    // Method 4: Mixed types
    volatile uint16_t statusFlags = 0b0101;  // Direct pointer coil
    volatile uint16_t alarmFlags = 0b0010;   // Direct pointer discrete input
    
    // Simulation counters
    uint32_t updateCounter = 0;
} demoData;

// Function prototypes
void setupWords();
void updateSimulation();
void logWordAccess(const char* operation, const Modbus::Word& word, const uint16_t* values = nullptr, size_t count = 1);

// Utility function to log Word access
void logWordAccess(const char* operation, const Modbus::Word& word, const uint16_t* values, size_t count) {
    Serial.print("[Word ");
    Serial.print(operation);
    Serial.print("] ");
    Serial.print("Addr:");
    Serial.print(word.startAddr);
    Serial.print(", regs:");
    Serial.print(word.nbRegs);
    Serial.print(") = [");
    for (size_t i = 0; i < count; i++) {
        if (i > 0) Serial.print(", ");
        Serial.print(values[i]);
    }
    Serial.println("]");
}

void setup() {
    // Initialize serial for debug output
    Serial.begin(115200);
    Serial.println("\nEZModbus RTU Server - Word Registration Demo");
    
    // Initialize UART driver
    uart.begin();
    
    // Initialize Modbus Server
    if (server.begin() != ModbusServer::SUCCESS) {
        Serial.println("Failed to initialize Modbus Server");
        while (1) { delay(1000); } // Halt
    }
    
    Serial.println("Modbus server initialized");
    
    // Setup Modbus Words with different methods
    setupWords();
}

void loop() {
    // Update demonstration data
    updateSimulation();
    
    // Small delay to prevent too frequent updates
    delay(1000);
}

// Word setup demonstrating all registration methods
void setupWords() {
    Serial.println("\n=== EZModbus Word Registration Methods Demo ===");
    
    Serial.println("\n--- Method 1: Direct Pointer Access (Single Registers) ---");
    
    // ===================================================================================
    // METHOD 1 - Direct Pointer Access (Single Registers)
    // ===================================================================================

    server.addWord({
        .type = Modbus::INPUT_REGISTER,
        .startAddr = RegAddr::SENSOR_TEMP,
        .nbRegs = 1,
        .value = &demoData.sensorTemp  // Direct access to volatile variable
    });
    
    // Or with condensed syntax
    server.addWord({Modbus::INPUT_REGISTER, RegAddr::SENSOR_HUMIDITY, 1, &demoData.sensorHumidity});
    server.addWord({Modbus::COIL, RegAddr::STATUS_FLAGS, 1, &demoData.statusFlags});
    server.addWord({Modbus::DISCRETE_INPUT, RegAddr::ALARM_FLAGS, 1, &demoData.alarmFlags});
    
    Serial.println("  ✓ Added 4 Words with direct pointer access");
    Serial.println("\n--- Method 2: Single Register Handlers (with validation/logging) ---");
    
    // ===================================================================================
    // METHOD 2 - Single Register Handlers (with validation/logging)
    // ===================================================================================

    // Read handler for configuration setpoint
    auto cfgReadHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        outVals[0] = demoData.configSetpoint;
        logWordAccess("READ", word, outVals, 1);
        return Modbus::NULL_EXCEPTION;
    };

    // Write handler for configuration setpoint
    auto cfgWriteHandler = [](const uint16_t* writeVals, const Modbus::Word& word, void* userCtx) -> Modbus::ExceptionCode {
        uint16_t value = writeVals[0];
        
        // Validate range (10°C to 40°C)
        if (value < 100 || value > 400) {
            Serial.printf("[VALIDATION ERROR] Holding Register %d: value %d out of range [100-400]\n", 
                         word.startAddr, value);
            return Modbus::ILLEGAL_DATA_VALUE;
        }
        
        demoData.configSetpoint = value;
        logWordAccess("WRITE", word, writeVals, 1);
        return Modbus::NULL_EXCEPTION;
    };
    
    // Add word with handlers
    server.addWord({
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = RegAddr::CONFIG_SETPOINT,
        .nbRegs = 1,
        .value = nullptr,
        .readHandler = cfgReadHandler,
        .writeHandler = cfgWriteHandler,
    });
    
    // Or with inline handler definitions
    server.addWord({
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = RegAddr::CONFIG_MODE,
        .nbRegs = 1,
        .value = nullptr,
        .readHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            outVals[0] = demoData.configMode;
            logWordAccess("READ", word, outVals, 1);
            return Modbus::NULL_EXCEPTION;
        },
        .writeHandler = [](const uint16_t* writeVals, const Modbus::Word& word, void* userCtx) -> Modbus::ExceptionCode {
            uint16_t value = writeVals[0];
            
            // Validate mode (0=manual, 1=auto, 2=debug)
            if (value > 2) {
                Serial.printf("[VALIDATION ERROR] Holding Register %d: invalid mode %d\n", 
                             word.startAddr, value);
                return Modbus::ILLEGAL_DATA_VALUE;
            }
            
            demoData.configMode = value;
            
            // Side effect: mode change triggers action
            if (value == 2) {
                Serial.println("[MODE CHANGE] Entering debug mode - resetting counters");
                demoData.updateCounter = 0;
            }
            
            logWordAccess("WRITE", word, writeVals, 1);
            return Modbus::NULL_EXCEPTION;
        }
    });
    
    Serial.println("  ✓ Added 2 Words with single register handlers");
    Serial.println("\n--- Method 3: Multi-Register Handlers (atomic operations) ---");
    
    // ===================================================================================
    // METHOD 3 - Multi-Register Handlers (atomic operations)
    // ===================================================================================

    // Multi-register structure (3 registers: min/max/avg)
    server.addWord({
        .type = Modbus::INPUT_REGISTER,
        .startAddr = RegAddr::STATS_BLOCK,
        .nbRegs = 3,
        .value = nullptr,  // Multi-register MUST use handlers
        .readHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            outVals[0] = demoData.tempStats.min;
            outVals[1] = demoData.tempStats.max;
            outVals[2] = demoData.tempStats.avg;
            logWordAccess("READ", word, outVals, 3);
            return Modbus::NULL_EXCEPTION;
        }
        // No writeHandler for INPUT_REGISTER
    });
    
    // Multi-register IEEE 754 float (2 registers)
    server.addWord({
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = RegAddr::FLOAT_VALUE,
        .nbRegs = 2,
        .value = nullptr,  // Multi-register MUST use handlers
        .readHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            // Convert float to two uint16_t using IEEE 754
            ModbusCodec::floatToRegisters(demoData.floatValue, outVals);
            logWordAccess("READ", word, outVals, 2);
            return Modbus::NULL_EXCEPTION;
        },
        .writeHandler = [](const uint16_t* writeVals, const Modbus::Word& word, void* userCtx) -> Modbus::ExceptionCode {
            // Convert two uint16_t back to float
            demoData.floatValue = ModbusCodec::registersToFloat(writeVals);
            logWordAccess("WRITE", word, writeVals, 2);
            Serial.printf("[FLOAT CONVERSION] New float value: %.6f\n", demoData.floatValue);
            return Modbus::NULL_EXCEPTION;
        }
    });
    
    // Multi-register timestamp (2 registers for 32-bit unix timestamp)
    server.addWord({
        .type = Modbus::INPUT_REGISTER,
        .startAddr = RegAddr::TIMESTAMP,
        .nbRegs = 2,
        .value = nullptr,  // Multi-register MUST use handlers
        .readHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            outVals[0] = (demoData.timestamp >> 16) & 0xFFFF;  // High word
            outVals[1] = demoData.timestamp & 0xFFFF;          // Low word
            logWordAccess("READ", word, outVals, 2);
            return Modbus::NULL_EXCEPTION;
        }
        // No writeHandler for INPUT_REGISTER
    });
    
    Serial.println("  ✓ Added 3 Words with multi-register handlers");
    Serial.println("\n=== Word Registration Complete ===");
    Serial.printf("Total Words registered: %d\n", 9);
    Serial.println("\nDemo shows:");
    Serial.println("  - Direct pointer: fastest, single registers only");
    Serial.println("  - Single handlers: validation, logging, computed values");
    Serial.println("  - Multi handlers: atomic operations, complex data types");
}

// Update demonstration data to show dynamic behavior
void updateSimulation() {
    demoData.updateCounter++;
    
    // Update sensor values (simulate changing conditions)
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    if (now - lastUpdate >= 2000) {  // Update every 2 seconds
        lastUpdate = now;
        
        // Simulate temperature sensor drift
        int16_t tempDelta = random(-20, 21);  // ±2.0°C
        demoData.sensorTemp = constrain(demoData.sensorTemp + tempDelta, 150, 350);
        
        // Simulate humidity sensor drift  
        int16_t humidDelta = random(-50, 51);  // ±5.0%
        demoData.sensorHumidity = constrain(demoData.sensorHumidity + humidDelta, 200, 900);
        
        // Update statistics based on current temperature
        demoData.tempStats.min = std::min(demoData.tempStats.min, (uint16_t)demoData.sensorTemp);
        demoData.tempStats.max = std::max(demoData.tempStats.max, (uint16_t)demoData.sensorTemp);
        demoData.tempStats.avg = (demoData.tempStats.min + demoData.tempStats.max) / 2;
        
        // Update timestamp 
        demoData.timestamp = now / 1000;  // Convert millis to seconds
        
        // Toggle status flags occasionally
        if (random(0, 10) < 2) {  // 20% chance
            demoData.statusFlags ^= 0b0001;  // Toggle bit 0
        }
        
        // Update alarm flags based on temperature
        if (demoData.sensorTemp > 300 || demoData.sensorTemp < 200) {
            demoData.alarmFlags |= 0b0001;   // Set temperature alarm
        } else {
            demoData.alarmFlags &= ~0b0001;  // Clear temperature alarm
        }
        
        Serial.printf("[UPDATE %lu] Temp: %.1f°C, Humidity: %.1f%%, Alarms: 0x%X\n",
                     demoData.updateCounter,
                     demoData.sensorTemp / 10.0f,
                     demoData.sensorHumidity / 10.0f,
                     demoData.alarmFlags);
    }
}