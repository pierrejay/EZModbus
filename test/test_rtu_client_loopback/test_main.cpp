#include <Arduino.h>
#include <unity.h>
#include <EZModbus.h>
#include "test_params.h"
#include <utils/ModbusLogger.hpp>
#include "test_helpers.hpp"

// Give some time for the application logs to be printed before asserting
#ifdef EZMODBUS_DEBUG
    #define TEST_ASSERT_START() { Modbus::Logger::waitQueueFlushed(); }
#else
    #define TEST_ASSERT_START() { vTaskDelay(pdMS_TO_TICKS(50)); }
#endif

// Pin / port / baud definitions - overridable per-environment via build flags.
// Defaults target the Seeed XIAO ESP32-S3 loopback wiring; the *_lpuart env overrides
// MBT_* to put the server on the LP-UART (see platformio.ini).
#ifndef EZM_SERIAL
    #define EZM_SERIAL Serial1        // Client HardwareSerial instance (HP UART)
#endif
#ifndef EZM_RX
    #define EZM_RX D5
#endif
#ifndef EZM_TX
    #define EZM_TX D6
#endif
#ifndef MBT_UART_NUM
    #define MBT_UART_NUM UART_NUM_2   // Server IDF port (can be LP_UART_NUM_0)
#endif
#ifndef MBT_RX
    #define MBT_RX D7
#endif
#ifndef MBT_TX
    #define MBT_TX D8
#endif
#ifndef MBT_DE
    #define MBT_DE -1                 // Server DE pin (-1 = none); exercises soft DE on LP-UART
#endif

// UART configurations
#define UART_BUFFER_SIZE 512
#ifndef UART_BAUD_RATE
    #define UART_BAUD_RATE 921600
#endif

// EZModbus RTU client: use ArduinoConfig
ModbusHAL::UART::ArduinoConfig ezm_cfg = {
    .serial = EZM_SERIAL,
    .baud = UART_BAUD_RATE,
    .config = SERIAL_8N1,
    .rxPin = EZM_RX,
    .txPin = EZM_TX
};
ModbusHAL::UART ezm_uart(ezm_cfg);
ModbusInterface::RTU ezm(ezm_uart, Modbus::MASTER);
Modbus::Client client(ezm);

// EZModbus RTU server for testing: use IDFConfig
ModbusHAL::UART::IDFConfig mbt_cfg = {
    .uartNum = MBT_UART_NUM,
    .baud = UART_BAUD_RATE,
    .config = ModbusHAL::UART::CONFIG_8N1,
    .rxPin = MBT_RX,
    .txPin = MBT_TX,
    .dePin = MBT_DE
};
ModbusHAL::UART mbt_uart(mbt_cfg);
ModbusInterface::RTU mbt(mbt_uart, Modbus::SERVER);
// Capacity follows the register count (4 word types per address). Kept in sync so a smaller
// MBT_INIT_REG_COUNT (e.g. on RAM-constrained chips without PSRAM like the ESP32-C6) also shrinks
// this heap reservation - a hardcoded 10000 would OOM at static-init and hang before USB comes up.
Modbus::DynamicWordStore dynamicStore(MBT_INIT_REG_COUNT * 4);  // Heap-allocated store
Modbus::Server server(mbt, dynamicStore);
uint16_t serverDiscreteInputs[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverCoils[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverHoldingRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverInputRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
Modbus::ReadWordHandler serverReadCallback = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
    switch (word.type) {
        case Modbus::HOLDING_REGISTER:
            outVals[0] = serverHoldingRegisters[word.startAddr];
            return Modbus::NULL_EXCEPTION;
        case Modbus::INPUT_REGISTER:
            outVals[0] = serverInputRegisters[word.startAddr];
            return Modbus::NULL_EXCEPTION;
        case Modbus::COIL:
            outVals[0] = serverCoils[word.startAddr];
            return Modbus::NULL_EXCEPTION;
        case Modbus::DISCRETE_INPUT:
            outVals[0] = serverDiscreteInputs[word.startAddr];
            return Modbus::NULL_EXCEPTION;
        default:
            return Modbus::SLAVE_DEVICE_FAILURE;
    }
};
Modbus::WriteWordHandler serverWriteCallback = [](const uint16_t* writeVals, const Modbus::Word& word, void* userCtx) -> Modbus::ExceptionCode {
    switch (word.type) {
        case Modbus::HOLDING_REGISTER:
            serverHoldingRegisters[word.startAddr] = writeVals[0];
            return Modbus::NULL_EXCEPTION;
        case Modbus::COIL:
            serverCoils[word.startAddr] = writeVals[0];
            return Modbus::NULL_EXCEPTION;
        default:
            return Modbus::SLAVE_DEVICE_FAILURE;
    }
};

// Tasks
TaskHandle_t modbusTestServerTaskHandle = NULL;
bool modbusTestServerTaskInitialized = false;

void flushSerialBuffer(HardwareSerial& port) {
    while (port.available()) {
        port.read();
    }
}

// Function to reset ModbusTestServer registers
void resetModbusTestServerRegisters() {
    for (int i = MBT_INIT_START_REG; i < MBT_INIT_START_REG + MBT_INIT_REG_COUNT; i++) {
        serverCoils[i] = MBT_INIT_COIL_VALUE(i);                           // Coils : all true
        serverDiscreteInputs[i] = MBT_INIT_DISCRETE_INPUT_VALUE(i);        // Discrete inputs : all true
        serverHoldingRegisters[i] = MBT_INIT_HOLDING_REGISTER_VALUE(i);    // Holding registers : 10 + their index
        serverInputRegisters[i] = MBT_INIT_INPUT_REGISTER_VALUE(i);        // Input registers : 20 + their index (arbitrary)
    }
}

// ModbusTestServer task
void ModbusTestServerTask(void* pvParameters) {
    // Initialize ModbusTestServer RTU server
    auto ifcInitRes = mbt.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[ModbusTestServerTask] EZModbus RTU interface initialization failed");
        return;
    }
    Modbus::Logger::logln("[ModbusTestServerTask] EZModbus RTU interface initialized");
    
    // Configure registers for each type (reduced for StaticWordStore testing)
    uint8_t regTypes[] = {
        Modbus::HOLDING_REGISTER,
        Modbus::INPUT_REGISTER,
        Modbus::COIL,
        Modbus::DISCRETE_INPUT
    };

    Modbus::Logger::logln("Adding words to ModbusTestServer...");
    uint32_t startTime = millis();    
    ssize_t freeHeapBefore = ESP.getFreeHeap();
    ssize_t freePsramBefore = ESP.getFreePsram(); // returns 0 on boards without PSRAM
    
    for (int i = MBT_INIT_START_REG; i < MBT_INIT_START_REG + MBT_INIT_REG_COUNT; i++) { // Insert in order
    // for (int i = MBT_INIT_START_REG + MBT_INIT_REG_COUNT - 1; i >= MBT_INIT_START_REG; i--) { //
        
        // Progress logging every 500 addresses to monitor for hangs
        if (i % 500 == 0) {
            Modbus::Logger::logf("Adding words for address %d/%d...\n", i, MBT_INIT_REG_COUNT);
        }
        
        for (uint8_t rt : regTypes) {
            Modbus::Word word;
            word.type = (Modbus::RegisterType)rt;
            word.startAddr = i;
            word.nbRegs = 1;
            word.value = nullptr;
            word.readHandler = serverReadCallback;
            if (rt == Modbus::HOLDING_REGISTER || rt == Modbus::COIL) {
                word.writeHandler = serverWriteCallback;
            } else {
                word.writeHandler = nullptr;
            }
            server.addWord(word);
        }
    }
    uint32_t endTime = millis();
    ssize_t freeHeapAfter = ESP.getFreeHeap();
    ssize_t freePsramAfter = ESP.getFreePsram(); // returns 0 on boards without PSRAM
    ssize_t memoryUsed = freeHeapBefore - freeHeapAfter;
    ssize_t psramUsed = freePsramBefore - freePsramAfter;
    if (ESP.getPsramSize() > 0) {
        Modbus::Logger::logf("Added %d words in %d ms, consuming %zd bytes of heap and %zd bytes of PSRAM\n", MBT_INIT_REG_COUNT * 4, endTime - startTime, memoryUsed, psramUsed);
    } else {
        Modbus::Logger::logf("Added %d words in %d ms, consuming %zd bytes of heap\n", MBT_INIT_REG_COUNT * 4, endTime - startTime, memoryUsed);
    }

    // Initialize Modbus::Server application
    auto srvInitRes = server.begin();
    if (srvInitRes != Modbus::Server::SUCCESS) {
        Modbus::Logger::logln("[ModbusTestServerTask] EZModbus Server initialization failed");
        return;
    }
    Modbus::Logger::logln("[ModbusTestServerTask] EZModbus Server initialized");

    // Set initial values
    resetModbusTestServerRegisters();

    // Server polling is handled by interface callbacks - no explicit poll needed
    while (true) {
        modbusTestServerTaskInitialized = true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setUp() {
    // Clear both RX buffers before each test
    vTaskDelay(pdMS_TO_TICKS(10));
    ezm_uart.flush_input();
    mbt_uart.flush_input();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Reset ModbusTestServer registers
    resetModbusTestServerRegisters();
}

void tearDown() {
    Serial.flush(); // Make sure all Unity logs are printed
}

// === Generation of read tests ===
#define X(Name, ReadSingle, ReadMulti, Addr, Expect, FC) \
void test_read_##Name##_sync() { \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_" #Name "_SYNC - CASE 1: SYNCHRONOUS READ W/ HELPER (NOW USING sendRequest)"); \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, response_case1.exceptionCode); \
    TEST_ASSERT_EQUAL(FC, response_case1.fc); \
    uint16_t readValue_case1; \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        auto coilValues = response_case1.getCoils(); \
        TEST_ASSERT_FALSE(coilValues.empty()); \
        readValue_case1 = coilValues[0]; \
    } else { \
        auto regValues = response_case1.getRegisters(); \
        TEST_ASSERT_FALSE(regValues.empty()); \
        readValue_case1 = regValues[0]; \
    } \
    TEST_ASSERT_EQUAL(Expect(Addr), readValue_case1); \
    \
    Modbus::Logger::logln("TEST_READ_" #Name "_SYNC - CASE 2: SYNCHRONOUS READ W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, nullptr); \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(FC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    uint16_t readValue; \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        auto coilValues = response.getCoils(); \
        TEST_ASSERT_FALSE(coilValues.empty()); \
        readValue = coilValues[0]; \
    } else { \
        auto regValues = response.getRegisters(); \
        TEST_ASSERT_FALSE(regValues.empty()); \
        readValue = regValues[0]; \
    } \
    TEST_ASSERT_EQUAL(Expect(Addr), readValue); \
} \
\
void test_read_##Name##_async() { \
    Modbus::Client::Result tracker_case1 = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_" #Name "_ASYNC - CASE 1: ASYNCHRONOUS READ W/ HELPER (NOW USING sendRequest)"); \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, &tracker_case1); \
    while (tracker_case1 == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, response_case1.exceptionCode); \
    TEST_ASSERT_EQUAL(FC, response_case1.fc); \
    TEST_ASSERT_EQUAL(Expect(Addr), response_case1.data[0]); \
    \
    Modbus::Client::Result tracker = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_" #Name "_ASYNC - CASE 2: ASYNCHRONOUS READ W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, &tracker); \
    /* Attente de fin de transfert */ \
    while (tracker == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker); \
    TEST_ASSERT_EQUAL(FC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    TEST_ASSERT_EQUAL(Expect(Addr), response.data[0]); \
} \
\
void test_read_multiple_##Name() { \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_MULTIPLE_" #Name " - CASE 1: READ MULTIPLE W/ HELPER (NOW USING sendRequest)"); \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, response_case1.exceptionCode); \
    TEST_ASSERT_EQUAL(FC, response_case1.fc); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, response_case1.regCount); \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        auto coilValues = response_case1.getCoils(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, coilValues.size()); \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), coilValues[i]); \
        } \
    } else { \
        TEST_ASSERT_EQUAL(MULTI_COUNT, response_case1.regCount); \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), response_case1.data[i]); \
        } \
    } \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_MULTIPLE_" #Name " - CASE 2: READ MULTIPLE W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, nullptr); \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(FC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    \
    Modbus::Logger::logf("TEST_READ_MULTIPLE_%s - CASE 3: FC = %s\n", #Name, Modbus::toString(response.fc)); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, response.regCount); \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        auto coilValues = response.getCoils(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, coilValues.size()); \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), coilValues[i]); \
        } \
    } else { \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), response.data[i]); \
        } \
    } \
} \
\
void test_read_multiple_##Name##_async() { \
    Modbus::Client::Result tracker_case1 = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_MULTIPLE_" #Name "_ASYNC - CASE 1: READ MULTIPLE W/ HELPER (NOW USING sendRequest)"); \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, &tracker_case1); \
    while (tracker_case1 == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, response_case1.exceptionCode); \
    TEST_ASSERT_EQUAL(FC, response_case1.fc); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, response_case1.regCount); \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        auto coilValues = response_case1.getCoils(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, coilValues.size()); \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), coilValues[i]); \
        } \
    } else { \
        TEST_ASSERT_EQUAL(MULTI_COUNT, response_case1.regCount); \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), response_case1.data[i]); \
        } \
    } \
    \
    Modbus::Client::Result tracker = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_MULTIPLE_" #Name "_ASYNC - CASE 2: READ MULTIPLE W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, &tracker); \
    /* Attente de fin de transfert */ \
    while (tracker == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker); \
    TEST_ASSERT_EQUAL(FC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, response.regCount); \
    \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        auto coilValues = response.getCoils(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, coilValues.size()); \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), coilValues[i]); \
        } \
    } else { \
        for (int i = 0; i < MULTI_COUNT; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), response.data[i]); \
        } \
    } \
} \
\
void test_read_max_##Name() { \
    uint16_t maxCount; \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        maxCount = Modbus::MAX_COILS_READ; \
    } else { \
        maxCount = Modbus::MAX_REGISTERS_READ; \
    } \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_READ_MAX_" #Name " - CASE 1: READ MAX W/ HELPER (NOW USING sendRequest)"); \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = FC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = maxCount, \
        .data = {}, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, response_case1.exceptionCode); \
    TEST_ASSERT_EQUAL(FC, response_case1.fc); \
    TEST_ASSERT_EQUAL(maxCount, response_case1.regCount); \
    if (FC == Modbus::READ_COILS || FC == Modbus::READ_DISCRETE_INPUTS) { \
        auto coilValues = response_case1.getCoils(); \
        TEST_ASSERT_EQUAL(maxCount, coilValues.size()); \
        for (int i = 0; i < maxCount; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), coilValues[i]); \
        } \
    } else { \
        for (int i = 0; i < maxCount; ++i) { \
            TEST_ASSERT_EQUAL(Expect(MULTI_START_ADDR + i), response_case1.data[i]); \
        } \
    } \
    \
}

// Generation of read tests
READ_TESTS
#undef X

// === Generation of write tests ===
#define X(Name, WriteSingle, WriteMulti, Addr, TestValue, SingleFC, MultiFC) \
void test_write_##Name##_sync() { \
    int32_t writeResult_case1; \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_" #Name "_SYNC - CASE 1: SYNCHRONOUS WRITE W/ HELPER (NOW USING sendRequest)"); \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = SingleFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = (SingleFC == Modbus::WRITE_COIL) ? \
                Modbus::packCoils({(uint16_t)TestValue}) : \
                Modbus::packRegisters({(uint16_t)TestValue}), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    if (result_case1 == Modbus::Client::SUCCESS && response_case1.exceptionCode == Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = 1; \
        TEST_ASSERT_EQUAL(SingleFC, response_case1.fc); \
        TEST_ASSERT_EQUAL(Addr, response_case1.regAddress); \
    } else if (response_case1.exceptionCode != Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = -response_case1.exceptionCode; \
    } else { \
        writeResult_case1 = 0; \
    } \
    TEST_ASSERT_EQUAL(1, writeResult_case1); \
    \
    /* Check that the value has been written by reading it */ \
    int32_t readValue; \
    Modbus::Frame readRequest_case1 = { \
        .type = Modbus::REQUEST, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    if (SingleFC == Modbus::WRITE_COIL) { \
        readRequest_case1.fc = Modbus::READ_COILS; \
    } else { \
        readRequest_case1.fc = Modbus::READ_HOLDING_REGISTERS; \
    } \
    Modbus::Frame readResponse_case1; \
    auto readOpResult_case1 = client.sendRequest(readRequest_case1, readResponse_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, readOpResult_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readResponse_case1.exceptionCode); \
    readValue = readResponse_case1.data[0]; \
    TEST_ASSERT_EQUAL(TestValue, readValue); \
    \
    /* Reset ModbusTestServer registers before next case */ \
    resetModbusTestServerRegisters(); \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_" #Name "_SYNC - CASE 2: SYNCHRONOUS WRITE W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = SingleFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = (SingleFC == Modbus::WRITE_COIL) ? \
                Modbus::packCoils({(uint16_t)TestValue}) : \
                Modbus::packRegisters({(uint16_t)TestValue}), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, nullptr); \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(SingleFC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    TEST_ASSERT_EQUAL(Addr, response.regAddress); \
    \
    /* Check that the value has been written by reading it */ \
    if (SingleFC == Modbus::WRITE_COIL) { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_COILS, .slaveId = TEST_SLAVE_ID, .regAddress = Addr, .regCount = 1}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) readValue = readResp.data[0]; \
    } else { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = TEST_SLAVE_ID, .regAddress = Addr, .regCount = 1}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) readValue = readResp.data[0]; \
    } \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(TestValue, readValue); \
} \
\
void test_write_##Name##_async() { \
    int32_t writeResult_case1; \
    Modbus::Client::Result tracker_case1 = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_" #Name "_ASYNC - CASE 1: ASYNCHRONOUS WRITE W/ HELPER (NOW USING sendRequest)"); \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = SingleFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = (SingleFC == Modbus::WRITE_COIL) ? \
                Modbus::packCoils({(uint16_t)TestValue}) : \
                Modbus::packRegisters({(uint16_t)TestValue}), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, &tracker_case1); \
    while (tracker_case1 == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker_case1); \
    if (result_case1 == Modbus::Client::SUCCESS && response_case1.exceptionCode == Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = 1; \
        TEST_ASSERT_EQUAL(SingleFC, response_case1.fc); \
        TEST_ASSERT_EQUAL(Addr, response_case1.regAddress); \
    } else if (response_case1.exceptionCode != Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = -response_case1.exceptionCode; \
    } else { \
        writeResult_case1 = 0; \
    } \
    TEST_ASSERT_EQUAL(1, writeResult_case1); \
    \
    /* Check that the value has been written by reading it */ \
    int32_t readValue; \
    Modbus::Frame readRequest_case1 = { \
        .type = Modbus::REQUEST, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    if (SingleFC == Modbus::WRITE_COIL) { \
        readRequest_case1.fc = Modbus::READ_COILS; \
    } else { \
        readRequest_case1.fc = Modbus::READ_HOLDING_REGISTERS; \
    } \
    Modbus::Frame readResponse_case1; \
    auto readOpResult_case1 = client.sendRequest(readRequest_case1, readResponse_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, readOpResult_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readResponse_case1.exceptionCode); \
    readValue = readResponse_case1.data[0]; \
    TEST_ASSERT_EQUAL(TestValue, readValue); \
    \
    /* Reset ModbusTestServer registers before next case */ \
    resetModbusTestServerRegisters(); \
    \
    Modbus::Client::Result tracker = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_" #Name "_ASYNC - CASE 2: ASYNCHRONOUS WRITE W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = SingleFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = Addr, \
        .regCount = 1, \
        .data = (SingleFC == Modbus::WRITE_COIL) ? \
                Modbus::packCoils({(uint16_t)TestValue}) : \
                Modbus::packRegisters({(uint16_t)TestValue}), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, &tracker); \
    /* Attente de fin de transfert */ \
    while (tracker == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker); \
    TEST_ASSERT_EQUAL(SingleFC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    TEST_ASSERT_EQUAL(Addr, response.regAddress); \
    \
    /* Check that the value has been written by reading it */ \
    if (SingleFC == Modbus::WRITE_COIL) { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_COILS, .slaveId = TEST_SLAVE_ID, .regAddress = Addr, .regCount = 1}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) { \
            auto coilValues = readResp.getCoils(); \
            TEST_ASSERT_EQUAL(1, coilValues.size()); \
            readValue = coilValues[0]; \
        } \
        TEST_ASSERT_START(); \
        TEST_ASSERT_EQUAL(TestValue, readValue); \
    } else { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = TEST_SLAVE_ID, .regAddress = Addr, .regCount = 1}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) { \
            TEST_ASSERT_EQUAL(1, readResp.regCount); \
            readValue = readResp.data[0]; \
        } \
        TEST_ASSERT_START(); \
        TEST_ASSERT_EQUAL(TestValue, readValue); \
    } \
} \
\
void test_write_multiple_##Name() { \
    int32_t writeResult_case1; \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_MULTIPLE_" #Name " - CASE 1: WRITE MULTIPLE W/ HELPER (NOW USING sendRequest)"); \
    std::vector<uint16_t> values_case1; \
    values_case1.reserve(MULTI_COUNT); \
    for (int i = 0; i < MULTI_COUNT; i++) { \
        values_case1.push_back(TestValue); \
    } \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = MultiFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = (MultiFC == Modbus::WRITE_MULTIPLE_COILS) ? Modbus::packCoils(values_case1) : Modbus::packRegisters(values_case1), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    if (result_case1 == Modbus::Client::SUCCESS && response_case1.exceptionCode == Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = 1; \
        TEST_ASSERT_EQUAL(MultiFC, response_case1.fc); \
        TEST_ASSERT_EQUAL(MULTI_START_ADDR, response_case1.regAddress); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, response_case1.regCount); \
    } else if (response_case1.exceptionCode != Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = -response_case1.exceptionCode; \
    } else { \
        writeResult_case1 = 0; \
    } \
    TEST_ASSERT_EQUAL(1, writeResult_case1); \
    \
    /* Check that the values have been written by reading them */ \
    std::vector<int32_t> readValues; \
    Modbus::Frame readRequest_case1 = { \
        .type = Modbus::REQUEST, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    if (MultiFC == Modbus::WRITE_MULTIPLE_COILS) { \
        readRequest_case1.fc = Modbus::READ_COILS; \
    } else { \
        readRequest_case1.fc = Modbus::READ_HOLDING_REGISTERS; \
    } \
    Modbus::Frame readResponse_case1; \
    auto readOpResult_case1 = client.sendRequest(readRequest_case1, readResponse_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, readOpResult_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readResponse_case1.exceptionCode); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, readResponse_case1.regCount); \
    readValues.resize(readResponse_case1.data.size()); \
    for (size_t i = 0; i < readResponse_case1.data.size(); ++i) readValues[i] = readResponse_case1.data[i]; \
    for (int i = 0; i < MULTI_COUNT; i++) { \
        TEST_ASSERT_EQUAL(TestValue, readValues[i]); \
    } \
    \
    /* Reset ModbusTestServer registers before next case */ \
    resetModbusTestServerRegisters(); \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_MULTIPLE_" #Name " - CASE 2: WRITE MULTIPLE W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = MultiFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = (MultiFC == Modbus::WRITE_MULTIPLE_COILS) ? Modbus::packCoils(values_case1) : Modbus::packRegisters(values_case1), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, nullptr); \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(MultiFC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    TEST_ASSERT_EQUAL(MULTI_START_ADDR, response.regAddress); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, response.regCount); \
    \
    /* Check that the values have been written by reading them */ \
    if (MultiFC == Modbus::WRITE_MULTIPLE_COILS) { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_COILS, .slaveId = TEST_SLAVE_ID, .regAddress = MULTI_START_ADDR, .regCount = MULTI_COUNT}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) { \
            auto coilValues = readResp.getCoils(); \
            TEST_ASSERT_EQUAL(MULTI_COUNT, coilValues.size()); \
            readValues.resize(coilValues.size()); \
            for(size_t i=0; i<coilValues.size(); ++i) readValues[i] = coilValues[i]; \
        } \
        TEST_ASSERT_START(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, readValues.size()); \
        for (int i = 0; i < MULTI_COUNT; i++) { \
            TEST_ASSERT_EQUAL(TestValue, readValues[i]); \
        } \
    } else { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = TEST_SLAVE_ID, .regAddress = MULTI_START_ADDR, .regCount = MULTI_COUNT}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) { \
            TEST_ASSERT_EQUAL(MULTI_COUNT, readResp.regCount); \
            readValues.resize(readResp.regCount); \
            for(size_t i=0; i<readResp.regCount; ++i) readValues[i] = readResp.data[i]; \
        } \
        TEST_ASSERT_START(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, readValues.size()); \
        for (int i = 0; i < MULTI_COUNT; i++) { \
            TEST_ASSERT_EQUAL(TestValue, readValues[i]); \
        } \
    } \
} \
\
void test_write_multiple_##Name##_async() { \
    int32_t writeResult_case1; \
    Modbus::Client::Result tracker_case1 = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_MULTIPLE_" #Name "_ASYNC - CASE 1: WRITE MULTIPLE W/ HELPER (NOW USING sendRequest)"); \
    std::vector<uint16_t> values_case1; \
    values_case1.reserve(MULTI_COUNT); \
    for (int i = 0; i < MULTI_COUNT; i++) { \
        values_case1.push_back(TestValue); \
    } \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = MultiFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = (MultiFC == Modbus::WRITE_MULTIPLE_COILS) ? Modbus::packCoils(values_case1) : Modbus::packRegisters(values_case1), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, &tracker_case1); \
    while (tracker_case1 == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker_case1); \
    if (result_case1 == Modbus::Client::SUCCESS && response_case1.exceptionCode == Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = 1; \
        TEST_ASSERT_EQUAL(MultiFC, response_case1.fc); \
        TEST_ASSERT_EQUAL(MULTI_START_ADDR, response_case1.regAddress); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, response_case1.regCount); \
    } else if (response_case1.exceptionCode != Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = -response_case1.exceptionCode; \
    } else { \
        writeResult_case1 = 0; \
    } \
    TEST_ASSERT_EQUAL(1, writeResult_case1); \
    \
    /* Check that the values have been written by reading them */ \
    std::vector<int32_t> readValues; \
    Modbus::Frame readRequest_case1 = { \
        .type = Modbus::REQUEST, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    if (MultiFC == Modbus::WRITE_MULTIPLE_COILS) { \
        readRequest_case1.fc = Modbus::READ_COILS; \
    } else { \
        readRequest_case1.fc = Modbus::READ_HOLDING_REGISTERS; \
    } \
    Modbus::Frame readResponse_case1; \
    auto readOpResult_case1 = client.sendRequest(readRequest_case1, readResponse_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, readOpResult_case1); \
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readResponse_case1.exceptionCode); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, readResponse_case1.regCount); \
    readValues.resize(readResponse_case1.data.size()); \
    for (size_t i = 0; i < readResponse_case1.data.size(); ++i) readValues[i] = readResponse_case1.data[i]; \
    for (int i = 0; i < MULTI_COUNT; i++) { \
        TEST_ASSERT_EQUAL(TestValue, readValues[i]); \
    } \
    \
    /* Reset ModbusTestServer registers before next case */ \
    resetModbusTestServerRegisters(); \
    \
    Modbus::Client::Result tracker = Modbus::Client::NODATA; \
    \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_MULTIPLE_" #Name "_ASYNC - CASE 2: WRITE MULTIPLE W/O HELPER"); \
    Modbus::Frame request = { \
        .type = Modbus::REQUEST, \
        .fc = MultiFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = MULTI_COUNT, \
        .data = (MultiFC == Modbus::WRITE_MULTIPLE_COILS) ? Modbus::packCoils(values_case1) : Modbus::packRegisters(values_case1), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response; \
    auto result = client.sendRequest(request, response, &tracker); \
    /* Attente de fin de transfert */ \
    while (tracker == Modbus::Client::NODATA) { \
        vTaskDelay(pdMS_TO_TICKS(1)); \
    } \
    \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker); \
    TEST_ASSERT_EQUAL(MultiFC, response.fc); \
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId); \
    TEST_ASSERT_EQUAL(MULTI_START_ADDR, response.regAddress); \
    TEST_ASSERT_EQUAL(MULTI_COUNT, response.regCount); \
    \
    /* Check that the values have been written by reading them */ \
    if (MultiFC == Modbus::WRITE_MULTIPLE_COILS) { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_COILS, .slaveId = TEST_SLAVE_ID, .regAddress = MULTI_START_ADDR, .regCount = MULTI_COUNT}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) { \
            auto coilValues = readResp.getCoils(); \
            TEST_ASSERT_EQUAL(MULTI_COUNT, coilValues.size()); \
            readValues.resize(coilValues.size()); \
            for(size_t i=0; i<coilValues.size(); ++i) readValues[i] = coilValues[i]; \
        } \
        TEST_ASSERT_START(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, readValues.size()); \
        for (int i = 0; i < MULTI_COUNT; i++) { \
            TEST_ASSERT_EQUAL(TestValue, readValues[i]); \
        } \
    } else { \
        Modbus::Frame readReq = {.type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = TEST_SLAVE_ID, .regAddress = MULTI_START_ADDR, .regCount = MULTI_COUNT}; \
        Modbus::Frame readResp; \
        result = client.sendRequest(readReq, readResp, nullptr); \
        if (result == Modbus::Client::SUCCESS && readResp.exceptionCode == Modbus::NULL_EXCEPTION) { \
            TEST_ASSERT_EQUAL(MULTI_COUNT, readResp.regCount); \
            readValues.resize(readResp.regCount); \
            for(size_t i=0; i<readResp.regCount; ++i) readValues[i] = readResp.data[i]; \
        } \
        TEST_ASSERT_START(); \
        TEST_ASSERT_EQUAL(MULTI_COUNT, readValues.size()); \
        for (int i = 0; i < MULTI_COUNT; i++) { \
            TEST_ASSERT_EQUAL(TestValue, readValues[i]); \
        } \
    } \
} \
\
void test_write_max_##Name() { \
    int32_t writeResult_case1; \
    Modbus::Logger::logln(); \
    Modbus::Logger::logln("TEST_WRITE_MAX_" #Name " - CASE 1: WRITE MAX REGISTERS W/ HELPER (NOW USING sendRequest)"); \
    std::vector<uint16_t> values_case1; \
    uint16_t maxCount; \
    if (MultiFC == Modbus::WRITE_MULTIPLE_COILS) { \
        maxCount = Modbus::MAX_COILS_WRITE; \
        values_case1.reserve(Modbus::MAX_COILS_WRITE); \
        for (int i = 0; i < Modbus::MAX_COILS_WRITE; i++) { \
            values_case1.push_back(TestValue); \
        } \
    } else { \
        maxCount = Modbus::MAX_REGISTERS_WRITE; \
        values_case1.reserve(Modbus::MAX_REGISTERS_WRITE); \
        for (int i = 0; i < Modbus::MAX_REGISTERS_WRITE; i++) { \
            values_case1.push_back(TestValue); \
        } \
    } \
    Modbus::Frame request_case1 = { \
        .type = Modbus::REQUEST, \
        .fc = MultiFC, \
        .slaveId = TEST_SLAVE_ID, \
        .regAddress = MULTI_START_ADDR, \
        .regCount = maxCount, \
        .data = (MultiFC == Modbus::WRITE_MULTIPLE_COILS) ? Modbus::packCoils(values_case1) : Modbus::packRegisters(values_case1), \
        .exceptionCode = Modbus::NULL_EXCEPTION \
    }; \
    Modbus::Frame response_case1; \
    auto result_case1 = client.sendRequest(request_case1, response_case1, nullptr); \
    TEST_ASSERT_START(); \
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result_case1); \
    if (result_case1 == Modbus::Client::SUCCESS && response_case1.exceptionCode == Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = 1; \
        TEST_ASSERT_EQUAL(MultiFC, response_case1.fc); \
        TEST_ASSERT_EQUAL(MULTI_START_ADDR, response_case1.regAddress); \
        TEST_ASSERT_EQUAL(maxCount, response_case1.regCount); \
    } else if (response_case1.exceptionCode != Modbus::NULL_EXCEPTION) { \
        writeResult_case1 = -response_case1.exceptionCode; \
    } else { \
        writeResult_case1 = 0; \
    } \
    TEST_ASSERT_EQUAL(1, writeResult_case1); \
    \
    /* Note: Original test_write_max did not have a read-back check. */ \
}

// Generation of write tests
WRITE_TESTS
#undef X

void test_request_timeout() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_REQUEST_TIMEOUT - CASE 1: TIMEOUT ON READ REQUEST");
    
    // Temporarily disable the Modbus test server's RX task processing new requests.
    TaskHandle_t serverRxTaskHandle = mbt.getRxTxTaskHandle(); // Updated to use new method name
    vTaskSuspend(serverRxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100)); // Give time for suspension to take effect

    // Try to read a register
    // int32_t value; // Not strictly needed if we only check tracker
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    Modbus::Frame readRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame readResponse; // Will not be filled
    auto result = client.sendRequest(
        readRequest,
        readResponse,
        &tracker // asynchronous mode to follow the timeout
    );

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); // The send itself should succeed (request queued)

    // Wait a bit more than the default timeout
    uint32_t startTime = Modbus::TIME_MS();
    // client.getRequestTimeoutMs() would be better if available, using Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS as fallback
    uint32_t timeoutDuration = Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS;
    while (Modbus::TIME_MS() - startTime < timeoutDuration + 100) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (tracker == Modbus::Client::ERR_TIMEOUT) break;
        // client.poll(); // Client poll is now internal via its own task. User doesn't call it.
        // EZMODBUS_DELAY_MS(1); // Not needed, vTaskDelay handles delay
    }
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TIMEOUT, tracker);
    
    // Reactivate the client's RX task
    mbt_uart.flush_input();
    vTaskResume(serverRxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow client's RX task to run
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_REQUEST_TIMEOUT - CASE 2: TIMEOUT ON WRITE REQUEST (SYNC)");
    
    // Temporarily disable the client's RX task again
    vTaskSuspend(serverRxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Try to write a register (synchronous call, so client.sendRequest will block)
    // int32_t writeOpResult; // From old helper, not directly applicable
    tracker = Modbus::Client::NODATA; // Tracker not used for sync call result, but can be observed if sendRequest is modified to update it even for sync.
                                      // The current sendRequest in ModbusClient.cpp does not use the tracker if the call is synchronous (tracker arg is nullptr internally).
                                      // However, the original test passed a tracker to a sync helper.
                                      // For a synchronous sendRequest (nullptr tracker), the `result` itself will be ERR_TIMEOUT.
    Modbus::Frame writeRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .data = {42},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame writeResponse;
    result = client.sendRequest(
        writeRequest,
        writeResponse,
        nullptr // Synchronous mode
    );

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TIMEOUT, result);

    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_REQUEST_TIMEOUT - CASE 2: TIMEOUT ON WRITE REQUEST (ASYNC)");
    tracker = Modbus::Client::NODATA;
     result = client.sendRequest(
        writeRequest, // Same write request
        writeResponse,
        &tracker // Asynchronous mode
    );

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); // Send initiated

    startTime = Modbus::TIME_MS();
    while (Modbus::TIME_MS() - startTime < timeoutDuration + 100) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (tracker == Modbus::Client::ERR_TIMEOUT) break;
    }
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TIMEOUT, tracker);


    // Reactivate the client's RX task
    mbt_uart.flush_input();
    vTaskResume(serverRxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wait for the server to be ready if any pending operations due to client being unresponsive
    // This delay might be important if the server tried to send something the client missed.
    vTaskDelay(pdMS_TO_TICKS(Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 200));
}

void test_modbus_exceptions() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_MODBUS_EXCEPTIONS - CASE 1: READ EXCEPTION (ILLEGAL DATA ADDRESS)");
    
    // Reading out of bounds (addr >= MBT_INIT_REG_COUNT)
    // int32_t value; // Not used directly for assertion
    Modbus::Frame readRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = MBT_INIT_REG_COUNT,  // First invalid address
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame readResponse;
    auto result = client.sendRequest(readRequest, readResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); // Communication success
    TEST_ASSERT_EQUAL(Modbus::READ_HOLDING_REGISTERS, readResponse.fc); // FC should indicate exception
    TEST_ASSERT_EQUAL(Modbus::ILLEGAL_DATA_ADDRESS, readResponse.exceptionCode);
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_MODBUS_EXCEPTIONS - CASE 2: WRITE EXCEPTION (ILLEGAL DATA ADDRESS)");
    
    // Writing out of bounds
    // int32_t writeOpResult; // Not used directly for assertion
     Modbus::Frame writeRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER, // Assuming single register write for this test
        .slaveId = TEST_SLAVE_ID,
        .regAddress = MBT_INIT_REG_COUNT,
        .regCount = 1,
        .data = {42},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame writeResponse;
    result = client.sendRequest(writeRequest, writeResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); // Communication success
    TEST_ASSERT_EQUAL(Modbus::WRITE_REGISTER, writeResponse.fc); // FC should indicate exception
    TEST_ASSERT_EQUAL(Modbus::ILLEGAL_DATA_ADDRESS, writeResponse.exceptionCode);
}

void test_invalid_parameters() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_INVALID_PARAMETERS - CASE 1: READ COILS WITH COUNT > MAX_COILS_READ");
    
    // std::vector<int32_t> values; // Not filled on error with sendRequest
    Modbus::Frame readCoilsRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_COILS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = Modbus::MAX_COILS_READ + 1, // Exceeds the max limit
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame readCoilsResponse; // Will not be properly filled
    auto result = client.sendRequest(readCoilsRequest, readCoilsResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, result);
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_INVALID_PARAMETERS - CASE 2: WRITE COILS WITH COUNT > MAX_COILS_WRITE");
    
    std::vector<uint16_t> writeCoilInputValues(Modbus::MAX_COILS_WRITE + 1, 1);  // Too many values
    // int32_t writeResult; // Not filled on error with sendRequest
    Modbus::Frame writeCoilsRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_MULTIPLE_COILS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = Modbus::MAX_COILS_WRITE + 1, // regCount should match data vector size for this check
        .data = Modbus::packCoils(writeCoilInputValues), 
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame writeCoilsResponse; // Will not be properly filled
    result = client.sendRequest(writeCoilsRequest, writeCoilsResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, result);
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_INVALID_PARAMETERS - CASE 3: READ REGISTERS WITH COUNT > MAX_REGISTERS_READ");
    
    Modbus::Frame readRegsRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS, // Could be any register read FC
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = Modbus::MAX_REGISTERS_READ + 1, // Exceeds the max limit
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame readRegsResponse; // Will not be properly filled
    result = client.sendRequest(readRegsRequest, readRegsResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, result);
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_INVALID_PARAMETERS - CASE 4: WRITE REGISTERS WITH COUNT > MAX_REGISTERS_WRITE");
    
    std::vector<uint16_t> writeRegInputValues(Modbus::MAX_REGISTERS_WRITE + 1, 42);  // Too many values
    Modbus::Frame writeRegsRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_MULTIPLE_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = Modbus::MAX_REGISTERS_WRITE + 1, // regCount should match data vector size
        .data = Modbus::packRegisters(writeRegInputValues),
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame writeRegsResponse; // Will not be properly filled
    result = client.sendRequest(writeRegsRequest, writeRegsResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, result);
}

void test_broadcast_read_rejected() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_BROADCAST_READ_REJECTED - CASE 1: SYNC BROADCAST READ W/ HELPER (NOW sendRequest)");
    
    // Try to read in broadcast (not allowed by Modbus spec)
    // int32_t value; // Not filled
    Modbus::Frame broadcastReadRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = BROADCAST_SLAVE_ID,  // Broadcast address
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame broadcastReadResponse;
    auto result = client.sendRequest(broadcastReadRequest, broadcastReadResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, result); // Should be caught by client before sending
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_BROADCAST_READ_REJECTED - CASE 2: SYNC BROADCAST READ W/O HELPER");
    
    // Also check with a raw request
    Modbus::Frame request = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = BROADCAST_SLAVE_ID,
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .data = {},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame response;
    result = client.sendRequest(request, response, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, result);
}

void test_broadcast() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_BROADCAST - CASE 1: SYNC BROADCAST WRITE W/ HELPER (NOW sendRequest)");
    
    // Test of broadcast write
    Modbus::Frame broadcastWriteRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = BROADCAST_SLAVE_ID,
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .data = Modbus::packRegisters({42}),
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame broadcastWriteResponse;
    auto result = client.sendRequest(
        broadcastWriteRequest,
        broadcastWriteResponse,
        nullptr
    );
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);

    while (!client.isReady()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Check that the value has been written on the server
    int32_t readValue;
    Modbus::Frame readCheckRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame readCheckResponse;
    result = client.sendRequest(readCheckRequest, readCheckResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readCheckResponse.exceptionCode);
    
    if (broadcastWriteRequest.fc == Modbus::WRITE_COIL) {
        auto coilValues = readCheckResponse.getCoils();
        TEST_ASSERT_FALSE(coilValues.empty());
        readValue = coilValues[0];
    } else {
        auto regValues = readCheckResponse.getRegisters();
        TEST_ASSERT_FALSE(regValues.empty());
        readValue = regValues[0];
    }
    TEST_ASSERT_EQUAL(42, readValue);
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_BROADCAST - CASE 2: ASYNC BROADCAST WRITE W/ HELPER");
    
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    broadcastWriteRequest.data = Modbus::packRegisters({43});
    result = client.sendRequest(
        broadcastWriteRequest,
        broadcastWriteResponse,
        &tracker
    );
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker);

    while (!client.isReady()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    
    result = client.sendRequest(readCheckRequest, readCheckResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readCheckResponse.exceptionCode);
    
    if (broadcastWriteRequest.fc == Modbus::WRITE_COIL) {
        auto coilValues = readCheckResponse.getCoils();
        TEST_ASSERT_FALSE(coilValues.empty());
        readValue = coilValues[0];
    } else {
        auto regValues = readCheckResponse.getRegisters();
        TEST_ASSERT_FALSE(regValues.empty());
        readValue = regValues[0];
    }
    TEST_ASSERT_EQUAL(43, readValue);
    
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_BROADCAST - CASE 3: ASYNC BROADCAST WRITE W/O HELPER");
    
    Modbus::Frame request = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = BROADCAST_SLAVE_ID,
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .data = Modbus::packRegisters({44}),
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame response;
    result = client.sendRequest(request, response, &tracker);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker);

    while (!client.isReady()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    
    result = client.sendRequest(readCheckRequest, readCheckResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readCheckResponse.exceptionCode);
    
    if (request.fc == Modbus::WRITE_COIL) {
        auto coilValues = readCheckResponse.getCoils();
        TEST_ASSERT_FALSE(coilValues.empty());
        readValue = coilValues[0];
    } else {
        auto regValues = readCheckResponse.getRegisters();
        TEST_ASSERT_FALSE(regValues.empty());
        readValue = regValues[0];
    }
    TEST_ASSERT_EQUAL(44, readValue);
}


// TEST_CONCURRENT_CALLS DATA STRUCTURES & TASKS

// Second task's core affinity: on single-core targets (e.g. ESP32-C6) there is no core 1,
// so both tasks run on core 0 (pinning to 1 would trip a configASSERT).
#ifdef CONFIG_FREERTOS_UNICORE
    #define TEST_SECOND_CORE 0
#else
    #define TEST_SECOND_CORE 1
#endif

// Synchronization data structure
struct SyncData {
    SemaphoreHandle_t startSignal;
    SemaphoreHandle_t resultsMutex;
    SemaphoreHandle_t doneSignal;
    int nextSlot;                        // Next result slot to claim (0 then 1), by task not by core
    Modbus::Client::Result core0Result;  // Result of the 1st task (slot 0)
    Modbus::Client::Result core1Result;  // Result of the 2nd task (slot 1)
};

// Task executed by each concurrent worker (may share a core on single-core targets)
auto concurrentTask = [](void* pv) {
    auto sd = static_cast<SyncData*>(pv);
    int core = xPortGetCoreID();
    Modbus::Logger::logf("[Core %d] Ready, waiting for start\n", core);

    // Wait for the start signal
    xSemaphoreTake(sd->startSignal, portMAX_DELAY);

    // Build the Modbus request
    Modbus::Frame req {
        .type         = Modbus::REQUEST,
        .fc           = Modbus::READ_HOLDING_REGISTERS,
        .slaveId      = TEST_SLAVE_ID,
        .regAddress   = READ_HOLDING_ADDR,
        .regCount     = 1,
        .data         = {},
        .exceptionCode= Modbus::NULL_EXCEPTION
    };
    Modbus::Frame resp;

    // In blocking mode for simplicity
    auto result = client.sendRequest(req, resp, nullptr); // Corrected: removed 'true', third arg is tracker*
    Modbus::Logger::logf("[Core %d] sendRequest -> %s\n", core,
            result == Modbus::Client::SUCCESS ? "SUCCESS" : 
            (result == Modbus::Client::ERR_BUSY ? "ERR_BUSY" : Modbus::Client::toString(result)) ); // More detailed error

    // Store the result in the next free slot (claimed atomically). Slots are per-task, not
    // per-core, so this stays correct when both tasks share core 0 on single-core targets.
    xSemaphoreTake(sd->resultsMutex, portMAX_DELAY);
    if (sd->nextSlot++ == 0) sd->core0Result = result;
    else                     sd->core1Result = result;
    xSemaphoreGive(sd->resultsMutex);

    // Signal the end
    xSemaphoreGive(sd->doneSignal);
    vTaskDelete(NULL);
};

void test_concurrent_calls() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CONCURRENT_CALLS: STRICT THREAD SAFETY WITH 2 SYNCHRONIZED CALLS");

    static SyncData sync = {};
    sync.startSignal   = xSemaphoreCreateCounting(2, 0);
    sync.resultsMutex  = xSemaphoreCreateMutex();
    sync.doneSignal    = xSemaphoreCreateCounting(2, 0);
    sync.nextSlot      = 0;
    sync.core0Result   = Modbus::Client::ERR_BUSY;
    sync.core1Result   = Modbus::Client::ERR_BUSY;

    // Launch the first task on core 0
    xTaskCreatePinnedToCore(
        [](void* p){ concurrentTask(p); },
        "Task0", 8192, &sync, 5, nullptr, 0
    );
    // … and the second on core 1 (or core 0 on single-core targets like the ESP32-C6)
    xTaskCreatePinnedToCore(
        [](void* p){ concurrentTask(p); },
        "Task1", 8192, &sync, 5, nullptr, TEST_SECOND_CORE
    );

    // Small delay to ensure the tasks are ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Launch the two tasks simultaneously
    xSemaphoreGive(sync.startSignal);
    xSemaphoreGive(sync.startSignal);

    // Wait for the two tasks to finish
    xSemaphoreTake(sync.doneSignal, portMAX_DELAY);
    xSemaphoreTake(sync.doneSignal, portMAX_DELAY);

    // Check the results
    Modbus::Logger::logln("=== TEST_CONCURRENT_CALLS Results ===");
    Modbus::Logger::logf("Task 0: %s\n", sync.core0Result == Modbus::Client::SUCCESS ? "SUCCESS" : "ERR_BUSY");
    Modbus::Logger::logf("Task 1: %s\n", sync.core1Result == Modbus::Client::SUCCESS ? "SUCCESS" : "ERR_BUSY");

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE(
        (sync.core0Result == Modbus::Client::SUCCESS && sync.core1Result == Modbus::Client::ERR_BUSY) ||
        (sync.core0Result == Modbus::Client::ERR_BUSY  && sync.core1Result == Modbus::Client::SUCCESS)
    );

    // Cleanup
    vSemaphoreDelete(sync.startSignal);
    vSemaphoreDelete(sync.resultsMutex);
    vSemaphoreDelete(sync.doneSignal);

    vTaskDelay(pdMS_TO_TICKS(50));
}


// -----------------------------------------------------------------------------
// CALLBACK PATH TESTS (SUCCESS & TIMEOUT)
// -----------------------------------------------------------------------------

struct CbData {
    SemaphoreHandle_t sem;
    Modbus::Client::Result res;
    uint16_t value;
};

static void cbStoreResult(Modbus::Client::Result res, const Modbus::Frame* resp, void* ctx) {
    if (ctx) {
        auto* d = static_cast<CbData*>(ctx);
        d->res = res;
        if (res == Modbus::Client::SUCCESS) {
            // For registers, take first value (addr 0)
            if (resp && resp->regCount > 0) d->value = resp->data[0];
        }
        xSemaphoreGive(d->sem);
    }
}

static SemaphoreHandle_t gNoCtxSem = nullptr;
static Modbus::Client::Result gNoCtxRes = Modbus::Client::NODATA;
static void cbNoCtx(Modbus::Client::Result res, const Modbus::Frame* resp, void* ctx) {
    gNoCtxRes = res;
    if (gNoCtxSem) xSemaphoreGive(gNoCtxSem);
}

void test_callback_success() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CALLBACK_SUCCESS: ASYNCHRONOUS CALLBACK SUCCESS");
    // ---------- With userCtx ----------
    CbData data{ xSemaphoreCreateBinary(), Modbus::Client::NODATA, 0 };

    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .data = {},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };

    auto res = client.sendRequest(req, cbStoreResult, &data);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);

    // Wait for callback (<= timeout + margin)
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(data.sem, pdMS_TO_TICKS(Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 200)));
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, data.res);

    // ---------- Without userCtx ----------
    gNoCtxSem = xSemaphoreCreateBinary();
    gNoCtxRes = Modbus::Client::NODATA;

    res = client.sendRequest(req, cbNoCtx, nullptr);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(gNoCtxSem, pdMS_TO_TICKS(Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 200)));
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, gNoCtxRes);

    vSemaphoreDelete(data.sem);
    vSemaphoreDelete(gNoCtxSem);
}

void test_callback_timeout() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CALLBACK_TIMEOUT: ASYNCHRONOUS CALLBACK TIMEOUT");
    // Suspend server RX task to force timeout
    TaskHandle_t serverRxTask = mbt.getRxTxTaskHandle();
    vTaskSuspend(serverRxTask);
    vTaskDelay(pdMS_TO_TICKS(50));

    // ---------- With userCtx ----------
    CbData data{ xSemaphoreCreateBinary(), Modbus::Client::NODATA, 0 };
    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .data = {},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    auto res = client.sendRequest(req, cbStoreResult, &data);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(data.sem, pdMS_TO_TICKS(Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 500)));
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TIMEOUT, data.res);

    // ---------- Without userCtx ----------
    gNoCtxSem = xSemaphoreCreateBinary();
    gNoCtxRes = Modbus::Client::NODATA;
    res = client.sendRequest(req, cbNoCtx);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(gNoCtxSem, pdMS_TO_TICKS(Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 500)));
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TIMEOUT, gNoCtxRes);

    vSemaphoreDelete(data.sem);
    vSemaphoreDelete(gNoCtxSem);

    // Resume server task
    mbt_uart.flush_input();
    vTaskResume(serverRxTask);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ===================================================================================
// RX ROBUSTNESS TESTS
// ===================================================================================

// Regression: a UART data event whose bytes were no longer in the driver (0-byte read) still grew
// the RX buffer: the stale bytes were counted as received and corrupted the next frame. The server
// task handles that event on an empty driver buffer, then is held before its frame-gap backstop
// (next tick) can discard the stale bytes, so that the request's event is handled right after.
void test_rx_zero_byte_read_keeps_buffer() {
    TaskHandle_t serverRxTask = mbt.getRxTxTaskHandle();
    QueueHandle_t serverEvents = mbt_uart.getRegisteredEventQueue();
    uart_event_t ghost = {};
    ghost.type = UART_DATA;
    ghost.size = 5;
    vTaskDelay(1); // Start of a tick
    bool injected = xQueueSend(serverEvents, &ghost, 0) == pdTRUE;
    while (uxQueueMessagesWaiting(serverEvents) > 0) { }
    delayMicroseconds(100); // Ghost event handled, server task back waiting on its queue
    vTaskSuspend(serverRxTask);

    serverHoldingRegisters[100] = 0x1234;
    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 100,
        .regCount = 1,
        .data = {},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    auto sendRes = client.sendRequest(req, resp, &tracker);
    vTaskDelay(pdMS_TO_TICKS(20)); // Request on the wire, queued behind the ghost event
    vTaskResume(serverRxTask);
    uint32_t t0 = Modbus::TIME_MS();
    while (tracker == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 500) {
        vTaskDelay(1);
    }

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(injected, "ghost event not injected");
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, sendRes);
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, tracker, "request after a 0-byte read not answered");
    TEST_ASSERT_EQUAL_UINT16(0x1234, resp.data[0]);
}

// ===================================================================================
// RUNTIME UART RECONFIGURATION TESTS
// ===================================================================================

void test_runtime_uart_reconfig() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_RUNTIME_UART_RECONFIG: CHANGE BAUDRATE + SERIAL CONFIG AT RUNTIME");

    // Save original config
    const uint32_t origBaud = ezm_uart.getBaudrate();
    const uint32_t origConfig = ezm_uart.getConfig();

    // 1) Sanity check: read at current config -> should work
    {
        Modbus::Frame req = {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_HOLDING_REGISTERS,
            .slaveId = TEST_SLAVE_ID,
            .regAddress = READ_HOLDING_ADDR,
            .regCount = 1,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };
        Modbus::Frame resp;
        auto res = client.sendRequest(req, resp, nullptr);  // nullptr = synchronous
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, res, "Baseline read before reconfig should succeed");
        TEST_ASSERT_EQUAL_MESSAGE(MBT_INIT_HOLDING_REGISTER_VALUE(READ_HOLDING_ADDR), resp.getRegister(0),
            "Baseline read value mismatch");
    }

    // 2) Reconfigure BOTH sides to 19200 8E1
    const uint32_t newBaud = 19200;
    const uint32_t newConfig = ModbusHAL::UART::CONFIG_8E1;

    TEST_ASSERT_EQUAL(ESP_OK, ezm_uart.setBaudrate(newBaud));
    TEST_ASSERT_EQUAL(ESP_OK, ezm_uart.setConfig(newConfig));
    TEST_ASSERT_EQUAL(ESP_OK, mbt_uart.setBaudrate(newBaud));
    TEST_ASSERT_EQUAL(ESP_OK, mbt_uart.setConfig(newConfig));
    vTaskDelay(pdMS_TO_TICKS(50)); // Let hardware settle

    // Flush stale data from both sides
    ezm_uart.flush_input();
    mbt_uart.flush_input();
    vTaskDelay(pdMS_TO_TICKS(20));

    // 3) Read again -> should succeed at new config
    {
        Modbus::Frame req = {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_HOLDING_REGISTERS,
            .slaveId = TEST_SLAVE_ID,
            .regAddress = READ_HOLDING_ADDR,
            .regCount = 1,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };
        Modbus::Frame resp;
        auto res = client.sendRequest(req, resp, nullptr);
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, res, "Read after reconfig to 19200 8E1 should succeed");
        TEST_ASSERT_EQUAL_MESSAGE(MBT_INIT_HOLDING_REGISTER_VALUE(READ_HOLDING_ADDR), resp.getRegister(0),
            "Read value after reconfig mismatch");
    }

    // 4) Restore original config
    TEST_ASSERT_EQUAL(ESP_OK, ezm_uart.setBaudrate(origBaud));
    TEST_ASSERT_EQUAL(ESP_OK, ezm_uart.setConfig(origConfig));
    TEST_ASSERT_EQUAL(ESP_OK, mbt_uart.setBaudrate(origBaud));
    TEST_ASSERT_EQUAL(ESP_OK, mbt_uart.setConfig(origConfig));
    vTaskDelay(pdMS_TO_TICKS(50));
    ezm_uart.flush_input();
    mbt_uart.flush_input();
    vTaskDelay(pdMS_TO_TICKS(20));

    // 5) Verify restored config works
    {
        Modbus::Frame req = {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_HOLDING_REGISTERS,
            .slaveId = TEST_SLAVE_ID,
            .regAddress = READ_HOLDING_ADDR,
            .regCount = 1,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };
        Modbus::Frame resp;
        auto res = client.sendRequest(req, resp, nullptr);
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, res, "Read after restoring original config should succeed");
        TEST_ASSERT_EQUAL_MESSAGE(MBT_INIT_HOLDING_REGISTER_VALUE(READ_HOLDING_ADDR), resp.getRegister(0),
            "Read value after restore mismatch");
    }
}

// Regression: the interface destructor slept 150 ms instead of waiting for its task,
// which can be busy for much longer (here transmitting a ~255-byte frame, ~270 ms at
// 9600 baud), then deleted its queue set while the UART event queue was not empty (the
// wake-up event it had just posted was never consumed): FreeRTOS refuses to remove a
// non-empty queue from a set, so the UART queue kept a reference to the freed set and
// no interface could be attached to that UART anymore.
// The request targets an absent slave, so that no RX traffic follows.
// Destroys & recreates the shared `ezm` interface (end of the suite).
void test_destroy_while_transmitting() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_DESTROY_WHILE_TRANSMITTING");

    // Slow line so that the transmission lasts ~270 ms (the suite runs at a high baud rate)
    const uint32_t origBaud = ezm_uart.getBaudrate();
    TEST_ASSERT_EQUAL(ESP_OK, ezm_uart.setBaudrate(9600));

    vTaskDelay(pdMS_TO_TICKS(50)); // Let the idle task clean up tasks deleted by previous tests
    const UBaseType_t baseline = uxTaskGetNumberOfTasks();
    client.~Client(); // The application goes first (rebuilt at the end)

    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_MULTIPLE_REGISTERS,
        .slaveId = 99, // No server answers this ID
        .regAddress = 0,
        .regCount = Modbus::MAX_REGISTERS_WRITE,
        .data = {},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    const auto sendRes = ezm.sendFrame(req, nullptr, nullptr);
    vTaskDelay(pdMS_TO_TICKS(20)); // The interface task is now transmitting

    ezm.~RTU();
    const UBaseType_t afterDestroy = uxTaskGetNumberOfTasks();
    vTaskDelay(pdMS_TO_TICKS(500)); // Before the fix, let the still-running task end before reusing the object

    ezm_uart.setBaudrate(origBaud);
    new (&ezm) ModbusInterface::RTU(ezm_uart, Modbus::MASTER);
    const auto beginRes = ezm.begin();
    new (&client) Modbus::Client(ezm);
    client.begin();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(ModbusInterface::IInterface::SUCCESS, sendRes, "cannot start the transmission");
    TEST_ASSERT_EQUAL_MESSAGE(baseline - 1, afterDestroy, "interface task still running after destruction");
    TEST_ASSERT_EQUAL_MESSAGE(ModbusInterface::IInterface::SUCCESS, beginRes, "cannot attach a new interface to the UART");
    TEST_ASSERT_EQUAL_MESSAGE(baseline, uxTaskGetNumberOfTasks(), "unexpected task count after re-init");

    Modbus::Logger::logln("TEST_DESTROY_WHILE_TRANSMITTING - OK");
    Modbus::Logger::logln();
}

// Interface destruction/re-creation while the UART keeps receiving: the UART driver posts
// events continuously. The interface waits on the UART event queue alone (no QueueSet), so
// there is nothing to detach: every re-creation must succeed.
static constexpr int RECREATE_STRESS_CYCLES = 20;
static volatile bool rxStressRunning = false;
static StaticTask_t rxStressTaskTcb;
static StackType_t rxStressTaskStack[2048];

// Sends garbage bytes to the EZModbus master's RX every 2 ms (from the server side UART)
static void rxStressSenderTask(void*) {
    const uint8_t garbage[8] = { 0x55, 0xAA, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
    while (rxStressRunning) {
        mbt_uart.write(garbage, sizeof(garbage));
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    rxStressRunning = true; // Ack: the task no longer uses the UART
    vTaskSuspend(nullptr);
}

// Last test of the suite: it destroys & recreates the shared `ezm` interface.
void test_recreate_under_rx_traffic() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_RECREATE_UNDER_RX_TRAFFIC");

    vTaskDelay(pdMS_TO_TICKS(50)); // Let the idle task clean up tasks deleted by previous tests
    const UBaseType_t baseline = uxTaskGetNumberOfTasks();

    rxStressRunning = true;
    TaskHandle_t sender = xTaskCreateStatic(rxStressSenderTask, "rxStress", sizeof(rxStressTaskStack) / sizeof(StackType_t),
                                            nullptr, tskIDLE_PRIORITY + 1, rxStressTaskStack, &rxStressTaskTcb);
    TEST_ASSERT_NOT_NULL(sender);
    client.~Client(); // The application goes first (rebuilt at the end)

    int failedCycle = -1;
    for (int i = 0; i < RECREATE_STRESS_CYCLES && failedCycle < 0; i++) {
        ezm.~RTU();
        vTaskDelay(pdMS_TO_TICKS(5)); // UART events keep coming meanwhile
        new (&ezm) ModbusInterface::RTU(ezm_uart, Modbus::MASTER);
        if (ezm.begin() != ModbusInterface::IInterface::SUCCESS) failedCycle = i;
        vTaskDelay(pdMS_TO_TICKS(10 + (i % 7) * 3));
    }

    // Stop the sender (it acks by setting the flag back)
    rxStressRunning = false;
    uint32_t t0 = millis();
    while (!rxStressRunning && millis() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(5));
    vTaskDelete(sender);
    new (&client) Modbus::Client(ezm);
    client.begin();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(-1, failedCycle, "interface begin() failed after a re-creation under RX traffic");
    TEST_ASSERT_EQUAL_MESSAGE(baseline, uxTaskGetNumberOfTasks(), "unexpected task count after the stress test");

    Modbus::Logger::logln("TEST_RECREATE_UNDER_RX_TRAFFIC - OK");
    Modbus::Logger::logln();
}

// User code may define its own global names matching EZModbus internal helpers (here a
// mutex, a buffer, a time function...): EZModbus keeps them out of the global namespace, so
// that including it never collides with user code (this test fails to compile otherwise).
class Mutex { public: int id = 1; };
struct Lock { int id = 2; };
struct ByteBuffer { int id = 3; };
class BinarySemaphore { public: int id = 4; };
inline uint32_t TIME_MS() { return 42; }
inline uint64_t TIME_US() { return 43; }
inline void WAIT_MS(uint32_t) {}
inline void WAIT_US(uint32_t) {}

void test_user_names_dont_collide() {
    Mutex mutex; Lock lock; ByteBuffer buffer; BinarySemaphore semaphore;
    WAIT_MS(0); WAIT_US(0);
    TEST_ASSERT_EQUAL(1, mutex.id);
    TEST_ASSERT_EQUAL(2, lock.id);
    TEST_ASSERT_EQUAL(3, buffer.id);
    TEST_ASSERT_EQUAL(4, semaphore.id);
    TEST_ASSERT_EQUAL(42, TIME_MS());
    TEST_ASSERT_EQUAL(43, (uint32_t)TIME_US());
}

void setup() {
    // Debug port
    Serial.setTxBufferSize(2048);
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);

    // Initialize UART HAL for client
    Modbus::Logger::logln("[setup] Initializing client UART HAL...");
    esp_err_t uart_init_res = ezm_uart.begin();
    if (uart_init_res != ESP_OK) {
        Modbus::Logger::logln("[setup] Client UART HAL initialization failed");
        return;
    }
    Modbus::Logger::logln("[setup] Client UART HAL initialized");

    // Initialize UART HAL for server
    Modbus::Logger::logln("[setup] Initializing server UART HAL...");
    uart_init_res = mbt_uart.begin();
    if (uart_init_res != ESP_OK) {
        Modbus::Logger::logln("[setup] Server UART HAL initialization failed");
        return;
    }
    Modbus::Logger::logln("[setup] Server UART HAL initialized");

    // EZModbus init
    auto ifcInitRes = ezm.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[setup] EZModbus RTU interface initialization failed");
        return;
    }
    Modbus::Logger::logln("[setup] EZModbus RTU interface initialized");
    auto clientInitRes = client.begin();
    if (clientInitRes != Modbus::Client::SUCCESS) {
        Modbus::Logger::logln("[setup] EZModbus Client initialization failed");
        return;
    }
    Modbus::Logger::logln("[setup] EZModbus Client initialized");

    // Run Modbus test server task on core 0 (main test thread on core 1)
    xTaskCreatePinnedToCore(ModbusTestServerTask, "ModbusTestServerTask", 16384, NULL, 5, &modbusTestServerTaskHandle, 0);
    while (!modbusTestServerTaskInitialized) {
        vTaskDelay(pdMS_TO_TICKS(1));
    } // Wait for the task to be looping
    Modbus::Logger::logln("[setup] ModbusTestServer task initialized, starting tests...");

    // Run tests
    UNITY_BEGIN();
    RUN_TEST(test_user_names_dont_collide);

    // Register all generated tests
    #define X(Name, ReadSingle, ReadMulti, Addr, Expect, FC) \
        RUN_TEST(test_read_##Name##_sync); \
        RUN_TEST(test_read_##Name##_async); \
        RUN_TEST(test_read_multiple_##Name); \
        RUN_TEST(test_read_multiple_##Name##_async); \
        RUN_TEST(test_read_max_##Name);
    READ_TESTS
    #undef X
    
    // Register all generated tests
    #define X(Name, WriteSingle, WriteMulti, Addr, TestValue, SingleFC, MultiFC) \
        RUN_TEST(test_write_##Name##_sync); \
        RUN_TEST(test_write_##Name##_async); \
        RUN_TEST(test_write_multiple_##Name); \
        RUN_TEST(test_write_multiple_##Name##_async); \
        RUN_TEST(test_write_max_##Name);
    WRITE_TESTS
    #undef X
    
    RUN_TEST(test_request_timeout);
    RUN_TEST(test_modbus_exceptions);
    RUN_TEST(test_invalid_parameters);
    RUN_TEST(test_broadcast_read_rejected);
    RUN_TEST(test_broadcast);
    RUN_TEST(test_callback_success);
    RUN_TEST(test_callback_timeout);
    // RUN_TEST(test_concurrent_calls);

    // Helper template tests
    RUN_TEST(test_helper_read_uint16_holding_registers);
    RUN_TEST(test_helper_read_uint16_coils);
    RUN_TEST(test_helper_write_uint16_holding_registers);
    RUN_TEST(test_helper_write_uint16_coils);
    RUN_TEST(test_helper_read_uint8_clamp);
    RUN_TEST(test_helper_read_int8_clamp);
    RUN_TEST(test_helper_write_uint8);
    RUN_TEST(test_helper_write_int8_negative_clamp);
    RUN_TEST(test_helper_write_uint32_clamp);
    RUN_TEST(test_helper_write_int32_clamp);
    RUN_TEST(test_helper_write_int32_negative_clamp);
    RUN_TEST(test_helper_read_bool_coils);
    RUN_TEST(test_helper_write_bool_coils);
    RUN_TEST(test_helper_write_bool_registers);
    RUN_TEST(test_helper_exception_illegal_address);
    RUN_TEST(test_helper_exception_vs_transport_error);
    RUN_TEST(test_helper_invalid_regtype);
    RUN_TEST(test_helper_null_buffer);
    RUN_TEST(test_helper_qty_zero);
    RUN_TEST(test_helper_write_readonly_regtype);

    // RX robustness tests
    RUN_TEST(test_rx_zero_byte_read_keeps_buffer);

    // Runtime reconfiguration tests
    RUN_TEST(test_runtime_uart_reconfig);

    // Lifecycle (last: destroys & recreates the shared interface)
    RUN_TEST(test_destroy_while_transmitting);
    RUN_TEST(test_recreate_under_rx_traffic);

    UNITY_END();
}

void loop() {
    // Reopen Serial port after Unity closes it
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.begin();
    
    // Nothing to do here
    Serial.println("Idling...");
    vTaskDelay(pdMS_TO_TICKS(1000));
}
