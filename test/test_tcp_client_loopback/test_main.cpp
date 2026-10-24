#include <Arduino.h>
#include <WiFi.h>
#include <unity.h>
#include <EZModbus.h>
#include <drivers/ModbusHAL_TCP.h>
#include "test_params.h"
#include <utils/ModbusLogger.hpp>


// Give some time for the application logs to be printed before asserting
#ifdef EZMODBUS_DEBUG
    #define TEST_ASSERT_START() { Modbus::Logger::waitQueueFlushed(); }
#else
    #define TEST_ASSERT_START() { vTaskDelay(pdMS_TO_TICKS(50)); }
#endif

// WiFi configuration
// Configuration WiFi
static constexpr char     WIFI_SSID[]   = "ESP32-LOOP";
static constexpr char     WIFI_PASS[]   = "loopesp32";
// SoftAP default IP
static IPAddress SOFTAP_IP    = IPAddress(192, 168, 4, 1);
static constexpr uint16_t  MODBUS_PORT  = 502;
static constexpr char LOOPBACK_IP_STR[] = "127.0.0.1";
static IPAddress LOOPBACK_IP = IPAddress(127, 0, 0, 1);

// Store the target IP for the client as a persistent String
static String clientTargetIpStr(LOOPBACK_IP_STR); // Will be initialized in setup() before ezm is constructed if ezm was local
                                // Since ezm is global, clientTargetIpStr must be initialized before ezm instance.
                                // Best approach is to initialize it at its declaration or ensure ezm is initialized after setup.
                                // For now, will initialize globally and rely on order of static initialization for safety, or make ezm local to setup.
                                // Let's try direct initialization here for simplicity for now:
// static String clientTargetIpStr = LOOPBACK_IP.toString(); // This might have static init order issues with LOOPBACK_IP

// RTU pin / port / baud definitions for the multi-interface tests - overridable per-environment
// via build flags. Defaults target the Seeed XIAO ESP32-S3 wiring; the *_c6 env puts the RTU
// client side on the LP-UART (see platformio.ini).
#ifndef EZM_UART_NUM
    #define EZM_UART_NUM UART_NUM_1   // Multi-interface RTU server port
#endif
#ifndef EZM_RX
    #define EZM_RX D5
#endif
#ifndef EZM_TX
    #define EZM_TX D6
#endif
#ifndef MBT_UART_NUM
    #define MBT_UART_NUM UART_NUM_2   // Multi-interface RTU client port (can be LP_UART_NUM_0)
#endif
#ifndef MBT_RX
    #define MBT_RX D7
#endif
#ifndef MBT_TX
    #define MBT_TX D8
#endif
#ifndef UART_BAUD_RATE
    #define UART_BAUD_RATE 9600
#endif

// HAL instances
ModbusHAL::TCP halForClient(LOOPBACK_IP_STR, MODBUS_PORT);
ModbusHAL::TCP halForServer(MODBUS_PORT);

// Isolated SERVER-only fixture for the safety-timeout idle-tick regression:
// a SERVER interface with a silent callback (no Modbus::Server on top), so
// the transaction can only be cleared by the rxTxTask itself. The client
// side is a plain POSIX socket (see test body) — we don't use a ModbusHAL
// CLIENT here because it would tight-loop its rx-event queue when responses
// are never consumed, and blocking the HAL task deadlocks the test.
//
// Port 510 is above the 502..504 range already used by the main fixture and
// the multi-interface tests (MODBUS_PORT + 1 / +2).
static constexpr uint16_t SAFETY_TEST_PORT = 510;
ModbusHAL::TCP halForSafetyServer(SAFETY_TEST_PORT);
ModbusInterface::TCP safetyServerIf(halForSafetyServer, Modbus::SERVER);
static std::atomic<uint32_t> safetyServerCbCount{0};

// Primary EZModbus TCP client
ModbusInterface::TCP ezm(halForClient, Modbus::CLIENT);
Modbus::Client client(ezm);

// EZModbus TCP server for testing with DynamicWordStore (heap allocation)
ModbusInterface::TCP mbt(halForServer, Modbus::SERVER);
// Coils + discrete inputs over the full range, holding + input registers over the small
// analog range (see MBT_ANALOG_COUNT). Sized exactly so the store stays off the internal
// heap budget the Wi-Fi stack needs on PSRAM-less chips.
Modbus::DynamicWordStore dynamicStore(2 * MBT_INIT_REG_COUNT + 2 * MBT_ANALOG_COUNT);
Modbus::Server server(mbt, dynamicStore);

uint16_t serverDiscreteInputs[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverCoils[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverHoldingRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverInputRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];

// Slow mode flag to artificially lengthen server processing during tests
volatile bool slowMode = false;

// Callbacks
Modbus::ReadWordHandler serverReadCallback = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
    if (slowMode) {
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms delay to keep transaction active
    }
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
    // Start HAL server (listening) if not already started
    halForServer.begin();
    // Initialize Modbus TCP test server interface and Modbus::Server application
    auto ifcInitRes = mbt.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[ModbusTestServerTask] EZModbus TCP interface initialization failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    Modbus::Logger::logln("[ModbusTestServerTask] EZModbus TCP interface initialized");
    
    // Configure registers for each type
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
    
    for (int i = MBT_INIT_START_REG; i < MBT_INIT_START_REG + MBT_INIT_REG_COUNT; i++) {
        for (uint8_t rt : regTypes) {
            // Holding/input registers only span the small analog range (see MBT_ANALOG_COUNT).
            bool isAnalog = (rt == Modbus::HOLDING_REGISTER || rt == Modbus::INPUT_REGISTER);
            if (isAnalog && i >= MBT_INIT_START_REG + MBT_ANALOG_COUNT) continue;
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
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
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
    // Reset ModbusTestServer registers
    resetModbusTestServerRegisters();
}

void tearDown() {
    slowMode = false; // Disable slow mode if enabled

    // Re-enable disabled client/server tasks if disabled
    // (avoids timeouting next tests if we fail during
    // a test that disables them)
    TaskHandle_t ht;
    ht = mbt.getRxTxTaskHandle();
    if (ht && eTaskGetState(ht) == eSuspended) vTaskResume(ht);
    ht = ezm.getRxTxTaskHandle();
    if (ht && eTaskGetState(ht) == eSuspended) vTaskResume(ht);

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

void test_timeout_client_interface() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_TIMEOUT_CLIENT_INTERFACE - CASE 1: TIMEOUT ON READ REQUEST");
    
    // Temporarily disable the Modbus client interface task.
    // The goal is to make the client interface unresponsive.
    TaskHandle_t rxTxTaskHandle = ezm.getRxTxTaskHandle(); // Get the TCP poll task handle
    vTaskSuspend(rxTxTaskHandle);
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
    vTaskResume(rxTxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow client's RX task to run
    
    Modbus::Logger::logln("TEST_TIMEOUT_CLIENT_INTERFACE - CASE 2: TIMEOUT ON WRITE REQUEST (SYNC)");
    
    // Temporarily disable the client's RX task again
    vTaskSuspend(rxTxTaskHandle);
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

    // Reactivate the Modbus test server
    vTaskResume(rxTxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));

    Modbus::Logger::logln("TEST_TIMEOUT_CLIENT_INTERFACE - CASE 2: TIMEOUT ON WRITE REQUEST (ASYNC)");

    // Temporarily disable the client's RX task again
    vTaskSuspend(rxTxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));

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
    vTaskResume(rxTxTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wait for the server to be ready if any pending operations due to client being unresponsive
    // This delay might be important if the server tried to send something the client missed.
    vTaskDelay(pdMS_TO_TICKS(Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 200));
}

void test_timeout_server() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_TIMEOUT_SERVER - Server receives but doesn't respond");
    
    // Enable slow mode on the server
    slowMode = true;

    // Temporarily suspend the server's TX processing to simulate no response
    // The server will receive the request but won't be able to send response
    TaskHandle_t serverTaskHandle = mbt.getRxTxTaskHandle();
    
    // Send a read request
    Modbus::Frame readRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame readResponse;
    
    // Use async mode to track timeout
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    auto result = client.sendRequest(readRequest, readResponse, &tracker);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result); // Request sent successfully
    
    // Let the request reach the server, then suspend its task before it sends its response
    vTaskDelay(pdMS_TO_TICKS(10));
    vTaskSuspend(serverTaskHandle);
    
    // Wait for client timeout
    uint32_t startTime = Modbus::TIME_MS();
    uint32_t timeoutDuration = Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 200; // Add margin
    while (Modbus::TIME_MS() - startTime < timeoutDuration) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (tracker != Modbus::Client::NODATA) break;
    }
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, tracker, 
        "Client should timeout when server doesn't respond");
    
    // Resume server
    vTaskResume(serverTaskHandle);
    slowMode = false;
    vTaskDelay(pdMS_TO_TICKS(100)); // Let server recover
    
    // Verify transaction was properly cleaned up via abortCurrentTransaction
    // Send another request to verify we're not stuck
    Modbus::Frame testRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame testResponse;
    
    result = client.sendRequest(testRequest, testResponse, nullptr); // Sync mode
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, result, 
        "Client should be able to send new requests after timeout cleanup");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::NULL_EXCEPTION, testResponse.exceptionCode,
        "Response should be valid after recovery");
    
    Modbus::Logger::logln("TEST_TIMEOUT_SERVER - Transaction cleanup verified");
    Modbus::Logger::logln();
}

// Regression: when a TCP SERVER holds an in-flight transaction and NO external
// signal ever reaches the interface (no response generated upstream, no
// abortCurrentTransaction() from a Client), the rxTxTask MUST still fire the
// safety-timeout on its own after TCP_TRANSACTION_SAFETY_TIMEOUT_MS.
//
// Before the fix: `continue` on `member == nullptr` in rxTxTask skipped the
// transaction-timeout check → state stuck forever, only cleared by a new
// event. Exposed by bridge-style topologies where no `Modbus::Client` sits
// above the interface to propagate a timeout cleanup signal.
//
// Probe: count how many times the silent receive-callback fires on an
// isolated SERVER-only pair (port 510). A second request sent while the
// first is still in-flight must be rejected as SLAVE_DEVICE_BUSY (no
// callback). A third request sent AFTER the safety-timeout elapsed must
// go through, proving the transaction was cleared idle.
void test_server_safety_timeout_clears_idle() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_SAFETY_TIMEOUT_CLEARS_IDLE - rxTxTask idle tick");

    // Reset the callback counter.
    safetyServerCbCount.store(0);

    // Open a plain POSIX TCP socket to the isolated server. We purposely
    // avoid ModbusHAL::TCP CLIENT here: that HAL tight-loops its rx-event
    // queue whenever responses sit unread on the socket (MSG_PEEK never
    // consumes), and the queue would deadlock the HAL task.
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "socket() failed");

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(SAFETY_TEST_PORT);
    inet_aton(LOOPBACK_IP_STR, &addr.sin_addr);
    int connRes = ::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    TEST_ASSERT_EQUAL_MESSAGE(0, connRes, "connect() to isolated server failed");

    auto sendReadHr = [sock](uint16_t tid) -> bool {
        uint8_t buf[12] = {
            static_cast<uint8_t>(tid >> 8), static_cast<uint8_t>(tid & 0xFF),
            0x00, 0x00,                     // protocol id
            0x00, 0x06,                     // length (unit + PDU = 6)
            0x01,                           // unit id
            0x03, 0x00, 0x00, 0x00, 0x01    // FC=0x03, addr=0, count=1
        };
        ssize_t n = ::send(sock, buf, sizeof(buf), 0);
        return n == static_cast<ssize_t>(sizeof(buf));
    };

    // Discard anything the server writes back — busy exceptions, late
    // responses, etc. We don't care about them, we only probe via the
    // server-side callback counter.
    auto drainSocket = [sock]() {
        uint8_t dump[64];
        while (::recv(sock, dump, sizeof(dump), MSG_DONTWAIT) > 0) { }
    };

    // 1) First request — transaction opens, callback fires.
    Modbus::Logger::logln("  [step 1/4] sending req1 -> expect cb=1");
    TEST_ASSERT_TRUE_MESSAGE(sendReadHr(1), "first raw send failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, safetyServerCbCount.load(),
        "first request must reach the silent callback");
    drainSocket();
    Modbus::Logger::logf("  [step 1/4] OK (cb=%u)\n", (unsigned)safetyServerCbCount.load());

    // 2) Second request immediately — server already in transaction →
    //    SLAVE_DEVICE_BUSY reply, callback NOT invoked.
    Modbus::Logger::logln("  [step 2/4] sending req2 (expect busy, cb stays at 1)");
    TEST_ASSERT_TRUE_MESSAGE(sendReadHr(2), "second raw send failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, safetyServerCbCount.load(),
        "second request must be rejected as busy without callback");
    drainSocket(); // discard the busy-exception response
    Modbus::Logger::logf("  [step 2/4] OK (cb=%u, server correctly rejected)\n",
        (unsigned)safetyServerCbCount.load());

    // 3) Sleep past the safety-timeout with zero activity. Before the fix
    //    the rxTxTask never ticks the timeout check on idle and the
    //    transaction remains pinned forever.
    const uint32_t safetyMs = ModbusInterface::TCP::TCP_TRANSACTION_SAFETY_TIMEOUT_MS;
    Modbus::Logger::logf("  [step 3/4] idling %u ms (safety=%u + 500) — no activity\n",
        (unsigned)(safetyMs + 500), (unsigned)safetyMs);
    vTaskDelay(pdMS_TO_TICKS(safetyMs + 500));
    drainSocket();
    Modbus::Logger::logln("  [step 3/4] wake up, checking if txn was cleared idle...");

    // 4) Third request — must go through since the transaction was
    //    cleared autonomously by the safety-timeout tick.
    TEST_ASSERT_TRUE_MESSAGE(sendReadHr(3), "third raw send failed");
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, safetyServerCbCount.load(),
        "idle safety-timeout failed to clear the stale transaction");
    drainSocket();
    Modbus::Logger::logf("  [step 4/4] OK (cb=%u, txn was cleared without any external signal)\n",
        (unsigned)safetyServerCbCount.load());

    ::close(sock);
    Modbus::Logger::logln("TEST_SERVER_SAFETY_TIMEOUT_CLEARS_IDLE - OK");
    Modbus::Logger::logln();
}

// ===================================================================================
// RAW TCP PEER HELPERS
// Plain POSIX sockets give the tests full control over the byte stream seen by
// EZModbus (timing, fragmentation, several ADUs in one segment).
// ===================================================================================

static int rawListen(uint16_t port) {
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    inet_aton(LOOPBACK_IP_STR, &addr.sin_addr);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(sock, 1) != 0) {
        ::close(sock);
        return -1;
    }
    return sock;
}

// 1 s receive timeout & no Nagle, so that each send() leaves as its own segment
static void rawSetupPeer(int sock) {
    timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

static int rawConnect(uint16_t port) {
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    inet_aton(LOOPBACK_IP_STR, &addr.sin_addr);
    if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(sock);
        return -1;
    }
    rawSetupPeer(sock);
    return sock;
}

// Returns the number of bytes received: len on success, less if closed by peer / timeout
static size_t rawRecvExact(int sock, uint8_t* buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = ::recv(sock, buf + got, len - got, 0);
        if (n <= 0) break;
        got += n;
    }
    return got;
}

static bool rawSendAll(int sock, const uint8_t* buf, size_t len) {
    return ::send(sock, buf, len, 0) == static_cast<ssize_t>(len);
}

static constexpr size_t RAW_READ_HR_REQ_SIZE  = 12;
static constexpr size_t RAW_READ_HR_RESP_SIZE = 11;

// FC03 request for 1 holding register
static void rawBuildReadHrRequest(uint8_t* buf, uint16_t tid, uint16_t addr, uint8_t unitId = TEST_SLAVE_ID) {
    const uint8_t frame[RAW_READ_HR_REQ_SIZE] = {
        static_cast<uint8_t>(tid >> 8), static_cast<uint8_t>(tid & 0xFF),
        0x00, 0x00,                     // protocol id
        0x00, 0x06,                     // length (unit + PDU = 6)
        unitId,                         // unit id
        0x03,                           // FC=0x03
        static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF),
        0x00, 0x01                      // count = 1
    };
    memcpy(buf, frame, sizeof(frame));
}

// FC03 response carrying 1 register value
static void rawBuildReadHrResponse(uint8_t* buf, uint16_t tid, uint16_t value) {
    const uint8_t frame[RAW_READ_HR_RESP_SIZE] = {
        static_cast<uint8_t>(tid >> 8), static_cast<uint8_t>(tid & 0xFF),
        0x00, 0x00,                     // protocol id
        0x00, 0x05,                     // length (unit + PDU = 5)
        TEST_SLAVE_ID,                  // unit id
        0x03, 0x02,                     // FC=0x03, byte count = 2
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)
    };
    memcpy(buf, frame, sizeof(frame));
}

static uint16_t rawTid(const uint8_t* frame) {
    return (static_cast<uint16_t>(frame[0]) << 8) | frame[1];
}

static uint16_t rawReadHrRespValue(const uint8_t* frame) {
    return (static_cast<uint16_t>(frame[9]) << 8) | frame[10];
}

static uint16_t localPort(int fd) {
    sockaddr_in addr;
    socklen_t len = sizeof(addr);
    return (getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0) ? ntohs(addr.sin_port) : 0;
}

// Socket number of the server side of a loopback connection (-1 if not accepted yet):
// the socket bound to serverPort whose peer is clientSock's local port
static int rawServerSideSocket(uint16_t serverPort, int clientSock) {
    const uint16_t clientPort = localPort(clientSock);
    if (clientPort == 0) return -1;
    for (int s = LWIP_SOCKET_OFFSET; s < LWIP_SOCKET_OFFSET + CONFIG_LWIP_MAX_SOCKETS; s++) {
        if (localPort(s) != serverPort) continue;
        sockaddr_in peer;
        socklen_t len = sizeof(peer);
        if (getpeername(s, reinterpret_cast<sockaddr*>(&peer), &len) != 0) continue; // Listening socket
        if (ntohs(peer.sin_port) == clientPort) return s;
    }
    return -1;
}

// ===================================================================================
// CLIENT: LATE RESPONSE SCENARIO
// Request A is received by the raw server but left unanswered until the client
// times out. Request B is then sent on the same connection, and the server
// answers A (late) then B, either in two segments or coalesced in one.
// B must complete with its own value.
// ===================================================================================

// The client tests against a raw server share one EZModbus client stack (RAM footprint).
// Each test leaves it disconnected: its next request reconnects to the test's raw server.
static constexpr uint16_t RAW_SERVER_TEST_PORT = 511;
static constexpr uint32_t RAW_SERVER_CLIENT_TIMEOUT_MS = 300;
ModbusHAL::TCP rawServerClientHal(LOOPBACK_IP_STR, RAW_SERVER_TEST_PORT);
ModbusInterface::TCP rawServerClientIfc(rawServerClientHal, Modbus::CLIENT);
Modbus::Client rawServerClient(rawServerClientIfc, RAW_SERVER_CLIENT_TIMEOUT_MS);

static void beginRawServerClient() {
    rawServerClientHal.begin(); // No-op (returns false) once running
    rawServerClientIfc.begin();
    rawServerClient.begin();
}

// Close the raw server side & wait for the client HAL to notice it (disconnected)
static void closeRawServer(int peer, int listenSock) {
    if (peer >= 0) ::close(peer);
    if (listenSock >= 0) ::close(listenSock);
    uint32_t t0 = Modbus::TIME_MS();
    while (rawServerClientHal.isClientConnected() && Modbus::TIME_MS() - t0 < 2000) vTaskDelay(pdMS_TO_TICKS(5));
}

static void runLateResponseScenario(bool coalesced) {
    ModbusInterface::TCP& ifc = rawServerClientIfc;
    Modbus::Client& cli = rawServerClient;
    int listenSock = rawListen(RAW_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(listenSock >= 0, "raw server listen failed");
    beginRawServerClient();

    auto waitTracker = [](volatile Modbus::Client::Result& tracker, uint32_t maxMs) {
        uint32_t t0 = Modbus::TIME_MS();
        while (tracker == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < maxMs) vTaskDelay(pdMS_TO_TICKS(5));
    };

    constexpr uint16_t VALUE_A = 0xAAAA;
    constexpr uint16_t VALUE_B = 0xBBBB;
    uint8_t reqBytes[RAW_READ_HR_REQ_SIZE];
    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };

    // 1) Request A: received by the server but left unanswered -> client timeout
    // (it (re)connects the client to the raw server)
    Modbus::Frame respA;
    Modbus::Client::Result trackerA = Modbus::Client::NODATA;
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, cli.sendRequest(req, respA, &trackerA));
    int peer = ::accept(listenSock, nullptr, nullptr);
    TEST_ASSERT_TRUE_MESSAGE(peer >= 0, "accept() failed");
    rawSetupPeer(peer);
    TEST_ASSERT_EQUAL_MESSAGE(RAW_READ_HR_REQ_SIZE, rawRecvExact(peer, reqBytes, sizeof(reqBytes)), "request A not received");
    uint16_t tidA = rawTid(reqBytes);
    waitTracker(trackerA, RAW_SERVER_CLIENT_TIMEOUT_MS + 200);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, trackerA, "request A should time out");

    // Let the interface process the abort signal before sending B
    uint32_t t0 = Modbus::TIME_MS();
    while (!ifc.isReady() && Modbus::TIME_MS() - t0 < 200) vTaskDelay(pdMS_TO_TICKS(1));

    // 2) Request B on the same connection
    req.regAddress = 1;
    Modbus::Frame respB;
    Modbus::Client::Result trackerB = Modbus::Client::NODATA;
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, cli.sendRequest(req, respB, &trackerB));
    TEST_ASSERT_EQUAL_MESSAGE(RAW_READ_HR_REQ_SIZE, rawRecvExact(peer, reqBytes, sizeof(reqBytes)), "request B not received");
    uint16_t tidB = rawTid(reqBytes);
    Modbus::Logger::logf("  TID A = %u, TID B = %u\n", (unsigned)tidA, (unsigned)tidB);

    // 3) Late response to A first, then response to B
    uint8_t respBytes[2 * RAW_READ_HR_RESP_SIZE];
    rawBuildReadHrResponse(respBytes, tidA, VALUE_A);
    rawBuildReadHrResponse(respBytes + RAW_READ_HR_RESP_SIZE, tidB, VALUE_B);
    if (coalesced) {
        TEST_ASSERT_TRUE_MESSAGE(rawSendAll(peer, respBytes, sizeof(respBytes)), "send coalesced responses failed");
    } else {
        TEST_ASSERT_TRUE_MESSAGE(rawSendAll(peer, respBytes, RAW_READ_HR_RESP_SIZE), "send late response A failed");
        vTaskDelay(pdMS_TO_TICKS(20));
        TEST_ASSERT_TRUE_MESSAGE(rawSendAll(peer, respBytes + RAW_READ_HR_RESP_SIZE, RAW_READ_HR_RESP_SIZE), "send response B failed");
    }

    waitTracker(trackerB, RAW_SERVER_CLIENT_TIMEOUT_MS + 200);
    closeRawServer(peer, listenSock);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, trackerB, "request B should succeed");
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(VALUE_B, respB.data[0],
        "request B completed with the late response to A");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(tidA, tidB, "consecutive requests must use different TIDs");
}

// Regression: the client TID counter used to live in the current transaction
// context, which is cleared when a transaction ends. Every request then went
// out with TID = 1, so a late response to a timed-out request A was accepted
// as the response to the next request B on the same connection.
void test_late_response_not_matched_to_next_request() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_LATE_RESPONSE_NOT_MATCHED_TO_NEXT_REQUEST");
    runLateResponseScenario(false);
    Modbus::Logger::logln("TEST_LATE_RESPONSE_NOT_MATCHED_TO_NEXT_REQUEST - OK");
    Modbus::Logger::logln();
}

// Regression: when the late response to A and the response to B arrived in the
// same TCP segment, the client kept the first ADU of the recv() buffer and
// silently dropped the second one, so B timed out.
void test_client_rx_coalesced_responses() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CLIENT_RX_COALESCED_RESPONSES");
    runLateResponseScenario(true);
    Modbus::Logger::logln("TEST_CLIENT_RX_COALESCED_RESPONSES - OK");
    Modbus::Logger::logln();
}

// Regression: when the client was disconnected, the request was handed to the HAL
// which reconnected (up to 5 s) and then sent it, even if the Client had already
// reported ERR_TIMEOUT to the application meanwhile: a write could reach the
// device after the application was told it had failed.
// A slow connect is obtained by filling the raw server's listen backlog: lwIP
// then ignores the SYN, and the connection completes on its retransmission (~3 s).

void test_client_no_late_write_after_reconnect() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CLIENT_NO_LATE_WRITE_AFTER_RECONNECT");

    // Shared client, left disconnected by the previous test (nothing listens yet)
    beginRawServerClient();
    TEST_ASSERT_FALSE_MESSAGE(rawServerClientHal.isClientConnected(), "client should start disconnected");

    // Raw server whose backlog (1) is filled by a connection we don't accept yet
    int listenSock = rawListen(RAW_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(listenSock >= 0, "raw server listen failed");
    int filler = rawConnect(RAW_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(filler >= 0, "filler connect failed");

    // Write request: the client reconnects first, which now takes ~3 s
    Modbus::Frame writeReq = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .data = Modbus::packRegisters({0x1234}),
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame writeResp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, rawServerClient.sendRequest(writeReq, writeResp, &tracker));
    uint32_t t0 = Modbus::TIME_MS();
    while (tracker == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < RAW_SERVER_CLIENT_TIMEOUT_MS + 200) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, tracker, "write should time out while reconnecting");

    // Free the backlog: the client's SYN retransmission is now accepted
    int fillerPeer = ::accept(listenSock, nullptr, nullptr);
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(listenSock, &rfds);
    timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    TEST_ASSERT_EQUAL_MESSAGE(1, ::select(listenSock + 1, &rfds, nullptr, nullptr, &tv), "client did not reconnect");
    int peer = ::accept(listenSock, nullptr, nullptr);
    TEST_ASSERT_TRUE_MESSAGE(peer >= 0, "accept() failed");
    rawSetupPeer(peer);

    // Nothing must be sent on the new connection: the request was already reported as timed out
    uint8_t buf[RAW_READ_HR_REQ_SIZE];
    size_t got = rawRecvExact(peer, buf, sizeof(buf));

    ::close(fillerPeer);
    ::close(filler);
    closeRawServer(peer, listenSock);

    TEST_ASSERT_EQUAL_MESSAGE(0, got, "timed-out write request was sent after reconnection");

    Modbus::Logger::logln("TEST_CLIENT_NO_LATE_WRITE_AFTER_RECONNECT - OK");
    Modbus::Logger::logln();
}

// Regression: a Client destroyed while the interface still held its request (e.g. reconnecting)
// returned at once: the interface later called its TX result callback on the destroyed object,
// and sent the request for nobody. The destructor must abort the request & wait until the
// interface released it. Slow reconnection as above (filled backlog).
alignas(Modbus::Client) static uint8_t destroyedClientStorage[sizeof(Modbus::Client)];

struct TcpClientDestroyer {
    Modbus::Client* client;
    volatile bool done;
    volatile bool busyAtReturn;
};
static StaticTask_t tcpClientDestroyerTcb;
static StackType_t tcpClientDestroyerStack[4096];

static void tcpClientDestroyerTask(void* arg) {
    auto* destroyer = static_cast<TcpClientDestroyer*>(arg);
    destroyer->client->~Client();
    destroyer->busyAtReturn = !rawServerClientIfc.isReady();
    destroyer->done = true;
    vTaskDelete(nullptr);
}

void test_client_destroyed_while_interface_holds_request() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CLIENT_DESTROYED_WHILE_INTERFACE_HOLDS_REQUEST");

    beginRawServerClient();
    TEST_ASSERT_FALSE_MESSAGE(rawServerClientHal.isClientConnected(), "client should start disconnected");
    int listenSock = rawListen(RAW_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(listenSock >= 0, "raw server listen failed");
    int filler = rawConnect(RAW_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(filler >= 0, "filler connect failed");

    // The request waits in the interface while it reconnects (~3 s)
    auto* client = new (destroyedClientStorage) Modbus::Client(rawServerClientIfc, RAW_SERVER_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());
    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    req.data[0] = 0x1234;
    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->sendRequest(req, resp, &tracker));
    vTaskDelay(pdMS_TO_TICKS(50));

    // Destroy the client meanwhile, then let the reconnection complete
    TcpClientDestroyer destroyer = { client, false, false };
    xTaskCreateStatic(tcpClientDestroyerTask, "tcpClientDestroyer", sizeof(tcpClientDestroyerStack) / sizeof(StackType_t),
                      &destroyer, 1, tcpClientDestroyerStack, &tcpClientDestroyerTcb);
    vTaskDelay(pdMS_TO_TICKS(100));
    int fillerPeer = ::accept(listenSock, nullptr, nullptr);
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(listenSock, &rfds);
    timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    int peer = (::select(listenSock + 1, &rfds, nullptr, nullptr, &tv) == 1) ? ::accept(listenSock, nullptr, nullptr) : -1;
    if (peer >= 0) rawSetupPeer(peer);
    uint32_t t0 = Modbus::TIME_MS();
    while (!destroyer.done && Modbus::TIME_MS() - t0 < 6000) vTaskDelay(pdMS_TO_TICKS(5));

    // Nothing must be sent for the destroyed client
    uint8_t buf[RAW_READ_HR_REQ_SIZE];
    size_t got = (peer >= 0) ? rawRecvExact(peer, buf, sizeof(buf)) : 0;
    ::close(fillerPeer);
    ::close(filler);
    closeRawServer(peer, listenSock);

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(destroyer.done, "client destructor didn't return");
    TEST_ASSERT_FALSE_MESSAGE(destroyer.busyAtReturn, "client destroyed while the interface still held its request");
    TEST_ASSERT_TRUE_MESSAGE(peer >= 0, "client did not reconnect");
    TEST_ASSERT_EQUAL_MESSAGE(0, got, "request of a destroyed client was sent");

    Modbus::Logger::logln("TEST_CLIENT_DESTROYED_WHILE_INTERFACE_HOLDS_REQUEST - OK");
    Modbus::Logger::logln();
}

// Regression: the interface released its TX slot before calling the TX result callback: in
// between, another task could submit request B, and the result of A (e.g. A dropped after its
// timeout, or a broadcast sent) was applied by the Client to B. The interface must stay busy
// until the callback returned: checked from inside the callback, for a failed send (no server)
// and for a broadcast (transaction ended right after sending).
struct TxCallbackProbe {
    volatile bool called;
    volatile bool readyInside;
};

static void txCallbackProbeFn(ModbusInterface::IInterface::Result, void* ctx) {
    auto* probe = static_cast<TxCallbackProbe*>(ctx);
    probe->readyInside = rawServerClientIfc.isReady();
    probe->called = true;
}

static TxCallbackProbe runTxCallbackProbe(const Modbus::Frame& frame) {
    TxCallbackProbe probe = { false, false };
    if (rawServerClientIfc.sendFrame(frame, txCallbackProbeFn, &probe) == ModbusInterface::IInterface::SUCCESS) {
        uint32_t t0 = Modbus::TIME_MS();
        while (!probe.called && Modbus::TIME_MS() - t0 < 2000) vTaskDelay(pdMS_TO_TICKS(5));
    }
    return probe;
}

void test_interface_busy_until_tx_callback_returned() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_INTERFACE_BUSY_UNTIL_TX_CALLBACK_RETURNED");

    beginRawServerClient();
    Modbus::Frame frame = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    frame.data[0] = 0x1234;

    // 1) Failed send: nobody listens, the connection is refused
    TxCallbackProbe failed = runTxCallbackProbe(frame);

    // 2) Broadcast sent to a raw server
    int listenSock = rawListen(RAW_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(listenSock >= 0, "raw server listen failed");
    frame.slaveId = 0;
    TxCallbackProbe broadcast = runTxCallbackProbe(frame);
    int peer = ::accept(listenSock, nullptr, nullptr);
    closeRawServer(peer, listenSock);

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(failed.called, "no TX callback for the failed send");
    TEST_ASSERT_FALSE_MESSAGE(failed.readyInside, "interface ready during the TX callback (failed send)");
    TEST_ASSERT_TRUE_MESSAGE(broadcast.called, "no TX callback for the broadcast");
    TEST_ASSERT_FALSE_MESSAGE(broadcast.readyInside, "interface ready during the TX callback (broadcast)");

    Modbus::Logger::logln("TEST_INTERFACE_BUSY_UNTIL_TX_CALLBACK_RETURNED - OK");
    Modbus::Logger::logln();
}

// Regression: a response PDU with the exception bit set but exception code 0 (e.g. 86 00)
// was decoded as a normal response without error: a write got a SUCCESS without any
// valid acknowledgement. The raw server answers a FC06 write with 86 00.
void test_client_null_exception_response_rejected() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CLIENT_NULL_EXCEPTION_RESPONSE_REJECTED");

    int listenSock = rawListen(RAW_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(listenSock >= 0, "raw server listen failed");
    beginRawServerClient();

    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    req.data[0] = 0x1234;
    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, rawServerClient.sendRequest(req, resp, &tracker));
    int peer = ::accept(listenSock, nullptr, nullptr);
    TEST_ASSERT_TRUE_MESSAGE(peer >= 0, "accept() failed");
    rawSetupPeer(peer);
    uint8_t reqBytes[RAW_READ_HR_REQ_SIZE]; // FC06 request: same size as FC03
    TEST_ASSERT_EQUAL_MESSAGE(sizeof(reqBytes), rawRecvExact(peer, reqBytes, sizeof(reqBytes)), "request not received");

    const uint8_t bogus[] = {reqBytes[0], reqBytes[1], 0x00, 0x00, 0x00, 0x03, TEST_SLAVE_ID, 0x86, 0x00};
    TEST_ASSERT_TRUE_MESSAGE(rawSendAll(peer, bogus, sizeof(bogus)), "send bogus response failed");

    volatile Modbus::Client::Result& result = tracker; // Written by the client
    uint32_t t0 = Modbus::TIME_MS();
    while (result == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < RAW_SERVER_CLIENT_TIMEOUT_MS + 200) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    closeRawServer(peer, listenSock);
    Modbus::Logger::logf("  result: %s\n", Modbus::Client::toString(tracker));

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, tracker, "86 00 must not complete the write");

    Modbus::Logger::logln("TEST_CLIENT_NULL_EXCEPTION_RESPONSE_REJECTED - OK");
    Modbus::Logger::logln();
}

// ===================================================================================
// SERVER: RX STREAM FRAMING
// A raw client talks to the main EZModbus test server (mbt).
// ===================================================================================

// Regression: a request split across two segments (MBAP header, then the rest)
// was discarded as soon as the header was read.
void test_server_rx_fragmented_request() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_RX_FRAGMENTED_REQUEST");

    int sock = rawConnect(MODBUS_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to server failed");

    uint8_t req[RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(req, 0x1234, 0);
    TEST_ASSERT_TRUE(rawSendAll(sock, req, 7));                 // MBAP header (incl. unit ID)
    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_TRUE(rawSendAll(sock, req + 7, sizeof(req) - 7)); // PDU

    uint8_t resp[RAW_READ_HR_RESP_SIZE];
    size_t got = rawRecvExact(sock, resp, sizeof(resp));
    ::close(sock);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(RAW_READ_HR_RESP_SIZE, got, "no response to a fragmented request");
    TEST_ASSERT_EQUAL_HEX16(0x1234, rawTid(resp));
    TEST_ASSERT_EQUAL_HEX16(MBT_INIT_HOLDING_REGISTER_VALUE(0), rawReadHrRespValue(resp));

    Modbus::Logger::logln("TEST_SERVER_RX_FRAGMENTED_REQUEST - OK");
    Modbus::Logger::logln();
}

// Read one ADU (MBAP length-framed), returns its size (0 if none / truncated)
static size_t rawRecvAdu(int sock, uint8_t* buf, size_t cap) {
    if (cap < 6 || rawRecvExact(sock, buf, 6) != 6) return 0;
    size_t len = 6 + ((static_cast<size_t>(buf[4]) << 8) | buf[5]);
    if (len > cap || rawRecvExact(sock, buf + 6, len - 6) != len - 6) return 0;
    return len;
}

// Regression: two requests received in the same segment: the second one was
// read from the socket and then silently dropped.
// The second one may be answered with SLAVE_DEVICE_BUSY instead (possibly before the
// first response) if it's read while the first transaction is open: that's allowed (the
// client pairs answers by TID), as long as no request goes unanswered.
void test_server_rx_coalesced_requests() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_RX_COALESCED_REQUESTS");

    int sock = rawConnect(MODBUS_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to server failed");

    uint8_t reqs[2 * RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(reqs, 0x0A01, 0);
    rawBuildReadHrRequest(reqs + RAW_READ_HR_REQ_SIZE, 0x0A02, 1);
    TEST_ASSERT_TRUE(rawSendAll(sock, reqs, sizeof(reqs)));

    uint8_t answers[2][RAW_READ_HR_RESP_SIZE] = {};
    size_t lens[2] = {0, 0};
    for (int i = 0; i < 2; i++) {
        uint8_t adu[RAW_READ_HR_RESP_SIZE];
        size_t len = rawRecvAdu(sock, adu, sizeof(adu));
        if (len == 0) break;
        int slot = (rawTid(adu) == 0x0A02) ? 1 : 0;
        memcpy(answers[slot], adu, len);
        lens[slot] = len;
    }
    ::close(sock);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(RAW_READ_HR_RESP_SIZE, lens[0], "first coalesced request not answered");
    TEST_ASSERT_EQUAL_HEX16(0x0A01, rawTid(answers[0]));
    TEST_ASSERT_EQUAL_HEX16(MBT_INIT_HOLDING_REGISTER_VALUE(0), rawReadHrRespValue(answers[0]));
    TEST_ASSERT_TRUE_MESSAGE(lens[1] > 0, "second coalesced request not answered");
    TEST_ASSERT_EQUAL_HEX16(0x0A02, rawTid(answers[1]));
    if (lens[1] == RAW_READ_HR_RESP_SIZE) {
        TEST_ASSERT_EQUAL_HEX16(MBT_INIT_HOLDING_REGISTER_VALUE(1), rawReadHrRespValue(answers[1]));
    } else {
        TEST_ASSERT_EQUAL_MESSAGE(9, lens[1], "unexpected answer to the second coalesced request");
        TEST_ASSERT_EQUAL_HEX8(0x83, answers[1][7]);
        TEST_ASSERT_EQUAL_HEX8(Modbus::SLAVE_DEVICE_BUSY, answers[1][8]);
    }

    Modbus::Logger::logln("TEST_SERVER_RX_COALESCED_REQUESTS - OK");
    Modbus::Logger::logln();
}

// A truncated ADU that never completes leaves the stream misaligned: the server
// must close the connection instead of reading the next bytes as a new header.
void test_server_rx_truncated_request_closes_connection() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_RX_TRUNCATED_REQUEST_CLOSES_CONNECTION");

    int sock = rawConnect(MODBUS_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to server failed");

    uint8_t req[RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(req, 0x0B01, 0);
    TEST_ASSERT_TRUE(rawSendAll(sock, req, 7)); // MBAP header only, PDU never sent

    // recv() returns 0 once the server has closed the connection (-1 on our 1 s timeout)
    uint8_t dump[16];
    ssize_t n = ::recv(sock, dump, sizeof(dump), 0);
    ::close(sock);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, n, "server should close a connection with a truncated ADU");

    // The server must still serve new connections
    sock = rawConnect(MODBUS_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw reconnect to server failed");
    rawBuildReadHrRequest(req, 0x0B02, 0);
    TEST_ASSERT_TRUE(rawSendAll(sock, req, sizeof(req)));
    uint8_t resp[RAW_READ_HR_RESP_SIZE];
    size_t got = rawRecvExact(sock, resp, sizeof(resp));
    ::close(sock);
    TEST_ASSERT_EQUAL_MESSAGE(RAW_READ_HR_RESP_SIZE, got, "server should answer on a new connection");
    TEST_ASSERT_EQUAL_HEX16(0x0B02, rawTid(resp));

    Modbus::Logger::logln("TEST_SERVER_RX_TRUNCATED_REQUEST_CLOSES_CONNECTION - OK");
    Modbus::Logger::logln();
}

// ===================================================================================
// UNIT ID 255
// 0xFF is the Unit ID recommended for devices addressed directly over TCP.
// ===================================================================================

// Regression: the Client validated TCP requests with the RTU address range (1-247)
void test_client_unit_id_255() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CLIENT_UNIT_ID_255");

    Modbus::Frame req = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = 0xFF,
        .regAddress = 0,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame resp;
    auto res = client.sendRequest(req, resp, nullptr); // Sync

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, res, "request to Unit ID 255 failed");
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, resp.exceptionCode);
    TEST_ASSERT_EQUAL_HEX8(0xFF, resp.slaveId);
    TEST_ASSERT_EQUAL_HEX16(MBT_INIT_HOLDING_REGISTER_VALUE(0), resp.data[0]);

    Modbus::Logger::logln("TEST_CLIENT_UNIT_ID_255 - OK");
    Modbus::Logger::logln();
}

// Regression: the server accepted a TCP request with Unit ID 255, but failed to
// encode its response (responses were validated with the RTU range)
void test_server_unit_id_255() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_UNIT_ID_255");

    int sock = rawConnect(MODBUS_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to server failed");

    uint8_t req[RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(req, 0x0F01, 0, 0xFF);
    TEST_ASSERT_TRUE(rawSendAll(sock, req, sizeof(req)));
    uint8_t resp[RAW_READ_HR_RESP_SIZE];
    size_t got = rawRecvExact(sock, resp, sizeof(resp));
    ::close(sock);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(RAW_READ_HR_RESP_SIZE, got, "no response to a request with Unit ID 255");
    TEST_ASSERT_EQUAL_HEX16(0x0F01, rawTid(resp));
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0xFF, resp[6], "response should echo Unit ID 255");
    TEST_ASSERT_EQUAL_HEX16(MBT_INIT_HOLDING_REGISTER_VALUE(0), rawReadHrRespValue(resp));

    Modbus::Logger::logln("TEST_SERVER_UNIT_ID_255 - OK");
    Modbus::Logger::logln();
}

// ===================================================================================
// MALFORMED REQUESTS
// A request received without communication error but that can't be handled must be
// answered with an exception (Modbus spec): ILLEGAL_FUNCTION for an unsupported function
// code, ILLEGAL_DATA_VALUE for an inconsistent structure. They used to be dropped silently.
// ===================================================================================

struct MalformedCase {
    const char* name;
    uint8_t unitId;
    uint8_t pdu[12];
    uint8_t pduLen;
    uint8_t expectedFc;  // 0 = no response expected
    uint8_t expectedEc;
};

static const MalformedCase MALFORMED_CASES[] = {
    { "unsupported FC 0x2B",        TEST_SLAVE_ID, {0x2B, 0x0E, 0x01, 0x00}, 4,                          0xAB, 0x01 },
    { "request FC with bit 7 set",  TEST_SLAVE_ID, {0x83, 0x00, 0x00, 0x00, 0x01}, 5,                    0x83, 0x01 },
    { "request FC 0x83, 2-byte PDU", TEST_SLAVE_ID, {0x83, 0x00}, 2,                                   0x83, 0x01 },
    { "FC03 quantity 126",          TEST_SLAVE_ID, {0x03, 0x00, 0x00, 0x00, 0x7E}, 5,                    0x83, 0x03 },
    { "FC03 truncated PDU",         TEST_SLAVE_ID, {0x03, 0x00, 0x00, 0x00}, 4,                          0x83, 0x03 },
    { "FC05 value not 0/FF00",      TEST_SLAVE_ID, {0x05, 0x00, 0x00, 0x12, 0x34}, 5,                    0x85, 0x03 },
    { "FC16 byte count mismatch",   TEST_SLAVE_ID, {0x10, 0x00, 0x00, 0x00, 0x02, 0x03, 0x00, 0x01, 0x00}, 9, 0x90, 0x03 },
    { "broadcast, unsupported FC",  0x00,          {0x2B, 0x0E, 0x01, 0x00}, 4,                          0, 0 },
    { "broadcast, FC16 malformed",  0x00,          {0x10, 0x00, 0x00, 0x00, 0x02, 0x03, 0x00, 0x01, 0x00}, 9, 0, 0 },
};

void test_server_malformed_requests_exceptions() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_MALFORMED_REQUESTS_EXCEPTIONS");

    int sock = rawConnect(MODBUS_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to server failed");

    uint16_t tid = 0x1001;
    for (const auto& c : MALFORMED_CASES) {
        uint8_t adu[7 + sizeof(c.pdu)] = {
            (uint8_t)(tid >> 8), (uint8_t)(tid & 0xFF), 0x00, 0x00,
            0x00, (uint8_t)(c.pduLen + 1), c.unitId
        };
        memcpy(adu + 7, c.pdu, c.pduLen);
        TEST_ASSERT_TRUE(rawSendAll(sock, adu, 7 + c.pduLen));

        uint8_t resp[9];
        size_t got = (c.expectedFc != 0) ? rawRecvExact(sock, resp, sizeof(resp)) : 0;
        if (c.expectedFc == 0) {
            // No response expected: make sure nothing comes within a short delay
            vTaskDelay(pdMS_TO_TICKS(200));
            got = ::recv(sock, resp, sizeof(resp), MSG_DONTWAIT) > 0 ? 1 : 0;
        }
        Modbus::Logger::logf("  %s: %u byte(s)\n", c.name, (unsigned)got);

        TEST_ASSERT_START();
        if (c.expectedFc == 0) {
            TEST_ASSERT_EQUAL_MESSAGE(0, got, c.name);
        } else {
            TEST_ASSERT_EQUAL_MESSAGE(sizeof(resp), got, c.name);
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(tid, rawTid(resp), c.name);
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(c.unitId, resp[6], c.name);
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(c.expectedFc, resp[7], c.name);
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(c.expectedEc, resp[8], c.name);
        }
        tid++;
    }
    ::close(sock);

    Modbus::Logger::logln("TEST_SERVER_MALFORMED_REQUESTS_EXCEPTIONS - OK");
    Modbus::Logger::logln();
}

// ===================================================================================
// ISOLATED SERVER
// Server tests needing a specific application (no Modbus::Server) share one server
// interface (RAM footprint): each test sets the application's behaviour.
// ===================================================================================

static constexpr uint16_t ISOLATED_SERVER_TEST_PORT = 518;
ModbusHAL::TCP isolatedServerHal(ISOLATED_SERVER_TEST_PORT);
ModbusInterface::TCP isolatedServerIf(isolatedServerHal, Modbus::SERVER);
static void (*isolatedServerApp)(const Modbus::Frame& request) = nullptr;

static void beginIsolatedServer(void (*app)(const Modbus::Frame& request)) {
    isolatedServerApp = app;
    static bool initialized = false;
    if (initialized) return;
    TEST_ASSERT_TRUE(isolatedServerHal.begin());
    isolatedServerIf.setRcvCallback([](const Modbus::Frame& request, void*) {
        if (isolatedServerApp) isolatedServerApp(request);
    }, nullptr);
    TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, isolatedServerIf.begin());
    initialized = true;
}

// Regression: when the application's response couldn't be encoded, nothing was sent and
// the TCP transaction stayed open: the client got no answer and every other request got
// SLAVE_DEVICE_BUSY until the 5 s safety timeout. The server must answer SLAVE_DEVICE_FAILURE.
// The application answers with an unencodable response (register count 0).
void test_server_unencodable_response_exception() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_UNENCODABLE_RESPONSE_EXCEPTION");

    beginIsolatedServer([](const Modbus::Frame& request) {
        Modbus::Frame response;
        response.clear();
        response.type = Modbus::RESPONSE;
        response.fc = request.fc;
        response.slaveId = request.slaveId;
        response.regAddress = request.regAddress;
        response.regCount = 0; // Invalid: can't be encoded
        isolatedServerIf.sendFrame(response, nullptr, nullptr);
    });

    int sock = rawConnect(ISOLATED_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to server failed");

    // Two requests in a row: the second one must not get SLAVE_DEVICE_BUSY
    for (uint16_t tid = 0x0E11; tid <= 0x0E12; tid++) {
        uint8_t req[RAW_READ_HR_REQ_SIZE];
        rawBuildReadHrRequest(req, tid, 0);
        TEST_ASSERT_TRUE(rawSendAll(sock, req, sizeof(req)));
        uint8_t resp[9];
        size_t got = rawRecvExact(sock, resp, sizeof(resp));

        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL_MESSAGE(sizeof(resp), got, "no exception for an unencodable response");
        TEST_ASSERT_EQUAL_HEX16(tid, rawTid(resp));
        TEST_ASSERT_EQUAL_HEX8(TEST_SLAVE_ID, resp[6]);
        TEST_ASSERT_EQUAL_HEX8(0x83, resp[7]);
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(Modbus::SLAVE_DEVICE_FAILURE, resp[8], "unexpected exception code");
    }
    ::close(sock);

    Modbus::Logger::logln("TEST_SERVER_UNENCODABLE_RESPONSE_EXCEPTION - OK");
    Modbus::Logger::logln();
}

// Regression: a transaction only remembered the socket number. When client A disconnected
// while its request was being processed, a new client B could get the same socket number,
// and received the response meant for A. The application answers slowly.
static constexpr uint32_t SOCKET_REUSE_HANDLER_DELAY_MS = 500;

void test_server_response_not_sent_to_reused_socket() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_RESPONSE_NOT_SENT_TO_REUSED_SOCKET");

    beginIsolatedServer([](const Modbus::Frame& request) {
        vTaskDelay(pdMS_TO_TICKS(SOCKET_REUSE_HANDLER_DELAY_MS)); // Slow application
        Modbus::Frame response;
        response.clear();
        response.type = Modbus::RESPONSE;
        response.fc = request.fc;
        response.slaveId = request.slaveId;
        response.regAddress = request.regAddress;
        response.regCount = 1;
        response.data[0] = 0xAAAA;
        isolatedServerIf.sendFrame(response, nullptr, nullptr);
    });

    // Client A: request, then disconnect while the application is processing it
    int sockA = rawConnect(ISOLATED_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sockA >= 0, "client A connect failed");
    uint8_t req[RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(req, 0x0A0A, 0);
    TEST_ASSERT_TRUE(rawSendAll(sockA, req, sizeof(req)));
    vTaskDelay(pdMS_TO_TICKS(50));
    ::close(sockA);
    vTaskDelay(pdMS_TO_TICKS(100)); // Let the server HAL notice & free A's socket

    // Client B connects meanwhile (typically gets the same socket number on the server side)
    int sockB = rawConnect(ISOLATED_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sockB >= 0, "client B connect failed");

    // B sent nothing: it must receive nothing, in particular not A's response
    uint8_t buf[RAW_READ_HR_RESP_SIZE];
    size_t got = rawRecvExact(sockB, buf, sizeof(buf)); // 1 s receive timeout
    ::close(sockB);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, got, "client B received the response meant for client A");

    Modbus::Logger::logln("TEST_SERVER_RESPONSE_NOT_SENT_TO_REUSED_SOCKET - OK");
    Modbus::Logger::logln();
}

// Regression: while assembling a frame, the interface only checked the connection after
// reading. When client A disconnected mid-frame and client B got the same socket number,
// B's first bytes were read then dropped: the rest of B's frame was taken for a header,
// found invalid, and B was disconnected. The interface task is held while A leaves & B
// connects, so that B's request is waiting when it resumes the assembly of A's frame.
// The test checks that B actually got A's server socket number.
void test_server_rx_reused_socket_mid_frame() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_RX_REUSED_SOCKET_MID_FRAME");

    beginIsolatedServer([](const Modbus::Frame& request) {
        Modbus::Frame response;
        response.clear();
        response.type = Modbus::RESPONSE;
        response.fc = request.fc;
        response.slaveId = request.slaveId;
        response.regAddress = request.regAddress;
        response.regCount = 1;
        response.data[0] = 0xBBBB;
        isolatedServerIf.sendFrame(response, nullptr, nullptr);
    });

    // Client A: start of a header, the interface waits for the rest
    int sockA = rawConnect(ISOLATED_SERVER_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sockA >= 0, "client A connect failed");
    uint8_t req[RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(req, 0x0A0A, 0);
    TEST_ASSERT_TRUE(rawSendAll(sockA, req, 2));
    vTaskDelay(pdMS_TO_TICKS(20));
    const int serverSockA = rawServerSideSocket(ISOLATED_SERVER_TEST_PORT, sockA);
    TEST_ASSERT_TRUE_MESSAGE(serverSockA >= 0, "client A not accepted");

    // Hold the interface task while it waits for the rest (blocked, so not holding a HAL lock)
    TaskHandle_t ifTask = isolatedServerIf.getRxTxTaskHandle();
    for (int i = 0; i < 100 && eTaskGetState(ifTask) != eBlocked; i++) vTaskDelay(1);
    TEST_ASSERT_EQUAL_MESSAGE(eBlocked, eTaskGetState(ifTask), "interface task not waiting");
    vTaskSuspend(ifTask);

    // A leaves, B connects on the same server socket number & sends a request
    ::close(sockA);
    vTaskDelay(pdMS_TO_TICKS(100)); // Let the server HAL notice & free A's socket
    int sockB = rawConnect(ISOLATED_SERVER_TEST_PORT);
    int serverSockB = -1;
    for (int i = 0; i < 50 && sockB >= 0 && serverSockB < 0; i++) { // Until the HAL accepts B
        vTaskDelay(pdMS_TO_TICKS(10));
        serverSockB = rawServerSideSocket(ISOLATED_SERVER_TEST_PORT, sockB);
    }
    rawBuildReadHrRequest(req, 0x0B0B, 0);
    bool sent = sockB >= 0 && rawSendAll(sockB, req, sizeof(req));
    vTaskDelay(pdMS_TO_TICKS(50));
    vTaskResume(ifTask); // Before any assert: the following tests need the task
    if (sockB >= 0 && !sent) ::close(sockB);
    TEST_ASSERT_TRUE_MESSAGE(sent, "client B connect/send failed");
    if (serverSockB != serverSockA) ::close(sockB);
    TEST_ASSERT_EQUAL_MESSAGE(serverSockA, serverSockB, "client B didn't get A's server socket number");

    uint8_t resp[RAW_READ_HR_RESP_SIZE];
    size_t got = rawRecvExact(sockB, resp, sizeof(resp));
    ::close(sockB);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(sizeof(resp), got, "client B got no response");
    TEST_ASSERT_EQUAL_HEX16(0x0B0B, rawTid(resp));
    TEST_ASSERT_EQUAL_HEX16(0xBBBB, rawReadHrRespValue(resp));

    Modbus::Logger::logln("TEST_SERVER_RX_REUSED_SOCKET_MID_FRAME - OK");
    Modbus::Logger::logln();
}

// Regression: the HAL task posts one RX event per select() round while data is
// unread on a socket, and used to post it with portMAX_DELAY while holding the
// socket mutex. Once the RX queue was full (consumer busy), the HAL task blocked
// holding the mutex, and any sendMsg() - which needs the same mutex - blocked
// forever: the interface task could no longer send, nor drain the queue.
// HAL-only fixture: nobody consumes the RX queue, as with a busy interface task.
// HAL-level tests (notifications, keepalive, stop, interface lifecycle) share these spare
// HALs & interface storage (RAM footprint): each test begins the HALs it needs & stops them.
static constexpr uint16_t SPARE_HAL_TEST_PORT = 513;
ModbusHAL::TCP spareServerHal(SPARE_HAL_TEST_PORT);
ModbusHAL::TCP spareClientHal(LOOPBACK_IP_STR, SPARE_HAL_TEST_PORT);
alignas(ModbusInterface::TCP) static uint8_t spareIfcStorage[sizeof(ModbusInterface::TCP)];

struct HalSendProbe {
    int sock;
    volatile bool done;
    volatile bool result;
};
static StaticTask_t halSendProbeTcb;
static StackType_t halSendProbeStack[4096];

static void halSendProbeTask(void* arg) {
    auto* probe = static_cast<HalSendProbe*>(arg);
    const uint8_t byte = 0x00;
    probe->result = spareServerHal.sendMsg(&byte, 1, probe->sock);
    probe->done = true;
    vTaskDelete(nullptr);
}

void test_hal_rx_notifications_never_block_send() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_HAL_RX_NOTIFICATIONS_NEVER_BLOCK_SEND");

    TEST_ASSERT_TRUE_MESSAGE(spareServerHal.begin(), "HAL server begin failed");
    QueueHandle_t rxQueue = spareServerHal.getRxQueueHandle();

    int sock = rawConnect(SPARE_HAL_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to HAL server failed");

    // Unread data on the connection: the HAL keeps signaling it until the queue is full
    uint8_t req[RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(req, 0x0D01, 0);
    TEST_ASSERT_TRUE(rawSendAll(sock, req, sizeof(req)));

    int serverSock = -1;
    TEST_ASSERT_EQUAL_MESSAGE(pdTRUE, xQueuePeek(rxQueue, &serverSock, pdMS_TO_TICKS(500)), "no RX event from HAL");
    uint32_t t0 = Modbus::TIME_MS();
    while (uxQueueMessagesWaiting(rxQueue) < ModbusHAL::TCP::RX_QUEUE_SIZE && Modbus::TIME_MS() - t0 < 500) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_EQUAL_MESSAGE(ModbusHAL::TCP::RX_QUEUE_SIZE, uxQueueMessagesWaiting(rxQueue), "RX queue should be full");
    vTaskDelay(pdMS_TO_TICKS(20)); // Let the HAL task hit the full queue

    // Send from another task (it would block forever on the socket mutex before the fix)
    static HalSendProbe probe;
    probe = { serverSock, false, false };
    TaskHandle_t probeTask = xTaskCreateStatic(halSendProbeTask, "halSendProbe", sizeof(halSendProbeStack) / sizeof(StackType_t),
                                               &probe, tskIDLE_PRIORITY + 1, halSendProbeStack, &halSendProbeTcb);
    TEST_ASSERT_NOT_NULL(probeTask);
    t0 = Modbus::TIME_MS();
    while (!probe.done && Modbus::TIME_MS() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(5));

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(probe.done, "sendMsg() blocked while the RX queue was full");
    TEST_ASSERT_TRUE_MESSAGE(probe.result, "sendMsg() failed");

    // Cleanup: close the connection, then drain the events left in the queue
    ::close(sock);
    vTaskDelay(pdMS_TO_TICKS(50));
    int dummy;
    while (xQueueReceive(rxQueue, &dummy, 0) == pdTRUE) {}
    spareServerHal.stop();

    Modbus::Logger::logln("TEST_HAL_RX_NOTIFICATIONS_NEVER_BLOCK_SEND - OK");
    Modbus::Logger::logln();
}

// ===================================================================================
// TCP KEEPALIVE
// Without keepalive, a half-open connection (peer powered off, cable cut) is never
// detected: it holds one of the server's connection slots forever, and a client keeps
// sending into the void. A dead peer can't be simulated on loopback (the local stack
// answers the probes), so we check the options set on the HAL sockets.
// ===================================================================================

static constexpr uint32_t MAX_DEAD_PEER_DETECTION_S = 60;

static void assertKeepAlive(int fd, const char* side) {
    int enabled = 0, idle = 0, intvl = 0, cnt = 0;
    socklen_t len = sizeof(int);
    getsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, &len);
    len = sizeof(int); getsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, &len);
    len = sizeof(int); getsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, &len);
    len = sizeof(int); getsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, &len);
    Modbus::Logger::logf("  %s socket %d: keepalive=%d idle=%ds intvl=%ds cnt=%d\n", side, fd, enabled, idle, intvl, cnt);
    TEST_ASSERT_TRUE_MESSAGE(enabled != 0, "SO_KEEPALIVE not enabled");
    TEST_ASSERT_TRUE_MESSAGE(idle > 0 && intvl > 0 && cnt > 0, "invalid keepalive parameters");
    TEST_ASSERT_TRUE_MESSAGE((uint32_t)(idle + intvl * cnt) <= MAX_DEAD_PEER_DETECTION_S, "dead peer detection too slow");
}

void test_hal_tcp_keepalive() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_HAL_TCP_KEEPALIVE");

    TEST_ASSERT_TRUE_MESSAGE(spareServerHal.begin(), "server HAL begin failed");
    TEST_ASSERT_TRUE_MESSAGE(spareClientHal.begin(), "client HAL begin failed");

    // Server side socket: signaled by the server HAL once the client sends something
    const uint8_t byte = 0x00;
    TEST_ASSERT_TRUE_MESSAGE(spareClientHal.sendMsg(&byte, 1), "client HAL send failed");
    int serverSock = -1;
    TEST_ASSERT_EQUAL_MESSAGE(pdTRUE, xQueuePeek(spareServerHal.getRxQueueHandle(), &serverSock, pdMS_TO_TICKS(500)),
                              "no RX event from server HAL");

    // Client side socket: the one whose local port is the server socket's peer port
    sockaddr_in peer;
    socklen_t len = sizeof(peer);
    TEST_ASSERT_EQUAL(0, getpeername(serverSock, reinterpret_cast<sockaddr*>(&peer), &len));
    int clientSock = -1;
    for (int fd = LWIP_SOCKET_OFFSET; fd < LWIP_SOCKET_OFFSET + CONFIG_LWIP_MAX_SOCKETS; fd++) {
        if (fd != serverSock && localPort(fd) == ntohs(peer.sin_port)) { clientSock = fd; break; }
    }

    TEST_ASSERT_START();
    assertKeepAlive(serverSock, "server");
    TEST_ASSERT_TRUE_MESSAGE(clientSock >= 0, "client HAL socket not found");
    assertKeepAlive(clientSock, "client");

    spareClientHal.stop();
    spareServerHal.stop();

    Modbus::Logger::logln("TEST_HAL_TCP_KEEPALIVE - OK");
    Modbus::Logger::logln();
}

// Regression: once the interface had checked that its request was not aborted, sendMsg()
// could reconnect by itself (the connection having dropped meanwhile) and send the request
// late on a new connection. Bound to a connection, sendMsg() must fail without reconnecting.
// Raw server, so that a reconnection shows as a pending connection on the listening socket.
void test_hal_send_bound_to_connection_no_reconnect() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_HAL_SEND_BOUND_TO_CONNECTION_NO_RECONNECT");

    int listenSock = rawListen(SPARE_HAL_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(listenSock >= 0, "raw server listen failed");
    bool begun = spareClientHal.begin(); // Connects
    int peer = begun ? ::accept(listenSock, nullptr, nullptr) : -1;

    // Connection ID of the current connection, then the server closes it
    const uint8_t byte = 0x00;
    uint32_t connId = 0;
    bool firstSent = peer >= 0 && spareClientHal.sendMsg(&byte, 1, -1, nullptr, 0, &connId);
    if (peer >= 0) ::close(peer);
    uint32_t t0 = Modbus::TIME_MS();
    while (spareClientHal.isClientConnected() && Modbus::TIME_MS() - t0 < 2000) vTaskDelay(pdMS_TO_TICKS(5));
    bool disconnected = !spareClientHal.isClientConnected();

    // Send bound to the closed connection: must fail, without connecting again
    bool lateSent = spareClientHal.sendMsg(&byte, 1, -1, nullptr, connId);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(listenSock, &fds);
    timeval tv = { .tv_sec = 0, .tv_usec = 300000 };
    bool reconnected = ::select(listenSock + 1, &fds, nullptr, nullptr, &tv) > 0;
    if (reconnected) {
        int peer2 = ::accept(listenSock, nullptr, nullptr);
        if (peer2 >= 0) ::close(peer2);
    }
    spareClientHal.stop();
    ::close(listenSock);

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(begun && peer >= 0, "client HAL didn't connect");
    TEST_ASSERT_TRUE_MESSAGE(firstSent && connId != 0, "first send failed");
    TEST_ASSERT_TRUE_MESSAGE(disconnected, "client HAL didn't notice the disconnection");
    TEST_ASSERT_FALSE_MESSAGE(lateSent, "send bound to a closed connection succeeded");
    TEST_ASSERT_FALSE_MESSAGE(reconnected, "send bound to a closed connection reconnected");

    Modbus::Logger::logln("TEST_HAL_SEND_BOUND_TO_CONNECTION_NO_RECONNECT - OK");
    Modbus::Logger::logln();
}

// ===================================================================================
// LIFECYCLE: STOP / DESTRUCTION MUST JOIN THE TASKS
// A task still running after stop() / destruction may access a destroyed object,
// or have its static TCB & stack reused by the next begin(). We check that the
// number of FreeRTOS tasks is back to its baseline as soon as the call returns.
// ===================================================================================

static UBaseType_t settledTaskCount() {
    vTaskDelay(pdMS_TO_TICKS(50)); // Let the idle task clean up tasks deleted by previous tests
    return uxTaskGetNumberOfTasks();
}

// Regression: HAL stop() slept 200 ms instead of waiting for its task (which can
// sit in select() for up to 1 s), and in client mode the task handle was not even
// stored, so stop() did not wait at all.

void test_hal_stop_joins_task() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_HAL_STOP_JOINS_TASK");

    const UBaseType_t baseline = settledTaskCount();

    for (int round = 0; round < 2; round++) { // Second round: restart after stop()
        TEST_ASSERT_TRUE_MESSAGE(spareServerHal.begin(), "server HAL begin failed");
        TEST_ASSERT_TRUE_MESSAGE(spareClientHal.begin(), "client HAL begin failed");
        vTaskDelay(pdMS_TO_TICKS(50)); // Both tasks now waiting in select()
        TEST_ASSERT_EQUAL(baseline + 2, uxTaskGetNumberOfTasks());

        spareClientHal.stop();
        TEST_ASSERT_EQUAL_MESSAGE(baseline + 1, uxTaskGetNumberOfTasks(), "client HAL task still running after stop()");
        spareServerHal.stop();
        TEST_ASSERT_EQUAL_MESSAGE(baseline, uxTaskGetNumberOfTasks(), "server HAL task still running after stop()");
    }

    Modbus::Logger::logln("TEST_HAL_STOP_JOINS_TASK - OK");
    Modbus::Logger::logln();
}

// Regression: the interface destructor slept 150 ms instead of waiting for its
// task (which can be in a 250 ms RX assembly wait), then deleted its queue set
// while the HAL RX queue was not empty: FreeRTOS refuses to remove a non-empty
// queue from a set, so the HAL queue kept a reference to the freed set and no
// other interface could be attached to that HAL.
// Interfaces live in static storage so that they can be destroyed and recreated.

void test_interface_destroy_joins_task_and_detaches_queue() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_INTERFACE_DESTROY_JOINS_TASK_AND_DETACHES_QUEUE");

    const UBaseType_t baseline = settledTaskCount();
    TEST_ASSERT_TRUE_MESSAGE(spareServerHal.begin(), "server HAL begin failed");

    auto* ifc1 = new (spareIfcStorage) ModbusInterface::TCP(spareServerHal, Modbus::SERVER);
    TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, ifc1->begin());
    TEST_ASSERT_EQUAL(baseline + 2, uxTaskGetNumberOfTasks());

    int sock1 = rawConnect(SPARE_HAL_TEST_PORT);
    int sock2 = rawConnect(SPARE_HAL_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock1 >= 0 && sock2 >= 0, "raw connect to server failed");
    vTaskDelay(pdMS_TO_TICKS(50)); // Let the HAL accept both connections

    // Connection 1: MBAP header only -> the interface task waits for the rest of the frame
    uint8_t req[RAW_READ_HR_REQ_SIZE];
    rawBuildReadHrRequest(req, 0x0E01, 0);
    TEST_ASSERT_TRUE(rawSendAll(sock1, req, 7));
    vTaskDelay(pdMS_TO_TICKS(20));
    // Connection 2: a full frame the interface task can't read meanwhile -> HAL RX queue not empty
    rawBuildReadHrRequest(req, 0x0E02, 0);
    TEST_ASSERT_TRUE(rawSendAll(sock2, req, sizeof(req)));
    vTaskDelay(pdMS_TO_TICKS(20));

    ifc1->~TCP();
    const UBaseType_t afterDestroy = uxTaskGetNumberOfTasks();

    // A new interface must be attachable to the same HAL
    auto* ifc2 = new (spareIfcStorage) ModbusInterface::TCP(spareServerHal, Modbus::SERVER);
    const auto beginRes = ifc2->begin();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(baseline + 1, afterDestroy, "interface task still running after destruction");
    TEST_ASSERT_EQUAL_MESSAGE(ModbusInterface::IInterface::SUCCESS, beginRes, "cannot attach a new interface to the HAL");

    ifc2->~TCP();
    ::close(sock1);
    ::close(sock2);
    spareServerHal.stop();
    TEST_ASSERT_EQUAL_MESSAGE(baseline, uxTaskGetNumberOfTasks(), "tasks left after full teardown");

    Modbus::Logger::logln("TEST_INTERFACE_DESTROY_JOINS_TASK_AND_DETACHES_QUEUE - OK");
    Modbus::Logger::logln();
}

// Interface destruction/re-creation while the HAL keeps signaling RX: once the interface task
// is gone nobody reads the data, so the HAL re-posts its notification continuously. Detaching
// the RX queue from the queue set must neither overflow the set (FreeRTOS assert) nor fail.
// It reuses the probe task buffers of test_hal_rx_notifications_never_block_send() (free by now).
static constexpr int DETACH_STRESS_CYCLES = 30;
static volatile bool detachStressRunning = false;

// Sends a request every 2 ms & drains whatever comes back (busy exceptions)
static void detachStressSenderTask(void* arg) {
    int sock = *static_cast<int*>(arg);
    uint8_t req[RAW_READ_HR_REQ_SIZE];
    uint8_t dump[64];
    uint16_t tid = 0;
    while (detachStressRunning) {
        rawBuildReadHrRequest(req, tid++, 0);
        ::send(sock, req, sizeof(req), MSG_DONTWAIT);
        while (::recv(sock, dump, sizeof(dump), MSG_DONTWAIT) > 0) {}
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    detachStressRunning = true; // Ack: the task no longer uses the socket
    vTaskSuspend(nullptr);
}

void test_interface_recreate_under_rx_traffic() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_INTERFACE_RECREATE_UNDER_RX_TRAFFIC");

    const UBaseType_t baseline = settledTaskCount();
    TEST_ASSERT_TRUE_MESSAGE(spareServerHal.begin(), "server HAL begin failed");
    static int sock;
    sock = rawConnect(SPARE_HAL_TEST_PORT);
    TEST_ASSERT_TRUE_MESSAGE(sock >= 0, "raw connect to server failed");

    detachStressRunning = true;
    TaskHandle_t sender = xTaskCreateStatic(detachStressSenderTask, "detachStress", sizeof(halSendProbeStack) / sizeof(StackType_t),
                                            &sock, tskIDLE_PRIORITY + 1, halSendProbeStack, &halSendProbeTcb);
    TEST_ASSERT_NOT_NULL(sender);

    int failedCycle = -1;
    for (int i = 0; i < DETACH_STRESS_CYCLES && failedCycle < 0; i++) {
        auto* ifc = new (spareIfcStorage) ModbusInterface::TCP(spareServerHal, Modbus::SERVER);
        if (ifc->begin() != ModbusInterface::IInterface::SUCCESS) failedCycle = i;
        vTaskDelay(pdMS_TO_TICKS(10 + (i % 7) * 3));
        ifc->~TCP();
        vTaskDelay(pdMS_TO_TICKS(5)); // Unread data keeps being signaled meanwhile
    }

    // Stop the sender (it acks by setting the flag back), then clean up
    detachStressRunning = false;
    uint32_t t0 = Modbus::TIME_MS();
    while (!detachStressRunning && Modbus::TIME_MS() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(5));
    vTaskDelete(sender);
    ::close(sock);
    spareServerHal.stop();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(-1, failedCycle, "interface begin() failed after a re-creation under RX traffic");
    TEST_ASSERT_EQUAL_MESSAGE(baseline, uxTaskGetNumberOfTasks(), "tasks left after the stress test");

    Modbus::Logger::logln("TEST_INTERFACE_RECREATE_UNDER_RX_TRAFFIC - OK");
    Modbus::Logger::logln();
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
    // int32_t writeResult; // Old helper artifact
    Modbus::Frame broadcastWriteRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER, // Assuming single register write
        .slaveId = BROADCAST_SLAVE_ID,  // Broadcast address
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .data = {42},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame broadcastWriteResponse; // Will not be filled for broadcast
    auto result = client.sendRequest(
        broadcastWriteRequest,
        broadcastWriteResponse, // Response object is not used by client for broadcast
        nullptr // Synchronous mode
    );
    
    // For broadcast, sendRequest should return SUCCESS if TX is OK. No response is waited for.
    // The old helper set writeResult = 1 immediately.
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    // TEST_ASSERT_EQUAL(1, writeResult); // Implicitly tested by SUCCESS from sendRequest for broadcast

    // Calls to Modbus::Client::poll() to release the UART bus after sending the request without response
    // This is now handled internally by the client's poll task.
    // We need to ensure the client becomes ready again.
    while (!client.isReady()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Add some time to let ModbusTestServer resynchronize
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Check that the value has been written on the server
    int32_t readValue;
    Modbus::Frame readCheckRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = TEST_SLAVE_ID, // Normal read to check
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame readCheckResponse;
    result = client.sendRequest(readCheckRequest, readCheckResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readCheckResponse.exceptionCode);
    readValue = readCheckResponse.data[0];
    TEST_ASSERT_EQUAL(42, readValue);
    
    Modbus::Logger::logln("TEST_BROADCAST - CASE 2: ASYNC BROADCAST WRITE W/ HELPER");
    
    // Test with a tracker to check that it passes to DONE immediately
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    // writeResult again is an old helper artifact for broadcast.
    broadcastWriteRequest.data = {43}; // Update value to write
    result = client.sendRequest(
        broadcastWriteRequest,
        broadcastWriteResponse, // Not used by client for broadcast
        &tracker
    );
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker);  // Must be DONE quickly

    while (!client.isReady()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Add some time to let ModbusTestServer resynchronize
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Check that the value has been written
    result = client.sendRequest(readCheckRequest, readCheckResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readCheckResponse.exceptionCode);
    readValue = readCheckResponse.data[0];
    TEST_ASSERT_EQUAL(43, readValue);
    
    Modbus::Logger::logln("TEST_BROADCAST - CASE 3: ASYNC BROADCAST WRITE W/O HELPER");
    
    // Test with a raw request
    Modbus::Frame request = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_REGISTER,
        .slaveId = BROADCAST_SLAVE_ID,  // Broadcast address
        .regAddress = BROADCAST_START_ADDR,
        .regCount = 1,
        .data = {44},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame response;
    result = client.sendRequest(request, response, &tracker);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker);  // Must be DONE immediately

    while (!client.isReady()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Add some time to let ModbusTestServer resynchronize
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Check that the value has been written
    result = client.sendRequest(readCheckRequest, readCheckResponse, nullptr);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, readCheckResponse.exceptionCode);
    readValue = readCheckResponse.data[0];
    TEST_ASSERT_EQUAL(44, readValue);
}


// TEST_CONCURRENT_CALLS DATA STRUCTURES & TASKS

// Synchronization data structure
struct SyncData {
    SemaphoreHandle_t startSignal;
    SemaphoreHandle_t resultsMutex;
    SemaphoreHandle_t doneSignal;
    Modbus::Client::Result core0Result;
    Modbus::Client::Result core1Result;
};

// Task executed on each core
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

    // Store the result
    xSemaphoreTake(sd->resultsMutex, portMAX_DELAY);
    if (core == 0) sd->core0Result = result;
    else           sd->core1Result = result;
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
    sync.core0Result   = Modbus::Client::ERR_BUSY;
    sync.core1Result   = Modbus::Client::ERR_BUSY;

    // Launch the task on core 0
    xTaskCreatePinnedToCore(
        [](void* p){ concurrentTask(p); },
        "Task0", 8192, &sync, 5, nullptr, 0
    );
    // … and on core 1
    xTaskCreatePinnedToCore(
        [](void* p){ concurrentTask(p); },
        "Task1", 8192, &sync, 5, nullptr, 1
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
    Modbus::Logger::logf("Core 0: %s\n", sync.core0Result == Modbus::Client::SUCCESS ? "SUCCESS" : "ERR_BUSY");
    Modbus::Logger::logf("Core 1: %s\n", sync.core1Result == Modbus::Client::SUCCESS ? "SUCCESS" : "ERR_BUSY");

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

void test_server_busy_exception() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_SERVER_BUSY_EXCEPTION");

    // -------- Activer slow mode sur le serveur --------
    extern volatile bool slowMode; // déclarée plus haut
    slowMode = true;

    // 1) Client principal envoie une requête asynchrone (sera ralentie)
    Modbus::Frame req {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = TEST_SLAVE_ID,
        .regAddress = 0,
        .regCount   = 1,
        .data       = {},
        .exceptionCode = Modbus::NULL_EXCEPTION
    };
    Modbus::Frame resp1;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    auto res = client.sendRequest(req, resp1, &tracker);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);

    // Laisser un tout petit délai pour s'assurer que la transaction est engagée
    vTaskDelay(pdMS_TO_TICKS(1));

    // 2) Créer à la volée un second client pour frapper le serveur pendant qu'il est occupé
    static ModbusHAL::TCP tempHal(LOOPBACK_IP_STR, MODBUS_PORT);
    tempHal.begin();
    static ModbusInterface::TCP tempIfc(tempHal, Modbus::CLIENT);
    tempIfc.begin();
    static Modbus::Client tempClient(tempIfc);
    tempClient.begin();

    Modbus::Frame busyResp;
    res = tempClient.sendRequest(req, busyResp, nullptr); // sync
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::SLAVE_DEVICE_BUSY, busyResp.exceptionCode);

    // 3) Attendre fin de la première transaction
    while (tracker == Modbus::Client::NODATA) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Désactiver slow mode
    slowMode = false;

    // 4) Retenter avec tempClient, doit passer
    Modbus::Frame okResp;
    res = tempClient.sendRequest(req, okResp, nullptr);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, okResp.exceptionCode);
}

void test_client_reconnect_on_first_request() {
    Modbus::Logger::logln();
    Modbus::Logger::logln("TEST_CLIENT_RECONNECT - Client starts before server is available");
    
    while (!client.isReady()) {
        Modbus::Logger::logln("Waiting for client to be ready from previous test...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Suspend the server task & stop the HAL server
    vTaskSuspend(modbusTestServerTaskHandle);
    halForServer.stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Stop the client HAL & destruct the client & interface
    halForClient.stop();
    ezm.~TCP();
    client.~Client();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Reconstruct the client & client interface
    new (&ezm) ModbusInterface::TCP(halForClient, Modbus::CLIENT);
    new (&client) Modbus::Client(ezm);
    
    // Restart the client HAL, client should appear disconnected
    halForClient.begin();
    TEST_ASSERT_START();
    TEST_ASSERT_FALSE(halForClient.isClientConnected());
    
    // Restart the client & client interface
    ezm.begin();
    client.begin();
    
    // Resume the server task & restart the server HAL
    vTaskResume(modbusTestServerTaskHandle);
    halForServer.begin();
    
    // Reconstruct server interface & restart it
    mbt.~TCP();
    new (&mbt) ModbusInterface::TCP(halForServer, Modbus::SERVER);
    mbt.begin();
    
    // Reconstruct Modbus server & restart it to register callbacks properly
    server.~Server();
    new (&server) Modbus::Server(mbt, dynamicStore);
    server.begin();
    
    // Let everything settle
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Try to send the request now that everything is ready, it should succeed
    uint16_t value;
    Modbus::Frame req {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = TEST_SLAVE_ID,
        .regAddress = MBT_INIT_START_REG,
        .regCount   = 1,
        .data       = {}
    };
    Modbus::Frame resp;
    auto result = client.sendRequest(req, resp);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, result);
    TEST_ASSERT_TRUE(halForClient.isClientConnected());
    TEST_ASSERT_EQUAL(MBT_INIT_HOLDING_REGISTER_VALUE(MBT_INIT_START_REG), resp.getRegister(0));
    
    Modbus::Logger::logln("Client successfully reconnected on first request");
}



// Multi-interface test instances - SEPARATE TCP PORTS to avoid catch-all conflicts
constexpr uint16_t TEST1_TCP_PORT = MODBUS_PORT + 1;
constexpr uint16_t TEST2_TCP_PORT = MODBUS_PORT + 2;

// TEST 1 instances (TCP on port 502+1)
ModbusHAL::TCP test1ServerTcpHal(TEST1_TCP_PORT);
ModbusHAL::TCP test1ClientTcpHal("127.0.0.1", TEST1_TCP_PORT);
ModbusInterface::TCP test1ServerTcp(test1ServerTcpHal, Modbus::SERVER);
ModbusInterface::TCP test1ClientTcp(test1ClientTcpHal, Modbus::CLIENT);
Modbus::Client test1TcpClient(test1ClientTcp);

// TEST 2 instances (TCP on port 502+2)
ModbusHAL::TCP test2ServerTcpHal(TEST2_TCP_PORT);
ModbusHAL::TCP test2ClientTcpHal("127.0.0.1", TEST2_TCP_PORT);
ModbusInterface::TCP test2ServerTcp(test2ServerTcpHal, Modbus::SERVER);
ModbusInterface::TCP test2ClientTcp(test2ClientTcpHal, Modbus::CLIENT);
Modbus::Client test2TcpClient(test2ClientTcp);

// SHARED RTU instances (no conflicts on RTU, filtered by slaveId)
ModbusHAL::UART multiInterfaceServerRtuHal(EZM_UART_NUM, UART_BAUD_RATE, ModbusHAL::UART::CONFIG_8N1, EZM_RX, EZM_TX);
ModbusHAL::UART multiInterfaceClientRtuHal(MBT_UART_NUM, UART_BAUD_RATE, ModbusHAL::UART::CONFIG_8N1, MBT_RX, MBT_TX);
ModbusInterface::RTU multiInterfaceServerRtu(multiInterfaceServerRtuHal, Modbus::SERVER);
ModbusInterface::RTU multiInterfaceClientRtu(multiInterfaceClientRtuHal, Modbus::CLIENT);
Modbus::Client multiInterfaceRtuClient(multiInterfaceClientRtu);

// Global test flag for test2 handler synchronization
static volatile bool g_test2HandlerEntered = false;

void test_multi_interface_server_reqmutex_maxtimeout() {
    Modbus::Logger::logln("TEST_MULTI_INTERFACE_SERVER_REQMUTEX_MAXTIMEOUT");

    // === MASTER TEST: Initialize all shared multi-interface components ===
    // This test initializes HAL + Interface pairs ONCE for both tests
    // Subsequent tests reuse these instances without re-initialization
    static bool sharedComponentsInitialized = false;
    if (!sharedComponentsInitialized) {
        Modbus::Logger::logln("[MASTER] Initializing shared multi-interface components");

        // Initialize TEST1 HAL + Interface pairs (TCP + RTU)
        test1ServerTcpHal.begin();
        test1ClientTcpHal.begin();
        multiInterfaceServerRtuHal.begin();
        multiInterfaceClientRtuHal.begin();

        auto test1TcpServerRes = test1ServerTcp.begin();
        auto test1TcpClientRes = test1ClientTcp.begin();
        auto rtuServerRes = multiInterfaceServerRtu.begin();
        auto rtuClientRes = multiInterfaceClientRtu.begin();
        auto test1TcpClientBeginRes = test1TcpClient.begin();
        auto rtuClientBeginRes = multiInterfaceRtuClient.begin();

        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, test1TcpServerRes);
        TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, test1TcpClientRes);
        TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, rtuServerRes);
        TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, rtuClientRes);
        TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, test1TcpClientBeginRes);
        TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, rtuClientBeginRes);

        sharedComponentsInitialized = true;
        Modbus::Logger::logln("[MASTER] Shared components initialized successfully");
    }

    // === TEST 1 SPECIFIC: Create server with slaveId=1 and UINT32_MAX timeout ===
    static Modbus::StaticWordStore<20> test1Store;
    static Modbus::Server test1Server({&test1ServerTcp, &multiInterfaceServerRtu}, test1Store, 1, true, UINT32_MAX);

    // 2. ADD WORDS TO TEST1 SERVER
    Modbus::Word testHoldingReg = {
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = 1000,
        .nbRegs = 1,
        .readHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            outVals[0] = 0x1234;  // Test value
            return Modbus::NULL_EXCEPTION;
        },
        .writeHandler = [](const uint16_t* writeVals, const Modbus::Word& word, void* userCtx) -> Modbus::ExceptionCode {
            // Accept any write for test
            return Modbus::NULL_EXCEPTION;
        },
        .userCtx = nullptr
    };

    auto addWordRes = test1Server.addWord(testHoldingReg);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, addWordRes);

    // 3. INITIALIZE SERVER AFTER ADDING WORDS
    auto serverBeginRes = test1Server.begin();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, serverBeginRes);

    // Test TCP client → multi-interface server (using global clients)
    // Clients are already initialized by master test

    // Test TCP read
    Modbus::Frame tcpRequest = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = 1,
        .regAddress = 1000,
        .regCount   = 1,
        .data       = {},
    };
    Modbus::Frame tcpResponse;

    vTaskDelay(pdMS_TO_TICKS(100)); // Let everything settle
    auto tcpResult = test1TcpClient.sendRequest(tcpRequest, tcpResponse);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tcpResult);
    TEST_ASSERT_EQUAL(0x1234, tcpResponse.getRegister(0));

    // Test RTU client → multi-interface server (using global clients)
    // Clients are already initialized by master test

    // Test RTU read
    Modbus::Frame rtuRequest = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = 1,
        .regAddress = 1000,
        .regCount   = 1,
        .data       = {},
    };
    Modbus::Frame rtuResponse;

    vTaskDelay(pdMS_TO_TICKS(100)); // Let server initialize
    auto rtuResult = multiInterfaceRtuClient.sendRequest(rtuRequest, rtuResponse);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, rtuResult);
    TEST_ASSERT_EQUAL(0x1234, rtuResponse.getRegister(0));

    // Test concurrent access

    // Send requests almost simultaneously (start by RTU -> higher delay)
    Modbus::Frame concurrentTcpResp, concurrentRtuResp;
    Modbus::Client::Result rtuTracker, tcpTracker = Modbus::Client::NODATA;
    // Async API = returns immediately
    multiInterfaceRtuClient.sendRequest(rtuRequest, concurrentRtuResp, &rtuTracker);
    vTaskDelay(pdMS_TO_TICKS(1));
    test1TcpClient.sendRequest(tcpRequest, concurrentTcpResp, &tcpTracker);

    TickType_t start = xTaskGetTickCount();
    TickType_t elapsed = 0;
    // Wait for both requests to complete
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed = xTaskGetTickCount() - start;
    } while ((tcpTracker == Modbus::Client::NODATA || rtuTracker == Modbus::Client::NODATA) 
            && elapsed < pdMS_TO_TICKS(1500)); // Timeout to avoid being stuck here

    // Both requests should succeed
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, rtuTracker);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tcpTracker);
    TEST_ASSERT_EQUAL(0x1234, concurrentRtuResp.getRegister(0));
    TEST_ASSERT_EQUAL(0x1234, concurrentTcpResp.getRegister(0));
}

void test_multi_interface_server_reqmutex_nolock() {
    Modbus::Logger::logln("TEST_MULTI_INTERFACE_SERVER_REQMUTEX_NOLOCK");

    // === SLAVE TEST: Initialize TEST2 specific components ===
    // Initialize TEST2 HAL + Interface pairs (separate TCP, shared RTU)
    test2ServerTcpHal.begin();
    test2ClientTcpHal.begin();
    // RTU already initialized by master test

    auto test2TcpServerRes = test2ServerTcp.begin();
    auto test2TcpClientRes = test2ClientTcp.begin();
    auto test2TcpClientBeginRes = test2TcpClient.begin();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, test2TcpServerRes);
    TEST_ASSERT_EQUAL(ModbusInterface::IInterface::SUCCESS, test2TcpClientRes);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, test2TcpClientBeginRes);

    // === TEST 2 SPECIFIC: Create server with slaveId=2 and timeout=0 (try-lock) ===
    static Modbus::StaticWordStore<20> test2Store;
    static Modbus::Server test2Server({&test2ServerTcp, &multiInterfaceServerRtu}, test2Store, 2, true, 0);

    // 2. ADD WORDS TO TEST2 SERVER (using different register address to avoid conflict)
    Modbus::Word test2HoldingReg = {
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = 2000,  // Different address from test1 (1000)
        .nbRegs = 1,
        .readHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            g_test2HandlerEntered = true;                    // Signal: mutex acquired
            vTaskDelay(pdMS_TO_TICKS(100));                   // Hold the mutex to force contention
            outVals[0] = 0x5678;  // Different test value for test2
            return Modbus::NULL_EXCEPTION;
        },
        .writeHandler = [](const uint16_t* writeVals, const Modbus::Word& word, void* userCtx) -> Modbus::ExceptionCode {
            // Accept any write for test
            return Modbus::NULL_EXCEPTION;
        },
        .userCtx = nullptr
    };

    auto addWordRes = test2Server.addWord(test2HoldingReg);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, addWordRes);

    // 3. INITIALIZE TEST2 SERVER AFTER ADDING WORDS
    auto serverBeginRes = test2Server.begin();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, serverBeginRes);

    // === SIMPLIFIED TEST: Just validate timeout=0 behavior (try-lock) ===
    // Test one simple TCP read to validate server works with timeout=0

    // Test TCP read (using test2 server slaveId=2 and address=2000)
    Modbus::Frame tcpRequest = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = 2,          // Use test2 server slaveId
        .regAddress = 2000,       // Use test2 register address
        .regCount   = 1,
        .data       = {},
    };
    Modbus::Frame tcpResponse;

    vTaskDelay(pdMS_TO_TICKS(100)); // Let everything settle
    auto tcpResult = test2TcpClient.sendRequest(tcpRequest, tcpResponse);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tcpResult);
    TEST_ASSERT_EQUAL(0x5678, tcpResponse.getRegister(0));  // Expect test2 value

    // === TEST CONCURRENT ACCESS WITH TIMEOUT=0 (try-lock behavior) ===
    // The key difference vs test1 is the mutex timeout behavior:
    // - test1: UINT32_MAX (blocking) - will wait for mutex
    // - test2: 0 (try-lock) - returns SLAVE_DEVICE_BUSY immediately if mutex is locked

    Modbus::Logger::logln("[TEST2] Testing concurrent access with timeout=0");

    // Test RTU read to test2 server
    Modbus::Frame rtuRequest = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = 2,          // Use test2 server slaveId
        .regAddress = 2000,       // Use test2 register address
        .regCount   = 1,
        .data       = {},
    };
    Modbus::Frame rtuResponse;

    vTaskDelay(pdMS_TO_TICKS(100)); // Let server initialize
    auto rtuResult = multiInterfaceRtuClient.sendRequest(rtuRequest, rtuResponse);

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, rtuResult);
    TEST_ASSERT_EQUAL(0x5678, rtuResponse.getRegister(0));  // Expect test2 value

    // Test concurrent access - both to test2 server
    Modbus::Frame concurrentTcpResp, concurrentRtuResp;
    Modbus::Client::Result rtuTracker = Modbus::Client::NODATA, tcpTracker = Modbus::Client::NODATA;

    // Reset flag and send RTU request first to grab the mutex
    g_test2HandlerEntered = false;
    multiInterfaceRtuClient.sendRequest(rtuRequest, concurrentRtuResp, &rtuTracker);

    // Wait for handler to enter (ensuring mutex is held)
    TickType_t waitStart = xTaskGetTickCount();
    while (!g_test2HandlerEntered && (xTaskGetTickCount() - waitStart) < pdMS_TO_TICKS(200)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Now send TCP request - should get SLAVE_DEVICE_BUSY due to try-lock with timeout=0
    test2TcpClient.sendRequest(tcpRequest, concurrentTcpResp, &tcpTracker);  // Use test2 TCP client!

    TickType_t start = xTaskGetTickCount();
    TickType_t elapsed = 0;
    // Wait for both requests to complete
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed = xTaskGetTickCount() - start;
    } while ((tcpTracker == Modbus::Client::NODATA || rtuTracker == Modbus::Client::NODATA)
            && elapsed < pdMS_TO_TICKS(1500));

    // With timeout=0: RTU should succeed, TCP should get SLAVE_DEVICE_BUSY immediately
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, rtuTracker);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tcpTracker);
    TEST_ASSERT_EQUAL(0x5678, concurrentRtuResp.getRegister(0));  // RTU got the value
    TEST_ASSERT_EQUAL(Modbus::SLAVE_DEVICE_BUSY, concurrentTcpResp.exceptionCode);  // TCP got busy!

    Modbus::Logger::logln("[TEST2] Concurrent access with timeout=0 works correctly!");
}

void test_multi_interface_server_overflow() {
    Modbus::Logger::logln("TEST_SERVER_MAX_INTERFACES_OVERFLOW");

    // Dummy store to satisfy ctor
    static Modbus::StaticWordStore<5> overflowStore;

    // Reuse global interface instance from previous test and try to register them 4 time
    static Modbus::Server overflowServer({&mbt, &mbt, &mbt, &mbt}, overflowStore);

    // begin() must fail because too many interfaces (default max: 2)
    auto overflowServerRes = overflowServer.begin();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Server::ERR_INIT_FAILED, overflowServerRes);
}

void setup() {
    // Debug port
    Serial.setTxBufferSize(2048);
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);

    // Initialize ModbusTestServer TCP server
    // WiFi.setSleep(WIFI_PS_NONE);
    // 1) Démarrage du SoftAP
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(SOFTAP_IP, SOFTAP_IP, IPAddress(255,255,255,0));
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Modbus::Logger::logf("[setup] SoftAP IP : %s\n", WiFi.softAPIP().toString().c_str());

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run Modbus test server task on core 0 (main test thread on core 1)
    xTaskCreatePinnedToCore(ModbusTestServerTask, "ModbusTestServerTask", 16384, NULL, 5, &modbusTestServerTaskHandle, 0);
    while (!modbusTestServerTaskInitialized) {
        Modbus::Logger::logln("[setup] ModbusTestServer task not initialized, waiting...");
        vTaskDelay(pdMS_TO_TICKS(100));
    } // Wait for the task to be looping
    Modbus::Logger::logln("[setup] ModbusTestServer task initialized, starting tests...");

    // Initialize the clientTargetIpStr before initializing the ModbusInterface::TCP client instance IF it were local.
    // Since 'ezm' is global, its constructor using clientTargetIpStr.c_str() runs before setup().
    // So, clientTargetIpStr must be initialized globally or ezm must be initialized in setup().
    // The safest for global 'ezm' is to initialize clientTargetIpStr globally too.
    clientTargetIpStr = LOOPBACK_IP.toString(); // Initialize it here before ezm.begin() is called.
                                                // This ensures it's valid when ezm.begin() -> _hal.beginClient() is called.

    // Démarrage des HAL clients
    halForClient.begin();

    // EZModbus Client init
    auto ifcInitRes = ezm.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[setup] EZModbus TCP interface initialization failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    Modbus::Logger::logln("[setup] EZModbus TCP interface initialized");
    auto clientInitRes = client.begin();
    if (clientInitRes != Modbus::Client::SUCCESS) {
        Modbus::Logger::logln("[setup] EZModbus Client initialization failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    Modbus::Logger::logln("[setup] EZModbus Client initialized");

    // Isolated SERVER on port 510 for the safety-timeout idle-tick regression
    // test (no Modbus::Server on top — the callback never responds).
    halForSafetyServer.begin();
    safetyServerIf.setRcvCallback(
        [](const Modbus::Frame& /*frame*/, void* /*ctx*/) { safetyServerCbCount++; },
        nullptr);
    auto safetyIfRes = safetyServerIf.begin();
    if (safetyIfRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[setup] Safety-timeout isolated SERVER interface init failed");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    Modbus::Logger::logf("[setup] Safety-timeout isolated SERVER ready on port %u\n",
                         (unsigned)SAFETY_TEST_PORT);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Run tests
    UNITY_BEGIN();
    
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
    
    RUN_TEST(test_timeout_client_interface);
    RUN_TEST(test_timeout_server);
    RUN_TEST(test_server_safety_timeout_clears_idle);
    RUN_TEST(test_late_response_not_matched_to_next_request);
    RUN_TEST(test_client_rx_coalesced_responses);
    RUN_TEST(test_client_no_late_write_after_reconnect);
    RUN_TEST(test_client_destroyed_while_interface_holds_request);
    RUN_TEST(test_client_null_exception_response_rejected);
    RUN_TEST(test_interface_busy_until_tx_callback_returned);
    RUN_TEST(test_server_rx_fragmented_request);
    RUN_TEST(test_server_rx_coalesced_requests);
    RUN_TEST(test_server_rx_truncated_request_closes_connection);
    RUN_TEST(test_client_unit_id_255);
    RUN_TEST(test_server_unit_id_255);
    RUN_TEST(test_server_malformed_requests_exceptions);
    RUN_TEST(test_server_unencodable_response_exception);
    RUN_TEST(test_server_response_not_sent_to_reused_socket);
    RUN_TEST(test_server_rx_reused_socket_mid_frame);
    RUN_TEST(test_hal_rx_notifications_never_block_send);
    RUN_TEST(test_hal_tcp_keepalive);
    RUN_TEST(test_hal_send_bound_to_connection_no_reconnect);
    RUN_TEST(test_hal_stop_joins_task);
    RUN_TEST(test_interface_destroy_joins_task_and_detaches_queue);
    RUN_TEST(test_interface_recreate_under_rx_traffic);
    RUN_TEST(test_modbus_exceptions);
    RUN_TEST(test_invalid_parameters);
    RUN_TEST(test_broadcast_read_rejected);
    RUN_TEST(test_broadcast);
    // RUN_TEST(test_concurrent_calls);
    RUN_TEST(test_server_busy_exception);
    RUN_TEST(test_client_reconnect_on_first_request);

    // Multi-interface server tests
    // Placed here (not in test_rtu_server_loopback) because the only
    // way to test it properly is to use TCP+RTU simultaneously
    RUN_TEST(test_multi_interface_server_reqmutex_maxtimeout);
    RUN_TEST(test_multi_interface_server_reqmutex_nolock);
    RUN_TEST(test_multi_interface_server_overflow);

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