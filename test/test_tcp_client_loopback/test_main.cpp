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

// HAL instances
ModbusHAL::TCP halForClient(LOOPBACK_IP_STR, MODBUS_PORT);
ModbusHAL::TCP halForServer(MODBUS_PORT);

// Primary EZModbus TCP client
ModbusInterface::TCP ezm(halForClient, Modbus::CLIENT);
Modbus::Client client(ezm);

// EZModbus TCP server for testing with DynamicWordStore (heap allocation)
ModbusInterface::TCP mbt(halForServer, Modbus::SERVER);
Modbus::DynamicWordStore dynamicStore(10000);  // Heap-allocated store for TCP server
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
    ssize_t freePsramBefore = BOARD_HAS_PSRAM ? ESP.getFreePsram() : 0;
    
    for (int i = MBT_INIT_START_REG; i < MBT_INIT_START_REG + MBT_INIT_REG_COUNT; i++) {
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
    ssize_t freePsramAfter = BOARD_HAS_PSRAM ? ESP.getFreePsram() : 0;
    ssize_t memoryUsed = freeHeapBefore - freeHeapAfter;
    ssize_t psramUsed = freePsramBefore - freePsramAfter;
    if (BOARD_HAS_PSRAM) {
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
    uint32_t startTime = TIME_MS();
    // client.getRequestTimeoutMs() would be better if available, using Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS as fallback
    uint32_t timeoutDuration = Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS;
    while (TIME_MS() - startTime < timeoutDuration + 100) {
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

    startTime = TIME_MS();
    while (TIME_MS() - startTime < timeoutDuration + 100) {
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
    uint32_t startTime = TIME_MS();
    uint32_t timeoutDuration = Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS + 200; // Add margin
    while (TIME_MS() - startTime < timeoutDuration) {
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
ModbusHAL::UART multiInterfaceServerRtuHal(UART_NUM_1, 9600, ModbusHAL::UART::CONFIG_8N1, D5, D6);
ModbusHAL::UART multiInterfaceClientRtuHal(UART_NUM_2, 9600, ModbusHAL::UART::CONFIG_8N1, D7, D8);
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
    // Nothing to do here
    Serial.println("Idling...");
    vTaskDelay(pdMS_TO_TICKS(1000));
}