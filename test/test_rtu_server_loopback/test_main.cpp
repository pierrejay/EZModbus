#include <Arduino.h>
#include <unity.h>
#include <EZModbus.h>
#include "test_helpers.h"
#include "ModbusTestAgent.h"
#include "ModbusTestClient.h"
#include <utils/ModbusDebug.hpp>

// ESP32 Logger compatibility wrapper
namespace Modbus {
namespace Logger {
    static ModbusTypeDef::Mutex logMutex;
    
    void logf(const char* fmt, ...) {
        ModbusTypeDef::Lock guard(logMutex);
        va_list args;
        va_start(args, fmt);
        Serial.printf(fmt, args);
        va_end(args);
    }
    
    void logln(const char* msg = "") {
        ModbusTypeDef::Lock guard(logMutex);
        Serial.printf("%s\n", msg);
    }
    
    void waitQueueFlushed() {
        Serial.flush();
    }
}
}

// ESP32 Arduino print function for EZModbus debug output
int ESP32_LogPrint_Serial(const char* msg, size_t len) {
    ModbusTypeDef::Lock guard(Modbus::Logger::logMutex);
    size_t written = Serial.write((const uint8_t*)msg, len);
    Serial.flush();
    return (written > 0) ? written : -1;
}

// Aliases for convenience
using UART = ModbusHAL::UART;
using UARTConfig = ModbusHAL::UART::Config;

// Pin definitions
#define MBT_RX D7 // ModbusTest RX
#define MBT_TX D8 // ModbusTest TX
#define EZM_RX D5 // EZModbus RX
#define EZM_TX D6 // EZModbus TX

// Serial ports definitions
#define EZM_SERIAL Serial1
#define MBT_SERIAL Serial2

// Create our test agent and client
ModbusTestAgent agent(MBT_SERIAL, 20000);
ModbusTestClient testClient(agent); 

#define HOLDING_REGISTERS 3
#define INPUT_REGISTERS 4
#define COILS 0
#define DISCRETE_INPUTS 1

// EZModbus RTU server with StaticWordStore
UARTConfig ezmConfig = {
    .serial = Serial1,
    .baud = 9600,
    .config = SERIAL_8N1,
    .rxPin = EZM_RX,
    .txPin = EZM_TX,
    .dePin = -1
};
UART ezm_uart(ezmConfig);
ModbusInterface::RTU rtu(ezm_uart, Modbus::SLAVE);
Modbus::StaticWordStore<150> staticStore;  // Stack-allocated store for test words
Modbus::Server server(rtu, staticStore, 1, true);  // NEW: External WordStore

// Tasks
TaskHandle_t ezmTaskHandle = NULL;
bool ezmTaskInitialized = false;

void flushSerialBuffer(HardwareSerial& port) {
    while (port.available()) {
        port.read();
    }
}

// EZModbus server task
void EZModbusServerTask(void* pModbusServer) {
    Modbus::Server* srv = (Modbus::Server*)pModbusServer;
    auto srvInitRes = srv->begin();
    if (srvInitRes != Modbus::Server::SUCCESS) {
        Modbus::Logger::logln("[EZModbusServerTask] EZModbus Server initialization failed");
        vTaskDelete(NULL);
        return;
    }
    Modbus::Logger::logln("[EZModbusServerTask] EZModbus Server (specific SlaveID) initialized.");

    while (true) {
        if (pModbusServer == &server) {
             ezmTaskInitialized = true;
        }
        // Server polling is handled by interface callbacks - no explicit poll needed
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setUp() {
    // Clear both RX buffers before each test
    vTaskDelay(pdMS_TO_TICKS(10));
    ezm_uart.flush_input();
    flushSerialBuffer(MBT_SERIAL);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void tearDown() {
    server.clearAllWords();
    Serial.flush(); // Make sure all Unity logs are printed
}

// ===================================================================================
// REGISTER STORE TESTS
// ===================================================================================

void test_add_single_registers() {
    Modbus::Logger::logln("\nTEST_ADD_SINGLE_REGISTERS");
    // Define 1 register of each type
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i(Modbus::INPUT_REGISTER, 1);
    DummyRegister c(Modbus::COIL, 1);
    DummyRegister d(Modbus::DISCRETE_INPUT, 1);
    // Add registers with single add command
    do_test_add_register(server, h.word, true, "holding add failed");
    do_test_add_register(server, i.word, true, "input add failed");
    do_test_add_register(server, c.word, true, "coil add failed");
    do_test_add_register(server, d.word, true, "discrete input add failed");
}

void test_add_multiple_registers() {
    Modbus::Logger::logln("\nTEST_ADD_MULTIPLE_REGISTERS");
    // Define 5 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER>   h(10,5);
    DummyRegSuite<Modbus::INPUT_REGISTER>     i(20,5);
    DummyRegSuite<Modbus::COIL>               c(30,5);
    DummyRegSuite<Modbus::DISCRETE_INPUT>     d(40,5);
    // Add registers with multiple add command
    do_test_add_registers(server, h.handle, true, "5 holding add failed");
    do_test_add_registers(server, i.handle, true, "5 input add failed");
    do_test_add_registers(server, c.handle, true, "5 coil add failed");
    do_test_add_registers(server, d.handle, true, "5 discrete input add failed");
}

void test_add_no_read_callback() {
    Modbus::Logger::logln("\nTEST_ADD_NO_READ_CALLBACK");
    // Define a Word with no read handler and no direct pointer
    ServerWord h;
    h.type = Modbus::HOLDING_REGISTER;
    h.startAddr = 1;
    h.nbRegs = 1;
    h.value = nullptr;       // No direct pointer
    h.readHandler = nullptr; // No read handler
    h.writeHandler = nullptr;
    // Add the Word, this should fail
    do_test_add_word(server, h, false, "no readHandler add failed");
}

void test_add_readonly_with_write_callback() {
    Modbus::Logger::logln("\nTEST_ADD_READONLY_WITH_WRITE_CALLBACK");
    // Define readonly Words with write handlers (should fail)
    ServerWord i, d;
    
    i.type = Modbus::INPUT_REGISTER;
    i.startAddr = 1;
    i.nbRegs = 1;
    i.value = nullptr;
    i.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        outVals[0] = 42;
        return Modbus::NULL_EXCEPTION;
    };
    i.writeHandler = [](const uint16_t* writeVals, const ServerWord& w, void* userCtx) -> Modbus::ExceptionCode {
        return Modbus::NULL_EXCEPTION; // Should not be allowed for read-only type
    };
    
    d.type = Modbus::DISCRETE_INPUT;
    d.startAddr = 1;
    d.nbRegs = 1;
    d.value = nullptr;
    d.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        outVals[0] = 1;
        return Modbus::NULL_EXCEPTION;
    };
    d.writeHandler = [](const uint16_t* writeVals, const ServerWord& w, void* userCtx) -> Modbus::ExceptionCode {
        return Modbus::NULL_EXCEPTION; // Should not be allowed for read-only type
    };
    
    // Add the Words, should fail
    do_test_add_word(server, i, false, "readonly with writeHandler add failed");
    do_test_add_word(server, d, false, "readonly with writeHandler add failed");
}

void test_add_write_without_write_callback() {
    Modbus::Logger::logln("\nTEST_ADD_WRITE_WITHOUT_WRITE_CALLBACK");
    // Define writable Words with read handler but no write handler (should fail)
    ServerWord h, c;
    
    h.type = Modbus::HOLDING_REGISTER;
    h.startAddr = 1;
    h.nbRegs = 1;
    h.value = nullptr;
    h.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        outVals[0] = 42;
        return Modbus::NULL_EXCEPTION;
    };
    h.writeHandler = nullptr; // Missing write handler for writable type
    
    c.type = Modbus::COIL;
    c.startAddr = 1;
    c.nbRegs = 1;
    c.value = nullptr;
    c.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        outVals[0] = 1;
        return Modbus::NULL_EXCEPTION;
    };
    c.writeHandler = nullptr; // Missing write handler for writable type
    
    // Add the Words, should fail
    do_test_add_word(server, h, false, "write without writeHandler add failed");
    do_test_add_word(server, c, false, "write without writeHandler add failed");
}

void test_register_bad_type() {
    Modbus::Logger::logln("\nTEST_REGISTER_BAD_TYPE");
    // Define a Word with a bad type (cast a random value to the RegisterType enum)
    const Modbus::RegisterType badType = static_cast<Modbus::RegisterType>(42);
    DummyRegister b(badType, 1);
    // Add the Word, should fail
    do_test_add_register(server, b.word, false, "bad type add failed");
}

// ===================================================================================
// NEW WORD-SPECIFIC TESTS  
// ===================================================================================

void test_direct_pointer_vs_handlers() {
    Modbus::Logger::logln("\nTEST_DIRECT_POINTER_VS_HANDLERS");
    // Test single-register Words with direct pointer vs handlers
    DummyWord1Reg h1(Modbus::HOLDING_REGISTER, 10, true);  // Direct pointer
    DummyWord1Reg h2(Modbus::HOLDING_REGISTER, 11, false); // Handlers
    
    do_test_add_word(server, h1.word, true, "direct pointer Word add failed");
    do_test_add_word(server, h2.word, true, "handler Word add failed");
    
    // Both should work for single-register Words
    auto w1 = server.getWord(Modbus::HOLDING_REGISTER, 10);
    auto w2 = server.getWord(Modbus::HOLDING_REGISTER, 11);
    TEST_ASSERT_TRUE(w1);
    TEST_ASSERT_TRUE(w2);
}

void test_multi_register_word_validation() {
    Modbus::Logger::logln("\nTEST_MULTI_REGISTER_WORD_VALIDATION");
    
    // Test that multi-register Words cannot use direct pointers
    volatile uint16_t testValue = 42;
    ServerWord invalidWord;
    invalidWord.type = Modbus::HOLDING_REGISTER;
    invalidWord.startAddr = 20;
    invalidWord.nbRegs = 3; // Multi-register
    invalidWord.value = &testValue; // Direct pointer not allowed for multi-reg
    invalidWord.readHandler = nullptr;
    invalidWord.writeHandler = nullptr;
    
    // Should fail due to thread safety validation
    do_test_add_word(server, invalidWord, false, "multi-reg with direct pointer should fail");
    
    // Test that multi-register Words require handlers
    ServerWord noHandlerWord;
    noHandlerWord.type = Modbus::HOLDING_REGISTER;
    noHandlerWord.startAddr = 25;
    noHandlerWord.nbRegs = 2; // Multi-register  
    noHandlerWord.value = nullptr;
    noHandlerWord.readHandler = nullptr; // Missing required handler
    noHandlerWord.writeHandler = nullptr;
    
    // Should fail due to missing handlers
    do_test_add_word(server, noHandlerWord, false, "multi-reg without handlers should fail");
    
    // Test valid multi-register Word
    DummyWordMultiReg validMulti(Modbus::HOLDING_REGISTER, 30, 4, 100);
    do_test_add_word(server, validMulti.word, true, "valid multi-reg Word should work");
}

void test_word_atomicity() {
    Modbus::Logger::logln("\nTEST_WORD_ATOMICITY");
    // Test that Words provide atomic read/write operations
    DummyWordMultiReg multiWord(Modbus::HOLDING_REGISTER, 40, 3, 200);
    do_test_add_word(server, multiWord.word, true, "atomic Word add failed");
    
    // The actual atomicity is tested through read/write operations
    // This is more of a structural test ensuring the Word is set up correctly
    auto retrievedWord = server.getWord(Modbus::HOLDING_REGISTER, 40);
    TEST_ASSERT_TRUE(retrievedWord);
    TEST_ASSERT_EQUAL(3, retrievedWord.nbRegs);
    TEST_ASSERT_NOT_NULL(retrievedWord.readHandler);
    TEST_ASSERT_NOT_NULL(retrievedWord.writeHandler);
}

void test_get_register() {
    Modbus::Logger::logln("\nTEST_GET_REGISTER");
    // Add Words (single-register, like old registers)
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i(Modbus::INPUT_REGISTER, 1);
    DummyRegister c(Modbus::COIL, 1);
    DummyRegister d(Modbus::DISCRETE_INPUT, 1);
    do_test_add_register(server, h.word, true, "holding add failed");
    do_test_add_register(server, i.word, true, "input add failed");
    do_test_add_register(server, c.word, true, "coil add failed");
    do_test_add_register(server, d.word, true, "discrete input add failed");
    // Test get Word
    TEST_ASSERT_TRUE(server.getWord(Modbus::HOLDING_REGISTER, 1));
    TEST_ASSERT_TRUE(server.getWord(Modbus::INPUT_REGISTER, 1));
    TEST_ASSERT_TRUE(server.getWord(Modbus::COIL, 1));
    TEST_ASSERT_TRUE(server.getWord(Modbus::DISCRETE_INPUT, 1));
    // Test get non-existing Word
    TEST_ASSERT_FALSE(server.getWord(Modbus::HOLDING_REGISTER, 2));
    TEST_ASSERT_FALSE(server.getWord(Modbus::INPUT_REGISTER, 2));
    TEST_ASSERT_FALSE(server.getWord(Modbus::COIL, 2));
    TEST_ASSERT_FALSE(server.getWord(Modbus::DISCRETE_INPUT, 2));
}

void test_clear_all_registers() {
    Modbus::Logger::logln("\nTEST_CLEAR_ALL_REGISTERS");
    // Add registers
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(10,5);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(20,5);
    DummyRegSuite<Modbus::COIL> c(30,5);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(40,5);
    // Clear all Words and test if all of them are cleared
    server.clearAllWords();
    for (auto& word : h.handle) TEST_ASSERT_FALSE(server.getWord(word.type, word.startAddr));
    for (auto& word : i.handle) TEST_ASSERT_FALSE(server.getWord(word.type, word.startAddr));
    for (auto& word : c.handle) TEST_ASSERT_FALSE(server.getWord(word.type, word.startAddr));
    for (auto& word : d.handle) TEST_ASSERT_FALSE(server.getWord(word.type, word.startAddr));
}

void test_add_registers_atomicity() {
    Modbus::Logger::logln("\nTEST_ADD_REGISTERS_ATOMICITY");
    // Test atomic Word addition - if one Word is invalid, none should be added
    std::vector<ServerWord> words;
    
    // Create valid Words
    DummyWord1Reg h1(Modbus::HOLDING_REGISTER, 10);
    DummyWord1Reg h2(Modbus::HOLDING_REGISTER, 11);
    words.push_back(h1.word);
    words.push_back(h2.word);
    
    // Create invalid Word (missing handlers and no direct pointer)
    ServerWord invalidWord;
    invalidWord.type = Modbus::HOLDING_REGISTER;
    invalidWord.startAddr = 12;
    invalidWord.nbRegs = 1;
    invalidWord.value = nullptr;        // No direct pointer
    invalidWord.readHandler = nullptr;  // No read handler
    invalidWord.writeHandler = nullptr;
    words.push_back(invalidWord);
    
    // Try to add Words, this must fail (atomicity)
    do_test_add_words(server, words, false, "Words with 1 invalid should fail atomically");
    
    // Check that NO Words were added
    TEST_ASSERT_FALSE(server.getWord(Modbus::HOLDING_REGISTER, 10));
    TEST_ASSERT_FALSE(server.getWord(Modbus::HOLDING_REGISTER, 11));
    TEST_ASSERT_FALSE(server.getWord(Modbus::HOLDING_REGISTER, 12));
}

/*void test_register_overwrite() {
    Modbus::Logger::logln("\nTEST_REGISTER_OVERWRITE");
    // Define a set of registers
    DummyRegister h(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i(Modbus::INPUT_REGISTER, 1);
    DummyRegister c(Modbus::COIL, 1);
    DummyRegister d(Modbus::DISCRETE_INPUT, 1);
    // Define another set of registers with a different read callback returning "42"
    DummyRegister h2(Modbus::HOLDING_REGISTER, 1);
    DummyRegister i2(Modbus::INPUT_REGISTER, 1);
    DummyRegister c2(Modbus::COIL, 1);
    DummyRegister d2(Modbus::DISCRETE_INPUT, 1);
    h2.reg.readCb = [](const ServerWord& r) -> uint16_t { return (uint16_t)42; };
    i2.reg.readCb = [](const ServerWord& r) -> uint16_t { return (uint16_t)42; };
    c2.reg.readCb = [](const ServerWord& r) -> uint16_t { return (uint16_t)42; };
    d2.reg.readCb = [](const ServerWord& r) -> uint16_t { return (uint16_t)42; };
    // Add the first set of registers
    do_test_add_register(server, h.word, true, "holding add failed");
    do_test_add_register(server, i.word, true, "input add failed");
    do_test_add_register(server, c.word, true, "coil add failed");
    do_test_add_register(server, d.word, true, "discrete input add failed");
    // Add the second set of registers
    do_test_add_register(server, h2.word, true, "overwrite holding add failed");
    do_test_add_register(server, i2.word, true, "overwrite input add failed");
    do_test_add_register(server, c2.word, true, "overwrite coil add failed");
    do_test_add_register(server, d2.word, true, "overwrite discrete input add failed");
    // Check that the second set of registers overwrote the first set
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(42, h2.reg.readCb(server.getWord(Modbus::HOLDING_REGISTER, 1)));
    TEST_ASSERT_EQUAL(42, i2.reg.readCb(server.getWord(Modbus::INPUT_REGISTER, 1)));
    TEST_ASSERT_EQUAL(42, c2.reg.readCb(server.getWord(Modbus::COIL, 1)));
    TEST_ASSERT_EQUAL(42, d2.reg.readCb(server.getWord(Modbus::DISCRETE_INPUT, 1)));
}*/

// DEFINES FOR TEST_CONCURRENT_ADD_REGISTERS

// Semaphores for barrier synchronization
static SemaphoreHandle_t readySem;
static SemaphoreHandle_t startSem;
static SemaphoreHandle_t doneSem;
// Results of the concurrent add
static int addResults[2];
// Register to add concurrently
static DummyRegister concurrentReg(Modbus::HOLDING_REGISTER, 42);

void addRegisterTask(void* pvParameters) {
    uintptr_t idx = (uintptr_t)pvParameters;
    // 1) Signal that we are ready
    xSemaphoreGive(readySem);
    // 2) Wait for the start
    xSemaphoreTake(startSem, portMAX_DELAY);
    // 3) Do the concurrent add
    addResults[idx] = server.addWord(concurrentReg.word);
    // 4) Signal that we are done
    xSemaphoreGive(doneSem);
    vTaskDelete(NULL);
}

// ===================================================================================
// READ/WRITE REQUESTS TESTS
// ===================================================================================

void test_single_read_request() {
    Modbus::Logger::logln("\nTEST_SINGLE_READ_REQUEST");
    // Create 20 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, i.handle, true, "20 input add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");
    do_test_add_registers(server, d.handle, true, "20 discrete input add failed");

    // Perform a single read request for each register & register type

    for (auto& word : h.handle) {
        auto val = testClient.holdingRegisterRead(1, word.startAddr);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(*word.value, val); // Check that the read value matches the direct pointer value
    }

    for (auto& word : i.handle) {
        auto val = testClient.inputRegisterRead(1, word.startAddr);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(*word.value, val); // Check that the read value matches the direct pointer value
    }

    for (auto& word : c.handle) {
        auto val = testClient.coilRead(1, word.startAddr);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(*word.value, val); // Check that the read value matches the direct pointer value
    }

    for (auto& word : d.handle) {
        auto val = testClient.discreteInputRead(1, word.startAddr);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(*word.value, val); // Check that the read value matches the direct pointer value
    }
}

void test_single_read_requests_undefined_registers() {
    Modbus::Logger::logln("\nTEST_SINGLE_READ_REQUESTS_UNDEFINED_REGISTERS");
    // Perform a single read request for 1 register of each type (should fail because of rejectUndefined)
    int val;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    val = testClient.holdingRegisterRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.inputRegisterRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.coilRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.discreteInputRead(1, 1);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(-1, val); // Check that the read callback returns the correct value    
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_read_request() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_READ_REQUEST");
    // Create 20 individual single-register Words of each type (multi-Word request)
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, i.handle, true, "20 input add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");
    do_test_add_registers(server, d.handle, true, "20 discrete input add failed");

    // Perform a multiple read request for 5 registers of each type

    const uint16_t startAddr = 1;
    const uint16_t numRegsToRead = 10;
    int numRead;

    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of holding registers");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(startAddr + k, testClient.read(), "Incorrect holding register value");
    }

    numRead = testClient.requestFrom(1, INPUT_REGISTERS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of input registers");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(startAddr + k, testClient.read(), "Incorrect input register value");
    }

    numRead = testClient.requestFrom(1, COILS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of coils");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of coils");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.read(), "Incorrect coil value");
    }

    numRead = testClient.requestFrom(1, DISCRETE_INPUTS, startAddr, numRegsToRead);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, numRead, "requestFrom did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_MESSAGE(numRegsToRead, testClient.available(), "available() did not return the expected number of discrete inputs");
    for (uint16_t k = 0; k < numRegsToRead; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.read(), "Incorrect discrete input value");
    }
}

void test_multiple_read_request_undefined_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_READ_REQUEST_UNDEFINED_REGISTERS");
    // Perform a multiple read request for 5 register of each type (should fail because of rejectUndefined)
    int numRead;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";
    const uint16_t startAddr = 1;
    const uint16_t numRegsToRead = 5;

    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, INPUT_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, COILS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of coils");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of coils");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, DISCRETE_INPUTS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_read_request_overlapping_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_READ_REQUEST_OVERLAPPING_REGISTERS");
    // Create 4 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,4);
    DummyRegSuite<Modbus::INPUT_REGISTER> i(1,4);
    DummyRegSuite<Modbus::COIL> c(1,4);
    DummyRegSuite<Modbus::DISCRETE_INPUT> d(1,4);
    // Add them
    do_test_add_registers(server, h.handle, true, "4 holding add failed");
    do_test_add_registers(server, i.handle, true, "4 input add failed");
    do_test_add_registers(server, c.handle, true, "4 coil add failed");
    do_test_add_registers(server, d.handle, true, "4 discrete input add failed");

    // Perform a multiple read request for 5 registers of each type, should fail TOTALLY because
    // of rejectUndefined & last register is undefined (atomicity)

    int numRead;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";
    const uint16_t startAddr = 1;
    const uint16_t numRegsToRead = 5;

    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of holding registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, INPUT_REGISTERS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of input registers");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, COILS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of coils");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of coils");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    numRead = testClient.requestFrom(1, DISCRETE_INPUTS, startAddr, numRegsToRead);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "requestFrom did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.available(), "available() did not return the expected number of discrete inputs");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_single_write_request() {
    Modbus::Logger::logln("\nTEST_SINGLE_WRITE_REQUEST");
    // Create 20 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");

    // Perform a single write request for each register & register type

    int coilVal = 0;
    int holdingVal = 42;

    for (auto& reg : h.handle) {
        auto ret = testClient.holdingRegisterWrite(1, reg.startAddr, holdingVal);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(1, ret); // Check that the write suceeded
        TEST_ASSERT_EQUAL(holdingVal, (int)*reg.value);
    }

    for (auto& reg : c.handle) {
        auto ret = testClient.coilWrite(1, reg.startAddr, coilVal);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL(1, ret); // Check that the write suceeded
        TEST_ASSERT_EQUAL(coilVal, (int)*reg.value);
    }
}

void test_single_write_requests_undefined_registers() {
    Modbus::Logger::logln("\nTEST_SINGLE_WRITE_REQUESTS_UNDEFINED_REGISTERS");
    // Perform a single write request for 1 register of each type (should fail because of rejectUndefined)
    int writeVal = 1;
    int val;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    val = testClient.holdingRegisterWrite(1, 1, writeVal);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(0, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    val = testClient.coilWrite(1, 1, writeVal);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(0, val); // Check that the read callback returns the correct value
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_write_request() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_WRITE_REQUEST");
    // Create 20 individual single-register Words of each type (multi-Word request)
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,20);
    DummyRegSuite<Modbus::COIL> c(1,20);
    // Add them
    do_test_add_registers(server, h.handle, true, "20 holding add failed");
    do_test_add_registers(server, c.handle, true, "20 coil add failed");

    // Perform a multiple write request for 5 registers of each type

    int coilVal = 0;
    int holdingVal = 42;

    // Test multiple write for holding registers
    const uint16_t startAddr = 1;
    const uint16_t numRegsToWrite = 10;

    // Write holding registers
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, startAddr, numRegsToWrite), 
        "beginTransmission failed for holding registers");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(holdingVal), 
            "write failed for holding register");
    }
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(), 
        "endTransmission failed for holding registers");

    // Verify the values were written correctly
    for (int i = 0; i < numRegsToWrite; i++) {
        auto reg = server.getWord(Modbus::HOLDING_REGISTER, startAddr + i);
        TEST_ASSERT_START();
        TEST_ASSERT_TRUE(reg);
        TEST_ASSERT_EQUAL(holdingVal, (int)*reg.value);
    }

    // Write coils
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, COILS, startAddr, numRegsToWrite),
        "beginTransmission failed for coils");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(coilVal),
            "write failed for coil");
    }
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(),
        "endTransmission failed for coils");

    // Verify the values were written correctly
    for (int i = 0; i < numRegsToWrite; i++) {
        auto reg = server.getWord(Modbus::COIL, startAddr + i);
        TEST_ASSERT_START();
        TEST_ASSERT_TRUE(reg);
        TEST_ASSERT_EQUAL(coilVal, (int)*reg.value);
    }
}

void test_multiple_write_request_undefined_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_WRITE_REQUEST_UNDEFINED_REGISTERS");
    // Perform a multiple write request for 5 registers of each type

    int coilVal = 0;
    int holdingVal = 42;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    // Test multiple write for holding registers
    const uint16_t startAddr = 1;
    const uint16_t numRegsToWrite = 5;

    // Write holding registers
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, startAddr, numRegsToWrite), 
        "beginTransmission failed for holding registers");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(holdingVal), 
            "write failed for holding register");
    }
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(), 
        "endTransmission must fail for holding registers");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
    // Write coils
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, COILS, startAddr, numRegsToWrite),
        "beginTransmission failed for coils");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(coilVal),
            "write failed for coil");
    }
    
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(),
        "endTransmission must fail for coils");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

void test_multiple_write_request_overlapping_registers() {
    Modbus::Logger::logln("\nTEST_MULTIPLE_WRITE_REQUEST_OVERLAPPING_REGISTERS");
    // Create 4 registers of each type
    DummyRegSuite<Modbus::HOLDING_REGISTER> h(1,4);
    DummyRegSuite<Modbus::COIL> c(1,4);
    // Add them
    do_test_add_registers(server, h.handle, true, "4 holding add failed");
    do_test_add_registers(server, c.handle, true, "4 coil add failed");

    // Perform a multiple write request for 5 registers of each type, should fail TOTALLY because
    // of rejectUndefined & last register is undefined (atomicity)

    int coilVal = 0;
    int holdingVal = 42;
    const char* errMsg;
    const char* expectedErrMsg = "Illegal data address";

    // Test multiple write for holding registers
    const uint16_t startAddr = 1;
    const uint16_t numRegsToWrite = 5;

    // Write holding registers
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, startAddr, numRegsToWrite), 
        "beginTransmission failed for holding registers");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(holdingVal), 
            "write failed for holding register");
    }
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(), 
        "endTransmission must fail for holding registers");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);

    // Write coils
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, COILS, startAddr, numRegsToWrite),
        "beginTransmission failed for coils");
    
    for (int i = 0; i < numRegsToWrite; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(coilVal),
            "write failed for coil");
    }
    
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(),
        "endTransmission must fail for coils");

    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
}

// ===================================================================================
// WORD TESTS - Basic functionality
// ===================================================================================

void test_add_single_words() {
    Modbus::Logger::logln("\nTEST_ADD_SINGLE_WORDS");
    
    // Test 1 : Word holding simple (2 registres)
    DummyWord hw(Modbus::HOLDING_REGISTER, 100, 2, 1000);
    do_test_add_word(server, hw.word, true, "holding word add failed");
    
    // Test 2 : Word input simple (3 registres)  
    DummyWord iw(Modbus::INPUT_REGISTER, 200, 3, 2000);
    do_test_add_word(server, iw.word, true, "input word add failed");
    
    // Test 3 : Vérifier qu'on peut récupérer les Words
    auto retrievedHW = server.getWord(Modbus::HOLDING_REGISTER, 100);
    TEST_ASSERT_TRUE_MESSAGE(retrievedHW, "Failed to retrieve holding word");
    TEST_ASSERT_EQUAL_MESSAGE(100, retrievedHW.startAddr, "Wrong startAddr");
    TEST_ASSERT_EQUAL_MESSAGE(2, retrievedHW.nbRegs, "Wrong nbRegs");
    
    auto retrievedIW = server.getWord(Modbus::INPUT_REGISTER, 200);
    TEST_ASSERT_TRUE_MESSAGE(retrievedIW, "Failed to retrieve input word");
    TEST_ASSERT_EQUAL_MESSAGE(200, retrievedIW.startAddr, "Wrong startAddr");
    TEST_ASSERT_EQUAL_MESSAGE(3, retrievedIW.nbRegs, "Wrong nbRegs");
}

void test_word_address_conflicts() {
    Modbus::Logger::logln("\nTEST_WORD_ADDRESS_CONFLICTS");
    
    // Test 1 : Deux Words qui ne se chevauchent pas → OK
    DummyWord w1(Modbus::HOLDING_REGISTER, 300, 2);  // 300-301
    DummyWord w2(Modbus::HOLDING_REGISTER, 302, 2);  // 302-303
    do_test_add_word(server, w1.word, true, "non-overlapping word 1 failed");
    do_test_add_word(server, w2.word, true, "non-overlapping word 2 failed");
    
    // Test 2 : Word qui chevauche → doit être rejetée (nouveau comportement)
    DummyWord w3(Modbus::HOLDING_REGISTER, 301, 2);  // 301-302 (chevauche w1 et w2)
    do_test_add_word(server, w3.word, false, "overlapping word should be rejected");
    
    // Vérifier que w1 et w2 existent toujours
    auto retrieved1 = server.getWord(Modbus::HOLDING_REGISTER, 300);
    TEST_ASSERT_TRUE_MESSAGE(retrieved1, "Original word w1 should still exist");
    TEST_ASSERT_EQUAL_MESSAGE(300, retrieved1.startAddr, "Wrong start address for w1");
    TEST_ASSERT_EQUAL_MESSAGE(2, retrieved1.nbRegs, "Wrong register count for w1");
    
    auto retrieved2 = server.getWord(Modbus::HOLDING_REGISTER, 302);
    TEST_ASSERT_TRUE_MESSAGE(retrieved2, "Original word w2 should still exist");
    TEST_ASSERT_EQUAL_MESSAGE(302, retrieved2.startAddr, "Wrong start address for w2");
    TEST_ASSERT_EQUAL_MESSAGE(2, retrieved2.nbRegs, "Wrong register count for w2");
    
    // Vérifier que w3 n'a pas été ajoutée (getWord cherche par adresse de début exacte)
    // On doit vérifier qu'il n'y a pas de Word qui commence exactement à 301
    bool w3Exists = false;
    auto retrieved3 = server.getWord(Modbus::HOLDING_REGISTER, 301);
    if (retrieved3 && retrieved3.startAddr == 301) {
        w3Exists = true;
    }
    TEST_ASSERT_FALSE_MESSAGE(w3Exists, "Overlapping word w3 should not exist");
}

void test_word_validation() {
    Modbus::Logger::logln("\nTEST_WORD_VALIDATION");
    
    // Test 1 : Word avec type coil (maintenant supporté)
    DummyWord coilWord(Modbus::COIL, 400, 2);
    do_test_add_word(server, coilWord.word, true, "coil word should now work");
    
    // Test 1b : Word avec type discrete input (maintenant supporté)
    DummyWord discreteWord(Modbus::DISCRETE_INPUT, 450, 3);
    do_test_add_word(server, discreteWord.word, true, "discrete word should now work");
    
    // Test 2 : Word avec 0 registres
    DummyWord zeroRegs(Modbus::HOLDING_REGISTER, 400, 0);
    do_test_add_word(server, zeroRegs.word, false, "zero register word should fail");
    
    // Test 3 : Word sans read handler
    ServerWord noReadHandler;
    noReadHandler.type = Modbus::HOLDING_REGISTER;
    noReadHandler.startAddr = 400;
    noReadHandler.nbRegs = 2;
    noReadHandler.readHandler = nullptr;
    noReadHandler.writeHandler = [](const uint16_t* vals, const ServerWord& w, void* userCtx) { return Modbus::NULL_EXCEPTION; };
    do_test_add_word(server, noReadHandler, false, "word without read handler should fail");
    
    // Test 4 : Word HOLDING sans write handler
    ServerWord noWriteHandler;
    noWriteHandler.type = Modbus::HOLDING_REGISTER;
    noWriteHandler.startAddr = 400;
    noWriteHandler.nbRegs = 2;
    noWriteHandler.readHandler = [](const ServerWord& w, uint16_t* vals, void* userCtx) { return Modbus::NULL_EXCEPTION; };
    noWriteHandler.writeHandler = nullptr;
    do_test_add_word(server, noWriteHandler, false, "holding word without write handler should fail");
    
    // Test 5 : Word INPUT avec write handler (read-only)
    ServerWord readOnlyWithWrite;
    readOnlyWithWrite.type = Modbus::INPUT_REGISTER;
    readOnlyWithWrite.startAddr = 400;
    readOnlyWithWrite.nbRegs = 2;
    readOnlyWithWrite.readHandler = [](const ServerWord& w, uint16_t* vals, void* userCtx) { return Modbus::NULL_EXCEPTION; };
    readOnlyWithWrite.writeHandler = [](const uint16_t* vals, const ServerWord& w, void* userCtx) { return Modbus::NULL_EXCEPTION; };
    do_test_add_word(server, readOnlyWithWrite, false, "input word with write handler should fail");
}

void test_word_read_requests() {
    // TODO: Temporarily commented out - uses readHoldingRegisters and lastErrorOccurred methods not implemented in ModbusTestClient
    /*
    Modbus::Logger::logln("\nTEST_WORD_READ_REQUESTS");
    
    // Setup : Créer un Word de test
    DummyWord testWord(Modbus::HOLDING_REGISTER, 500, 4, 7000);  // 500-503, valeurs 7000-7003
    do_test_add_word(server, testWord.word, true, "test word setup failed");
    
    // Test 1 : Lecture complète du Word (4 registres) → doit marcher
    std::vector<uint16_t> readVals = testClient.readHoldingRegisters(1, 500, 4);
    TEST_ASSERT_FALSE_MESSAGE(testClient.lastErrorOccurred(), "Full word read should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(4, readVals.size(), "Wrong read size");
    TEST_ASSERT_EQUAL_MESSAGE(7000, readVals[0], "Wrong value at offset 0");
    TEST_ASSERT_EQUAL_MESSAGE(7001, readVals[1], "Wrong value at offset 1");
    TEST_ASSERT_EQUAL_MESSAGE(7002, readVals[2], "Wrong value at offset 2");
    TEST_ASSERT_EQUAL_MESSAGE(7003, readVals[3], "Wrong value at offset 3");
    
    // Test 2 : Lecture partielle (2 sur 4) → doit échouer avec ILLEGAL_DATA_ADDRESS
    readVals = testClient.readHoldingRegisters(1, 500, 2);
    TEST_ASSERT_TRUE_MESSAGE(testClient.lastErrorOccurred(), "Partial word read should fail");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("ILLEGAL_DATA_ADDRESS", testClient.lastError(), "Wrong error for partial read");
    
    // Test 3 : Lecture débordant le Word → doit échouer  
    readVals = testClient.readHoldingRegisters(1, 502, 3);  // 502-504, mais Word finit à 503
    TEST_ASSERT_TRUE_MESSAGE(testClient.lastErrorOccurred(), "Overflowing word read should fail");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("ILLEGAL_DATA_ADDRESS", testClient.lastError(), "Wrong error for overflow read");
    
    // Test 4 : Lecture commençant avant le Word → doit échouer
    readVals = testClient.readHoldingRegisters(1, 499, 2);  // 499-500, mais Word commence à 500
    TEST_ASSERT_TRUE_MESSAGE(testClient.lastErrorOccurred(), "Underflowing word read should fail");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("ILLEGAL_DATA_ADDRESS", testClient.lastError(), "Wrong error for underflow read");
    */
}

void test_word_write_requests() {
    // TODO: Temporarily commented out - uses writeHoldingRegisters and lastErrorOccurred methods not implemented in ModbusTestClient
    /*
    Modbus::Logger::logln("\nTEST_WORD_WRITE_REQUESTS");
    
    // Setup : Créer un Word de test
    DummyWord testWord(Modbus::HOLDING_REGISTER, 600, 4, 8000);  // 600-603, valeurs 8000-8003
    do_test_add_word(server, testWord.word, true, "test word setup failed");
    
    // Test 1 : Écriture complète → doit marcher
    std::vector<uint16_t> writeVals = {9000, 9001, 9002, 9003};
    bool writeOk = testClient.writeHoldingRegisters(1, 600, writeVals);
    TEST_ASSERT_TRUE_MESSAGE(writeOk, "Full word write should succeed");
    TEST_ASSERT_FALSE_MESSAGE(testClient.lastErrorOccurred(), "Full word write should not error");
    
    // Vérifier que les valeurs ont changé
    std::vector<uint16_t> readVals = testClient.readHoldingRegisters(1, 600, 4);
    TEST_ASSERT_FALSE_MESSAGE(testClient.lastErrorOccurred(), "Read after write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(9000, readVals[0], "Write verification failed at offset 0");
    TEST_ASSERT_EQUAL_MESSAGE(9001, readVals[1], "Write verification failed at offset 1");
    TEST_ASSERT_EQUAL_MESSAGE(9002, readVals[2], "Write verification failed at offset 2");
    TEST_ASSERT_EQUAL_MESSAGE(9003, readVals[3], "Write verification failed at offset 3");
    
    // Test 2 : Écriture partielle (2 sur 4) → doit échouer avec ILLEGAL_DATA_ADDRESS
    std::vector<uint16_t> partialWrite = {9999, 9998};
    writeOk = testClient.writeHoldingRegisters(1, 600, partialWrite);
    TEST_ASSERT_FALSE_MESSAGE(writeOk, "Partial word write should fail");
    TEST_ASSERT_TRUE_MESSAGE(testClient.lastErrorOccurred(), "Partial word write should error");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("ILLEGAL_DATA_ADDRESS", testClient.lastError(), "Wrong error for partial write");
    
    // Test 3 : Écriture d'un seul registre dans le Word → doit échouer
    writeOk = testClient.writeHoldingRegister(1, 601, 5555);
    TEST_ASSERT_FALSE_MESSAGE(writeOk, "Single register write in word should fail");
    TEST_ASSERT_TRUE_MESSAGE(testClient.lastErrorOccurred(), "Single register write should error");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("ILLEGAL_DATA_ADDRESS", testClient.lastError(), "Wrong error for single write");
    */
}

void test_word_handler_failures() {
    Modbus::Logger::logln("\nTEST_WORD_HANDLER_FAILURES");
    
    // Test 1 : Word avec handler de lecture qui échoue
    FailingReadWord failRead(Modbus::HOLDING_REGISTER, 700, 2);
    do_test_add_word(server, failRead.word, true, "failing read word add failed");
    
    // Tentative de lecture - doit échouer avec SLAVE_DEVICE_FAILURE
    int numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 700, 2);
    const char* errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "Failing read handler should cause error");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("Slave device failure", errMsg, "Wrong error for read handler failure");
    
    // Test 2 : Word avec handler d'écriture qui échoue
    FailingWriteWord failWrite(800, 2, 5000);
    do_test_add_word(server, failWrite.word, true, "failing write word add failed");
    
    // D'abord vérifier qu'on peut lire les valeurs initiales
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 800, 2);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(2, numRead, "Read before write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(5000, testClient.read(), "Initial value 0 wrong");
    TEST_ASSERT_EQUAL_MESSAGE(5001, testClient.read(), "Initial value 1 wrong");
    
    // Tentative d'écriture - doit échouer avec SLAVE_DEVICE_FAILURE
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, 800, 2),
        "beginTransmission should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(6000), "write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(6001), "write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(),
        "Failing write handler should fail at endTransmission");
    
    errMsg = testClient.lastError();
    TEST_ASSERT_EQUAL_STRING_MESSAGE("Slave device failure", errMsg, "Wrong error for write handler failure");
}

void test_float_conversion_utils() {
    Modbus::Logger::logln("\nTEST_FLOAT_CONVERSION_UTILS");
    
    // Test 1 : Conversion float vers registres et retour
    float originalValue = 123.456f;
    uint16_t registers[2];
    
    // Convertir float vers registres
    ModbusCodec::floatToRegisters(originalValue, registers);
    
    // Convertir registres vers float
    float convertedValue = ModbusCodec::registersToFloat(registers);
    
    // Vérifier que la valeur est identique (IEEE 754 exact)
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(originalValue, convertedValue, "Float conversion round-trip failed");
    
    // Test 2 : Cas particuliers
    float specialValues[] = {0.0f, -0.0f, 1.0f, -1.0f, 3.14159f, -999.999f};
    
    for (size_t i = 0; i < sizeof(specialValues) / sizeof(float); i++) {
        ModbusCodec::floatToRegisters(specialValues[i], registers);
        float result = ModbusCodec::registersToFloat(registers);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(specialValues[i], result, "Special value conversion failed");
    }
}

void test_float_word_integration() {
    Modbus::Logger::logln("\nTEST_FLOAT_WORD_INTEGRATION");
    
    // Setup : Variable float et Word associé
    static float temperature = 25.5f;
    
    Modbus::Word floatWord;
    floatWord.type = Modbus::HOLDING_REGISTER;
    floatWord.startAddr = 1000;
    floatWord.nbRegs = 2;
    
    floatWord.readHandler = [](const ServerWord& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        if (word.nbRegs != 2) return Modbus::ILLEGAL_DATA_VALUE;
        float* temp = (float*)userCtx;
        ModbusCodec::floatToRegisters(*temp, outVals);
        return Modbus::NULL_EXCEPTION;
    };
    
    floatWord.writeHandler = [](const uint16_t* writeVals, const ServerWord& word, void* userCtx) -> Modbus::ExceptionCode {
        if (word.nbRegs != 2) return Modbus::ILLEGAL_DATA_VALUE;
        float* temp = (float*)userCtx;
        *temp = ModbusCodec::registersToFloat(writeVals);
        return Modbus::NULL_EXCEPTION;
    };

    floatWord.userCtx = (void*)&temperature;
    
    do_test_add_word(server, floatWord, true, "float word add failed");
    
    // Test 1 : Lecture du float via Modbus
    int numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 1000, 2);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(2, numRead, "Float word read should succeed");
    
    // Lire les deux registres et reconvertir en float
    uint16_t readRegs[2];
    readRegs[0] = testClient.read();
    readRegs[1] = testClient.read();
    
    // Vérifier que la conversion est correcte
    uint16_t expectedRegs[2];
    ModbusCodec::floatToRegisters(25.5f, expectedRegs);
    TEST_ASSERT_EQUAL_MESSAGE(expectedRegs[0], readRegs[0], "Float upper register wrong");
    TEST_ASSERT_EQUAL_MESSAGE(expectedRegs[1], readRegs[1], "Float lower register wrong");
    
    float readBackValue = ModbusCodec::registersToFloat(readRegs);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(25.5f, readBackValue, "Read float value mismatch");
    
    // Test 2 : Écriture d'un nouveau float via Modbus
    float newValue = -123.789f;
    uint16_t writeRegs[2];
    ModbusCodec::floatToRegisters(newValue, writeRegs);
    
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, 1000, 2),
        "beginTransmission should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(writeRegs[0]), "write reg[0] should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(writeRegs[1]), "write reg[1] should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(),
        "Float word write should succeed at endTransmission");
    
    // Vérifier que la variable a été mise à jour
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(newValue, temperature, "Temperature variable not updated correctly");
    
    // Test 3 : Relire pour vérifier la persistance
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 1000, 2);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(2, numRead, "Float word re-read should succeed");
    
    readRegs[0] = testClient.read();
    readRegs[1] = testClient.read();
    readBackValue = ModbusCodec::registersToFloat(readRegs);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(newValue, readBackValue, "Read-back value mismatch");
}

void test_unified_word_validation() {
    Modbus::Logger::logln("\nTEST_UNIFIED_WORD_VALIDATION");
    
    // Test 1 : Tous les types de registres sont maintenant supportés pour Words
    DummyWord coilWord(Modbus::COIL, 1100, 1);
    do_test_add_word(server, coilWord.word, true, "single coil word should work");
    
    DummyWord discreteWord(Modbus::DISCRETE_INPUT, 1200, 1);
    do_test_add_word(server, discreteWord.word, true, "single discrete word should work");
    
    DummyWord inputWord(Modbus::INPUT_REGISTER, 1300, 1);
    do_test_add_word(server, inputWord.word, true, "single input word should work");
    
    DummyWord holdingWord(Modbus::HOLDING_REGISTER, 1400, 1);
    do_test_add_word(server, holdingWord.word, true, "single holding word should work");
    
    // Test 2 : Multi-register Words avec handlers (requis pour thread safety)
    DummyWord multiCoil(Modbus::COIL, 1500, 4);
    do_test_add_word(server, multiCoil.word, true, "multi-coil word with handlers should work");
    
    DummyWord multiHolding(Modbus::HOLDING_REGISTER, 1600, 3);
    do_test_add_word(server, multiHolding.word, true, "multi-holding word with handlers should work");
    
    // Test 3 : Vérifier que les Words sont récupérables via getWord
    auto retrievedCoil = server.getWord(Modbus::COIL, 1100);
    TEST_ASSERT_TRUE_MESSAGE(retrievedCoil, "Failed to retrieve coil word");
    TEST_ASSERT_EQUAL_MESSAGE(1, retrievedCoil.nbRegs, "Wrong coil word nbRegs");
    
    auto retrievedMulti = server.getWord(Modbus::HOLDING_REGISTER, 1600);
    TEST_ASSERT_TRUE_MESSAGE(retrievedMulti, "Failed to retrieve multi-holding word");
    TEST_ASSERT_EQUAL_MESSAGE(3, retrievedMulti.nbRegs, "Wrong multi-holding word nbRegs");
}

void test_second_server_on_same_interface() {
    Modbus::Logger::logln("\nTEST_SECOND_SERVER_ON_SAME_INTERFACE");

    static Modbus::StaticWordStore<50> staticStore2;
    static Modbus::Server server2(rtu, staticStore2, 3, true); // Slave ID 2 is already used by gap handling test
    TaskHandle_t ezmTaskHandle2 = NULL;
    xTaskCreatePinnedToCore(EZModbusServerTask, "EZModbusServerTask2", 16384, &server2, 5, &ezmTaskHandle2, 0);

    Modbus::Logger::logln("[test_second_server] Waiting for Server 2 to initialize...");
    vTaskDelay(pdMS_TO_TICKS(200));

    // Créer 1 Word holding sur chaque serveur
    DummyWord1Reg h1(Modbus::HOLDING_REGISTER, 1);
    DummyWord1Reg h2(Modbus::HOLDING_REGISTER, 1);
    
    // Vérifier les valeurs initiales
    TEST_ASSERT_EQUAL_MESSAGE(1, h1.value, "h1 initial value should be 1");
    TEST_ASSERT_EQUAL_MESSAGE(1, h2.value, "h2 initial value should be 1");
    
    do_test_add_word(server, h1.word, true, "add holding word to server1 failed");
    do_test_add_word(server2, h2.word, true, "add holding word to server2 failed");

    // Écrire sur le serveur 1 (slave ID 1)
    uint16_t valSrv1 = 3;
    auto ret = testClient.holdingRegisterWrite(1, 1, valSrv1);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(1, ret); // Check that the write succeeded
    
    // Vérifier la valeur via lecture
    int readVal1 = testClient.holdingRegisterRead(1, 1);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, readVal1, "Read from server1 should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(valSrv1, readVal1, "Server1 value wrong");

    // Écrire sur le serveur 2 (slave ID 3) 
    uint16_t valSrv2 = 4;
    ret = testClient.holdingRegisterWrite(3, 1, valSrv2);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(1, ret); // Check that the write succeeded
    
    // Vérifier que h2.value a été modifié
    TEST_ASSERT_EQUAL_MESSAGE(valSrv2, h2.value, "h2.value should be updated after write");
    
    // Vérifier la valeur via lecture
    int readVal2 = testClient.holdingRegisterRead(3, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, readVal2, "Read from server2 should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(valSrv2, readVal2, "Server2 value wrong");

    // Test broadcast (slave ID 0) - pas de réponse attendue
    uint16_t valBroadcast = 5;
    ret = testClient.holdingRegisterWrite(0, 1, valBroadcast);

    // Attendre un peu pour que les serveurs traitent le broadcast
    vTaskDelay(pdMS_TO_TICKS(100));

    // Vérifier que les deux serveurs ont reçu la valeur broadcast
    int readBcast1 = testClient.holdingRegisterRead(1, 1);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, readBcast1, "Read from server1 after broadcast should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(valBroadcast, readBcast1, "Server1 broadcast value wrong");
    
    int readBcast2 = testClient.holdingRegisterRead(3, 1);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, readBcast2, "Read from server2 after broadcast should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(valBroadcast, readBcast2, "Server2 broadcast value wrong");
    
    // Cleanup
    if(ezmTaskHandle2) {
        vTaskDelete(ezmTaskHandle2);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

// ===================================================================================
// CRITICAL MISSING TESTS - COMPREHENSIVE COVERAGE
// ===================================================================================

void test_partial_multi_register_word_access() {
    Modbus::Logger::logln("\nTEST_PARTIAL_MULTI_REGISTER_WORD_ACCESS");
    
    // Add a 4-register Word at addresses 100-103
    DummyWordMultiReg multiWord(Modbus::HOLDING_REGISTER, 100, 4, 1000);
    do_test_add_word(server, multiWord.word, true, "4-register word add failed");
    
    const char* expectedErrMsg = "Illegal data address";
    int numRead;
    const char* errMsg;
    
    // Test 1: Try to read partial Word (registers 100-101) - MUST FAIL
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 100, 2);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "Partial multi-register Word read should fail");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
    
    // Test 2: Try to read extending beyond Word (registers 102-105) - MUST FAIL
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 102, 4);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "Read extending beyond Word should fail");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
    
    // Test 3: Try to read starting before Word (registers 99-101) - MUST FAIL
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 99, 3);
    errMsg = testClient.lastError();
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, numRead, "Read starting before Word should fail");
    TEST_ASSERT_EQUAL_STRING(expectedErrMsg, errMsg);
    
    // Test 4: Complete Word read (registers 100-103) - MUST SUCCEED
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 100, 4);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(4, numRead, "Complete multi-register Word read should succeed");
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(1000 + i, testClient.read(), "Incorrect multi-register Word value");
    }
    
    // Test 5: Try partial write (registers 100-101) - MUST FAIL
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, 100, 2), 
        "beginTransmission should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(999), "write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(888), "write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.endTransmission(), 
        "Partial multi-register Word write should fail at endTransmission");
}

void test_mixed_single_multi_word_scenarios() {
    Modbus::Logger::logln("\nTEST_MIXED_SINGLE_MULTI_WORD_SCENARIOS");
    
    // Industrial scenario: Mix of single and multi-register Words
    // Single Words: status flags at 1, 2, 3
    DummyWord1Reg flag1(Modbus::COIL, 1);
    DummyWord1Reg flag2(Modbus::COIL, 2); 
    DummyWord1Reg flag3(Modbus::COIL, 3);
    
    // Multi-register Word: temperature sensor at 4-5
    DummyWordMultiReg tempSensor(Modbus::INPUT_REGISTER, 4, 2, 2500);
    
    // Single Words: config parameters at 6, 7
    DummyWord1Reg config1(Modbus::HOLDING_REGISTER, 6);
    DummyWord1Reg config2(Modbus::HOLDING_REGISTER, 7);
    
    // Multi-register Word: setpoints at 8-11
    DummyWordMultiReg setpoints(Modbus::HOLDING_REGISTER, 8, 4, 3000);
    
    // Add all Words
    do_test_add_word(server, flag1.word, true, "flag1 add failed");
    do_test_add_word(server, flag2.word, true, "flag2 add failed");
    do_test_add_word(server, flag3.word, true, "flag3 add failed");
    do_test_add_word(server, tempSensor.word, true, "temp sensor add failed");
    do_test_add_word(server, config1.word, true, "config1 add failed");
    do_test_add_word(server, config2.word, true, "config2 add failed");
    do_test_add_word(server, setpoints.word, true, "setpoints add failed");
    
    // Test 1: Read across single Words (coils 1-3) - MUST SUCCEED
    int numRead = testClient.requestFrom(1, COILS, 1, 3);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(3, numRead, "Read across multiple single-register Words should succeed");
    
    // Test 2: Read complete multi-register Word (input 4-5) - MUST SUCCEED  
    numRead = testClient.requestFrom(1, INPUT_REGISTERS, 4, 2);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(2, numRead, "Complete multi-register Word read should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(2500, testClient.read(), "Incorrect temp sensor value 1");
    TEST_ASSERT_EQUAL_MESSAGE(2501, testClient.read(), "Incorrect temp sensor value 2");
    
    // Test 3: Read mix single + multi (holding 6-11) - MUST SUCCEED
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 6, 6);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(6, numRead, "Read mix of single and multi Words should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(6, testClient.read(), "Incorrect config1 value");
    TEST_ASSERT_EQUAL_MESSAGE(7, testClient.read(), "Incorrect config2 value");  
    TEST_ASSERT_EQUAL_MESSAGE(3000, testClient.read(), "Incorrect setpoint value 1");
    TEST_ASSERT_EQUAL_MESSAGE(3001, testClient.read(), "Incorrect setpoint value 2");
    TEST_ASSERT_EQUAL_MESSAGE(3002, testClient.read(), "Incorrect setpoint value 3");
    TEST_ASSERT_EQUAL_MESSAGE(3003, testClient.read(), "Incorrect setpoint value 4");
}

void test_address_boundary_conditions() {
    Modbus::Logger::logln("\nTEST_ADDRESS_BOUNDARY_CONDITIONS");
    
    // Test 1: Word at address 0 (minimum address)
    DummyWord1Reg wordAtZero(Modbus::HOLDING_REGISTER, 0);
    do_test_add_word(server, wordAtZero.word, true, "Word at address 0 should succeed");
    
    // Test 2: Word at high address (near max but not overflowing)
    DummyWord1Reg wordAtHigh(Modbus::HOLDING_REGISTER, 65530);
    do_test_add_word(server, wordAtHigh.word, true, "Word at high address should succeed");
    
    // Test 3: Multi-register Word that would overflow address space
    Modbus::Word overflowWord;
    overflowWord.type = Modbus::HOLDING_REGISTER;
    overflowWord.startAddr = 65534;
    overflowWord.nbRegs = 5;  // Would go to address 65538 (overflow)
    overflowWord.readHandler = [](const auto& w, uint16_t* out, void* userCtx) -> Modbus::ExceptionCode {
        for (uint16_t i = 0; i < w.nbRegs; ++i) out[i] = i;
        return Modbus::NULL_EXCEPTION;
    };
    
    auto res = server.addWord(overflowWord);
    TEST_ASSERT_START();
    TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, "Address overflow Word should be rejected");
    
    // Test 4: Zero-register Word
    Modbus::Word zeroRegWord;
    zeroRegWord.type = Modbus::HOLDING_REGISTER;
    zeroRegWord.startAddr = 100;
    zeroRegWord.nbRegs = 0;  // Invalid
    zeroRegWord.readHandler = [](const auto& w, uint16_t* out, void* userCtx) -> Modbus::ExceptionCode {
        return Modbus::NULL_EXCEPTION;
    };
    
    res = server.addWord(zeroRegWord);
    TEST_ASSERT_START();
    TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, "Zero-register Word should be rejected");
}

void test_batch_overlap_validation() {
    Modbus::Logger::logln("\nTEST_BATCH_OVERLAP_VALIDATION");
    
    // Test 1: Batch with internal overlaps - MUST FAIL COMPLETELY
    DummyWord1Reg word1(Modbus::HOLDING_REGISTER, 50);
    DummyWord1Reg word2(Modbus::HOLDING_REGISTER, 51);
    DummyWord1Reg word3(Modbus::HOLDING_REGISTER, 50); // Overlaps with word1
    
    std::vector<Modbus::Word> overlappingBatch = {word1.word, word2.word, word3.word};
    auto res = server.addWords(overlappingBatch);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_WORD_OVERLAP, res, "Batch with internal overlaps should fail");
    
    // Verify no Words were added
    auto retrieved1 = server.getWord(Modbus::HOLDING_REGISTER, 50);
    auto retrieved2 = server.getWord(Modbus::HOLDING_REGISTER, 51);
    TEST_ASSERT_START();
    TEST_ASSERT_FALSE_MESSAGE(retrieved1, "No Words should be added from failed batch");
    TEST_ASSERT_FALSE_MESSAGE(retrieved2, "No Words should be added from failed batch");
    
    // Test 2: Valid batch (no overlaps) - MUST SUCCEED
    DummyWord1Reg word4(Modbus::HOLDING_REGISTER, 60);
    DummyWord1Reg word5(Modbus::HOLDING_REGISTER, 61);
    DummyWordMultiReg word6(Modbus::HOLDING_REGISTER, 62, 3, 6200);
    
    std::vector<Modbus::Word> validBatch = {word4.word, word5.word, word6.word};
    res = server.addWords(validBatch);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, "Valid batch should succeed");
    
    // Test 3: Batch overlapping with existing Words - MUST FAIL
    DummyWord1Reg word7(Modbus::HOLDING_REGISTER, 61); // Overlaps with existing word5
    DummyWord1Reg word8(Modbus::HOLDING_REGISTER, 70);
    
    std::vector<Modbus::Word> conflictBatch = {word7.word, word8.word};
    res = server.addWords(conflictBatch);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_WORD_OVERLAP, res, "Batch overlapping existing should fail");
    
    // Verify word8 was not added even though it doesn't overlap
    auto retrieved8 = server.getWord(Modbus::HOLDING_REGISTER, 70);
    TEST_ASSERT_START();
    TEST_ASSERT_FALSE_MESSAGE(retrieved8, "No Words should be added from failed batch");
}

void test_industrial_control_pattern() {
    Modbus::Logger::logln("\nTEST_INDUSTRIAL_CONTROL_PATTERN");
    
    // Realistic industrial control scenario
    // Device status: single coils
    DummyWord1Reg pumpRunning(Modbus::COIL, 1);
    DummyWord1Reg valveOpen(Modbus::COIL, 2);
    DummyWord1Reg alarmActive(Modbus::COIL, 3);
    
    // Sensor readings: multi-register input registers (float values)
    DummyWordMultiReg temperature(Modbus::INPUT_REGISTER, 10, 2, 2350); // 23.50°C as int
    DummyWordMultiReg pressure(Modbus::INPUT_REGISTER, 12, 2, 1250);    // 12.50 bar as int
    DummyWordMultiReg flowRate(Modbus::INPUT_REGISTER, 14, 2, 850);     // 8.50 L/min as int
    
    // Configuration: mix of single and multi holding registers
    DummyWord1Reg operationMode(Modbus::HOLDING_REGISTER, 20);          // 0=manual, 1=auto
    DummyWordMultiReg pidSetpoint(Modbus::HOLDING_REGISTER, 21, 3, 2500); // PID parameters
    DummyWord1Reg maxRpm(Modbus::HOLDING_REGISTER, 24);
    
    // Device info: discrete inputs (read-only status)
    DummyWord1Reg powerOk(Modbus::DISCRETE_INPUT, 30);
    DummyWord1Reg commsOk(Modbus::DISCRETE_INPUT, 31);
    
    // Add all Words
    do_test_add_word(server, pumpRunning.word, true, "pump status add failed");
    do_test_add_word(server, valveOpen.word, true, "valve status add failed");
    do_test_add_word(server, alarmActive.word, true, "alarm status add failed");
    do_test_add_word(server, temperature.word, true, "temperature sensor add failed");
    do_test_add_word(server, pressure.word, true, "pressure sensor add failed");
    do_test_add_word(server, flowRate.word, true, "flow rate sensor add failed");
    do_test_add_word(server, operationMode.word, true, "operation mode add failed");
    do_test_add_word(server, pidSetpoint.word, true, "PID setpoint add failed");
    do_test_add_word(server, maxRpm.word, true, "max RPM add failed");
    do_test_add_word(server, powerOk.word, true, "power OK add failed");
    do_test_add_word(server, commsOk.word, true, "comms OK add failed");
    
    // Test 1: Read all status coils (1-3)
    int numRead = testClient.requestFrom(1, COILS, 1, 3);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(3, numRead, "Status coils read should succeed");
    
    // Test 2: Read all sensor data (10-15) - spans multiple multi-register Words
    numRead = testClient.requestFrom(1, INPUT_REGISTERS, 10, 6);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(6, numRead, "Sensor data read should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(2350, testClient.read(), "Temperature value 1");
    TEST_ASSERT_EQUAL_MESSAGE(2351, testClient.read(), "Temperature value 2");
    TEST_ASSERT_EQUAL_MESSAGE(1250, testClient.read(), "Pressure value 1");
    TEST_ASSERT_EQUAL_MESSAGE(1251, testClient.read(), "Pressure value 2");
    TEST_ASSERT_EQUAL_MESSAGE(850, testClient.read(), "Flow rate value 1");
    TEST_ASSERT_EQUAL_MESSAGE(851, testClient.read(), "Flow rate value 2");
    
    // Test 3: Read configuration (20-24) - mix of single and multi Words
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 20, 5);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(5, numRead, "Configuration read should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(20, testClient.read(), "Operation mode");
    TEST_ASSERT_EQUAL_MESSAGE(2500, testClient.read(), "PID setpoint 1");
    TEST_ASSERT_EQUAL_MESSAGE(2501, testClient.read(), "PID setpoint 2");
    TEST_ASSERT_EQUAL_MESSAGE(2502, testClient.read(), "PID setpoint 3");
    TEST_ASSERT_EQUAL_MESSAGE(24, testClient.read(), "Max RPM");
    
    // Test 4: Write operation mode (single Word write)
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, 20, 1), 
        "Operation mode write begin should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(1), "Operation mode write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(), "Operation mode write end should succeed");
    
    // Verify write
    auto retrieved = server.getWord(Modbus::HOLDING_REGISTER, 20);
    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(retrieved, "Operation mode Word should exist");
    TEST_ASSERT_EQUAL_MESSAGE(1, (int)*retrieved.value, "Operation mode should be updated");
    
    // Test 5: Try to write to read-only discrete inputs - MUST FAIL
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, testClient.beginTransmission(1, DISCRETE_INPUTS, 30, 1), 
        "Write to discrete input should fail");
}

void test_handler_priority_over_value() {
    Modbus::Logger::logln("\nTEST_HANDLER_PRIORITY_OVER_VALUE");
    
    // Create a Word with BOTH value pointer AND handler
    // Handler should take priority (universal case)
    volatile uint16_t directValue = 123;
    
    Modbus::Word mixedWord;
    mixedWord.type = Modbus::HOLDING_REGISTER;
    mixedWord.startAddr = 500;
    mixedWord.nbRegs = 1;
    mixedWord.value = &directValue;  // Direct pointer value = 123
    
    // Handler returns different value (999) to prove it takes priority
    mixedWord.readHandler = [](const auto& w, uint16_t* out, void* userCtx) -> Modbus::ExceptionCode {
        out[0] = 999;  // Handler value = 999 (different from direct value)
        return Modbus::NULL_EXCEPTION;
    };
    mixedWord.writeHandler = [](const uint16_t* vals, const auto& w, void* userCtx) -> Modbus::ExceptionCode {
        // Handler updates to 888, ignoring direct pointer
        volatile uint16_t* directValuePtr = static_cast<volatile uint16_t*>(userCtx);
        *directValuePtr = 888;
        return Modbus::NULL_EXCEPTION;
    };
    mixedWord.userCtx = (void*)&directValue;
    
    // Add the mixed Word
    do_test_add_word(server, mixedWord, true, "Mixed word (value+handler) should succeed");
    
    // Test 1: Read should return handler value (999), not direct value (123)
    int numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 500, 1);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, numRead, "Read from mixed word should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(999, testClient.read(), "Handler should take priority over direct value");
    
    // Test 2: Write should use handler, not direct pointer
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(1, HOLDING_REGISTERS, 500, 1), 
        "Write transmission should begin");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(777), "Write should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(), "Write should complete successfully");
    
    // Verify that the handler was called (sets directValue to 888), not direct write (would set to 777)
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(888, (int)directValue, "Write handler should have been called, not direct write");
    
    // Verify read still returns handler value
    numRead = testClient.requestFrom(1, HOLDING_REGISTERS, 500, 1);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, numRead, "Second read should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(999, testClient.read(), "Handler should still take priority after write");
}

// ===================================================================================
// GAP HANDLING TESTS (rejectUndefined = false)
// ===================================================================================

void test_gap_handling_comprehensive() {
    Modbus::Logger::logln("\\nTEST_GAP_HANDLING_COMPREHENSIVE");
    
    // Initialize gap test server with rejectUndefined = false at the beginning
    static Modbus::StaticWordStore<100> gapStore;
    static Modbus::Server gapTestServer(rtu, gapStore, 2, false);
    static bool gapServerInitialized = false;
    
    if (!gapServerInitialized) {
        Modbus::Logger::logln("[test_gap_handling] Initializing gap test server (SlaveID=2)...");
        auto initRes = gapTestServer.begin();
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, initRes, "Gap test server initialization failed");
        gapServerInitialized = true;
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow initialization to complete
        Modbus::Logger::logln("[test_gap_handling] Gap test server initialized successfully");
    }
    
    gapTestServer.clearAllWords();  // Ensure clean state
    
    // Test 1: Leading gap scenario
    // Word at 110-112, request 105-114
    // Expected response: [0,0,0,0,0, word_data, 0,0]
    Modbus::Logger::logln("[test_gap_handling] Testing leading gap scenario...");
    {
        DummyWord testWord(Modbus::HOLDING_REGISTER, 110, 3, 1000);  // Words at 110,111,112 with values 1000,1001,1002
        do_test_add_word(gapTestServer, testWord.word, true, "Leading gap word should be added successfully");
        
        // Request 105-114 (10 registers) from SlaveID=2 (gapTestServer)
        int numRead = testClient.requestFrom(2, HOLDING_REGISTERS, 105, 10);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL_MESSAGE(10, numRead, "Leading gap: Should read 10 registers");
        
        // Verify leading gap (105-109) is zeros
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 105 should be zero");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 106 should be zero");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 107 should be zero");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 108 should be zero");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 109 should be zero");
        
        // Verify Word data (110-112)
        TEST_ASSERT_EQUAL_MESSAGE(1000, testClient.read(), "Address 110 should be 1000");
        TEST_ASSERT_EQUAL_MESSAGE(1001, testClient.read(), "Address 111 should be 1001");
        TEST_ASSERT_EQUAL_MESSAGE(1002, testClient.read(), "Address 112 should be 1002");
        
        // Verify trailing gap (113-114) is zeros
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 113 should be zero");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 114 should be zero");
        
        gapTestServer.clearAllWords();  // Clean for next test
    }
    
    // Test 2: Trailing gap scenario
    // Word at 100-102, request 100-110
    // Expected response: [word_data, 0,0,0,0,0,0,0]
    Modbus::Logger::logln("[test_gap_handling] Testing trailing gap scenario...");
    {
        DummyWord testWord(Modbus::INPUT_REGISTER, 100, 3, 2000);  // Words at 100,101,102 with values 2000,2001,2002
        do_test_add_word(gapTestServer, testWord.word, true, "Trailing gap word should be added successfully");
        
        // Request 100-110 (11 registers) from SlaveID=2
        int numRead = testClient.requestFrom(2, INPUT_REGISTERS, 100, 11);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL_MESSAGE(11, numRead, "Trailing gap: Should read 11 registers");
        
        // Verify Word data (100-102)
        TEST_ASSERT_EQUAL_MESSAGE(2000, testClient.read(), "Address 100 should be 2000");
        TEST_ASSERT_EQUAL_MESSAGE(2001, testClient.read(), "Address 101 should be 2001");
        TEST_ASSERT_EQUAL_MESSAGE(2002, testClient.read(), "Address 102 should be 2002");
        
        // Verify trailing gap (103-110) is zeros
        for (int i = 103; i <= 110; i++) {
            TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Trailing gap should be zero");
        }
        
        gapTestServer.clearAllWords();  // Clean for next test
    }
    
    // Test 3: Total gap scenario
    // No Words, request 200-210
    // Expected response: [0,0,0,0,0,0,0,0,0,0,0]
    Modbus::Logger::logln("[test_gap_handling] Testing total gap scenario...");
    {
        // Request 200-210 (11 registers) from SlaveID=2 with no Words defined
        int numRead = testClient.requestFrom(2, DISCRETE_INPUTS, 200, 11);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL_MESSAGE(11, numRead, "Total gap: Should read 11 discrete inputs");
        
        // Verify entire response is zeros
        for (int i = 200; i <= 210; i++) {
            TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Total gap should be zero");
        }
    }
    
    // Test 4: Middle gap scenario
    // Words at 300-301 and 305-306, request 300-306
    // Expected response: [word1_data, 0,0,0, word2_data]
    Modbus::Logger::logln("[test_gap_handling] Testing middle gap scenario...");
    {
        DummyWord word1(Modbus::HOLDING_REGISTER, 300, 2, 3000);  // 300-301 with values 3000,3001
        DummyWord word2(Modbus::HOLDING_REGISTER, 305, 2, 3050);  // 305-306 with values 3050,3051
        do_test_add_word(gapTestServer, word1.word, true, "Middle gap word1 should be added successfully");
        do_test_add_word(gapTestServer, word2.word, true, "Middle gap word2 should be added successfully");
        
        // Request 300-306 (7 registers) from SlaveID=2
        int numRead = testClient.requestFrom(2, HOLDING_REGISTERS, 300, 7);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL_MESSAGE(7, numRead, "Middle gap: Should read 7 registers");
        
        // Verify first Word data (300-301)
        TEST_ASSERT_EQUAL_MESSAGE(3000, testClient.read(), "Address 300 should be 3000");
        TEST_ASSERT_EQUAL_MESSAGE(3001, testClient.read(), "Address 301 should be 3001");
        
        // Verify middle gap (302-304) is zeros
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 302 should be zero");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 303 should be zero");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 304 should be zero");
        
        // Verify second Word data (305-306)
        TEST_ASSERT_EQUAL_MESSAGE(3050, testClient.read(), "Address 305 should be 3050");
        TEST_ASSERT_EQUAL_MESSAGE(3051, testClient.read(), "Address 306 should be 3051");
        
        gapTestServer.clearAllWords();  // Clean for next test
    }
    
    // Test 5: Gap handling with write operations
    Modbus::Logger::logln("[test_gap_handling] Testing gap handling with write operations...");
    {
        DummyWord writeWord(Modbus::HOLDING_REGISTER, 400, 2, 4000);  // 400-401 with values 4000,4001
        do_test_add_word(gapTestServer, writeWord.word, true, "Gap write word should be added successfully");
        
        // Try to write across gap: 399-402 (4 registers)
        // This should only affect the defined Words (400-401), gaps should be ignored
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.beginTransmission(2, HOLDING_REGISTERS, 399, 4), 
            "Gap write: beginTransmission should succeed");
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(9999), "Write to 399 should succeed");  // Gap, ignored
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(5000), "Write to 400 should succeed");  // Word, applied
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(5001), "Write to 401 should succeed");  // Word, applied
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.write(9998), "Write to 402 should succeed");  // Gap, ignored
        TEST_ASSERT_EQUAL_MESSAGE(1, testClient.endTransmission(), 
            "Gap write: endTransmission should succeed");
        
        // Verify that only the Word data was updated
        int numRead = testClient.requestFrom(2, HOLDING_REGISTERS, 399, 4);
        TEST_ASSERT_START();
        TEST_ASSERT_EQUAL_MESSAGE(4, numRead, "Gap write verification: Should read 4 registers");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 399 should remain zero (gap)");
        TEST_ASSERT_EQUAL_MESSAGE(5000, testClient.read(), "Address 400 should be updated to 5000");
        TEST_ASSERT_EQUAL_MESSAGE(5001, testClient.read(), "Address 401 should be updated to 5001");
        TEST_ASSERT_EQUAL_MESSAGE(0, testClient.read(), "Address 402 should remain zero (gap)");
        
        gapTestServer.clearAllWords();  // Clean after test
    }
    
    Modbus::Logger::logln("[test_gap_handling] All gap handling tests completed successfully");
}


void setup() {
    // Debug port
    Serial.setTxBufferSize(2048);
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);
    
    // Configure EZModbus debug output
    #ifdef EZMODBUS_DEBUG
    Modbus::Debug::setPrintFunction(ESP32_LogPrint_Serial);
    #endif
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ModbusTestClient init
    MBT_SERIAL.begin(9600, SERIAL_8N1, MBT_RX, MBT_TX);
    vTaskDelay(pdMS_TO_TICKS(50));
    flushSerialBuffer(MBT_SERIAL);

    Modbus::Logger::logln("[setup] Initializing EzmUart...");
    esp_err_t uart_init_res = ezm_uart.begin();
    if (uart_init_res != ESP_OK) {
        Modbus::Logger::logln("[setup] EzmUart initialization failed)");
        return;
    }
    Modbus::Logger::logln("[setup] EzmUart initialized.");

    Modbus::Logger::logln("[setup] Initializing RTU interface...");
    auto ifcInitRes = rtu.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[setup] EZModbus RTU interface initialization failed");
        return;
    }
    Modbus::Logger::logln("[setup] EZModbus RTU interface initialized.");

    Modbus::Logger::logln("[setup] Creating EZModbusServerTask for main server...");
    xTaskCreatePinnedToCore(EZModbusServerTask, "EZModbusServerTask", 16384, &server, 5, &ezmTaskHandle, 0);
    while (!ezmTaskInitialized) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    Modbus::Logger::logln("[setup] EZModbus server task initialized, starting tests...");

    // Run tests
    UNITY_BEGIN();
 
    // Word API tests
    RUN_TEST(test_direct_pointer_vs_handlers);
    RUN_TEST(test_multi_register_word_validation);
    RUN_TEST(test_word_atomicity);
    RUN_TEST(test_add_no_read_callback);
    RUN_TEST(test_add_readonly_with_write_callback);
    RUN_TEST(test_add_write_without_write_callback);
    RUN_TEST(test_register_bad_type);
    RUN_TEST(test_get_register);
    RUN_TEST(test_add_single_registers);
    RUN_TEST(test_add_multiple_registers);  
    RUN_TEST(test_clear_all_registers);
    RUN_TEST(test_add_registers_atomicity);

    // Read/write requests tests - Server side uses Word API, client side uses ModbusTestClient
    RUN_TEST(test_single_read_request);
    RUN_TEST(test_single_read_requests_undefined_registers);
    RUN_TEST(test_multiple_read_request);
    RUN_TEST(test_multiple_read_request_undefined_registers);
    RUN_TEST(test_multiple_read_request_overlapping_registers);
    RUN_TEST(test_single_write_request);
    RUN_TEST(test_single_write_requests_undefined_registers);
    RUN_TEST(test_multiple_write_request);
    RUN_TEST(test_multiple_write_request_undefined_registers);
    RUN_TEST(test_multiple_write_request_overlapping_registers);

    // Word tests
    RUN_TEST(test_add_single_words);
    RUN_TEST(test_word_address_conflicts);
    RUN_TEST(test_word_validation);
    RUN_TEST(test_word_handler_failures);
    
    // Float utility tests
    RUN_TEST(test_float_conversion_utils);
    RUN_TEST(test_float_word_integration);
    
    // Edge case tests
    RUN_TEST(test_partial_multi_register_word_access);
    RUN_TEST(test_mixed_single_multi_word_scenarios);
    RUN_TEST(test_address_boundary_conditions);
    RUN_TEST(test_batch_overlap_validation);
    RUN_TEST(test_industrial_control_pattern);
    RUN_TEST(test_handler_priority_over_value);
    
    // Unified Word validation tests
    RUN_TEST(test_unified_word_validation);
    
    // Gap handling tests (rejectUndefined = false)
    RUN_TEST(test_gap_handling_comprehensive);

    // Multiple servers test
    RUN_TEST(test_second_server_on_same_interface);

    UNITY_END();
}

void loop() {
    Modbus::Logger::logln("Idling in loop()...");
    vTaskDelay(pdMS_TO_TICKS(1000));
}