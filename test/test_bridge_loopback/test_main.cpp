#include <Arduino.h>
#include <unity.h>
#include <EZModbus.h>
#include "test_params.h"
#include "dummy_interface.h"
#include <WiFi.h>
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

// Pin / port / baud definitions - overridable per-environment via build flags.
// Defaults target the Seeed XIAO ESP32-S3 loopback wiring; the *_c6 env puts the
// dummy server (MBT side) on the LP-UART (see platformio.ini).
#ifndef EZM_UART_NUM
    #define EZM_UART_NUM UART_NUM_1   // Bridge RTU master port
#endif
#ifndef EZM_RX
    #define EZM_RX D5
#endif
#ifndef EZM_TX
    #define EZM_TX D6
#endif
#ifndef MBT_UART_NUM
    #define MBT_UART_NUM UART_NUM_2   // Dummy server port (can be LP_UART_NUM_0)
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

ModbusHAL::UART ezmUart(EZM_UART_NUM, UART_BAUD_RATE, ModbusHAL::UART::CONFIG_8N1, EZM_RX, EZM_TX);
ModbusHAL::UART mbtUart(MBT_UART_NUM, UART_BAUD_RATE, ModbusHAL::UART::CONFIG_8N1, MBT_RX, MBT_TX);
ModbusHAL::TCP tcpServer(MODBUS_PORT); // Server for the bridge
WiFiClient tcpClient;             // Client for injecting requests in the bridge

// EZModbus RTU master interface that will relay requests to the test server
ModbusInterface::RTU ezm1(ezmUart, Modbus::MASTER);

// EZModbus TCP server interface that will ingest requests from the bridge end
ModbusInterface::TCP ezm2(tcpServer, Modbus::SERVER);

// The bridge
Modbus::Bridge bridge(ezm1, ezm2);

// EZModbus RTU server acting as a remote Modbus RTU server + list of registers
ModbusInterface::RTU mbt(mbtUart, Modbus::SERVER);
Modbus::DynamicWordStore wordStore(MBT_INIT_REG_COUNT * 4);
Modbus::Server server(mbt, wordStore);
uint16_t serverDiscreteInputs[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverCoils[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverHoldingRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverInputRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
// Holding register whose read takes SLOW_READ_DELAY_MS when slowReadEnabled (late response tests)
#define SLOW_READ_ADDR 6
static constexpr uint32_t SLOW_READ_DELAY_MS = 1500;
volatile bool slowReadEnabled = false;

Modbus::ReadWordHandler serverReadHandler = [](const Modbus::Word& word, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
    if (slowReadEnabled && word.type == Modbus::HOLDING_REGISTER && word.startAddr == SLOW_READ_ADDR) {
        vTaskDelay(pdMS_TO_TICKS(SLOW_READ_DELAY_MS));
    }
    switch (word.type) {
        case Modbus::HOLDING_REGISTER:
            outVals[0] = serverHoldingRegisters[word.startAddr];
            break;
        case Modbus::INPUT_REGISTER:
            outVals[0] = serverInputRegisters[word.startAddr];
            break;
        case Modbus::COIL:
            outVals[0] = serverCoils[word.startAddr];
            break;
        case Modbus::DISCRETE_INPUT:
            outVals[0] = serverDiscreteInputs[word.startAddr];
            break;
        default:
            return Modbus::ILLEGAL_DATA_ADDRESS;
    }
    return Modbus::NULL_EXCEPTION;
};
Modbus::WriteWordHandler serverWriteHandler = [](const uint16_t* writeVals, const Modbus::Word& word, void* userCtx) -> Modbus::ExceptionCode {
    switch (word.type) {
        case Modbus::HOLDING_REGISTER:
            serverHoldingRegisters[word.startAddr] = writeVals[0];
            break;
        case Modbus::COIL:
            serverCoils[word.startAddr] = writeVals[0];
            break;
        default:
            return Modbus::ILLEGAL_FUNCTION;
    }
    return Modbus::NULL_EXCEPTION;
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

    // Initialize Modbus RTU test server
    auto ifcInitRes = mbt.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[ModbusTestServerTask] EZModbus RTU interface initialization failed");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    Modbus::Logger::logln("[ModbusTestServerTask] EZModbus RTU interface initialized");
    
    // Configure registers for each type
    uint8_t regTypes[] = {
        Modbus::HOLDING_REGISTER,
        Modbus::INPUT_REGISTER,
        Modbus::COIL,
        Modbus::DISCRETE_INPUT
    };
    
    for (int i = MBT_INIT_START_REG; i < MBT_INIT_START_REG + MBT_INIT_REG_COUNT; i++) {
        for (uint8_t rt : regTypes) {
            Modbus::Word word;
            word.type = (Modbus::RegisterType)rt;
            word.startAddr = i;
            word.nbRegs = 1;
            word.value = nullptr;
            word.readHandler = serverReadHandler;
            if (rt == Modbus::HOLDING_REGISTER || rt == Modbus::COIL) {
                word.writeHandler = serverWriteHandler;
            } else {
                word.writeHandler = nullptr;
            }
            server.addWord(word);
        }
    }

    // Initialize ModbusServer
    auto srvInitRes = server.begin();
    if (srvInitRes != Modbus::Server::SUCCESS) {
        Modbus::Logger::logln("[ModbusTestServerTask] EZModbus Server initialization failed");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
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

void test_read_holding_register() {
    // Create a frame and encode it with the TCP codec
    Modbus::Frame request = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::READ_HOLDING_REGISTERS,
        .slaveId    = TEST_SLAVE_ID,
        .regAddress = READ_HOLDING_ADDR,
        .regCount   = 1,
        .data       = {}
    };
    uint8_t _rawRequest[ModbusCodec::TCP::MAX_FRAME_SIZE];
    ByteBuffer requestBytes(_rawRequest, sizeof(_rawRequest));
    ModbusCodec::TCP::encode(request, requestBytes, 0);

    // Connect our dummy client & inject the frame in the bridge
    uint32_t connStartTime = millis();
    while (!tcpClient.connected()) {
        if (millis() - connStartTime > 1000) {
            TEST_ASSERT_START();
            TEST_FAIL_MESSAGE("Timeout waiting for connection to bridge");
            return;
        }
        tcpClient.connect(LOOPBACK_IP, MODBUS_PORT);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    tcpClient.write(requestBytes.data(), requestBytes.size());

    
    uint8_t _rawResponse[ModbusCodec::TCP::MAX_FRAME_SIZE];
    ByteBuffer responseBytes(_rawResponse, sizeof(_rawResponse));
    uint32_t reqStartTime = millis();
    while (!tcpClient.available()) {
        if (millis() - reqStartTime > 1000) {
            TEST_ASSERT_START();
            TEST_FAIL_MESSAGE("Timeout waiting for response from bridge");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    while (tcpClient.available()) {
        responseBytes.push_back(tcpClient.read());
    }
    Modbus::Frame response;
    ModbusCodec::TCP::decode(responseBytes, response, Modbus::RESPONSE);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::RESPONSE, response.type);
    TEST_ASSERT_EQUAL(Modbus::READ_HOLDING_REGISTERS, response.fc);
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId);
    TEST_ASSERT_EQUAL(1, response.regCount);
    TEST_ASSERT_EQUAL(MBT_INIT_HOLDING_REGISTER_VALUE(READ_HOLDING_ADDR), response.data[0]);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, response.exceptionCode);
}

void test_write_holding_register() {
    // Create a frame and encode it with the TCP codec
    Modbus::Frame request = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::WRITE_REGISTER,
        .slaveId    = TEST_SLAVE_ID,
        .regAddress = WRITE_HOLDING_ADDR,
        .regCount   = 1,
        .data       = {42}
    };
    uint8_t _rawRequest[ModbusCodec::TCP::MAX_FRAME_SIZE];
    ByteBuffer requestBytes(_rawRequest, sizeof(_rawRequest));
    ModbusCodec::TCP::encode(request, requestBytes, 1);

    // Connect our dummy client & inject the frame in the bridge
    uint32_t connStartTime = millis();
    while (!tcpClient.connected()) {
        if (millis() - connStartTime > 1000) {
            TEST_ASSERT_START();
            TEST_FAIL_MESSAGE("Timeout waiting for connection to bridge");
            return;
        }
        tcpClient.connect(LOOPBACK_IP, MODBUS_PORT);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    tcpClient.write(requestBytes.data(), requestBytes.size());

    
    uint8_t _rawResponse[ModbusCodec::TCP::MAX_FRAME_SIZE];
    ByteBuffer responseBytes(_rawResponse, sizeof(_rawResponse));
    uint32_t reqStartTime = millis();
    while (!tcpClient.available()) {
        if (millis() - reqStartTime > 1000) {
            TEST_ASSERT_START();
            TEST_FAIL_MESSAGE("Timeout waiting for response from bridge");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    while (tcpClient.available()) {
        responseBytes.push_back(tcpClient.read());
    }
    Modbus::Frame response;
    ModbusCodec::TCP::decode(responseBytes, response, Modbus::RESPONSE);
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::RESPONSE, response.type);
    TEST_ASSERT_EQUAL(Modbus::WRITE_REGISTER, response.fc);
    TEST_ASSERT_EQUAL(TEST_SLAVE_ID, response.slaveId);
    TEST_ASSERT_EQUAL(1, response.regCount);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, response.exceptionCode);
}

// ===================================================================================
// GATEWAY BEHAVIOUR WHEN THE RTU SIDE CAN'T ANSWER
// ===================================================================================

static constexpr uint32_t BRIDGE_TIMEOUT_MS = 1000; // Bridge default RTU response timeout
#define ABSENT_SLAVE_ID 7                           // No RTU server answers this ID

static bool bridgeConnect() {
    uint32_t t0 = millis();
    while (!tcpClient.connected()) {
        if (millis() - t0 > 1000) return false;
        tcpClient.connect(LOOPBACK_IP, MODBUS_PORT);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

// Read one ADU (MBAP length-framed) from the bridge, returns its size (0 if none within timeoutMs)
static size_t bridgeReceive(uint8_t* buf, size_t cap, uint32_t timeoutMs) {
    size_t got = 0, expected = 6;
    uint32_t t0 = millis();
    while (got < expected) {
        if (millis() - t0 > timeoutMs) return 0;
        if (!tcpClient.available()) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        buf[got++] = tcpClient.read();
        if (got == 6) {
            expected = 6 + ((buf[4] << 8) | buf[5]);
            if (expected > cap) return 0;
        }
    }
    return got;
}

static bool bridgeSend(const Modbus::Frame& request, uint16_t tid) {
    uint8_t raw[ModbusCodec::TCP::MAX_FRAME_SIZE];
    ByteBuffer bytes(raw, sizeof(raw));
    if (ModbusCodec::TCP::encode(request, bytes, tid) != ModbusCodec::SUCCESS) return false;
    if (!bridgeConnect()) return false;
    while (tcpClient.available()) tcpClient.read(); // Drop leftovers from a previous exchange
    return tcpClient.write(bytes.data(), bytes.size()) == bytes.size();
}

// Check a raw TCP ADU is the exception `ec` for (tid, slaveId, fc)
static void assertGatewayException(const uint8_t* adu, size_t len, uint16_t tid, uint8_t slaveId,
                                   Modbus::FunctionCode fc, uint8_t ec) {
    TEST_ASSERT_EQUAL_MESSAGE(9, len, "expected a 9-byte exception response");
    TEST_ASSERT_EQUAL_HEX16(tid, (adu[0] << 8) | adu[1]);
    TEST_ASSERT_EQUAL_HEX8(slaveId, adu[6]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)fc | 0x80, adu[7]);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(ec, adu[8], "unexpected exception code");
}

// Regression: when the RS-485 slave doesn't answer, the bridge sent nothing back and the
// TCP side stayed busy until its 5 s safety timeout. A gateway must answer 0x0B.
void test_bridge_silent_slave_gateway_exception() {
    Modbus::Frame request = {
        .type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = ABSENT_SLAVE_ID,
        .regAddress = READ_HOLDING_ADDR, .regCount = 1, .data = {}
    };
    TEST_ASSERT_TRUE_MESSAGE(bridgeSend(request, 0x0B01), "cannot send request to bridge");
    uint32_t t0 = millis();
    uint8_t adu[ModbusCodec::TCP::MAX_FRAME_SIZE];
    size_t len = bridgeReceive(adu, sizeof(adu), BRIDGE_TIMEOUT_MS + 500);
    uint32_t elapsed = millis() - t0;

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(len > 0, "no answer from the bridge for a silent slave");
    assertGatewayException(adu, len, 0x0B01, ABSENT_SLAVE_ID, Modbus::READ_HOLDING_REGISTERS, 0x0B);
    TEST_ASSERT_TRUE_MESSAGE(elapsed >= BRIDGE_TIMEOUT_MS - 50, "0x0B sent before the bridge timeout");

    // The bridge must be ready for the next request right away
    request.slaveId = TEST_SLAVE_ID;
    TEST_ASSERT_TRUE(bridgeSend(request, 0x0B02));
    len = bridgeReceive(adu, sizeof(adu), 1000);
    TEST_ASSERT_EQUAL_MESSAGE(11, len, "next request not answered");
    TEST_ASSERT_EQUAL_HEX16(0x0B02, (adu[0] << 8) | adu[1]);
    TEST_ASSERT_EQUAL(MBT_INIT_HOLDING_REGISTER_VALUE(READ_HOLDING_ADDR), (adu[9] << 8) | adu[10]);
}

// Regression: when the request can't be forwarded to the RTU side (here Unit ID 255, valid
// over TCP but not on RTU), the bridge sent nothing back. A gateway must answer 0x0A.
void test_bridge_forward_failure_gateway_exception() {
    Modbus::Frame request = {
        .type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = 0xFF,
        .regAddress = READ_HOLDING_ADDR, .regCount = 1, .data = {}
    };
    TEST_ASSERT_TRUE_MESSAGE(bridgeSend(request, 0x0A01), "cannot send request to bridge");
    uint8_t adu[ModbusCodec::TCP::MAX_FRAME_SIZE];
    size_t len = bridgeReceive(adu, sizeof(adu), 500);

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(len > 0, "no answer from the bridge when forwarding fails");
    assertGatewayException(adu, len, 0x0A01, 0xFF, Modbus::READ_HOLDING_REGISTERS, 0x0A);
}

// A response arriving after the bridge timeout must be dropped, not forwarded to the TCP
// side, and the next request (same slave & function code) must get its own answer.
// Note: RTU has no transaction ID, so a late response can't be told apart from the answer
// to a request sent meanwhile to the same slave with the same function code: the bridge
// timeout must stay above the slowest slave's response time.
void test_bridge_late_response_not_forwarded() {
    slowReadEnabled = true;
    Modbus::Frame request = {
        .type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = TEST_SLAVE_ID,
        .regAddress = SLOW_READ_ADDR, .regCount = 1, .data = {}
    };
    TEST_ASSERT_TRUE_MESSAGE(bridgeSend(request, 0x0C01), "cannot send request to bridge");
    uint32_t t0 = millis();
    uint8_t adu[ModbusCodec::TCP::MAX_FRAME_SIZE];
    size_t len = bridgeReceive(adu, sizeof(adu), BRIDGE_TIMEOUT_MS + 300);

    // Wait for the late response to reach the bridge: nothing must come out on TCP
    while (millis() - t0 < SLOW_READ_DELAY_MS + 300) vTaskDelay(pdMS_TO_TICKS(5));
    slowReadEnabled = false;
    int leftover = tcpClient.available();

    // Next request to the same slave with the same function code
    request.regAddress = READ_HOLDING_ADDR;
    bool sent = bridgeSend(request, 0x0C02);
    uint8_t nextAdu[ModbusCodec::TCP::MAX_FRAME_SIZE];
    size_t nextLen = sent ? bridgeReceive(nextAdu, sizeof(nextAdu), BRIDGE_TIMEOUT_MS) : 0;

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(len > 0, "no answer from the bridge for a slow slave");
    assertGatewayException(adu, len, 0x0C01, TEST_SLAVE_ID, Modbus::READ_HOLDING_REGISTERS, 0x0B);
    TEST_ASSERT_EQUAL_MESSAGE(0, leftover, "late response forwarded to the TCP side");
    TEST_ASSERT_TRUE_MESSAGE(sent, "cannot send next request to bridge");
    TEST_ASSERT_EQUAL_MESSAGE(11, nextLen, "next request not answered");
    TEST_ASSERT_EQUAL_HEX16(0x0C02, (nextAdu[0] << 8) | nextAdu[1]);
    TEST_ASSERT_EQUAL(MBT_INIT_HOLDING_REGISTER_VALUE(READ_HOLDING_ADDR), (nextAdu[9] << 8) | nextAdu[10]);
}

// A request the bridge can't decode (here an unsupported function code) is answered with
// the exception required by the Modbus spec, instead of being dropped silently.
void test_bridge_malformed_request_exception() {
    if (!bridgeConnect()) { TEST_FAIL_MESSAGE("cannot connect to bridge"); return; }
    while (tcpClient.available()) tcpClient.read();
    const uint8_t adu[] = { 0x0D, 0x01, 0x00, 0x00, 0x00, 0x05, TEST_SLAVE_ID, 0x2B, 0x0E, 0x01, 0x00 };
    tcpClient.write(adu, sizeof(adu));
    uint8_t resp[ModbusCodec::TCP::MAX_FRAME_SIZE];
    size_t len = bridgeReceive(resp, sizeof(resp), 500);

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(len > 0, "no answer from the bridge for an unsupported function code");
    TEST_ASSERT_EQUAL_MESSAGE(9, len, "expected a 9-byte exception response");
    TEST_ASSERT_EQUAL_HEX16(0x0D01, (resp[0] << 8) | resp[1]);
    TEST_ASSERT_EQUAL_HEX8(0xAB, resp[7]);
    TEST_ASSERT_EQUAL_HEX8(Modbus::ILLEGAL_FUNCTION, resp[8]);
}

void setUp() {
    // Clear both RX buffers before each test
    vTaskDelay(pdMS_TO_TICKS(10));
    ezmUart.flush_input();
    mbtUart.flush_input();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Reset ModbusTestServer registers
    resetModbusTestServerRegisters();
}

void tearDown() {
    Serial.flush(); // Make sure all Unity logs are printed
}

void setup() {
    // Debug port
    Serial.setTxBufferSize(2048);
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);
    
    // Initialize UART HAL objects first (like in working RTU client/server code)
    Modbus::Logger::logln("[setup] Initializing EZModbus UART HAL...");
    esp_err_t ezmUartRes = ezmUart.begin();
    if (ezmUartRes != ESP_OK) {
        Modbus::Logger::logln("[setup] EZModbus UART HAL initialization failed");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    Modbus::Logger::logln("[setup] EZModbus UART HAL initialized");
    
    Modbus::Logger::logln("[setup] Initializing ModbusTestServer UART HAL...");
    esp_err_t mbtUartRes = mbtUart.begin();
    if (mbtUartRes != ESP_OK) {
        Modbus::Logger::logln("[setup] ModbusTestServer UART HAL initialization failed");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    Modbus::Logger::logln("[setup] ModbusTestServer UART HAL initialized");
    
    // EZModbus init
    ezm1.setSilenceTimeMs(10);

    // Initialize ModbusTestServer TCP server
    // WiFi.setSleep(WIFI_PS_NONE);
    // 1) Démarrage du SoftAP
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(SOFTAP_IP, SOFTAP_IP, IPAddress(255,255,255,0));
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Modbus::Logger::logf("[setup] SoftAP IP : %s\n", WiFi.softAPIP().toString().c_str());

    vTaskDelay(pdMS_TO_TICKS(1000));

    tcpServer.begin();

    auto bridgeInitRes = bridge.begin();
    if (bridgeInitRes != Modbus::Bridge::SUCCESS) {
        Modbus::Logger::logln("[setup] Modbus Bridge initialization failed");
        return;
    }
    Modbus::Logger::logln("[setup] Modbus Bridge initialized");

    // Run Modbus test server task on core 0 (main test thread on core 1)
    xTaskCreatePinnedToCore(ModbusTestServerTask, "ModbusTestServerTask", 16384, NULL, 5, &modbusTestServerTaskHandle, 0);
    while (!modbusTestServerTaskInitialized) {
        vTaskDelay(pdMS_TO_TICKS(1));
    } // Wait for the task to be looping
    Modbus::Logger::logln("[setup] ModbusTestServer task initialized, starting tests...");

    // Run tests
    UNITY_BEGIN();
    
    RUN_TEST(test_read_holding_register);
    RUN_TEST(test_write_holding_register);
    RUN_TEST(test_bridge_silent_slave_gateway_exception);
    RUN_TEST(test_bridge_forward_failure_gateway_exception);
    RUN_TEST(test_bridge_late_response_not_forwarded);
    RUN_TEST(test_bridge_malformed_request_exception);
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