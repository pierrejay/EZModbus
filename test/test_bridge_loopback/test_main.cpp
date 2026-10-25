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
    Modbus::ByteBuffer requestBytes(_rawRequest, sizeof(_rawRequest));
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
    Modbus::ByteBuffer responseBytes(_rawResponse, sizeof(_rawResponse));
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
    Modbus::ByteBuffer requestBytes(_rawRequest, sizeof(_rawRequest));
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
    Modbus::ByteBuffer responseBytes(_rawResponse, sizeof(_rawResponse));
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
    Modbus::ByteBuffer bytes(raw, sizeof(raw));
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

// ===================================================================================
// BRIDGE LIFECYCLE (in-memory bridges between dummy interfaces)
// The slave side injects requests, the master side swallows them (silent device).
// ===================================================================================

struct DummyMaster : public ModbusInterface::Dummy {
    DummyMaster() { _role = Modbus::MASTER; }
};
static constexpr uint32_t LOCAL_BRIDGE_TIMEOUT_MS = 100;
static const Modbus::Frame LOCAL_BRIDGE_REQUEST = {
    .type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = TEST_SLAVE_ID,
    .regAddress = READ_HOLDING_ADDR, .regCount = 1, .data = {}
};

// Regression: the timeout timer kept a pointer to the bridge and fired after its destruction
void test_bridge_destroyed_with_armed_timer() {
    static ModbusInterface::Dummy slaveSide;
    static DummyMaster masterSide;
    alignas(Modbus::Bridge) static uint8_t storage[sizeof(Modbus::Bridge)];
    auto* localBridge = new (storage) Modbus::Bridge(masterSide, slaveSide, LOCAL_BRIDGE_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Bridge::SUCCESS, localBridge->begin());

    slaveSide.transmitFrame(LOCAL_BRIDGE_REQUEST); // Forwarded, timeout armed
    Modbus::Frame frame;
    TEST_ASSERT_TRUE_MESSAGE(masterSide.getReceivedFrame(frame), "request not forwarded");

    localBridge->~Bridge();
    vTaskDelay(pdMS_TO_TICKS(LOCAL_BRIDGE_TIMEOUT_MS * 3));
    bool answered = slaveSide.getReceivedFrame(frame);

    TEST_ASSERT_START();
    TEST_ASSERT_FALSE_MESSAGE(answered, "timeout fired after the bridge was destroyed");
}

// Regression: applications left their receive callbacks registered in the interfaces after
// their destruction: the next frame received was handed to a destroyed object
void test_apps_remove_rcv_callbacks_on_destruction() {
    static ModbusInterface::Dummy slaveSide;
    static DummyMaster masterSide;
    static Modbus::StaticWordStore<1> store;

    alignas(Modbus::Client) static uint8_t clientStorage[sizeof(Modbus::Client)];
    auto* client = new (clientStorage) Modbus::Client(masterSide);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());
    size_t clientRegistered = masterSide.callbackCount();
    client->~Client();
    size_t clientLeft = masterSide.callbackCount();

    alignas(Modbus::Server) static uint8_t serverStorage[sizeof(Modbus::Server)];
    auto* server = new (serverStorage) Modbus::Server(slaveSide, store);
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, server->begin());
    size_t serverRegistered = slaveSide.callbackCount();
    server->~Server();
    size_t serverLeft = slaveSide.callbackCount();

    alignas(Modbus::Bridge) static uint8_t bridgeStorage[sizeof(Modbus::Bridge)];
    auto* bridge = new (bridgeStorage) Modbus::Bridge(masterSide, slaveSide, LOCAL_BRIDGE_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Bridge::SUCCESS, bridge->begin());
    size_t bridgeRegistered = masterSide.callbackCount() + slaveSide.callbackCount();
    bridge->~Bridge();
    size_t bridgeLeft = masterSide.callbackCount() + slaveSide.callbackCount();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, clientRegistered, "client callback not registered");
    TEST_ASSERT_EQUAL_MESSAGE(0, clientLeft, "client callback left after destruction");
    TEST_ASSERT_EQUAL_MESSAGE(1, serverRegistered, "server callback not registered");
    TEST_ASSERT_EQUAL_MESSAGE(0, serverLeft, "server callback left after destruction");
    TEST_ASSERT_EQUAL_MESSAGE(2, bridgeRegistered, "bridge callbacks not registered");
    TEST_ASSERT_EQUAL_MESSAGE(0, bridgeLeft, "bridge callbacks left after destruction");
}

// Regression: the client destructor only stopped its timeout timer if it was still active: a
// timeout callback already running (timer expired) kept using the destroyed client, and the
// timer deletion was left pending. The interface's abort blocks to hold that callback while
// another task destroys the client: the destructor must wait for it.
struct BlockingAbortMaster : public DummyMaster {
    StaticSemaphore_t releaseBuf;
    SemaphoreHandle_t release = xSemaphoreCreateBinaryStatic(&releaseBuf);
    volatile bool entered = false;
    void abortCurrentTransaction() override {
        if (entered) return; // Only the timeout callback's abort is held
        entered = true;
        xSemaphoreTake(release, portMAX_DELAY);
    }
};
static constexpr uint32_t LOCAL_CLIENT_TIMEOUT_MS = 20;

struct ClientDestroyer {
    Modbus::Client* client;
    volatile bool done;
};
static StaticTask_t clientDestroyerTcb;
static StackType_t clientDestroyerStack[4096];

static void clientDestroyerTask(void* arg) {
    auto* destroyer = static_cast<ClientDestroyer*>(arg);
    destroyer->client->~Client();
    destroyer->done = true;
    vTaskDelete(nullptr);
}

void test_client_destroyed_while_timeout_runs() {
    static BlockingAbortMaster masterSide;
    alignas(Modbus::Client) static uint8_t storage[sizeof(Modbus::Client)];
    auto* client = new (storage) Modbus::Client(masterSide, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());

    // The request times out: its callback runs, held in the interface's abort
    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->sendRequest(LOCAL_BRIDGE_REQUEST, resp, &tracker));
    uint32_t t0 = Modbus::TIME_MS();
    while (!masterSide.entered && Modbus::TIME_MS() - t0 < 500) vTaskDelay(1);
    bool held = masterSide.entered;

    // Destroy the client meanwhile, from another task
    ClientDestroyer destroyer = { client, false };
    if (held) {
        xTaskCreateStatic(clientDestroyerTask, "clientDestroyer", sizeof(clientDestroyerStack) / sizeof(StackType_t),
                          &destroyer, 1, clientDestroyerStack, &clientDestroyerTcb);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    bool returnedWhileRunning = destroyer.done;
    xSemaphoreGive(masterSide.release);
    t0 = Modbus::TIME_MS();
    while (held && !destroyer.done && Modbus::TIME_MS() - t0 < 1000) vTaskDelay(1);

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(held, "timeout callback not reached");
    TEST_ASSERT_FALSE_MESSAGE(returnedWhileRunning, "client destroyed while its timeout callback was running");
    TEST_ASSERT_TRUE_MESSAGE(destroyer.done, "client destructor didn't return");
}

// Regression: when Server::begin() failed on its second interface, the callback registered on
// the first one stayed in place (calling a server that isn't initialized), and a retry added a
// second one: each request was then handled twice.
struct FailingBeginDummy : public ModbusInterface::Dummy {
    bool failBegin = true;
    Result begin() override { return failBegin ? ERR_INIT_FAILED : SUCCESS; }
};

void test_server_begin_retry_registers_once() {
    static ModbusInterface::Dummy firstSide;
    static FailingBeginDummy secondSide;
    static Modbus::StaticWordStore<1> store;
    alignas(Modbus::Server) static uint8_t storage[sizeof(Modbus::Server)];
    auto* server = new (storage) Modbus::Server({&firstSide, &secondSide}, store);

    Modbus::Server::Result failedRes = server->begin();
    size_t leftAfterFailure = firstSide.callbackCount();
    secondSide.failBegin = false;
    Modbus::Server::Result retryRes = server->begin();
    size_t firstAfterRetry = firstSide.callbackCount();
    size_t secondAfterRetry = secondSide.callbackCount();
    server->~Server();

    TEST_ASSERT_START();
    TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, failedRes, "begin() should fail on the second interface");
    TEST_ASSERT_EQUAL_MESSAGE(0, leftAfterFailure, "callback left on the first interface after a failed begin()");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, retryRes, "begin() retry failed");
    TEST_ASSERT_EQUAL_MESSAGE(1, firstAfterRetry, "first interface callback registered more than once");
    TEST_ASSERT_EQUAL_MESSAGE(1, secondAfterRetry, "second interface callback not registered once");
}

// Regression: begin() only checked the first interface pointer: a null one further in the list
// was dereferenced (crash).
void test_server_begin_rejects_null_interface() {
    static ModbusInterface::Dummy side;
    static Modbus::StaticWordStore<1> store;
    alignas(Modbus::Server) static uint8_t storage[sizeof(Modbus::Server)];
    auto* server = new (storage) Modbus::Server({&side, nullptr}, store);
    Modbus::Server::Result res = server->begin();
    size_t callbacksLeft = side.callbackCount();
    server->~Server();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_INIT_FAILED, res, "begin() accepted a null interface");
    TEST_ASSERT_EQUAL_MESSAGE(0, callbacksLeft, "callback left on the valid interface");
}

// Regression: begin() validated overlapping words starting its scan strictly after address 0,
// and jumping from start address to start address: a word at address 0, or two words with the
// same start address, were never compared. Returns true if the server accepts both words.
static bool overlapAccepted(Modbus::IWordStore& store, uint16_t start1, uint16_t nbRegs1,
                            uint16_t start2, uint16_t nbRegs2) {
    static ModbusInterface::Dummy side;
    alignas(Modbus::Server) static uint8_t storage[sizeof(Modbus::Server)];
    auto* server = new (storage) Modbus::Server(side, store);
    Modbus::Word word;
    word.type = Modbus::HOLDING_REGISTER;
    word.readHandler = serverReadHandler;
    word.writeHandler = serverWriteHandler;
    word.startAddr = start1;
    word.nbRegs = nbRegs1;
    bool added = server->addWord(word) == Modbus::Server::SUCCESS;
    word.startAddr = start2;
    word.nbRegs = nbRegs2;
    added = (server->addWord(word) == Modbus::Server::SUCCESS) && added;
    bool begun = server->begin() == Modbus::Server::SUCCESS;
    server->~Server();
    return added && begun;
}

void test_server_rejects_overlaps_at_zero_and_same_start() {
    // Dynamic store: words added before begin() aren't checked one by one (bulk mode)
    static Modbus::DynamicWordStore dynAtZero(4), dynSameStart(4), dynDisjoint(4);

    TEST_ASSERT_START();
    TEST_ASSERT_FALSE_MESSAGE(overlapAccepted(dynAtZero, 0, 2, 1, 1), "dynamic store: overlap at address 0 accepted");
    TEST_ASSERT_FALSE_MESSAGE(overlapAccepted(dynSameStart, 5, 2, 5, 1), "dynamic store: words with the same start accepted");
    TEST_ASSERT_TRUE_MESSAGE(overlapAccepted(dynDisjoint, 0, 1, 1, 1), "disjoint words rejected");
}

// Regression: a busy server answered SLAVE_DEVICE_BUSY before checking the slave ID: on a shared
// RTU bus, it answered requests addressed to another device. A first request received on another
// interface (requests of a same interface are handled one by one) is held in its Word handler.
struct HeldHandler {
    StaticSemaphore_t releaseBuf;
    SemaphoreHandle_t release = xSemaphoreCreateBinaryStatic(&releaseBuf);
    volatile bool entered = false;
};
struct HeldRequest {
    ModbusInterface::Dummy* side;
    Modbus::Frame request;
    volatile bool done;
};
static StaticTask_t heldRequestTcb;
static StackType_t heldRequestStack[4096];

static void heldRequestTask(void* arg) {
    auto* held = static_cast<HeldRequest*>(arg);
    held->side->transmitFrame(held->request);
    held->done = true;
    vTaskDelete(nullptr);
}

void test_server_busy_ignores_other_slave_ids() {
    static ModbusInterface::Dummy busySide, side;
    static Modbus::StaticWordStore<1> store;
    static HeldHandler handler;
    alignas(Modbus::Server) static uint8_t storage[sizeof(Modbus::Server)];
    auto* server = new (storage) Modbus::Server({&busySide, &side}, store, 1, true, 10);
    Modbus::Word word;
    word.type = Modbus::HOLDING_REGISTER;
    word.nbRegs = 1;
    word.readHandler = [](const Modbus::Word&, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        auto* held = static_cast<HeldHandler*>(userCtx);
        held->entered = true;
        xSemaphoreTake(held->release, portMAX_DELAY);
        outVals[0] = 0;
        return Modbus::NULL_EXCEPTION;
    };
    word.writeHandler = serverWriteHandler;
    word.userCtx = &handler;
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, server->addWord(word));
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, server->begin());

    Modbus::Frame request = {
        .type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = 1,
        .regAddress = 0, .regCount = 1, .data = {}
    };
    HeldRequest held = { &busySide, request, false };
    xTaskCreateStatic(heldRequestTask, "heldRequest", sizeof(heldRequestStack) / sizeof(StackType_t),
                      &held, 1, heldRequestStack, &heldRequestTcb);
    uint32_t t0 = Modbus::TIME_MS();
    while (!handler.entered && Modbus::TIME_MS() - t0 < 500) vTaskDelay(1);
    bool busy = handler.entered;

    // While busy: silent for another slave ID, SLAVE_DEVICE_BUSY for our own
    Modbus::Frame frame;
    request.slaveId = 2;
    side.transmitFrame(request);
    bool otherAnswered = side.getReceivedFrame(frame);
    request.slaveId = 1;
    side.transmitFrame(request);
    bool ownBusy = side.getReceivedFrame(frame) && frame.exceptionCode == Modbus::SLAVE_DEVICE_BUSY;

    xSemaphoreGive(handler.release);
    t0 = Modbus::TIME_MS();
    while (!held.done && Modbus::TIME_MS() - t0 < 1000) vTaskDelay(1);
    bool heldAnswered = held.done && busySide.getReceivedFrame(frame) && frame.exceptionCode == Modbus::NULL_EXCEPTION;
    if (held.done) server->~Server();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(busy, "first request not held in its handler");
    TEST_ASSERT_FALSE_MESSAGE(otherAnswered, "busy server answered a request for another slave ID");
    TEST_ASSERT_TRUE_MESSAGE(ownBusy, "busy server didn't answer SLAVE_DEVICE_BUSY to its own slave ID");
    TEST_ASSERT_TRUE_MESSAGE(heldAnswered, "held request not answered once released");
}

// Regression: the slave ID was only checked before taking the mutex: a request waiting for it
// with the old ID was handled after setSlaveId() returned. A first request (other interface) holds
// the mutex; setSlaveId(), from a higher priority task, takes it before the waiting request.
struct SlaveIdSetter {
    Modbus::Server* server;
    volatile bool done;
};
static StaticTask_t waitingRequestTcb, slaveIdSetterTcb;
static StackType_t waitingRequestStack[4096], slaveIdSetterStack[4096];

static void slaveIdSetterTask(void* arg) {
    auto* setter = static_cast<SlaveIdSetter*>(arg);
    setter->server->setSlaveId(2);
    setter->done = true;
    vTaskDelete(nullptr);
}

static bool waitBlocked(TaskHandle_t task) {
    uint32_t t0 = Modbus::TIME_MS();
    while (eTaskGetState(task) != eBlocked && Modbus::TIME_MS() - t0 < 500) vTaskDelay(1);
    return eTaskGetState(task) == eBlocked;
}

void test_server_slave_id_change_spares_waiting_request() {
    static ModbusInterface::Dummy busySide, side;
    static Modbus::StaticWordStore<1> store;
    static HeldHandler handler;
    alignas(Modbus::Server) static uint8_t storage[sizeof(Modbus::Server)];
    auto* server = new (storage) Modbus::Server({&busySide, &side}, store, 1, true, 2000);
    Modbus::Word word;
    word.type = Modbus::HOLDING_REGISTER;
    word.nbRegs = 1;
    word.readHandler = [](const Modbus::Word&, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
        auto* held = static_cast<HeldHandler*>(userCtx);
        if (!held->entered) { // Only the first request is held
            held->entered = true;
            xSemaphoreTake(held->release, portMAX_DELAY);
        }
        outVals[0] = 0;
        return Modbus::NULL_EXCEPTION;
    };
    word.writeHandler = serverWriteHandler;
    word.userCtx = &handler;
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, server->addWord(word));
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, server->begin());

    Modbus::Frame request = {
        .type = Modbus::REQUEST, .fc = Modbus::READ_HOLDING_REGISTERS, .slaveId = 1,
        .regAddress = 0, .regCount = 1, .data = {}
    };
    // First request holds the mutex
    HeldRequest held = { &busySide, request, false };
    xTaskCreateStatic(heldRequestTask, "heldRequest", sizeof(heldRequestStack) / sizeof(StackType_t),
                      &held, 1, heldRequestStack, &heldRequestTcb);
    uint32_t t0 = Modbus::TIME_MS();
    while (!handler.entered && Modbus::TIME_MS() - t0 < 500) vTaskDelay(1);
    bool busy = handler.entered;

    // Request for slave 1 waiting for the mutex, then setSlaveId(2) waiting with a higher priority
    HeldRequest waiting = { &side, request, false };
    TaskHandle_t waitingTask = xTaskCreateStatic(heldRequestTask, "waitingRequest",
        sizeof(waitingRequestStack) / sizeof(StackType_t), &waiting, 1, waitingRequestStack, &waitingRequestTcb);
    bool waitingBlocked = waitBlocked(waitingTask);
    SlaveIdSetter setter = { server, false };
    TaskHandle_t setterTask = xTaskCreateStatic(slaveIdSetterTask, "slaveIdSetter",
        sizeof(slaveIdSetterStack) / sizeof(StackType_t), &setter, 3, slaveIdSetterStack, &slaveIdSetterTcb);
    bool setterBlocked = waitBlocked(setterTask);

    xSemaphoreGive(handler.release);
    t0 = Modbus::TIME_MS();
    while (!(held.done && waiting.done && setter.done) && Modbus::TIME_MS() - t0 < 3000) vTaskDelay(1);
    Modbus::Frame frame;
    bool waitingAnswered = side.getReceivedFrame(frame);
    bool allDone = held.done && waiting.done && setter.done;
    if (allDone) server->~Server();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(busy && waitingBlocked && setterBlocked, "requests / setSlaveId() not waiting for the mutex");
    TEST_ASSERT_TRUE_MESSAGE(allDone, "a task didn't return");
    TEST_ASSERT_FALSE_MESSAGE(waitingAnswered, "request for the old slave ID answered after setSlaveId() returned");
}

// Regression: with rejectUndefined = false, a request starting inside a multi-register Word was
// handled as a gap (zeros read, write dropped) instead of a partial Word access. And looking up the
// Word holding an address gave up on the first Word of another type before it (the store keeps all
// types sorted together), so such a Word was missed. Returns the exception answered, -1 if none.
static int serverException(ModbusInterface::Dummy& side, Modbus::FunctionCode fc, uint16_t addr) {
    Modbus::Frame request = {
        .type = Modbus::REQUEST, .fc = fc, .slaveId = 1,
        .regAddress = addr, .regCount = 1, .data = {}
    };
    Modbus::Frame response;
    side.transmitFrame(request);
    return side.getReceivedFrame(response) ? (int)response.exceptionCode : -1;
}

static void checkStartInsideWordRejected(Modbus::IWordStore& store, const char* storeName) {
    static ModbusInterface::Dummy side;
    alignas(Modbus::Server) static uint8_t storage[sizeof(Modbus::Server)];
    auto* server = new (storage) Modbus::Server(side, store, 1, false);
    Modbus::Word words[2];
    words[0].type = Modbus::HOLDING_REGISTER; // Registers 10-13
    words[0].startAddr = 10;
    words[0].nbRegs = 4;
    words[1].type = Modbus::COIL;             // Coil 12, in between in the store
    words[1].startAddr = 12;
    words[1].nbRegs = 1;
    for (auto& word : words) {
        word.readHandler = serverReadHandler;
        word.writeHandler = serverWriteHandler;
    }
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, server->addWords(words, 2));
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, server->begin());

    Modbus::Word found = server->getWord(Modbus::HOLDING_REGISTER, 13);
    int readInside = serverException(side, Modbus::READ_HOLDING_REGISTERS, 11);
    int readBehindCoil = serverException(side, Modbus::READ_HOLDING_REGISTERS, 13);
    int writeInside = serverException(side, Modbus::WRITE_REGISTER, 11);
    server->~Server();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(found.type == Modbus::HOLDING_REGISTER && found.startAddr == 10, storeName);
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::ILLEGAL_DATA_ADDRESS, readInside, storeName);
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::ILLEGAL_DATA_ADDRESS, readBehindCoil, storeName);
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::ILLEGAL_DATA_ADDRESS, writeInside, storeName);
}

void test_server_rejects_start_inside_word() {
    static Modbus::StaticWordStore<2> staticStore;
    static Modbus::DynamicWordStore dynamicStore(2);
    checkStartInsideWordRejected(staticStore, "static store");
    checkStartInsideWordRejected(dynamicStore, "dynamic store");
}

// Regression: addWords() wasn't all-or-nothing. Before begin(), the static store rejected an
// overlapping Word while inserting it, after the previous ones of the batch (reported as a capacity
// overflow); after begin(), the vector overload's in-batch check wrapped the end address of a Word
// ending at 65535, letting the store reject it in the same way.
static Modbus::Word handledWord(uint16_t startAddr, uint16_t nbRegs) {
    Modbus::Word word;
    word.type = Modbus::HOLDING_REGISTER;
    word.startAddr = startAddr;
    word.nbRegs = nbRegs;
    word.readHandler = serverReadHandler;
    word.writeHandler = serverWriteHandler;
    return word;
}

void test_server_add_words_all_or_nothing() {
    static ModbusInterface::Dummy side;
    static Modbus::StaticWordStore<4> store;
    alignas(Modbus::Server) static uint8_t storage[sizeof(Modbus::Server)];
    auto* server = new (storage) Modbus::Server(side, store);

    // Before begin(): overlap inside the batch, then the valid batch
    const Modbus::Word overlapping[] = { handledWord(10, 1), handledWord(11, 1), handledWord(10, 1) };
    auto overlapRes = server->addWords(overlapping, 3);
    bool leftover = (bool)server->getWord(Modbus::HOLDING_REGISTER, 10);
    auto retryRes = server->addWords(overlapping, 2);
    auto singleRes = server->addWord(handledWord(11, 1)); // Overlaps a Word already added
    auto begun = server->begin();

    // After begin(): vector batch with a Word ending at 65535
    std::vector<Modbus::Word> lastAddr = { handledWord(65530, 6), handledWord(65532, 1) };
    auto lastAddrRes = server->addWords(lastAddr);
    bool lastAddrLeftover = (bool)server->getWord(Modbus::HOLDING_REGISTER, 65530);
    server->~Server();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_WORD_OVERLAP, overlapRes, "overlapping batch not reported as such");
    TEST_ASSERT_FALSE_MESSAGE(leftover, "overlapping batch partly added");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, retryRes, "valid batch rejected after a failed one");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_WORD_OVERLAP, singleRes, "overlapping Word not reported as such");
    TEST_ASSERT_EQUAL(Modbus::Server::SUCCESS, begun);
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::ERR_WORD_OVERLAP, lastAddrRes, "overlap ending at 65535 not reported as such");
    TEST_ASSERT_FALSE_MESSAGE(lastAddrLeftover, "batch ending at 65535 partly added");
}

// Regression: when Bridge::begin() couldn't register its slave callback, the master one stayed
// in place, and every call to begin() (retry, or a second call after a success) registered the
// callbacks again: each request was then forwarded several times. The slave side's callback
// slots are filled to make the slave registration fail.
void test_bridge_begin_registers_once() {
    static ModbusInterface::Dummy slaveSide;
    static DummyMaster masterSide;
    static uint8_t fillers[ModbusInterface::IInterface::MAX_RCV_CALLBACKS];
    for (auto& filler : fillers) {
        slaveSide.setRcvCallback([](const Modbus::Frame&, void*) {}, &filler);
    }
    alignas(Modbus::Bridge) static uint8_t storage[sizeof(Modbus::Bridge)];
    auto* bridge = new (storage) Modbus::Bridge(masterSide, slaveSide, LOCAL_BRIDGE_TIMEOUT_MS);

    Modbus::Bridge::Result failedRes = bridge->begin();
    size_t masterAfterFailure = masterSide.callbackCount();
    for (auto& filler : fillers) slaveSide.removeRcvCallbacks(&filler);
    Modbus::Bridge::Result retryRes = bridge->begin();
    Modbus::Bridge::Result againRes = bridge->begin();
    size_t masterRegistered = masterSide.callbackCount();
    size_t slaveRegistered = slaveSide.callbackCount();
    bridge->~Bridge();

    TEST_ASSERT_START();
    TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Bridge::SUCCESS, failedRes, "begin() should fail on the slave callback");
    TEST_ASSERT_EQUAL_MESSAGE(0, masterAfterFailure, "master callback left after a failed begin()");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Bridge::SUCCESS, retryRes, "begin() retry failed");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Bridge::SUCCESS, againRes, "second begin() failed");
    TEST_ASSERT_EQUAL_MESSAGE(1, masterRegistered, "master callback registered more than once");
    TEST_ASSERT_EQUAL_MESSAGE(1, slaveRegistered, "slave callback registered more than once");
}

// Guard: an answered request's timer is no longer stopped (stopped after the mutex, it could
// cancel the timer of a request forwarded meanwhile). Its later event must neither expire the
// next request early nor prevent that request's own timeout.
void test_bridge_answered_request_timer_spares_next() {
    static ModbusInterface::Dummy slaveSide;
    static DummyMaster masterSide;
    alignas(Modbus::Bridge) static uint8_t storage[sizeof(Modbus::Bridge)];
    auto* bridge = new (storage) Modbus::Bridge(masterSide, slaveSide, LOCAL_BRIDGE_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Bridge::SUCCESS, bridge->begin());
    Modbus::Frame frame;

    // Request A, answered at once
    uint32_t tA = Modbus::TIME_MS();
    slaveSide.transmitFrame(LOCAL_BRIDGE_REQUEST);
    bool aForwarded = masterSide.getReceivedFrame(frame);
    Modbus::Frame response = LOCAL_BRIDGE_REQUEST;
    response.type = Modbus::RESPONSE;
    response.data[0] = 0x1234;
    masterSide.transmitFrame(response);
    bool aAnswered = slaveSide.getReceivedFrame(frame) && frame.exceptionCode == Modbus::NULL_EXCEPTION;

    // Request B, half a timeout later, never answered
    vTaskDelay(pdMS_TO_TICKS(LOCAL_BRIDGE_TIMEOUT_MS / 2));
    uint32_t tB = Modbus::TIME_MS();
    slaveSide.transmitFrame(LOCAL_BRIDGE_REQUEST);
    bool bForwarded = masterSide.getReceivedFrame(frame);
    while (Modbus::TIME_MS() - tA < LOCAL_BRIDGE_TIMEOUT_MS + 20) vTaskDelay(1); // A's deadline passed
    bool bExpiredEarly = slaveSide.getReceivedFrame(frame);
    while (Modbus::TIME_MS() - tB < LOCAL_BRIDGE_TIMEOUT_MS + 50) vTaskDelay(1); // B's deadline passed
    bool bExpired = slaveSide.getReceivedFrame(frame)
                    && frame.exceptionCode == Modbus::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND;
    bridge->~Bridge();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(aForwarded && aAnswered, "request A not forwarded & answered");
    TEST_ASSERT_TRUE_MESSAGE(bForwarded, "request B not forwarded");
    TEST_ASSERT_FALSE_MESSAGE(bExpiredEarly, "request B expired at A's deadline");
    TEST_ASSERT_TRUE_MESSAGE(bExpired, "request B didn't time out at its own deadline");
}

// Regression: when sendFrame() failed at once, the failure was applied twice: by the TX callback,
// then by sendRequest() itself. If the first completion started request B (here from the
// completion callback), the second one ended B with A's failure.
struct FailFirstSendMaster : public DummyMaster {
    bool failNext = true;
    Result sendFrame(const Modbus::Frame& frame, TxResultCallback txCallback, void* ctx) override {
        if (failNext) {
            failNext = false;
            if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
            return ERR_SEND_FAILED;
        }
        return DummyMaster::sendFrame(frame, txCallback, ctx);
    }
};
static constexpr uint32_t LOCAL_CLIENT_LONG_TIMEOUT_MS = 500;
static Modbus::Client* retryClient = nullptr;
static Modbus::Frame retryResp;
static volatile Modbus::Client::Result retryTracker = Modbus::Client::NODATA;
static volatile Modbus::Client::Result retryStartRes = Modbus::Client::NODATA;

void test_client_sync_send_failure_applied_once() {
    static FailFirstSendMaster masterSide;
    alignas(Modbus::Client) static uint8_t storage[sizeof(Modbus::Client)];
    retryClient = new (storage) Modbus::Client(masterSide, LOCAL_CLIENT_LONG_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, retryClient->begin());

    // Request A fails at once; its completion callback starts request B
    auto onA = [](Modbus::Client::Result, const Modbus::Frame*, void*) {
        retryStartRes = retryClient->sendRequest(LOCAL_BRIDGE_REQUEST, retryResp, (Modbus::Client::Result*)&retryTracker);
    };
    Modbus::Client::Result aRes = retryClient->sendRequest(LOCAL_BRIDGE_REQUEST, onA, nullptr);
    Modbus::Client::Result bState = retryTracker; // B is sent & waits for its response
    retryClient->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TX_FAILED, aRes, "request A should fail");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, retryStartRes, "request B not started");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::NODATA, bState, "request B ended by the failure of request A");
}

// Regression: a timeout shorter than a tick (here 0 ms) gave a timer period of 0 ticks, which
// FreeRTOS rejects. It must be rounded up to 1 tick: the request times out.
void test_client_zero_timeout() {
    static DummyMaster masterSide;
    alignas(Modbus::Client) static uint8_t storage[sizeof(Modbus::Client)];
    auto* client = new (storage) Modbus::Client(masterSide, 0);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());

    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    volatile Modbus::Client::Result& result = tracker; // Written by the timer daemon
    Modbus::Client::Result sendRes = client->sendRequest(LOCAL_BRIDGE_REQUEST, resp, &tracker);
    uint32_t t0 = Modbus::TIME_MS();
    while (result == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < 100) vTaskDelay(1);
    client->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, sendRes, "request not started");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, tracker, "request didn't time out");
}

// ===================================================================================
// CLIENT REQUEST ENGINE: CONCURRENCY
// Interleavings made deterministic by holding a task where the engine waits: the interface's
// abort (called under the Client mutex), the timer daemon (blocked by a pending function, or
// suspended while it waits for the Client mutex), or a synchronous caller (suspended likewise).
// ===================================================================================

struct ControlledMaster : public DummyMaster {
    // When holdAbort is set, the first abort is held until abortRelease is given
    StaticSemaphore_t abortReleaseBuf;
    SemaphoreHandle_t abortRelease = xSemaphoreCreateBinaryStatic(&abortReleaseBuf);
    volatile bool holdAbort = false;
    volatile bool abortEntered = false;
    volatile uint32_t aborts = 0;
    void abortCurrentTransaction() override {
        aborts = aborts + 1;
        if (!holdAbort || abortEntered) return;
        abortEntered = true;
        xSemaphoreTake(abortRelease, portMAX_DELAY);
    }

    // When holdTx is set, the TX result is held until deliverTxResult()
    volatile bool holdTx = false;
    TxResultCallback heldTxCallback = nullptr;
    void* heldTxCtx = nullptr;
    Result sendFrame(const Modbus::Frame& frame, TxResultCallback txCallback, void* ctx) override {
        if (!holdTx) return DummyMaster::sendFrame(frame, txCallback, ctx);
        heldTxCallback = txCallback;
        heldTxCtx = ctx;
        return SUCCESS;
    }
    void deliverTxResult(Result result) {
        TxResultCallback cb = heldTxCallback;
        heldTxCallback = nullptr;
        if (cb) cb(result, heldTxCtx);
    }

    void reset() {
        holdAbort = false;
        abortEntered = false;
        aborts = 0;
        holdTx = false;
        heldTxCallback = nullptr;
        xSemaphoreTake(abortRelease, 0);
    }
};

// Timer daemon held by a pending function until released
static StaticSemaphore_t daemonReleaseBuf;
static SemaphoreHandle_t daemonRelease = xSemaphoreCreateBinaryStatic(&daemonReleaseBuf);
static void holdTimerDaemon() {
    xTimerPendFunctionCall([](void*, uint32_t) { xSemaphoreTake(daemonRelease, portMAX_DELAY); },
                           nullptr, 0, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5));
}
static void releaseTimerDaemon() {
    xSemaphoreGive(daemonRelease);
    vTaskDelay(pdMS_TO_TICKS(5));
}

// Synchronous request A sent from its own task
struct SyncCaller {
    Modbus::Client* client;
    volatile bool done;
    volatile Modbus::Client::Result result;
};
static StaticTask_t syncCallerTcb;
static StackType_t syncCallerStack[4096];
static TaskHandle_t startSyncCaller(SyncCaller& caller) {
    return xTaskCreateStatic([](void* arg) {
        auto* c = static_cast<SyncCaller*>(arg);
        Modbus::Frame resp;
        c->result = c->client->sendRequest(LOCAL_BRIDGE_REQUEST, resp);
        c->done = true;
        vTaskDelete(nullptr);
    }, "syncCaller", sizeof(syncCallerStack) / sizeof(StackType_t), &caller, 2, syncCallerStack, &syncCallerTcb);
}

static bool waitFlag(volatile bool& flag, uint32_t maxMs) {
    uint32_t t0 = Modbus::TIME_MS();
    while (!flag && Modbus::TIME_MS() - t0 < maxMs) vTaskDelay(1);
    return flag;
}

static ControlledMaster controlledMaster;
alignas(Modbus::Client) static uint8_t concurrencyClientStorage[sizeof(Modbus::Client)];
static constexpr uint32_t SYNC_WAIT_MS = LOCAL_CLIENT_TIMEOUT_MS + 100; // Timeout + fallback margin

// A timeout callback of A already running (waiting for the Client mutex) while A ends and B
// starts: it must not end B before B's own deadline. A ends by its synchronous fallback, held in
// the interface's abort; the daemon, which reached A's timeout callback meanwhile, is suspended
// while B starts, then resumed.
void test_client_late_timeout_callback_spares_next() {
    controlledMaster.reset();
    controlledMaster.holdAbort = true;
    auto* client = new (concurrencyClientStorage) Modbus::Client(controlledMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());
    TaskHandle_t daemon = xTimerGetTimerDaemonTaskHandle();

    holdTimerDaemon(); // A's timer can't fire: A's synchronous wait expires first
    SyncCaller a = { client, false, Modbus::Client::NODATA };
    startSyncCaller(a);
    bool fallbackHeld = waitFlag(controlledMaster.abortEntered, SYNC_WAIT_MS + 200);
    releaseTimerDaemon(); // A's timeout callback runs & waits for the Client mutex
    vTaskDelay(pdMS_TO_TICKS(20));
    vTaskSuspend(daemon);
    xSemaphoreGive(controlledMaster.abortRelease); // A's fallback ends A
    bool aDone = waitFlag(a.done, 500);

    Modbus::Frame respB;
    Modbus::Client::Result trackerB = Modbus::Client::NODATA;
    volatile Modbus::Client::Result& resultB = trackerB; // Written by the timer daemon
    uint32_t tB = Modbus::TIME_MS();
    Modbus::Client::Result sendB = client->sendRequest(LOCAL_BRIDGE_REQUEST, respB, &trackerB);
    vTaskResume(daemon); // A's timeout callback resumes while B is active
    vTaskDelay(pdMS_TO_TICKS(5));
    Modbus::Client::Result bAfterLateCallback = resultB;
    while (resultB == Modbus::Client::NODATA && Modbus::TIME_MS() - tB < LOCAL_CLIENT_TIMEOUT_MS + 100) vTaskDelay(1);
    uint32_t bElapsed = Modbus::TIME_MS() - tB;
    client->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(fallbackHeld && aDone, "request A not ended by its synchronous fallback");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, a.result, "request A should time out");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, sendB, "request B not started");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::NODATA, bAfterLateCallback, "request B ended by A's late timeout callback");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, trackerB, "request B didn't time out");
    TEST_ASSERT_TRUE_MESSAGE(bElapsed >= LOCAL_CLIENT_TIMEOUT_MS, "request B timed out early");
}

// The synchronous fallback of A, resumed after A ended and B started, must not end B. A ends by
// its timeout callback, held in the interface's abort; A's caller, whose wait expired meanwhile,
// is suspended while it waits for the Client mutex, B starts, then the caller is resumed.
void test_client_sync_fallback_spares_next() {
    controlledMaster.reset();
    controlledMaster.holdAbort = true;
    auto* client = new (concurrencyClientStorage) Modbus::Client(controlledMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());

    SyncCaller a = { client, false, Modbus::Client::NODATA };
    uint32_t tA = Modbus::TIME_MS();
    TaskHandle_t caller = startSyncCaller(a);
    bool timeoutHeld = waitFlag(controlledMaster.abortEntered, LOCAL_CLIENT_TIMEOUT_MS + 200);
    while (Modbus::TIME_MS() - tA < SYNC_WAIT_MS + 30) vTaskDelay(1); // A's wait expired: its fallback waits
    vTaskSuspend(caller);
    xSemaphoreGive(controlledMaster.abortRelease); // A's timeout callback ends A
    vTaskDelay(pdMS_TO_TICKS(5));

    Modbus::Frame respB;
    Modbus::Client::Result trackerB = Modbus::Client::NODATA;
    volatile Modbus::Client::Result& resultB = trackerB; // Written by the timer daemon
    Modbus::Client::Result sendB = client->sendRequest(LOCAL_BRIDGE_REQUEST, respB, &trackerB);
    vTaskResume(caller); // A's fallback resumes while B is active
    bool aDone = waitFlag(a.done, 500);
    Modbus::Client::Result bAfterFallback = resultB;
    uint32_t abortsAfterFallback = controlledMaster.aborts;
    uint32_t t0 = Modbus::TIME_MS();
    while (resultB == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < LOCAL_CLIENT_TIMEOUT_MS + 100) vTaskDelay(1);
    client->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(timeoutHeld, "request A's timeout callback not reached");
    TEST_ASSERT_TRUE_MESSAGE(aDone, "request A's caller didn't return");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, a.result, "request A should time out");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, sendB, "request B not started");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::NODATA, bAfterFallback, "request B ended by A's synchronous fallback");
    TEST_ASSERT_EQUAL_MESSAGE(1, abortsAfterFallback, "request B aborted by A's synchronous fallback");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, trackerB, "request B didn't time out");
}

// A's TX result held beyond its timeout (e.g. interface reconnecting): B is refused until it
// comes back, and the late result doesn't complete A a second time.
void test_client_late_tx_result_gates_next() {
    controlledMaster.reset();
    controlledMaster.holdTx = true;
    auto* client = new (concurrencyClientStorage) Modbus::Client(controlledMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());

    Modbus::Frame respA;
    Modbus::Client::Result trackerA = Modbus::Client::NODATA;
    volatile Modbus::Client::Result& resultA = trackerA; // Written by the timer daemon
    Modbus::Client::Result sendA = client->sendRequest(LOCAL_BRIDGE_REQUEST, respA, &trackerA);
    uint32_t t0 = Modbus::TIME_MS();
    while (resultA == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < LOCAL_CLIENT_TIMEOUT_MS + 100) vTaskDelay(1);
    Modbus::Client::Result aTimedOut = trackerA;

    Modbus::Frame respB;
    Modbus::Client::Result trackerB = Modbus::Client::NODATA;
    Modbus::Client::Result sendBWhileHeld = client->sendRequest(LOCAL_BRIDGE_REQUEST, respB, &trackerB);
    controlledMaster.holdTx = false;
    controlledMaster.deliverTxResult(ModbusInterface::IInterface::ERR_SEND_FAILED); // Late TX result of A
    Modbus::Client::Result aAfterLateResult = trackerA;
    Modbus::Client::Result sendBAfter = client->sendRequest(LOCAL_BRIDGE_REQUEST, respB, &trackerB);
    client->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, sendA, "request A not started");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, aTimedOut, "request A should time out");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_BUSY, sendBWhileHeld, "request B accepted before A's TX result");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, aAfterLateResult, "request A completed a second time");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, sendBAfter, "request B refused after A's TX result");
}

// ===================================================================================
// TRANSACTION OWNER (client side): the requester owns the interface's transaction until it
// releases it, at the end of its request; a given-up request aborts what's left of it.
// OwnedMaster tracks the owner like the TCP & RTU interfaces, with barriers: sendFrame() held
// at its entry, a synchronous send failure, a held TX result (ControlledMaster).
// ===================================================================================

struct OwnedMaster : public ControlledMaster {
    OwnedMaster() { _tracksOwner = true; }

    StaticSemaphore_t sendGateBuf;
    SemaphoreHandle_t sendGate = xSemaphoreCreateBinaryStatic(&sendGateBuf);
    volatile bool holdBeforeSend = false; // The next sendFrame() waits for sendGate at its entry
    volatile bool sendEntered = false;
    volatile bool failNextSend = false;   // The next sendFrame() fails at once, once claimed

    Result sendFrame(const Modbus::Frame& frame, TxResultCallback txCallback, void* ctx) override {
        if (holdBeforeSend) {
            holdBeforeSend = false;
            sendEntered = true;
            xSemaphoreTake(sendGate, portMAX_DELAY);
        }
        if (ownedByOther(ctx)) {
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return ERR_BUSY;
        }
        const bool claimed = claimTransaction(ctx);
        if (failNextSend) {
            failNextSend = false;
            if (claimed) unclaimTransaction(ctx);
            if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
            return ERR_SEND_FAILED;
        }
        return ControlledMaster::sendFrame(frame, txCallback, ctx);
    }
    bool isReady() override { return !isOwned(); }
    void* owner() { return currentOwner(); }

    void resetOwned() {
        reset();
        holdBeforeSend = false;
        sendEntered = false;
        failNextSend = false;
        xSemaphoreTake(sendGate, 0);
        Modbus::Lock guard(_ownerMutex);
        _owner = nullptr;
    }
};

static OwnedMaster ownedMaster;
alignas(Modbus::Client) static uint8_t ownerClientBStorage[sizeof(Modbus::Client)];

static Modbus::Frame localResponse(uint16_t value) {
    Modbus::Frame response = LOCAL_BRIDGE_REQUEST;
    response.type = Modbus::RESPONSE;
    response.data[0] = value;
    return response;
}

// B, sharing the interface, sends & gets its own response
static bool requesterServed(OwnedMaster& master, Modbus::Client& client, uint16_t value) {
    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    if (client.sendRequest(LOCAL_BRIDGE_REQUEST, resp, &tracker) != Modbus::Client::SUCCESS) return false;
    master.transmitFrame(localResponse(value));
    return tracker == Modbus::Client::SUCCESS && resp.data[0] == value;
}

// Runs a function from its own task (when it blocks at a barrier)
struct OwnerJob {
    void (*fn)(OwnerJob&);
    Modbus::Client* client;
    bool sync;
    volatile bool done;
    Modbus::Client::Result sendResult;
    volatile Modbus::Client::Result tracker;
    Modbus::Frame resp;
};
static StaticTask_t ownerJobTcb;
static StackType_t ownerJobStack[4096];
static void startOwnerJob(OwnerJob& job) {
    xTaskCreateStatic([](void* arg) {
        auto* job = static_cast<OwnerJob*>(arg);
        job->fn(*job);
        job->done = true;
        vTaskDelete(nullptr);
    }, "ownerJob", sizeof(ownerJobStack) / sizeof(StackType_t), &job, 2, ownerJobStack, &ownerJobTcb);
}
static void ownerJobSend(OwnerJob& job) {
    job.sendResult = job.sync ? job.client->sendRequest(LOCAL_BRIDGE_REQUEST, job.resp)
                              : job.client->sendRequest(LOCAL_BRIDGE_REQUEST, job.resp, (Modbus::Client::Result*)&job.tracker);
}

// A's timer fires after it was armed, before its frame is queued: A ends without owning the
// transaction, then its frame claims it. Its TX result (always delivered) must release it.
static void runTimeoutBeforeSend(bool sync) {
    ownedMaster.resetOwned();
    ownedMaster.holdBeforeSend = true;
    auto* clientA = new (concurrencyClientStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    auto* clientB = new (ownerClientBStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientA->begin());
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientB->begin());

    static OwnerJob a;
    a.fn = ownerJobSend;
    a.client = clientA;
    a.sync = sync;
    a.done = false;
    a.tracker = Modbus::Client::NODATA;
    startOwnerJob(a);
    bool held = waitFlag(ownedMaster.sendEntered, 500);
    vTaskDelay(pdMS_TO_TICKS(LOCAL_CLIENT_TIMEOUT_MS + 50)); // A's timer fires
    xSemaphoreGive(ownedMaster.sendGate); // A's frame claims the transaction, its TX result follows
    bool aDone = waitFlag(a.done, 500);
    Modbus::Client::Result resultA = sync ? a.sendResult : a.tracker;
    void* ownerAfter = ownedMaster.owner();
    bool bServed = requesterServed(ownedMaster, *clientB, 0xB0B0);
    clientB->~Client();
    clientA->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(held && aDone, "A not held before its send");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::ERR_TIMEOUT, resultA, "A should time out");
    TEST_ASSERT_NULL_MESSAGE(ownerAfter, "transaction left claimed by a request that timed out before its send");
    TEST_ASSERT_TRUE_MESSAGE(bServed, "B not served after A");
}

void test_owner_timeout_before_send_async() { runTimeoutBeforeSend(false); }
void test_owner_timeout_before_send_sync() { runTimeoutBeforeSend(true); }

// A owns the transaction, its TX result is held; A times out (released & aborted), B claims the
// transaction, then A's late TX result comes: it must leave B's transaction alone.
void test_owner_late_tx_result_spares_next() {
    ownedMaster.resetOwned();
    ownedMaster.holdTx = true;
    auto* clientA = new (concurrencyClientStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    auto* clientB = new (ownerClientBStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientA->begin());
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientB->begin());

    Modbus::Frame respA;
    Modbus::Client::Result trackerA = Modbus::Client::NODATA;
    volatile Modbus::Client::Result& resultA = trackerA; // Written by the timer daemon
    Modbus::Client::Result sendA = clientA->sendRequest(LOCAL_BRIDGE_REQUEST, respA, &trackerA);
    void* ownerWhileHeld = ownedMaster.owner();
    uint32_t t0 = Modbus::TIME_MS();
    while (resultA == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < LOCAL_CLIENT_TIMEOUT_MS + 100) vTaskDelay(1);
    uint32_t abortsAfterTimeout = ownedMaster.aborts;

    ownedMaster.holdTx = false;
    Modbus::Frame respB;
    Modbus::Client::Result trackerB = Modbus::Client::NODATA;
    Modbus::Client::Result sendB = clientB->sendRequest(LOCAL_BRIDGE_REQUEST, respB, &trackerB);
    ownedMaster.deliverTxResult(ModbusInterface::IInterface::SUCCESS); // A's late TX result
    void* ownerAfterLate = ownedMaster.owner();
    ownedMaster.transmitFrame(localResponse(0xB0B0));
    uint32_t abortsAfter = ownedMaster.aborts;
    clientB->~Client();
    clientA->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, sendA);
    TEST_ASSERT_TRUE_MESSAGE(ownerWhileHeld == clientA, "A doesn't own its transaction");
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TIMEOUT, trackerA);
    TEST_ASSERT_EQUAL_MESSAGE(1, abortsAfterTimeout, "A's timeout didn't abort its transaction");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, sendB, "B refused after A released");
    TEST_ASSERT_TRUE_MESSAGE(ownerAfterLate == clientB, "A's late TX result released B's transaction");
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, trackerB);
    TEST_ASSERT_EQUAL_HEX16(0xB0B0, respB.data[0]);
    TEST_ASSERT_EQUAL_MESSAGE(1, abortsAfter, "B's transaction aborted");
}

// The send fails at once once claimed: the claim is undone, A ends (TX failure, no abort)
void test_owner_sync_send_failure_releases() {
    ownedMaster.resetOwned();
    ownedMaster.failNextSend = true;
    auto* clientA = new (concurrencyClientStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    auto* clientB = new (ownerClientBStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientA->begin());
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientB->begin());

    Modbus::Frame respA;
    Modbus::Client::Result trackerA = Modbus::Client::NODATA;
    Modbus::Client::Result sendA = clientA->sendRequest(LOCAL_BRIDGE_REQUEST, respA, &trackerA);
    void* ownerAfter = ownedMaster.owner();
    bool bServed = requesterServed(ownedMaster, *clientB, 0xB0B0);
    uint32_t aborts = ownedMaster.aborts;
    clientB->~Client();
    clientA->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TX_FAILED, sendA);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TX_FAILED, trackerA);
    TEST_ASSERT_NULL_MESSAGE(ownerAfter, "transaction left claimed after a failed send");
    TEST_ASSERT_TRUE_MESSAGE(bServed, "B not served after A");
    TEST_ASSERT_EQUAL_MESSAGE(0, aborts, "a failed send aborted the transaction");
}

// A's completion callback runs once the transaction is released (no abort): it can start B,
// which doesn't get A's response (still being delivered) but its own.
struct ChainedOwnerCtx {
    Modbus::Client* clientB;
    Modbus::Client::Result resultA;
    bool readyInCallback;
    Modbus::Client::Result startB;
    Modbus::Frame respB;
    Modbus::Client::Result trackerB;
};

void test_owner_released_before_completion_callback() {
    ownedMaster.resetOwned();
    auto* clientA = new (concurrencyClientStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    auto* clientB = new (ownerClientBStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientA->begin());
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, clientB->begin());

    static ChainedOwnerCtx ctx;
    ctx = { clientB, Modbus::Client::NODATA, false, Modbus::Client::NODATA, {}, Modbus::Client::NODATA };
    Modbus::Client::Result sendA = clientA->sendRequest(LOCAL_BRIDGE_REQUEST,
        [](Modbus::Client::Result result, const Modbus::Frame*, void* arg) {
            auto* c = static_cast<ChainedOwnerCtx*>(arg);
            c->resultA = result;
            c->readyInCallback = ownedMaster.isReady();
            c->startB = c->clientB->sendRequest(LOCAL_BRIDGE_REQUEST, c->respB, &c->trackerB);
        }, &ctx);
    ownedMaster.transmitFrame(localResponse(0xA0A0));
    Modbus::Client::Result bAfterA = ctx.trackerB;
    ownedMaster.transmitFrame(localResponse(0xB0B0));
    uint32_t aborts = ownedMaster.aborts;
    clientB->~Client();
    clientA->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, sendA);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, ctx.resultA);
    TEST_ASSERT_TRUE_MESSAGE(ctx.readyInCallback, "interface not released before A's completion callback");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, ctx.startB, "B refused from A's completion callback");
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::NODATA, bAfterA, "B took A's response");
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, ctx.trackerB);
    TEST_ASSERT_EQUAL_HEX16(0xB0B0, ctx.respB.data[0]);
    TEST_ASSERT_EQUAL_MESSAGE(0, aborts, "a completed request aborted the transaction");
}

// A response delivered while the active request's TX result isn't back answers an earlier
// request (e.g. a previous one of the same slave & FC): dropped, the request gets its own.
void test_owner_response_before_tx_result_dropped() {
    ownedMaster.resetOwned();
    ownedMaster.holdTx = true;
    auto* client = new (concurrencyClientStorage) Modbus::Client(ownedMaster, LOCAL_CLIENT_TIMEOUT_MS * 10);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());

    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    Modbus::Client::Result send = client->sendRequest(LOCAL_BRIDGE_REQUEST, resp, &tracker);
    ownedMaster.transmitFrame(localResponse(0xDEAD)); // Before the TX result
    Modbus::Client::Result beforeTx = tracker;
    ownedMaster.deliverTxResult(ModbusInterface::IInterface::SUCCESS);
    ownedMaster.transmitFrame(localResponse(0xA0A0));
    client->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, send);
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Client::NODATA, beforeTx, "response taken before the request's TX result");
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, tracker);
    TEST_ASSERT_EQUAL_HEX16(0xA0A0, resp.data[0]);
}

// Bridge: its timeout closes the request before its frame is queued (gateway exception), then
// the frame claims the transaction: its TX result must release it, the next request goes through.
static ModbusInterface::Dummy ownerBridgeSlave;
alignas(Modbus::Bridge) static uint8_t ownerBridgeStorage[sizeof(Modbus::Bridge)];

void test_owner_bridge_timeout_before_send() {
    ownedMaster.resetOwned();
    Modbus::Frame frame;
    while (ownerBridgeSlave.getReceivedFrame(frame)) {}
    auto* bridge = new (ownerBridgeStorage) Modbus::Bridge(ownedMaster, ownerBridgeSlave, LOCAL_BRIDGE_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Bridge::SUCCESS, bridge->begin());

    ownedMaster.holdBeforeSend = true;
    static OwnerJob inject;
    inject.fn = [](OwnerJob&) { ownerBridgeSlave.transmitFrame(LOCAL_BRIDGE_REQUEST); };
    inject.done = false;
    startOwnerJob(inject);
    bool held = waitFlag(ownedMaster.sendEntered, 500);
    vTaskDelay(pdMS_TO_TICKS(LOCAL_BRIDGE_TIMEOUT_MS + 50)); // The bridge times out
    xSemaphoreGive(ownedMaster.sendGate);
    bool injected = waitFlag(inject.done, 500);
    void* ownerAfter = ownedMaster.owner();
    Modbus::Frame exception;
    bool gotException = ownerBridgeSlave.getReceivedFrame(exception);

    ownerBridgeSlave.transmitFrame(LOCAL_BRIDGE_REQUEST); // Next request
    ownedMaster.transmitFrame(localResponse(0xB0B0));
    Modbus::Frame forwarded;
    bool gotResponse = ownerBridgeSlave.getReceivedFrame(forwarded);
    void* ownerAtEnd = ownedMaster.owner();
    bridge->~Bridge();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(held && injected, "request not held before its send");
    TEST_ASSERT_TRUE_MESSAGE(gotException, "no gateway exception on timeout");
    TEST_ASSERT_EQUAL_HEX8(Modbus::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND, exception.exceptionCode);
    TEST_ASSERT_NULL_MESSAGE(ownerAfter, "transaction left claimed by a request closed before its send");
    TEST_ASSERT_TRUE_MESSAGE(gotResponse, "next request not answered");
    TEST_ASSERT_EQUAL_HEX16(0xB0B0, forwarded.data[0]);
    TEST_ASSERT_NULL_MESSAGE(ownerAtEnd, "transaction not released after the forwarded response");
}

// The bridge timeout must stay below the TCP server's transaction safety timeout: otherwise the
// server side could close a request's transaction while the bridge still waits for its device,
// and the late answer would go to the next request (with its transaction ID)
void test_bridge_timeout_below_tcp_safety_timeout() {
    static ModbusInterface::Dummy slaveSide;
    static DummyMaster masterSide;
    constexpr uint32_t safetyMs = ModbusInterface::TCP::TCP_TRANSACTION_SAFETY_TIMEOUT_MS;

    auto* bridge = new (ownerBridgeStorage) Modbus::Bridge(masterSide, slaveSide, safetyMs);
    Modbus::Bridge::Result tooLong = bridge->begin();
    bridge->~Bridge();
    bridge = new (ownerBridgeStorage) Modbus::Bridge(masterSide, slaveSide, safetyMs - 1);
    Modbus::Bridge::Result below = bridge->begin();
    bridge->~Bridge();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(Modbus::Bridge::ERR_INIT_FAILED, tooLong, "bridge timeout not below the TCP safety timeout accepted");
    TEST_ASSERT_EQUAL(Modbus::Bridge::SUCCESS, below);
}

// Interfaces without owners keep their abort calls: on timeout & on destruction while active
// only (not after a response, nor on destruction while idle), Client & Bridge alike.
void test_custom_interface_abort_calls() {
    controlledMaster.reset();
    auto* client = new (concurrencyClientStorage) Modbus::Client(controlledMaster, LOCAL_CLIENT_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());
    Modbus::Frame resp;
    Modbus::Client::Result tracker = Modbus::Client::NODATA;
    volatile Modbus::Client::Result& result = tracker; // Written by the timer daemon

    client->sendRequest(LOCAL_BRIDGE_REQUEST, resp, &tracker);
    controlledMaster.transmitFrame(localResponse(1));
    uint32_t afterResponse = controlledMaster.aborts;

    tracker = Modbus::Client::NODATA;
    client->sendRequest(LOCAL_BRIDGE_REQUEST, resp, &tracker);
    uint32_t t0 = Modbus::TIME_MS();
    while (result == Modbus::Client::NODATA && Modbus::TIME_MS() - t0 < LOCAL_CLIENT_TIMEOUT_MS + 100) vTaskDelay(1);
    uint32_t afterTimeout = controlledMaster.aborts;

    client->~Client(); // Idle
    uint32_t afterIdleDestroy = controlledMaster.aborts;
    client = new (concurrencyClientStorage) Modbus::Client(controlledMaster, LOCAL_CLIENT_TIMEOUT_MS);
    client->begin();
    client->sendRequest(LOCAL_BRIDGE_REQUEST, resp, &tracker);
    client->~Client(); // Active
    uint32_t afterActiveDestroy = controlledMaster.aborts;

    static ModbusInterface::Dummy slaveSide;
    auto* bridge = new (ownerBridgeStorage) Modbus::Bridge(controlledMaster, slaveSide, LOCAL_BRIDGE_TIMEOUT_MS);
    bridge->begin();
    slaveSide.transmitFrame(LOCAL_BRIDGE_REQUEST);
    controlledMaster.transmitFrame(localResponse(2));
    uint32_t afterForwarded = controlledMaster.aborts;
    slaveSide.transmitFrame(LOCAL_BRIDGE_REQUEST);
    vTaskDelay(pdMS_TO_TICKS(LOCAL_BRIDGE_TIMEOUT_MS + 50));
    uint32_t afterBridgeTimeout = controlledMaster.aborts;
    bridge->~Bridge();

    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(0, afterResponse, "aborted after a response");
    TEST_ASSERT_EQUAL_MESSAGE(1, afterTimeout, "not aborted on timeout");
    TEST_ASSERT_EQUAL_MESSAGE(1, afterIdleDestroy, "aborted on destruction while idle");
    TEST_ASSERT_EQUAL_MESSAGE(2, afterActiveDestroy, "not aborted on destruction while active");
    TEST_ASSERT_EQUAL_MESSAGE(2, afterForwarded, "bridge aborted after a forwarded response");
    TEST_ASSERT_EQUAL_MESSAGE(3, afterBridgeTimeout, "bridge didn't abort on timeout");
}

// The timer can't be re-armed (timer command queue full): the request isn't started, and nothing
// is published nor delivered.
static volatile uint32_t rearmFailureCallbacks = 0;

void test_client_timer_rearm_failure() {
    controlledMaster.reset();
    auto* client = new (concurrencyClientStorage) Modbus::Client(controlledMaster, LOCAL_CLIENT_LONG_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, client->begin());

    holdTimerDaemon();
    uint32_t queued = 0;
    while (xTimerPendFunctionCall([](void*, uint32_t) {}, nullptr, 0, 0) == pdPASS) queued++;
    rearmFailureCallbacks = 0;
    Modbus::Client::Result sendRes = client->sendRequest(LOCAL_BRIDGE_REQUEST,
        [](Modbus::Client::Result, const Modbus::Frame*, void*) { rearmFailureCallbacks = rearmFailureCallbacks + 1; }, nullptr);
    bool readyAfter = client->isReady();
    releaseTimerDaemon();
    vTaskDelay(pdMS_TO_TICKS(20));
    uint32_t callbacks = rearmFailureCallbacks;
    client->~Client();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(queued > 0, "timer command queue not filled");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Client::SUCCESS, sendRes, "request started without its timer");
    TEST_ASSERT_TRUE_MESSAGE(readyAfter, "client not ready after a failed start");
    TEST_ASSERT_EQUAL_MESSAGE(0, callbacks, "failed start delivered a completion");
}

// Regression: if the timer command couldn't be queued, the request was forwarded without
// any timeout. The bridge must answer instead (the requester can retry).
void test_bridge_timer_command_failure() {
    static ModbusInterface::Dummy slaveSide;
    static DummyMaster masterSide;
    static Modbus::Bridge localBridge(masterSide, slaveSide, LOCAL_BRIDGE_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Bridge::SUCCESS, localBridge.begin());

    // Saturate the timer command queue: suspend the timer daemon so it can't drain it
    static StaticTimer_t floodTimerBuf;
    TimerHandle_t floodTimer = xTimerCreateStatic("flood", pdMS_TO_TICKS(1000), pdFALSE, nullptr,
                                                  [](TimerHandle_t) {}, &floodTimerBuf);
    TaskHandle_t daemon = xTimerGetTimerDaemonTaskHandle();
    vTaskSuspend(daemon);
    while (xTimerReset(floodTimer, 0) == pdPASS) {}

    slaveSide.transmitFrame(LOCAL_BRIDGE_REQUEST);
    Modbus::Frame forwardedFrame, answer;
    bool forwarded = masterSide.getReceivedFrame(forwardedFrame);
    bool answered = slaveSide.getReceivedFrame(answer);

    vTaskResume(daemon);
    xTimerStop(floodTimer, pdMS_TO_TICKS(100));

    TEST_ASSERT_START();
    TEST_ASSERT_FALSE_MESSAGE(forwarded, "request forwarded without a timeout");
    TEST_ASSERT_TRUE_MESSAGE(answered, "no answer when the timeout can't be armed");
    TEST_ASSERT_EQUAL(Modbus::SLAVE_DEVICE_BUSY, answer.exceptionCode);
}

// Regression: the TX result of a request that already timed out closed the request forwarded
// meanwhile (it closed "the pending request", whichever it was): B was sent, then answered
// GATEWAY_PATH_UNAVAILABLE because of A, and B's response was dropped. A request arriving while
// the previous TX result is missing is now answered SLAVE_DEVICE_BUSY, like a requester the
// bridge can't serve yet.
void test_bridge_late_tx_result_spares_next() {
    static ModbusInterface::Dummy slaveSide;
    controlledMaster.reset();
    Modbus::Frame frame;
    while (controlledMaster.getReceivedFrame(frame)) {} // Frames left by the previous tests
    alignas(Modbus::Bridge) static uint8_t storage[sizeof(Modbus::Bridge)];
    auto* bridge = new (storage) Modbus::Bridge(controlledMaster, slaveSide, LOCAL_BRIDGE_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(Modbus::Bridge::SUCCESS, bridge->begin());

    // A forwarded, its TX result held, then it times out
    controlledMaster.holdTx = true;
    slaveSide.transmitFrame(LOCAL_BRIDGE_REQUEST);
    vTaskDelay(pdMS_TO_TICKS(LOCAL_BRIDGE_TIMEOUT_MS + 50));
    bool aTimedOut = slaveSide.getReceivedFrame(frame) && frame.exceptionCode == Modbus::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND;

    // B (another address) while A's TX result is missing
    controlledMaster.holdTx = false;
    Modbus::Frame requestB = LOCAL_BRIDGE_REQUEST;
    requestB.regAddress = LOCAL_BRIDGE_REQUEST.regAddress + 1;
    slaveSide.transmitFrame(requestB);
    bool bForwardedEarly = controlledMaster.getReceivedFrame(frame);
    bool bBusy = slaveSide.getReceivedFrame(frame) && frame.exceptionCode == Modbus::SLAVE_DEVICE_BUSY;

    // A's late TX failure: nothing left to close
    controlledMaster.deliverTxResult(ModbusInterface::IInterface::ERR_SEND_FAILED);
    bool lateAnswer = slaveSide.getReceivedFrame(frame);

    // B again: forwarded & answered
    slaveSide.transmitFrame(requestB);
    bool bForwarded = controlledMaster.getReceivedFrame(frame);
    Modbus::Frame responseB = requestB;
    responseB.type = Modbus::RESPONSE;
    responseB.data[0] = 0x1234;
    controlledMaster.transmitFrame(responseB);
    bool bAnswered = slaveSide.getReceivedFrame(frame) && frame.exceptionCode == Modbus::NULL_EXCEPTION && frame.data[0] == 0x1234;

    bridge->~Bridge();
    controlledMaster.reset();

    TEST_ASSERT_START();
    TEST_ASSERT_TRUE_MESSAGE(aTimedOut, "A didn't time out");
    TEST_ASSERT_FALSE_MESSAGE(bForwardedEarly, "B forwarded while A's TX result was missing");
    TEST_ASSERT_TRUE_MESSAGE(bBusy, "B not answered SLAVE_DEVICE_BUSY");
    TEST_ASSERT_FALSE_MESSAGE(lateAnswer, "A's late TX result answered the slave side");
    TEST_ASSERT_TRUE_MESSAGE(bForwarded, "B not forwarded once A's TX result came back");
    TEST_ASSERT_TRUE_MESSAGE(bAnswered, "B's response not forwarded");
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

// Regression: the Logger's first use wasn't serialized: two tasks logging for the first time at
// once could both initialize it (two log tasks, a leaked queue). Probed at the start of setup(),
// before anything logs: two tasks, one per core, log at the same time. Log tasks are counted by
// name: other tasks may end meanwhile (e.g. Arduino's main task on the C6).
static int loggerTasksCreated = -1;
static StaticEventGroup_t loggerRaceStartBuf;
static EventGroupHandle_t loggerRaceStart;
static StaticTask_t loggerRacerTcb[2];
static StackType_t loggerRacerStack[2][4096];
static volatile bool loggerRacerDone[2];

static void loggerRacer(void* arg) {
    int idx = (int)(intptr_t)arg;
    xEventGroupWaitBits(loggerRaceStart, BIT0, pdFALSE, pdTRUE, portMAX_DELAY);
    Modbus::Logger::logln("[setup] Logger first use");
    loggerRacerDone[idx] = true;
    vTaskDelete(nullptr);
}

static int countLogTasks() {
    static TaskStatus_t tasks[32];
    UBaseType_t n = uxTaskGetSystemState(tasks, 32, nullptr);
    int count = 0;
    for (UBaseType_t i = 0; i < n; i++) count += strcmp(tasks[i].pcTaskName, "LogTask") == 0;
    return count;
}

static void probeLoggerFirstUse() {
    loggerRaceStart = xEventGroupCreateStatic(&loggerRaceStartBuf);
    int tasksBefore = countLogTasks();
    for (int i = 0; i < 2; i++) {
        xTaskCreateStaticPinnedToCore(loggerRacer, "loggerRacer", sizeof(loggerRacerStack[i]) / sizeof(StackType_t),
                                      (void*)(intptr_t)i, 2, loggerRacerStack[i], &loggerRacerTcb[i],
                                      i < portNUM_PROCESSORS ? i : tskNO_AFFINITY); // Single core: no race
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Both racers waiting
    xEventGroupSetBits(loggerRaceStart, BIT0);
    uint32_t t0 = Modbus::TIME_MS();
    while (!(loggerRacerDone[0] && loggerRacerDone[1]) && Modbus::TIME_MS() - t0 < 1000) vTaskDelay(1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Racers deleted
    loggerTasksCreated = countLogTasks() - tasksBefore;
}

void test_logger_first_use_starts_one_task() {
#ifdef EZMODBUS_DEBUG
    TEST_IGNORE_MESSAGE("debug build: the library logs before setup(), the probe doesn't see the first use");
#endif
    TEST_ASSERT_START();
    TEST_ASSERT_EQUAL_MESSAGE(1, loggerTasksCreated, "tasks created by the Logger's first use");
}

void setup() {
    // Debug port
    Serial.setTxBufferSize(2048);
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);
    probeLoggerFirstUse(); // Before anything logs
    
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
    
    RUN_TEST(test_logger_first_use_starts_one_task);
    RUN_TEST(test_read_holding_register);
    RUN_TEST(test_write_holding_register);
    RUN_TEST(test_bridge_silent_slave_gateway_exception);
    RUN_TEST(test_bridge_forward_failure_gateway_exception);
    RUN_TEST(test_bridge_late_response_not_forwarded);
    RUN_TEST(test_bridge_malformed_request_exception);
    RUN_TEST(test_bridge_destroyed_with_armed_timer);
    RUN_TEST(test_apps_remove_rcv_callbacks_on_destruction);
    RUN_TEST(test_client_destroyed_while_timeout_runs);
    RUN_TEST(test_server_begin_retry_registers_once);
    RUN_TEST(test_server_begin_rejects_null_interface);
    RUN_TEST(test_server_rejects_overlaps_at_zero_and_same_start);
    RUN_TEST(test_server_busy_ignores_other_slave_ids);
    RUN_TEST(test_server_slave_id_change_spares_waiting_request);
    RUN_TEST(test_server_rejects_start_inside_word);
    RUN_TEST(test_server_add_words_all_or_nothing);
    RUN_TEST(test_bridge_begin_registers_once);
    RUN_TEST(test_bridge_answered_request_timer_spares_next);
    RUN_TEST(test_client_sync_send_failure_applied_once);
    RUN_TEST(test_client_late_timeout_callback_spares_next);
    RUN_TEST(test_client_sync_fallback_spares_next);
    RUN_TEST(test_client_late_tx_result_gates_next);
    RUN_TEST(test_owner_timeout_before_send_async);
    RUN_TEST(test_owner_timeout_before_send_sync);
    RUN_TEST(test_owner_late_tx_result_spares_next);
    RUN_TEST(test_owner_sync_send_failure_releases);
    RUN_TEST(test_owner_released_before_completion_callback);
    RUN_TEST(test_owner_response_before_tx_result_dropped);
    RUN_TEST(test_owner_bridge_timeout_before_send);
    RUN_TEST(test_bridge_timeout_below_tcp_safety_timeout);
    RUN_TEST(test_custom_interface_abort_calls);
    RUN_TEST(test_client_timer_rearm_failure);
    RUN_TEST(test_client_zero_timeout); // Last: crashed (assert) before the fix
    RUN_TEST(test_bridge_timer_command_failure);
    RUN_TEST(test_bridge_late_tx_result_spares_next);
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