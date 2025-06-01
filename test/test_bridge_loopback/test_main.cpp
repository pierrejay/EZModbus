#include <unity.h>
#include <EZModbus.h>
#include "test_params.h"
#include "dummy_interface.h"
#include <WiFi.h>
#include <utils/ModbusLogger.h>
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

// Pin definitions
#define MBT_RX D7
#define MBT_TX D8
#define EZM_RX D5
#define EZM_TX D6

ModbusHAL::UART ezmUart(UART_NUM_1, 9600, ModbusHAL::UART::CONFIG_8N1, EZM_RX, EZM_TX);
ModbusHAL::UART mbtUart(UART_NUM_2, 9600, ModbusHAL::UART::CONFIG_8N1, MBT_RX, MBT_TX);
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
Modbus::Server server(mbt);
uint16_t serverDiscreteInputs[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverCoils[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverHoldingRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
uint16_t serverInputRegisters[MBT_INIT_START_REG + MBT_INIT_REG_COUNT];
Modbus::Server::ReadCallback serverReadCallback = [](const Modbus::Server::Register& reg) -> uint16_t {
    switch (reg.type) {
        case Modbus::HOLDING_REGISTER:
            return serverHoldingRegisters[reg.address];
        case Modbus::INPUT_REGISTER:
            return serverInputRegisters[reg.address];
        case Modbus::COIL:
            return serverCoils[reg.address];
        case Modbus::DISCRETE_INPUT:
            return serverDiscreteInputs[reg.address];
        default:
            return 0;
    }
};
Modbus::Server::WriteCallback serverWriteCallback = [](uint16_t value, const Modbus::Server::Register& reg) -> bool {
    switch (reg.type) {
        case Modbus::HOLDING_REGISTER:
            serverHoldingRegisters[reg.address] = value;
            return true;
        case Modbus::COIL:
            serverCoils[reg.address] = value;
            return true;
        default:
            return false;
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

    // Initialize Modbus RTU test server
    auto ifcInitRes = mbt.begin();
    if (ifcInitRes != ModbusInterface::IInterface::SUCCESS) {
        Modbus::Logger::logln("[ModbusTestServerTask] EZModbus RTU interface initialization failed");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    Modbus::Logger::logln("[ModbusTestServerTask] EZModbus RTU interface initialized");
    auto srvInitRes = server.begin();
    if (srvInitRes != Modbus::Server::SUCCESS) {
        Modbus::Logger::logln("[ModbusTestServerTask] EZModbus Server initialization failed");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    Modbus::Logger::logln("[ModbusTestServerTask] EZModbus Server initialized");
    
    // Configure registers for each type
    uint8_t regTypes[] = {
        Modbus::HOLDING_REGISTER,
        Modbus::INPUT_REGISTER,
        Modbus::COIL,
        Modbus::DISCRETE_INPUT
    };
    
    for (int i = MBT_INIT_START_REG; i < MBT_INIT_START_REG + MBT_INIT_REG_COUNT; i++) {
        for (uint8_t rt : regTypes) {
            Modbus::Server::Register reg;
            reg.type = (Modbus::RegisterType)rt;
            reg.address = i;
            reg.readCb = serverReadCallback;
            if (rt == Modbus::HOLDING_REGISTER || rt == Modbus::COIL) {
                reg.writeCb = serverWriteCallback;
            }
            server.addRegister(reg);
        }
    }

    // Set initial values
    resetModbusTestServerRegisters();

    // Poll the Modbus test server continuously (dummy server)
    while (true) {
        modbusTestServerTaskInitialized = true;
        server.poll();
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
    UNITY_END();
}

void loop() {
    // Nothing to do here
    Serial.println("Idling...");
    vTaskDelay(pdMS_TO_TICKS(1000));
}