/**
 * @file main.cpp
 * @brief EZModbus ESP-IDF smoke test: clients, server & bridge wired together on one chip
 *
 *   tcpDirect (TCP client) ──127.0.0.1:5020──> tcpSlave ─┐
 *                                                         ├─> server (slave ID 1)
 *   tcpGw (TCP client) ──127.0.0.1:5021──> bridge         │
 *                            └─> rtuMaster ══UART══> rtuSlave
 *
 * Self-checking: prints "SMOKE PASS" or "SMOKE FAIL" once done (ci/smoke.sh waits for it).
 * Wiring: cross-connect the two UARTs (see the pin map below).
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "EZModbus.h"

using UART         = ModbusHAL::UART;
using TCP          = ModbusHAL::TCP;
using ModbusRTU    = ModbusInterface::RTU;
using ModbusTCP    = ModbusInterface::TCP;
using ModbusClient = Modbus::Client;
using ModbusServer = Modbus::Server;
using ModbusBridge = Modbus::Bridge;

// ===================================================================================
// PIN MAP (two UARTs cross-wired: MASTER_TX -> SLAVE_RX, SLAVE_TX -> MASTER_RX)
// ===================================================================================

#if defined(CONFIG_IDF_TARGET_ESP32S3) // Seeed XIAO ESP32-S3 (same jumpers as the PlatformIO suites)
    #define MASTER_UART UART_NUM_1
    #define MASTER_RX   GPIO_NUM_6
    #define MASTER_TX   GPIO_NUM_43
    #define SLAVE_UART  UART_NUM_2
    #define SLAVE_RX    GPIO_NUM_44
    #define SLAVE_TX    GPIO_NUM_7
#elif defined(CONFIG_IDF_TARGET_ESP32C6) // HP UART1 <-> LP-UART (same jumpers as the PlatformIO suites)
    #define MASTER_UART UART_NUM_1
    #define MASTER_RX   GPIO_NUM_22
    #define MASTER_TX   GPIO_NUM_21
    #define SLAVE_UART  LP_UART_NUM_0
    #define SLAVE_RX    GPIO_NUM_4
    #define SLAVE_TX    GPIO_NUM_5
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    #define MASTER_UART UART_NUM_1
    #define MASTER_RX   GPIO_NUM_6
    #define MASTER_TX   GPIO_NUM_43
    #define SLAVE_UART  UART_NUM_2
    #define SLAVE_RX    GPIO_NUM_44
    #define SLAVE_TX    GPIO_NUM_7
#else // ESP32 & others with a 3rd UART, avoiding flash-bound pins
    #define MASTER_UART UART_NUM_1
    #define MASTER_RX   GPIO_NUM_18
    #define MASTER_TX   GPIO_NUM_19
    #define SLAVE_UART  UART_NUM_2
    #define SLAVE_RX    GPIO_NUM_16
    #define SLAVE_TX    GPIO_NUM_17
#endif

#define BAUD_RATE        115200
#define SLAVE_ID         1
#define UNKNOWN_SLAVE_ID 9
#define DIRECT_PORT      5020
#define GATEWAY_PORT     5021
#define LOOPBACK_IP      "127.0.0.1"

#define DIRECT_TIMEOUT_MS  500  // tcpDirect client timeout (the slow Word outlasts it)
#define SLOW_HANDLER_MS    800
#define BRIDGE_TIMEOUT_MS  300
#define GATEWAY_TIMEOUT_MS 1500 // tcpGw client timeout (outlasts the bridge timeout)

// ===================================================================================
// INSTANCES
// ===================================================================================

UART uartMaster({ .uartNum = MASTER_UART, .baud = BAUD_RATE, .config = UART::CONFIG_8N1,
                  .rxPin = MASTER_RX, .txPin = MASTER_TX, .dePin = GPIO_NUM_NC });
UART uartSlave ({ .uartNum = SLAVE_UART,  .baud = BAUD_RATE, .config = UART::CONFIG_8N1,
                  .rxPin = SLAVE_RX,  .txPin = SLAVE_TX,  .dePin = GPIO_NUM_NC });
ModbusRTU rtuMaster(uartMaster, Modbus::CLIENT);
ModbusRTU rtuSlave(uartSlave, Modbus::SERVER);

TCP halDirectSrv(DIRECT_PORT);
TCP halGatewaySrv(GATEWAY_PORT);
TCP halDirectCli(LOOPBACK_IP, DIRECT_PORT);
TCP halGatewayCli(LOOPBACK_IP, GATEWAY_PORT);
ModbusTCP tcpSlave(halDirectSrv, Modbus::SERVER);
ModbusTCP tcpBridgeSlave(halGatewaySrv, Modbus::SERVER);
ModbusTCP tcpDirectMaster(halDirectCli, Modbus::CLIENT);
ModbusTCP tcpGatewayMaster(halGatewayCli, Modbus::CLIENT);

// Server data
#define NB_COILS 64
#define NB_DI    16
#define NB_HR    125
#define NB_IR    10
#define HR32_ADDR 200 // 2-register Word behind handlers
#define SLOW_ADDR 300 // Input register whose handler outlasts the tcpDirect timeout
volatile uint16_t coils[NB_COILS];
volatile uint16_t dis[NB_DI];
volatile uint16_t hrs[NB_HR];
volatile uint16_t irs[NB_IR];
uint32_t hr32 = 0;

Modbus::StaticWordStore<NB_COILS + NB_DI + NB_HR + NB_IR + 3> store; // +2-register Word, slow Word, runtime Word
ModbusServer server({ &rtuSlave, &tcpSlave }, store, SLAVE_ID);
ModbusBridge bridge(tcpBridgeSlave, rtuMaster, BRIDGE_TIMEOUT_MS);
ModbusClient tcpDirect(tcpDirectMaster, DIRECT_TIMEOUT_MS);
ModbusClient tcpGw(tcpGatewayMaster, GATEWAY_TIMEOUT_MS);

// ===================================================================================
// CHECK HELPERS
// ===================================================================================

static int nbChecks = 0;
static int nbFails = 0;

#define CHECK(cond, fmt, ...) do {                                              \
    nbChecks++;                                                                 \
    if (!(cond)) {                                                              \
        nbFails++;                                                              \
        printf("  [FAIL] %s:%d " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);  \
    }                                                                           \
} while (0)

// A request that went through and got a regular (non-exception) answer
#define CHECK_OK(res, exc, what) \
    CHECK((res) == ModbusClient::SUCCESS && (exc) == Modbus::NULL_EXCEPTION, \
          "%s: %s / exception 0x%02X", what, ModbusClient::toString(res), (unsigned)(exc))

#define CHECK_EXC(res, exc, expected, what) \
    CHECK((res) == ModbusClient::SUCCESS && (exc) == (expected), \
          "%s: %s / exception 0x%02X (expected 0x%02X)", what, ModbusClient::toString(res), (unsigned)(exc), (unsigned)(expected))

static void section(const char* name) { printf("-- %s\n", name); }

// ===================================================================================
// SERVER SETUP
// ===================================================================================

static Modbus::ExceptionCode readHr32(const Modbus::Word&, uint16_t* out, void* ctx) {
    const uint32_t v = *static_cast<uint32_t*>(ctx);
    out[0] = (uint16_t)(v >> 16);
    out[1] = (uint16_t)(v & 0xFFFF);
    return Modbus::NULL_EXCEPTION;
}

static Modbus::ExceptionCode writeHr32(const uint16_t* in, const Modbus::Word&, void* ctx) {
    *static_cast<uint32_t*>(ctx) = ((uint32_t)in[0] << 16) | in[1];
    return Modbus::NULL_EXCEPTION;
}

static Modbus::ExceptionCode readSlow(const Modbus::Word&, uint16_t* out, void*) {
    vTaskDelay(pdMS_TO_TICKS(SLOW_HANDLER_MS));
    out[0] = 0x5A5A;
    return Modbus::NULL_EXCEPTION;
}

static bool setupServer() {
    // Coils through the array overload, holding registers through the vector one
    static Modbus::Word coilWords[NB_COILS];
    for (int i = 0; i < NB_COILS; i++) {
        coils[i] = (i % 3 == 0);
        coilWords[i] = { .type = Modbus::COIL, .startAddr = (uint16_t)i, .nbRegs = 1, .value = &coils[i] };
    }
    if (server.addWords(coilWords, NB_COILS) != ModbusServer::SUCCESS) return false;

    std::vector<Modbus::Word> hrWords;
    for (int i = 0; i < NB_HR; i++) {
        hrs[i] = 10 + i;
        hrWords.push_back({ .type = Modbus::HOLDING_REGISTER, .startAddr = (uint16_t)i, .nbRegs = 1, .value = &hrs[i] });
    }
    if (server.addWords(hrWords) != ModbusServer::SUCCESS) return false;

    for (int i = 0; i < NB_DI; i++) {
        dis[i] = (i % 2);
        if (server.addWord({ .type = Modbus::DISCRETE_INPUT, .startAddr = (uint16_t)i, .nbRegs = 1, .value = &dis[i] }) != ModbusServer::SUCCESS) return false;
    }
    for (int i = 0; i < NB_IR; i++) {
        irs[i] = 1000 + i;
        if (server.addWord({ .type = Modbus::INPUT_REGISTER, .startAddr = (uint16_t)i, .nbRegs = 1, .value = &irs[i] }) != ModbusServer::SUCCESS) return false;
    }

    Modbus::Word w32 = { .type = Modbus::HOLDING_REGISTER, .startAddr = HR32_ADDR, .nbRegs = 2 };
    w32.readHandler = readHr32;
    w32.writeHandler = writeHr32;
    w32.userCtx = &hr32;
    if (server.addWord(w32) != ModbusServer::SUCCESS) return false;

    Modbus::Word slow = { .type = Modbus::INPUT_REGISTER, .startAddr = SLOW_ADDR, .nbRegs = 1 };
    slow.readHandler = readSlow;
    return server.addWord(slow) == ModbusServer::SUCCESS;
}

// Runtime store changes (server running)
static void testStore() {
    section("word store");
    static volatile uint16_t extra = 77;
    Modbus::Word overlap = { .type = Modbus::HOLDING_REGISTER, .startAddr = HR32_ADDR + 1, .nbRegs = 1, .value = &extra };
    ModbusServer::Result res = server.addWord(overlap);
    CHECK(res == ModbusServer::ERR_WORD_OVERLAP, "overlapping Word: %s", ModbusServer::toString(res));
    CHECK(server.getWord(Modbus::HOLDING_REGISTER, HR32_ADDR + 1).startAddr == HR32_ADDR, "getWord inside a Word");
    res = server.addWord({ .type = Modbus::HOLDING_REGISTER, .startAddr = 400, .nbRegs = 1, .value = &extra });
    CHECK(res == ModbusServer::SUCCESS, "Word added at runtime: %s", ModbusServer::toString(res));
    uint16_t v = 0;
    Modbus::ExceptionCode exc;
    ModbusClient::Result rd = tcpDirect.read(SLAVE_ID, Modbus::HOLDING_REGISTER, 400, 1, &v, &exc);
    CHECK_OK(rd, exc, "read runtime Word");
    CHECK(v == 77, "runtime Word: %u", v);
}

// ===================================================================================
// SCENARIOS (run against both paths: direct TCP & TCP -> bridge -> RTU)
// ===================================================================================

static void testReadWrite(ModbusClient& c, const char* path) {
    printf("-- read / write (%s)\n", path);
    Modbus::ExceptionCode exc;
    ModbusClient::Result res;

    // Reads: every register type, single & full-range
    uint16_t regs[NB_HR];
    bool bits[NB_COILS];
    res = c.read(SLAVE_ID, Modbus::COIL, 0, NB_COILS, bits, &exc);
    CHECK_OK(res, exc, "read coils");
    for (int i = 0; i < NB_COILS; i++) CHECK(bits[i] == (coils[i] != 0), "coil %d", i);

    res = c.read(SLAVE_ID, Modbus::DISCRETE_INPUT, 0, NB_DI, bits, &exc);
    CHECK_OK(res, exc, "read discrete inputs");
    for (int i = 0; i < NB_DI; i++) CHECK(bits[i] == (dis[i] != 0), "discrete input %d", i);

    res = c.read(SLAVE_ID, Modbus::HOLDING_REGISTER, 0, NB_HR, regs, &exc); // 125 registers: longest frame
    CHECK_OK(res, exc, "read 125 holding registers");
    for (int i = 0; i < NB_HR; i++) CHECK(regs[i] == hrs[i], "holding register %d: %u != %u", i, regs[i], hrs[i]);

    res = c.read(SLAVE_ID, Modbus::INPUT_REGISTER, 3, 1, regs, &exc);
    CHECK_OK(res, exc, "read input register");
    CHECK(regs[0] == 1003, "input register 3: %u", regs[0]);

    // Writes: single & multiple, coils & registers
    const bool one = true;
    res = c.write(SLAVE_ID, Modbus::COIL, 1, 1, &one, &exc);
    CHECK_OK(res, exc, "write coil");
    CHECK(coils[1] != 0, "coil 1 not set");

    bool pattern[20];
    for (int i = 0; i < 20; i++) pattern[i] = (i % 2 == 0);
    res = c.write(SLAVE_ID, Modbus::COIL, 40, 20, pattern, &exc);
    CHECK_OK(res, exc, "write coils");
    for (int i = 0; i < 20; i++) CHECK((coils[40 + i] != 0) == pattern[i], "coil %d", 40 + i);

    const uint16_t single = 0xBEEF;
    res = c.write(SLAVE_ID, Modbus::HOLDING_REGISTER, 5, 1, &single, &exc);
    CHECK_OK(res, exc, "write register");
    CHECK(hrs[5] == 0xBEEF, "holding register 5: 0x%04X", hrs[5]);

    for (int i = 0; i < 123; i++) regs[i] = (uint16_t)(0x1000 + i); // 123 registers: max write
    res = c.write(SLAVE_ID, Modbus::HOLDING_REGISTER, 2, 123, regs, &exc);
    CHECK_OK(res, exc, "write 123 registers");
    for (int i = 0; i < 123; i++) CHECK(hrs[2 + i] == 0x1000 + i, "holding register %d", 2 + i);

    // Multi-register Word behind handlers, raw Frame API
    Modbus::Frame req = { .type = Modbus::REQUEST, .fc = Modbus::WRITE_MULTIPLE_REGISTERS,
                          .slaveId = SLAVE_ID, .regAddress = HR32_ADDR, .regCount = 2 };
    req.setRegisters({ 0x1234, 0x5678 });
    Modbus::Frame rsp;
    res = c.sendRequest(req, rsp);
    CHECK_OK(res, rsp.exceptionCode, "write 32-bit Word");
    CHECK(hr32 == 0x12345678, "32-bit Word: 0x%08lX", (unsigned long)hr32);

    res = c.read(SLAVE_ID, Modbus::HOLDING_REGISTER, HR32_ADDR + 1, 1, regs, &exc); // Inside the Word
    CHECK_EXC(res, exc, Modbus::ILLEGAL_DATA_ADDRESS, "read inside a Word");

    // Exceptions
    res = c.read(SLAVE_ID, Modbus::HOLDING_REGISTER, 1000, 1, regs, &exc);
    CHECK_EXC(res, exc, Modbus::ILLEGAL_DATA_ADDRESS, "read undefined register");
    req = { .type = Modbus::REQUEST, .fc = Modbus::WRITE_REGISTER, .slaveId = SLAVE_ID, .regAddress = 1000, .regCount = 1 };
    req.setRegisters({ 1 });
    res = c.sendRequest(req, rsp);
    CHECK_EXC(res, rsp.exceptionCode, Modbus::ILLEGAL_DATA_ADDRESS, "write undefined register");
}

// Every arithmetic type through read<T> / write<T>
template<typename T>
static void roundTrip(ModbusClient& c, const char* name, const T (&values)[3]) {
    Modbus::ExceptionCode exc;
    T back[3] = {};
    ModbusClient::Result res = c.write(SLAVE_ID, Modbus::HOLDING_REGISTER, 50, 3, values, &exc);
    CHECK_OK(res, exc, name);
    res = c.read(SLAVE_ID, Modbus::HOLDING_REGISTER, 50, 3, back, &exc);
    CHECK_OK(res, exc, name);
    for (int i = 0; i < 3; i++) CHECK(back[i] == values[i], "%s value %d", name, i);
}

static void testTypes(ModbusClient& c) {
    section("read<T> / write<T>");
    roundTrip<bool>(c, "bool", { true, false, true });
    roundTrip<int8_t>(c, "int8_t", { 0, 1, 127 });
    roundTrip<uint8_t>(c, "uint8_t", { 0, 128, 255 });
    roundTrip<int16_t>(c, "int16_t", { 0, 1000, 32767 });
    roundTrip<uint16_t>(c, "uint16_t", { 0, 1, 65535 });
    roundTrip<int32_t>(c, "int32_t", { 0, 2, 65535 });
    roundTrip<uint32_t>(c, "uint32_t", { 0, 3, 65535 });
    roundTrip<float>(c, "float", { 0.0f, 4.0f, 65535.0f });
    roundTrip<double>(c, "double", { 0.0, 5.0, 65535.0 });

    // Clamping: negative -> 0, above 16 bits -> 65535, register above T -> T max
    Modbus::ExceptionCode exc;
    const int32_t wide[2] = { -5, 70000 };
    ModbusClient::Result res = c.write(SLAVE_ID, Modbus::HOLDING_REGISTER, 50, 2, wide, &exc);
    CHECK_OK(res, exc, "clamped write");
    CHECK(hrs[50] == 0 && hrs[51] == 65535, "clamped write: %u %u", hrs[50], hrs[51]);
    int8_t narrow[2];
    res = c.read(SLAVE_ID, Modbus::HOLDING_REGISTER, 50, 2, narrow, &exc);
    CHECK_OK(res, exc, "clamped read");
    CHECK(narrow[0] == 0 && narrow[1] == 127, "clamped read: %d %d", narrow[0], narrow[1]);

    // Refused before sending
    const float notANumber = NAN;
    CHECK(c.write(SLAVE_ID, Modbus::HOLDING_REGISTER, 50, 1, &notANumber, &exc) == ModbusClient::ERR_INVALID_FRAME, "NaN write accepted");
}

static SemaphoreHandle_t asyncDone;
static StaticSemaphore_t asyncDoneBuf;
static ModbusClient::Result asyncResult;
static uint16_t asyncValue;

static void testAsync(ModbusClient& c, const char* path) {
    printf("-- async request (%s)\n", path);
    Modbus::Frame req = { .type = Modbus::REQUEST, .fc = Modbus::READ_INPUT_REGISTERS,
                          .slaveId = SLAVE_ID, .regAddress = 7, .regCount = 1 };
    asyncResult = ModbusClient::NODATA;
    ModbusClient::Result res = c.sendRequest(req, [](ModbusClient::Result r, const Modbus::Frame* rsp, void*) {
        asyncResult = r;
        asyncValue = rsp ? rsp->getRegister(0) : 0;
        xSemaphoreGive(asyncDone);
    });
    CHECK(res == ModbusClient::SUCCESS, "async send: %s", ModbusClient::toString(res));
    CHECK(xSemaphoreTake(asyncDone, pdMS_TO_TICKS(3000)) == pdTRUE, "async callback never called");
    CHECK(asyncResult == ModbusClient::SUCCESS && asyncValue == 1007, "async: %s / %u", ModbusClient::toString(asyncResult), asyncValue);
}

// Client timeout on a slow Word, late answer dropped, next request answered
static void testDirectTimeout() {
    section("client timeout & recovery (direct)");
    uint16_t v = 0;
    Modbus::ExceptionCode exc;
    ModbusClient::Result res = tcpDirect.read(SLAVE_ID, Modbus::INPUT_REGISTER, SLOW_ADDR, 1, &v, &exc);
    CHECK(res == ModbusClient::ERR_TIMEOUT, "slow Word: %s", ModbusClient::toString(res));
    vTaskDelay(pdMS_TO_TICKS(SLOW_HANDLER_MS)); // Let the late answer arrive
    res = tcpDirect.read(SLAVE_ID, Modbus::INPUT_REGISTER, 2, 1, &v, &exc);
    CHECK_OK(res, exc, "after timeout");
    CHECK(v == 1002, "after timeout: got %u (late answer taken?)", v);
}

// Silent RTU slave behind the bridge, then slave ID change
static void testGatewayTimeout() {
    section("bridge timeout & recovery, slave ID change (RTU)");
    uint16_t v = 0;
    Modbus::ExceptionCode exc;
    ModbusClient::Result res = tcpGw.read(UNKNOWN_SLAVE_ID, Modbus::INPUT_REGISTER, 2, 1, &v, &exc);
    CHECK_EXC(res, exc, Modbus::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND, "unknown slave");
    res = tcpGw.read(SLAVE_ID, Modbus::INPUT_REGISTER, 2, 1, &v, &exc);
    CHECK_OK(res, exc, "after bridge timeout");
    CHECK(v == 1002, "after bridge timeout: got %u", v);

    server.setSlaveId(2);
    res = tcpGw.read(SLAVE_ID, Modbus::INPUT_REGISTER, 2, 1, &v, &exc);
    CHECK_EXC(res, exc, Modbus::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND, "old slave ID");
    res = tcpGw.read(2, Modbus::INPUT_REGISTER, 2, 1, &v, &exc);
    CHECK_OK(res, exc, "new slave ID");
    server.setSlaveId(SLAVE_ID);
}

// Both clients hammer the server at once (one per interface)
#define BURST_ITER 100
struct BurstCtx { ModbusClient* client; uint16_t addr; int errors; TaskHandle_t parent; char firstError[64]; };

static void burstError(BurstCtx* ctx, uint16_t i, const char* step, ModbusClient::Result res, Modbus::ExceptionCode exc) {
    if (ctx->errors++ == 0) {
        snprintf(ctx->firstError, sizeof(ctx->firstError), "#%u %s: %s / 0x%02X", i, step, ModbusClient::toString(res), (unsigned)exc);
    }
}

static void burstTask(void* arg) {
    BurstCtx* ctx = static_cast<BurstCtx*>(arg);
    for (uint16_t i = 1; i <= BURST_ITER; i++) {
        Modbus::ExceptionCode exc = Modbus::NULL_EXCEPTION;
        uint16_t back = 0;
        ModbusClient::Result res = ctx->client->write(SLAVE_ID, Modbus::HOLDING_REGISTER, ctx->addr, 1, &i, &exc);
        if (res != ModbusClient::SUCCESS || exc != Modbus::NULL_EXCEPTION) { burstError(ctx, i, "write", res, exc); continue; }
        res = ctx->client->read(SLAVE_ID, Modbus::HOLDING_REGISTER, ctx->addr, 1, &back, &exc);
        if (res != ModbusClient::SUCCESS || exc != Modbus::NULL_EXCEPTION) { burstError(ctx, i, "read", res, exc); continue; }
        if (back != i) burstError(ctx, i, "read value", res, exc);
    }
    xTaskNotifyGive(ctx->parent);
    vTaskDelete(nullptr);
}

static void testBurst() {
    section("concurrent burst (both interfaces)");
    BurstCtx a = { &tcpDirect, 100, 0, xTaskGetCurrentTaskHandle(), "" };
    BurstCtx b = { &tcpGw, 101, 0, xTaskGetCurrentTaskHandle(), "" };
    const TickType_t start = xTaskGetTickCount();
    xTaskCreate(burstTask, "burstA", 4096, &a, 5, nullptr);
    xTaskCreate(burstTask, "burstB", 4096, &b, 5, nullptr);
    int done = 0;
    while (done < 2 && ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(30000)) > 0) done++;
    CHECK(done == 2, "burst tasks didn't finish");
    CHECK(a.errors == 0, "direct: %d errors, first %s", a.errors, a.firstError);
    CHECK(b.errors == 0, "gateway: %d errors, first %s", b.errors, b.firstError);
    CHECK(hrs[100] == BURST_ITER && hrs[101] == BURST_ITER, "burst final values: %u %u", hrs[100], hrs[101]);
    printf("   %d x 2 transactions per client in %lu ms\n", BURST_ITER, (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - start));
}

// ===================================================================================
// MAIN
// ===================================================================================

static bool waitReady(ModbusClient& c) {
    for (int i = 0; i < 50 && !c.isReady(); i++) vTaskDelay(pdMS_TO_TICKS(100));
    return c.isReady();
}

#define INIT(expr, ok) do { if ((expr) != (ok)) { printf("SMOKE FAIL: init %s\n", #expr); return; } } while (0)

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(1500)); // Let the host (re)open the console
    printf("SMOKE START idf=%s target=%s\n", esp_get_idf_version(), CONFIG_IDF_TARGET);
    asyncDone = xSemaphoreCreateBinaryStatic(&asyncDoneBuf);

    INIT(esp_netif_init(), ESP_OK); // lwIP stack, no network interface needed for 127.0.0.1
    INIT(uartMaster.begin(), ESP_OK);
    INIT(uartSlave.begin(), ESP_OK);
    INIT(halDirectSrv.begin(), true);
    INIT(halGatewaySrv.begin(), true);
    INIT(halDirectCli.begin(), true);
    INIT(halGatewayCli.begin(), true);
    INIT(rtuMaster.begin(), ModbusRTU::SUCCESS);
    INIT(rtuSlave.begin(), ModbusRTU::SUCCESS);
    INIT(tcpSlave.begin(), ModbusTCP::SUCCESS);
    INIT(tcpBridgeSlave.begin(), ModbusTCP::SUCCESS);
    INIT(tcpDirectMaster.begin(), ModbusTCP::SUCCESS);
    INIT(tcpGatewayMaster.begin(), ModbusTCP::SUCCESS);
    INIT(setupServer(), true);
    INIT(server.begin(), ModbusServer::SUCCESS);
    INIT(bridge.begin(), ModbusBridge::SUCCESS);
    INIT(tcpDirect.begin(), ModbusClient::SUCCESS);
    INIT(tcpGw.begin(), ModbusClient::SUCCESS);
    INIT(waitReady(tcpDirect), true);
    INIT(waitReady(tcpGw), true);

    const uint32_t heapStart = esp_get_free_heap_size();
    const TickType_t start = xTaskGetTickCount();

    testStore();
    testReadWrite(tcpDirect, "direct TCP");
    testReadWrite(tcpGw, "TCP -> bridge -> RTU");
    testTypes(tcpDirect);
    testAsync(tcpDirect, "direct TCP");
    testAsync(tcpGw, "TCP -> bridge -> RTU");
    testDirectTimeout();
    testGatewayTimeout();
    testBurst();

    // Timer callbacks (client & bridge timeouts) run on the timer daemon's stack
    printf("   timer daemon stack: %lu of %d bytes never used\n",
           (unsigned long)uxTaskGetStackHighWaterMark(xTimerGetTimerDaemonTaskHandle()) * sizeof(StackType_t),
           CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH);
    printf("   heap: %lu -> %lu bytes free (min %lu), %lu ms\n",
           (unsigned long)heapStart, (unsigned long)esp_get_free_heap_size(),
           (unsigned long)esp_get_minimum_free_heap_size(),
           (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - start));
    if (nbFails == 0) printf("SMOKE PASS (%d checks)\n", nbChecks);
    else printf("SMOKE FAIL (%d of %d checks failed)\n", nbFails, nbChecks);
}
