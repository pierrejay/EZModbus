/**
 * @file ModbusHAL_TCP.h
 * @brief Multi-platform HAL wrapper for TCP sockets (header)
 */

#pragma once

#include "core/ModbusCore.h"
#include "utils/ModbusDebug.hpp"

// ===================================================================================
// EXTERNAL DEPENDENCIES (PLATFORM SPECIFIC)
// ===================================================================================

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
    #include <sys/socket.h>
    #include <netdb.h>
    #include <fcntl.h>
    #include <string.h>
    #include <arpa/inet.h>
    #include "esp_netif.h"
#elif defined(PICO_SDK)
    #include "../../external/pico-freertos-CH9120Driver/include/CH9120Driver.hpp"
#endif

// ===================================================================================
// CONFIGURATION CONSTANTS
// ===================================================================================

#ifndef EZMODBUS_HAL_TCP_MAX_ACTIVE_SOCKETS // TCP max active sockets (#)
    #define EZMODBUS_HAL_TCP_MAX_ACTIVE_SOCKETS 4
#endif
#ifndef EZMODBUS_HAL_TCP_RX_Q_SIZE // TCP RX queue size (# of frames to be signaled)
    #define EZMODBUS_HAL_TCP_RX_Q_SIZE 16
#endif
#ifndef EZMODBUS_HAL_TCP_TASK_STACK_SIZE // TCP RX/TX task stack size (bytes)
    #define EZMODBUS_HAL_TCP_TASK_STACK_SIZE BYTES_TO_STACK_SIZE(4096)
#endif

namespace ModbusHAL {

class TCP {
public:
    enum CfgMode { UNINIT, SERVER, CLIENT };

    static constexpr size_t MAX_ACTIVE_SOCKETS = (size_t)EZMODBUS_HAL_TCP_MAX_ACTIVE_SOCKETS;
    static constexpr size_t RX_QUEUE_SIZE = (size_t)EZMODBUS_HAL_TCP_RX_Q_SIZE;
    static constexpr size_t TCP_TASK_STACK_SIZE = (size_t)EZMODBUS_HAL_TCP_TASK_STACK_SIZE;
    static constexpr size_t MAX_MODBUS_FRAME_SIZE = 260;  // Modbus TCP max frame size (MBAP + PDU)
    static constexpr uint32_t SELECT_TIMEOUT_MS = 1000;  // Select timeout in milliseconds (1s = low CPU usage)
    
    // Recovery configuration constants
    static constexpr int MAX_SELECT_ERRORS = 5;               // Max select() errors before long sleep
    static constexpr uint32_t SELECT_RECOVERY_SLEEP_MS = 10000; // Sleep after MAX_SELECT_ERRORS (10s)
    static constexpr uint32_t SELECT_BACKOFF_BASE_MS = 1000;   // Progressive backoff base (1s)
    static constexpr int MAX_EMPTY_HITS = 3;                   // Max empty rounds before anti-spin pause
    static constexpr uint32_t ANTI_SPIN_DELAY_MS = 10;        // Anti-spin EAGAIN delay (10ms)

    // Structure for messages exchanged between HAL and Modbus layer
    struct TCPMsg {
        uint8_t payload[MAX_MODBUS_FRAME_SIZE];
        size_t len;
        int socketNum;  // Socket descriptor: source (RX) or destination (TX)
    };

    // ===================================================================================
    // PLATFORM-SPECIFIC CONSTRUCTORS & CONFIGURATION
    // ===================================================================================

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
    TCP();
    explicit TCP(uint16_t serverPort);                       // Server
    TCP(const char* serverIP, uint16_t port);                // Client
#elif defined(PICO_SDK)
    using CH9120Config = CH9120Driver::HardwareConfig;  // Alias for backward compatibility
    using Config = CH9120Config;  // Default config type for RP2040

    // Unified constructor - auto-detects mode from NetworkConfig
    TCP(const CH9120Config& hwConfig, const CH9120Driver::NetworkConfig& netConfig);

    // Raw constructors (not used in public API)
    TCP(uart_inst_t* uart, uint32_t baudrate, 
        uint8_t txPin, uint8_t rxPin,
        uint8_t cfgPin, uint8_t resPin, uint8_t statusPin);

    explicit TCP(const CH9120Config& cfg);
#endif

    ~TCP();

    // Storage for FreeRTOS objects
    StaticQueue_t _rxQueueBuf;
    uint8_t _rxQueueStorage[RX_QUEUE_SIZE * sizeof(int)];
    
    // Disable copy and assign
    TCP(const TCP&) = delete;
    TCP& operator=(const TCP&) = delete;

    // ===================================================================================
    // COMMON PUBLIC API
    // ===================================================================================

    // Setup methods
    bool begin();
    bool beginServer(uint16_t port, uint32_t ip = 0); // ip parameter meaning varies by platform
    bool beginClient(const char* serverIP, uint16_t port);
    void stop();

    // Main API (compatible across platforms)
    bool sendMsg(const uint8_t* payload, const size_t len, const int destSocket = -1, int* actualSocket = nullptr);
    size_t readSocketData(int socketNum, uint8_t* dst, size_t maxLen);

    // Monitoring
    size_t getActiveSocketCount();
    bool isServerRunning() const;
    bool isClientConnected();

    // Get HAL configuration mode (server/client/uninit)
    CfgMode getMode() const { return _cfgMode; }

    // Give access to RX queue for QueueSet integration (read only)
    QueueHandle_t getRxQueueHandle() const { return _rxQueue; }

private:
    // ===================================================================================
    // PLATFORM-SPECIFIC PRIVATE MEMBERS
    // ===================================================================================

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
    // ESP32-specific members
    static void tcpTask(void* param);
    StaticTask_t _tcpTaskBuf;
    StackType_t _tcpTaskStack[TCP_TASK_STACK_SIZE];
    void runTcpTask();

    bool setupServerSocket(uint16_t port, uint32_t ip);
    bool setupClientSocket(const char* serverIP, uint16_t port);
    void closeSocket(int sock);
    void cleanupDeadSockets();  // Clean up dead sockets (EBADF recovery)

    TaskHandle_t _tcpTaskHandle;
    QueueHandle_t _rxQueue;
    int _activeSockets[MAX_ACTIVE_SOCKETS] = {0};
    size_t _activeSocketCount = 0;
    int _listenSocket;
    int _clientSocket;
    bool _isServer;
    volatile bool _isRunning;
    Mutex _socketsMutex;

#elif defined(PICO_SDK)
    // RP2040/CH9120-specific members
    static void tcpTask(void* param);
    StaticTask_t _tcpTaskBuf;
    StackType_t _tcpTaskStack[TCP_TASK_STACK_SIZE];
    void runTcpTask();

    bool setupConnection();
    void closeConnection();

    CH9120Driver _ch9120;  // Owned instance, not a reference!
    TaskHandle_t _tcpTaskHandle;
    QueueHandle_t _rxQueue;
    
    volatile bool _isConnected;
    volatile bool _isRunning;
    Mutex _connectionMutex;
    CH9120Driver::NetworkConfig _networkConfig;
#endif

    // Config stored if constructor parameters are provided
    CfgMode _cfgMode = CfgMode::UNINIT;
    char _cfgIP[16] = {0}; // Enough to hold standard dotted IPv4 string
    uint16_t _cfgPort = 0;
};

} // namespace ModbusHAL