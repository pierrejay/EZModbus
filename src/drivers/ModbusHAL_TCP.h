/**
 * @file ModbusHAL_TCP.h
 * @brief Event-driven HAL wrapper for TCP sockets using ESP socket API (header)
 */

#pragma once

#include "core/ModbusCore.h"
#include "utils/ModbusDebug.hpp"

#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <arpa/inet.h>
#include "esp_netif.h"

#ifndef EZMODBUS_HAL_TCP_MAX_ACTIVE_SOCKETS // TCP max active sockets (#)
    #define EZMODBUS_HAL_TCP_MAX_ACTIVE_SOCKETS 4
#endif
#ifndef EZMODBUS_HAL_TCP_RX_Q_SIZE // TCP RX queue size (# of frames)
    #define EZMODBUS_HAL_TCP_RX_Q_SIZE 16
#endif
#ifndef EZMODBUS_HAL_TCP_TASK_STACK_SIZE // TCP RX/TX task stack size (bytes)
    #define EZMODBUS_HAL_TCP_TASK_STACK_SIZE 4096
#endif


namespace ModbusHAL {

class TCP {
public:
    enum CfgMode { UNINIT, SERVER, CLIENT };

    static constexpr size_t MAX_ACTIVE_SOCKETS = (size_t)EZMODBUS_HAL_TCP_MAX_ACTIVE_SOCKETS;
    static constexpr size_t RX_QUEUE_SIZE = (size_t)EZMODBUS_HAL_TCP_RX_Q_SIZE; // Number of Modbus frames the RX queue can hold
    static constexpr size_t TCP_TASK_STACK_SIZE = (size_t)EZMODBUS_HAL_TCP_TASK_STACK_SIZE;
    static constexpr size_t MAX_MODBUS_FRAME_SIZE = 260;  // Modbus TCP max frame size (MBAP + PDU)

    // Structure for messages exchanged between HAL and Modbus layer
    struct TCPMsg {
        uint8_t payload[MAX_MODBUS_FRAME_SIZE];
        size_t len;
        int socketNum;  // Socket descriptor: source (RX) or destination (TX)
    };

    TCP();
    explicit TCP(uint16_t serverPort);                       // Serveur
    TCP(const char* serverIP, uint16_t port);                // Client
    ~TCP();

    // // Storage for FreeRTOS objects
    // StaticTask_t _tcpTaskBuf;
    // StackType_t _tcpTaskStack[TCP_TASK_STACK_SIZE];
    StaticQueue_t _rxQueueBuf;
    uint8_t _rxQueueStorage[RX_QUEUE_SIZE * sizeof(int)];
    // Disable copy and assign
    TCP(const TCP&) = delete;
    TCP& operator=(const TCP&) = delete;

    // Setup methods
    bool begin();
    bool beginServer(uint16_t port, uint32_t ip = INADDR_ANY);
    bool beginClient(const char* serverIP, uint16_t port);
    void stop();

    // Main API
    bool sendMsg(const uint8_t* payload, const size_t len, const int destSocket = -1, int* actualSocket = nullptr);
    size_t readSocketData(int socketNum, uint8_t* dst, size_t maxLen);

    // Monitoring
    size_t getActiveSocketCount();
    bool isServerRunning() const;
    bool isClientConnected();
    bool isReady();

    // Get HAL configuration mode (server/client/uninit)
    CfgMode getMode() const { return _cfgMode; }

    // Give access to RX queue for QueueSet integration (read only)
    QueueHandle_t getRxQueueHandle() const { return _rxQueue; }

private:
    // Task for handling socket events and data
    static void tcpTask(void* param);
    // TaskHandle_t _tcpTaskHandle; -> defined in public to allow access from tests code
    StaticTask_t _tcpTaskBuf;
    StackType_t _tcpTaskStack[TCP_TASK_STACK_SIZE];
    void runTcpTask(); // Internal method called by the FreeRTOS task

    // Socket management
    bool setupServerSocket(uint16_t port, uint32_t ip);
    bool setupClientSocket(const char* serverIP, uint16_t port);
    void closeSocket(int sock);

    // Internal state
    TaskHandle_t _tcpTaskHandle;
    QueueHandle_t _rxQueue;
    // Fixed-size storage for active sockets (used in server mode)
    int _activeSockets[MAX_ACTIVE_SOCKETS] = {0};
    size_t _activeSocketCount = 0; // Current number of active sockets
    int _listenSocket;              // Server listen socket
    int _clientSocket;              // Client mode connected socket
    bool _isServer;                 // True if running in server mode
    volatile bool _isRunning;       // Task control flag
    
    // Mutex for protecting shared resources like _activeSockets
    Mutex _socketsMutex; 

    // Config stored if constructor parameters are provided
    CfgMode _cfgMode = CfgMode::UNINIT;
    char _cfgIP[16] = {0}; // Enough to hold standard dotted IPv4 string
    uint16_t _cfgPort = 0;
};

} // namespace ModbusHAL
