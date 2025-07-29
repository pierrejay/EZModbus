/**
 * @file ModbusHAL_TCP_Pico_CH9120.inl
 * @brief Pico/CH9120 implementation of ModbusHAL::TCP
 */

#include "ModbusHAL_TCP.h"
#include <cstring>
#include <cstdio>

namespace ModbusHAL {

// ===================================================================================
// CONSTRUCTOR & DESTRUCTOR
// ===================================================================================

// Direct parameters constructor
TCP::TCP(uart_inst_t* uart, uint32_t baudrate, 
    uint8_t txPin, uint8_t rxPin,
    uint8_t cfgPin, uint8_t resPin, uint8_t statusPin)
: _ch9120(uart, baudrate, txPin, rxPin, cfgPin, resPin, statusPin),
 _tcpTaskHandle(nullptr),
 _rxQueue(nullptr),
 _isConnected(false),
 _isRunning(false)
{
Modbus::Debug::LOG_MSG("ModbusHAL::TCP (Pico/CH9120) created");
}

// Unified constructor - auto-detects mode from NetworkConfig
TCP::TCP(const CH9120Config& hwConfig, const CH9120Driver::NetworkConfig& netConfig)
: _ch9120(hwConfig.uart, hwConfig.baudrate, hwConfig.txPin, hwConfig.rxPin, 
         hwConfig.cfgPin, hwConfig.resPin, hwConfig.statusPin),
 _tcpTaskHandle(nullptr),
 _rxQueue(nullptr),
 _isConnected(false),
 _isRunning(false)
{
// Validate that mode is TCP (not UDP) - security feature
if (netConfig.mode == CH9120Driver::Mode::UDP_SERVER || 
   netConfig.mode == CH9120Driver::Mode::UDP_CLIENT) {
   Modbus::Debug::LOG_MSG("ModbusHAL::TCP ERROR: UDP modes not supported for Modbus TCP");
   _cfgMode = CfgMode::UNINIT;
   return;
}

// Store network config as-is (user has full control)
_networkConfig = netConfig;

// Auto-detect and configure based on mode
if (netConfig.mode == CH9120Driver::Mode::TCP_SERVER) {
   _cfgMode = CfgMode::SERVER;
   _cfgPort = netConfig.localPort;
   Modbus::Debug::LOG_MSGF("ModbusHAL::TCP configured as server on port %u", _cfgPort);
} else if (netConfig.mode == CH9120Driver::Mode::TCP_CLIENT) {
   _cfgMode = CfgMode::CLIENT;
   _cfgPort = netConfig.targetPort;
   
   Modbus::Debug::LOG_MSGF("ModbusHAL::TCP configured as client to %d.%d.%d.%d:%u", 
                          netConfig.targetIp[0], netConfig.targetIp[1], 
                          netConfig.targetIp[2], netConfig.targetIp[3], _cfgPort);
} else {
   Modbus::Debug::LOG_MSG("ModbusHAL::TCP ERROR: Invalid NetworkConfig mode");
   _cfgMode = CfgMode::UNINIT;
}

Modbus::Debug::LOG_MSG("ModbusHAL::TCP (Pico/CH9120) created with unified constructor");
}

TCP::~TCP() {
stop();
Modbus::Debug::LOG_MSG("ModbusHAL::TCP (Pico/CH9120) destroyed");
}

// ===================================================================================
// PUBLIC API
// ===================================================================================

bool TCP::begin() {
    if (_cfgMode == CfgMode::UNINIT) {
        Modbus::Debug::LOG_MSG("TCP: No configuration set, call beginServer() or beginClient() first");
        return false;
    }

    // Initialize CH9120 driver if not already done
    if (_ch9120.getDriverState() == CH9120Driver::STOPPED) {
        auto result = _ch9120.begin();
        if (result != CH9120Driver::SUCCESS) {
            Modbus::Debug::LOG_MSGF("TCP: Failed to initialize CH9120 driver: %s", 
                                   CH9120Driver::toString(result));
            return false;
        }
    }

    // Create RX queue for socket events (int = socket number, always 0 for CH9120)
    _rxQueue = xQueueCreateStatic(RX_QUEUE_SIZE, sizeof(int), _rxQueueStorage, &_rxQueueBuf);
    if (!_rxQueue) {
        Modbus::Debug::LOG_MSG("TCP: Failed to create RX queue");
        return false;
    }

    // Setup connection based on mode
    if (!setupConnection()) {
        vQueueDelete(_rxQueue);
        _rxQueue = nullptr;
        return false;
    }

    // Start monitoring task
    _isRunning = true;
    _tcpTaskHandle = xTaskCreateStatic(
        tcpTask,
        "ModbusHAL_TCP_Pico_CH9120",
        TCP_TASK_STACK_SIZE,
        this,
        tskIDLE_PRIORITY + 2,
        _tcpTaskStack,
        &_tcpTaskBuf
    );

    if (!_tcpTaskHandle) {
        Modbus::Debug::LOG_MSG("TCP: Failed to create TCP task");
        stop();
        return false;
    }

    Modbus::Debug::LOG_MSGF("TCP: Started in %s mode", 
                           (_cfgMode == CfgMode::CLIENT ? "CLIENT" : "SERVER"));
    return true;
}

bool TCP::beginServer(uint16_t port, uint32_t ip) {
    if (_isRunning) {
        Modbus::Debug::LOG_MSG("TCP: Already running, stop first");
        return false;
    }

    _cfgMode = CfgMode::SERVER;
    _cfgPort = port;
    strcpy(_cfgIP, "0.0.0.0"); // Server listens on all interfaces

    // Configure CH9120 for server mode
    _networkConfig.mode = CH9120Driver::Mode::TCP_SERVER;
    _networkConfig.localPort = port;
    _networkConfig.useDhcp = true; // Use DHCP by default

    Modbus::Debug::LOG_MSGF("TCP: Configured as server on port %d", port);
    return true;
}

bool TCP::beginClient(const char* serverIP, uint16_t port) {
    if (_isRunning) {
        Modbus::Debug::LOG_MSG("TCP: Already running, stop first");
        return false;
    }

    if (!serverIP || strlen(serverIP) >= sizeof(_cfgIP)) {
        Modbus::Debug::LOG_MSG("TCP: Invalid server IP");
        return false;
    }

    _cfgMode = CfgMode::CLIENT;
    _cfgPort = port;
    strcpy(_cfgIP, serverIP);

    // Configure CH9120 for client mode
    _networkConfig.mode = CH9120Driver::Mode::TCP_CLIENT;
    _networkConfig.targetPort = port;
    _networkConfig.useDhcp = true; // Use DHCP by default
    
    // Parse IP address
    if (sscanf(serverIP, "%hhu.%hhu.%hhu.%hhu", 
               &_networkConfig.targetIp[0], &_networkConfig.targetIp[1],
               &_networkConfig.targetIp[2], &_networkConfig.targetIp[3]) != 4) {
        Modbus::Debug::LOG_MSG("TCP: Failed to parse server IP");
        return false;
    }

    Modbus::Debug::LOG_MSGF("TCP: Configured as client to %s:%d", serverIP, port);
    return true;
}

void TCP::stop() {
    if (_isRunning) {
        _isRunning = false;
        
        // Wait for task to finish
        if (_tcpTaskHandle) {
            while (eTaskGetState(_tcpTaskHandle) != eDeleted) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            _tcpTaskHandle = nullptr;
        }
    }

    closeConnection();

    // Clean up RX queue
    if (_rxQueue) {
        vQueueDelete(_rxQueue);
        _rxQueue = nullptr;
    }

    _cfgMode = CfgMode::UNINIT;
    Modbus::Debug::LOG_MSG("TCP: Stopped");
}

bool TCP::sendMsg(const uint8_t* payload, const size_t len, const int destSocket, int* actualSocket) {
    if (!_isRunning) {
        return false;
    }
    // For a server we try to send even if previous client disconnected.
    // For a client the connection must be active.
    if (_cfgMode == CfgMode::CLIENT && !_isConnected) {
        return false;
    }

    if (!payload || len == 0 || len > MAX_MODBUS_FRAME_SIZE) {
        return false;
    }

    // CH9120 handles single connection, ignore destSocket
    int result = _ch9120.send(payload, len, pdMS_TO_TICKS(1000));
    
    if (result > 0) {
        if (actualSocket) *actualSocket = 0; // CH9120 uses single socket
        return true;
    }

    Modbus::Debug::LOG_MSGF("TCP: Send failed with error %d", -result);
    return false;
}

size_t TCP::readSocketData(int socketNum, uint8_t* dst, size_t maxLen) {
    if (!_isRunning || socketNum != 0) {
        return 0;
    }

    // Cohérence avec la logique de la tâche : en mode serveur, on essaie de lire les données
    // même si l'état _isConnected n'est pas encore à jour.
    if (_cfgMode == CfgMode::CLIENT && !_isConnected) {
        return 0;
    }

    if (!dst || maxLen == 0) {
        return 0;
    }

    // Try to read data from CH9120 (non-blocking)
    int result = _ch9120.recv(dst, maxLen, 0);
    
    if (result > 0) {
        return (size_t)result;
    } else if (result == 0) {
        return 0; // No data available
    } else {
        // Error occurred
        Modbus::Debug::LOG_MSGF("TCP: Recv failed with error %d", -result);
        return 0;
    }
}

size_t TCP::getActiveSocketCount() {
    return _isConnected ? 1 : 0;
}

bool TCP::isServerRunning() const {
    return (_cfgMode == CfgMode::SERVER) && _isRunning;
}

bool TCP::isClientConnected() {
    return (_cfgMode == CfgMode::CLIENT) && _isConnected;
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

bool TCP::setupConnection() {
    auto result = _ch9120.setConfig(_networkConfig);
    if (result != CH9120Driver::SUCCESS) {
        Modbus::Debug::LOG_MSGF("TCP: Failed to configure CH9120: %s", 
                               CH9120Driver::toString(result));
        return false;
    }

    if (_cfgMode == CfgMode::CLIENT) {
        // Client mode: connect to server
        int connectResult = _ch9120.connect(_networkConfig, pdMS_TO_TICKS(10000));
        if (connectResult != 0) {
            Modbus::Debug::LOG_MSGF("TCP: Client connection failed with error %d", -connectResult);
            return false;
        }
        _isConnected = true;
        Modbus::Debug::LOG_MSG("TCP: Client connected successfully");
    } else {
        // Server mode: bind and listen
        int bindResult = _ch9120.bind(_networkConfig);
        if (bindResult != 0) {
            Modbus::Debug::LOG_MSGF("TCP: Server bind failed with error %d", -bindResult);
            return false;
        }
        
        int listenResult = _ch9120.listen(1);
        if (listenResult != 0) {
            Modbus::Debug::LOG_MSGF("TCP: Server listen failed with error %d", -listenResult);
            return false;
        }
        
        Modbus::Debug::LOG_MSG("TCP: Server listening for connections");
    }

    return true;
}

void TCP::closeConnection() {
    if (_isConnected || _isRunning) {
        _ch9120.close();
        _isConnected = false;
        Modbus::Debug::LOG_MSG("TCP: Connection closed");
    }
}

void TCP::tcpTask(void* param) {
    TCP* self = static_cast<TCP*>(param);
    self->runTcpTask();
    vTaskDelete(nullptr);
}

void TCP::runTcpTask() {
    Modbus::Debug::LOG_MSG("TCP: Task started");

    // Get CH9120 event queues
    QueueHandle_t dataEventQueue = _ch9120.getDataEventQueueHandle();
    QueueHandle_t linkEventQueue = _ch9120.getLinkEventQueueHandle();

    // Create queue set for efficient multi-queue handling
    QueueSetHandle_t queueSet = xQueueCreateSet(20); // Size = sum of both queue lengths
    if (!queueSet) {
        Modbus::Debug::LOG_MSG("TCP: Failed to create queue set");
        return;
    }

    // Add both queues to the set with error checking
    if (dataEventQueue && xQueueAddToSet(dataEventQueue, queueSet) != pdPASS) {
        Modbus::Debug::LOG_MSG("TCP: ERROR - dataEventQueue already in another QueueSet!");
    }
    if (linkEventQueue && xQueueAddToSet(linkEventQueue, queueSet) != pdPASS) {
        Modbus::Debug::LOG_MSG("TCP: ERROR - linkEventQueue already in another QueueSet!");
    }

    while (_isRunning) {
        // Block until ANY queue has data (with timeout for system breathing)
        QueueHandle_t activeQueue = (QueueHandle_t)xQueueSelectFromSet(queueSet, pdMS_TO_TICKS(100));
        
        if (activeQueue == dataEventQueue) {
            // Handle data events - DRAIN PATTERN (process all pending)
            UartDmaDriver::Event dataEvent;
            while (xQueueReceive(dataEventQueue, &dataEvent, 0) == pdTRUE) {
                printf("[DEBUG] dataEvent received: type=%d, _isConnected=%d, _cfgMode=%d\n", 
                       dataEvent.type, _isConnected, (int)_cfgMode);
                
                // EXPERT 1 FIX: En mode serveur, traiter les données même si _isConnected=false
                // car un client peut envoyer des données avant que l'événement de lien soit traité
                bool shouldProcessData = (_cfgMode == CfgMode::SERVER) || (_isConnected);

                if (dataEvent.type == UartDmaDriver::EVT_DATA && shouldProcessData) {
                    // Si des données arrivent en mode serveur, c'est qu'un client est forcément connecté
                    // On s'assure que notre état interne est à jour
                    if (_cfgMode == CfgMode::SERVER && !_isConnected) {
                        _isConnected = true;
                        Modbus::Debug::LOG_MSG("TCP: Data arrived, ensuring connected state is true.");
                    }
                    
                    // Signal data available to ModbusTCP layer
                    int socketNum = 0; // CH9120 single connection
                    xQueueSend(_rxQueue, &socketNum, 0);
                    printf("[DEBUG] Data forwarded to ModbusTCP (_rxQueue)\n");
                } else if (dataEvent.type == UartDmaDriver::EVT_DATA && !shouldProcessData) {
                    printf("[DEBUG] Data DROPPED - client mode and not connected yet!\n");
                }
            }
        }
        else if (activeQueue == linkEventQueue) {
            // Handle link events - DRAIN PATTERN (process all pending)
            CH9120Driver::LinkEvent linkEvent;
            while (xQueueReceive(linkEventQueue, &linkEvent, 0) == pdTRUE) {
                bool linkIsUp = (linkEvent.type == CH9120Driver::LINK_CONNECTED);
                
                printf("[DEBUG] linkEvent received: type=%d (%s), _isConnected was %d\n", 
                       linkEvent.type, linkIsUp ? "CONNECTED" : "DISCONNECTED", _isConnected);

                if (_cfgMode == CfgMode::SERVER) {
                    if (!linkIsUp) {
                        Modbus::Debug::LOG_MSG("TCP: Client disconnected from server.");
                    } else {
                         Modbus::Debug::LOG_MSG("TCP: Client connected to server.");
                         // Accept the new client connection
                         int acceptResult = _ch9120.accept(pdMS_TO_TICKS(1000));
                         if (acceptResult == 0) {
                             Modbus::Debug::LOG_MSG("TCP: Server accepted client connection");
                         } else {
                             printf("[DEBUG] accept() failed with result=%d\n", acceptResult);
                         }
                    }
                    _isConnected = linkIsUp;
                    printf("[DEBUG] _isConnected updated to %d\n", _isConnected);

                } else { // CfgMode::CLIENT
                    if (_isConnected != linkIsUp) {
                        _isConnected = linkIsUp;
                        Modbus::Debug::LOG_MSGF("TCP: Link state changed - %s", 
                                               _isConnected ? "CONNECTED" : "DISCONNECTED");
                        printf("[DEBUG] _isConnected updated to %d\n", _isConnected);
                    }
                }
            }
        }
        // If activeQueue is NULL (timeout), continue loop to allow system breathing
    }

    // Cleanup
    if (dataEventQueue) xQueueRemoveFromSet(dataEventQueue, queueSet);
    if (linkEventQueue) xQueueRemoveFromSet(linkEventQueue, queueSet);
    vQueueDelete(queueSet);
    
    Modbus::Debug::LOG_MSG("TCP: Task stopping");
}

} // namespace ModbusHAL