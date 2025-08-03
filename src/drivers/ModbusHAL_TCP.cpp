/**
 * @file ModbusHAL_TCP.cpp
 * @brief Event-driven HAL wrapper for TCP sockets using ESP socket API (implementation)
 */

#include "drivers/ModbusHAL_TCP.h"
#include "core/ModbusTypes.hpp" // For Mutex, Lock, TIME_MS, WAIT_MS

#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h> // For memset, memcpy
#include <arpa/inet.h> // For htons, inet_pton
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stddef.h> // For size_t
#include <algorithm> // For std::max

namespace ModbusHAL {

// Default constructor
TCP::TCP()
    : _tcpTaskHandle(nullptr),
      _rxQueue(nullptr),
      _listenSocket(-1),
      _clientSocket(-1),
      _isServer(false),
      _isRunning(false),
      _cfgMode(CfgMode::UNINIT),
      _cfgPort(0) {
    _activeSocketCount = 0;
    _cfgIP[0] = '\0'; // Clear the IP string
}

// Server constructor
TCP::TCP(uint16_t serverPort) : TCP() {
    _cfgMode = CfgMode::SERVER;
    _cfgPort = serverPort;
}

// Client constructor
TCP::TCP(const char* serverIP, uint16_t port) : TCP() {
    _cfgMode = CfgMode::CLIENT;
    if (serverIP) {
        strncpy(_cfgIP, serverIP, sizeof(_cfgIP) - 1);
        _cfgIP[sizeof(_cfgIP) - 1] = '\0';
    } else {
        _cfgIP[0] = '\0';
    }
    _cfgPort = port;
}

TCP::~TCP() {
    stop();
}

bool TCP::begin() {
    switch (_cfgMode) {
        case CfgMode::SERVER:
            return beginServer(_cfgPort);
        case CfgMode::CLIENT:
            return beginClient(_cfgIP, _cfgPort);
        default:
            return false; // No config
    }
}

void TCP::stop() {
    Modbus::Debug::LOG_MSG("Stopping TCP HAL...");
    _isRunning = false;

    if (_tcpTaskHandle) {
        Modbus::Debug::LOG_MSG("Waiting for TCP task to terminate...");
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait a bit for the task to self-terminate
        _tcpTaskHandle = nullptr;
    }

    {
        Lock guard(_socketsMutex);
        if (_listenSocket != -1) {
            Modbus::Debug::LOG_MSGF("Closing listen socket %d", _listenSocket);
            ::close(_listenSocket);
            _listenSocket = -1;
        }
        if (_clientSocket != -1) {
            Modbus::Debug::LOG_MSGF("Closing client socket %d", _clientSocket);
            ::close(_clientSocket);
            _clientSocket = -1;
        }
        for (size_t i = 0; i < _activeSocketCount; ++i) {
            int sock = _activeSockets[i];
            Modbus::Debug::LOG_MSGF("Closing active client socket %d", sock);
            ::close(sock);
        }
        _activeSocketCount = 0;
    }

    if (_rxQueue) {
        Modbus::Debug::LOG_MSG("Deleting RX queue.");
        vQueueDelete(_rxQueue);
        _rxQueue = nullptr;
    }
    Modbus::Debug::LOG_MSG("TCP HAL stopped.");
}

bool TCP::setupServerSocket(uint16_t port, uint32_t ip) {
    _listenSocket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (_listenSocket < 0) {
        Modbus::Debug::LOG_MSGF("Failed to create listen socket, errno: %d", errno);
        return false;
    }
    Modbus::Debug::LOG_MSGF("Listen socket created: %d", _listenSocket);

    int opt = 1;
    if (setsockopt(_listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        Modbus::Debug::LOG_MSGF("setsockopt(SO_REUSEADDR) failed, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }

    // Set to non-blocking
    int flags = fcntl(_listenSocket, F_GETFL, 0);
    if (flags == -1) {
        Modbus::Debug::LOG_MSGF("fcntl(F_GETFL) failed for listen socket, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }
    if (fcntl(_listenSocket, F_SETFL, flags | O_NONBLOCK) == -1) {
        Modbus::Debug::LOG_MSGF("fcntl(F_SETFL, O_NONBLOCK) failed for listen socket, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }

    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = ip; // INADDR_ANY or specific IP
    server_addr.sin_port = htons(port);

    if (::bind(_listenSocket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        Modbus::Debug::LOG_MSGF("Bind failed for port %u, errno: %d", port, errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }
    Modbus::Debug::LOG_MSGF("Socket bound to port %u", port);

    if (::listen(_listenSocket, MAX_ACTIVE_SOCKETS) < 0) {
        Modbus::Debug::LOG_MSGF("Listen failed, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }
    Modbus::Debug::LOG_MSGF("Server listening on port %u, socket %d", port, _listenSocket);
    return true;
}

static uint32_t stringToIP(const char* ip_str) {
    struct in_addr addr;
    if (inet_pton(AF_INET, ip_str, &addr) == 1) {
        return addr.s_addr;
    }
    Modbus::Debug::LOG_MSGF("Invalid IP address format: %s", ip_str);
    return 0;
}

bool TCP::setupClientSocket(const char* serverIP, uint16_t port) {
    
    uint32_t ip_addr = stringToIP(serverIP);
    if (ip_addr == 0) {
        return false; // Error logged in stringToIP
    }

    // Create local socket (don't touch _clientSocket yet)
    int newSocket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (newSocket < 0) {
        Modbus::Debug::LOG_MSGF("Failed to create client socket, errno: %d", errno);
        return false;
    }
    Modbus::Debug::LOG_MSGF("Client socket created: %d", newSocket);
    
    // Configure non-blocking
    int flags = fcntl(newSocket, F_GETFL, 0);
    if (flags == -1) {
        Modbus::Debug::LOG_MSGF("fcntl(F_GETFL) failed for client socket, errno: %d", errno);
        ::close(newSocket);
        return false;
    }
    if (fcntl(newSocket, F_SETFL, flags | O_NONBLOCK) == -1) {
        Modbus::Debug::LOG_MSGF("fcntl(F_SETFL, O_NONBLOCK) failed, errno: %d", errno);
        ::close(newSocket);
        return false;
    }

    // Prepare address
    sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(port);
    remote_addr.sin_addr.s_addr = ip_addr;

    // Attempt connection
    Modbus::Debug::LOG_MSGF("Attempting to connect to %s:%u (socket %d)...", serverIP, port, newSocket);
    int ret = ::connect(newSocket, (struct sockaddr*)&remote_addr, sizeof(remote_addr));

    if (ret < 0) {
        if (errno == EINPROGRESS) {
            Modbus::Debug::LOG_MSGF("Connection in progress...");
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(newSocket, &wfds);

            struct timeval tv;
            tv.tv_sec = CONNECT_TIMEOUT_SEC; // 5 second timeout
            tv.tv_usec = 0;

            ret = ::select(newSocket + 1, nullptr, &wfds, nullptr, &tv);
            if (ret < 0) {
                Modbus::Debug::LOG_MSGF("select() for connect failed, errno: %d", errno);
                ::close(newSocket);
                return false;
            } else if (ret == 0) {
                Modbus::Debug::LOG_MSGF("Connect timeout to %s:%u", serverIP, port);
                ::close(newSocket);
                return false;
            } else {
                // Check SO_ERROR
                int so_error;
                socklen_t len = sizeof(so_error);
                if (getsockopt(newSocket, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0) {
                    Modbus::Debug::LOG_MSGF("getsockopt(SO_ERROR) failed, errno: %d", errno);
                    ::close(newSocket);
                    return false;
                }
                if (so_error != 0) {
                    Modbus::Debug::LOG_MSGF("Connect failed with SO_ERROR: %d", so_error);
                    ::close(newSocket);
                    return false;
                }
                Modbus::Debug::LOG_MSGF("Connected to %s:%u successfully!", serverIP, port);
            }
        } else {
            Modbus::Debug::LOG_MSGF("Connect failed immediately, errno: %d", errno);
            ::close(newSocket);
            return false;
        }
    } else {
        Modbus::Debug::LOG_MSGF("Connected to %s:%u immediately!", serverIP, port);
    }
    
    // Atomic assignment at the end (under Mutex)
    {
        Lock guard(_socketsMutex);
        if (_clientSocket != -1) {
            Modbus::Debug::LOG_MSGF("Closing previous client socket %d", _clientSocket);
            ::close(_clientSocket);
        }
        _clientSocket = newSocket;
    }
    
    return true;
}

bool TCP::beginServer(uint16_t port, uint32_t ip) {
    if (_isRunning) {
        Modbus::Debug::LOG_MSG("Server already running.");
        return false;
    }
    Modbus::Debug::LOG_MSGF("Beginning server on port %u", port);
    _isServer = true;

    if (!setupServerSocket(port, ip)) {
        return false; // Error logged in setupServerSocket
    }

    _rxQueue = xQueueCreateStatic(RX_QUEUE_SIZE, sizeof(int), _rxQueueStorage, &_rxQueueBuf);
    if (!_rxQueue) {
        Modbus::Debug::LOG_MSG("Failed to create RX queue.");
        stop(); // Cleanup listen socket
        return false;
    }
    Modbus::Debug::LOG_MSG("RX queue created.");

    _isRunning = true;
    _tcpTaskHandle = xTaskCreateStatic(
        tcpTask,
        "ModbusHALtcpSrv",
        TCP_TASK_STACK_SIZE, // Stack size
        this, // Task parameter
        tskIDLE_PRIORITY + 1,    // Priority
        _tcpTaskStack,
        &_tcpTaskBuf
    );

    if (_tcpTaskHandle == nullptr) {
        Modbus::Debug::LOG_MSG("Failed to create server TCP task.");
        stop(); // Cleanup queue and socket
        return false;
    }
    Modbus::Debug::LOG_MSG("Server TCP task created and started.");
    return true;
}

bool TCP::beginClient(const char* serverIP, uint16_t port) {
    if (_isRunning) {
        Modbus::Debug::LOG_MSG("Client already running or server mode active.");
        return false;
    }
    Modbus::Debug::LOG_MSGF("Beginning client to %s:%u", serverIP, port);
    _isServer = false;

    if (!setupClientSocket(serverIP, port)) {
        Modbus::Debug::LOG_MSGF("WARNING: Initial connection to %s:%u failed, will retry on first sendMsg", serverIP, port);
        // Continue anyway, auto-reconnect will handle it
    }

    _rxQueue = xQueueCreateStatic(RX_QUEUE_SIZE, sizeof(int), _rxQueueStorage, &_rxQueueBuf);
    if (!_rxQueue) {
        Modbus::Debug::LOG_MSG("Failed to create RX queue for client.");
        stop(); // Cleanup client socket
        return false;
    }
    Modbus::Debug::LOG_MSG("RX queue created for client.");
    
    _isRunning = true;
    TaskHandle_t taskCreated = xTaskCreateStatic(
        tcpTask,
        "ModbusHALtcpCli",
        TCP_TASK_STACK_SIZE, // Stack size
        this, // Task parameter
        tskIDLE_PRIORITY + 1,    // Priority
        _tcpTaskStack,
        &_tcpTaskBuf
    );

    if (taskCreated == nullptr) {
        Modbus::Debug::LOG_MSG("Failed to create client TCP task.");
        _tcpTaskHandle = nullptr; // Ensure it's null
        stop(); // Cleanup queue and socket
        return false;
    }
    Modbus::Debug::LOG_MSG("Client TCP task created and started.");
    return true;
}

void TCP::tcpTask(void* param) {
    TCP* self = static_cast<TCP*>(param);
    Modbus::Debug::LOG_MSGF("tcpTask started on core %d", xPortGetCoreID());
    if (self) {
        self->runTcpTask();
    }
    Modbus::Debug::LOG_MSGF("tcpTask exiting for %s.", self->_isServer ? "server" : "client");
    vTaskDelete(nullptr);
}

void TCP::runTcpTask() {
    struct timeval tv; // For select timeout
    int error_count = 0;  // Consecutive error counter for recovery
    int empty_hits = 0;   // Empty rounds counter (anti-spin EAGAIN)

    while (_isRunning) {
        tv.tv_sec = SELECT_TIMEOUT_MS / 1000;        // Seconds part
        tv.tv_usec = (SELECT_TIMEOUT_MS % 1000) * 1000; // Microseconds part

        fd_set readfds;
        FD_ZERO(&readfds);
        int max_fd = -1;

        { // Scope for socketsMutex lock
            Lock guard(_socketsMutex);
            if (_isServer && _listenSocket != -1) {
                FD_SET(_listenSocket, &readfds);
                max_fd = std::max(max_fd, _listenSocket);
                // Modbus::Debug::LOG_MSGF("Server: Added listen_socket %d to fd_set", _listenSocket);
            } else if (!_isServer && _clientSocket != -1) {
                FD_SET(_clientSocket, &readfds);
                max_fd = std::max(max_fd, _clientSocket);
                // Modbus::Debug::LOG_MSGF("Client: Added client_socket %d to fd_set", _clientSocket);
            }

            for (size_t i = 0; i < _activeSocketCount; ++i) {
                int sock = _activeSockets[i]; // Primarily for server mode client connections
                FD_SET(sock, &readfds);
                max_fd = std::max(max_fd, sock);
                // Modbus::Debug::LOG_MSGF("Server: Added active_socket %d to fd_set", sock);
            }
        } // Unlock socketsMutex

        if (max_fd == -1) { // No sockets to monitor (e.g., client not connected yet, or server not started)
            vTaskDelay(pdMS_TO_TICKS(20)); // Sleep a bit
            continue;
        }
        
        // Modbus::Debug::LOG_MSGF("Calling select() with max_fd = %d", max_fd);
        int activity = ::select(max_fd + 1, &readfds, nullptr, nullptr, &tv);

        if (!_isRunning) break; // Check flag again after select

        if (activity < 0) {
            if (errno == EINTR) {
                error_count = 0;  // Normal signal, reset counter
                continue;
            }
            
            if (errno == EBADF) {
                Modbus::Debug::LOG_MSGF("EBADF detected, cleaning dead sockets");
                cleanupDeadSockets();
                error_count = 0;  // Reset after cleanup
                continue;
            }
            
            error_count++;
            Modbus::Debug::LOG_MSGF("select() error #%d, errno: %d (%s)", error_count, errno, strerror(errno));
            
            if (error_count >= MAX_SELECT_ERRORS) {
                Modbus::Debug::LOG_MSGF("Too many select errors, sleeping %lums then retrying...", SELECT_RECOVERY_SLEEP_MS);
                
                // Long sleep then reset and retry
                vTaskDelay(pdMS_TO_TICKS(SELECT_RECOVERY_SLEEP_MS));
                error_count = 0;
                continue;
            }
            
            // Progressive backoff and retry (prevent overflow)
            uint32_t backoff_ms = std::min(SELECT_BACKOFF_BASE_MS * error_count, SELECT_RECOVERY_SLEEP_MS);
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            continue;
        }
 
        // Success! Reset error counter
        error_count = 0;

        if (activity == 0) {
            // Timeout, just loop again
            // Modbus::Debug::LOG_MSGF("select() timeout");
            continue;
        }

        // Activity detected, process sockets
        { // Scope for socketsMutex lock
            Lock guard(_socketsMutex);
            if (!_isRunning) break; // Check flag again after acquiring lock

            // 1. Handle new connections for server
            if (_isServer && _listenSocket != -1 && FD_ISSET(_listenSocket, &readfds)) {
                Modbus::Debug::LOG_MSGF("Activity on listen socket %d", _listenSocket);
                sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                int new_socket = ::accept(_listenSocket, (struct sockaddr*)&client_addr, &client_len);

                if (new_socket >= 0) {
                    if (_activeSocketCount < MAX_ACTIVE_SOCKETS) {
                        // Set new socket to non-blocking
                        int flags = fcntl(new_socket, F_GETFL, 0);
                        if (flags != -1 && fcntl(new_socket, F_SETFL, flags | O_NONBLOCK) != -1) {
                            _activeSockets[_activeSocketCount++] = new_socket;
                            char client_ip[INET_ADDRSTRLEN];
                            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
                            Modbus::Debug::LOG_MSGF("New connection from %s on socket %d. Active clients: %d", client_ip, new_socket, static_cast<int>(_activeSocketCount));
                        } else {
                            Modbus::Debug::LOG_MSGF("Failed to set new client socket %d to non-blocking, errno: %d. Closing.", new_socket, errno);
                            ::close(new_socket);
                        }
                    } else {
                        Modbus::Debug::LOG_MSGF("Max clients (%d) reached. Rejecting new connection from socket %d.", MAX_ACTIVE_SOCKETS, new_socket);
                        ::close(new_socket);
                    }
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                         Modbus::Debug::LOG_MSGF("accept() failed on listen socket %d, errno: %d", _listenSocket, errno);
                         // Potentially a serious issue with the listen socket, consider re-init or stopping.
                    }
                }
            }

            // 2. Simply check which sockets have data to read
            // Build the list of sockets to check for data
            int sockets_to_check[MAX_ACTIVE_SOCKETS + 1];
            size_t check_count = 0;
            if (_isServer) {
                for (size_t i = 0; i < _activeSocketCount; ++i) {
                    sockets_to_check[check_count++] = _activeSockets[i];
                }
            } else if (_clientSocket != -1) {
                sockets_to_check[check_count++] = _clientSocket;
            }

            int sockets_to_remove[MAX_ACTIVE_SOCKETS];
            size_t remove_count = 0;
            bool no_data_read = true;  // Tracker for anti-spin detection
 
            uint8_t dummy;
            for (size_t idx = 0; idx < check_count; ++idx) {
                int sock = sockets_to_check[idx];
                if (!FD_ISSET(sock, &readfds)) continue;
 
                ssize_t peek = ::recv(sock, &dummy, 1, MSG_PEEK);
 
                if (peek > 0) {
                    no_data_read = false;  // We have data!
                    if (xQueueSend(_rxQueue, &sock, portMAX_DELAY) != pdTRUE) {
                        Modbus::Debug::LOG_MSGF("RX queue full. Dropping event for socket %d.", sock);
                    }
                } else if (peek == 0) {
                    no_data_read = false;  // Connection closed = activity
                    Modbus::Debug::LOG_MSGF("Socket %d closed by peer.", sock);
                    sockets_to_remove[remove_count++] = sock;
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        no_data_read = false;  // Error = activity
                        Modbus::Debug::LOG_MSGF("recv(MSG_PEEK) error on socket %d, errno: %d. Closing.", sock, errno);
                        sockets_to_remove[remove_count++] = sock;
                    }
                }
            }
 
            for (size_t i = 0; i < remove_count; ++i) {
                closeSocket(sockets_to_remove[i]);
            }
 
            // Anti-spin: if select() says OK but no socket has data
            if (activity > 0 && no_data_read) {
                empty_hits++;
                if (empty_hits > MAX_EMPTY_HITS) {
                    vTaskDelay(pdMS_TO_TICKS(ANTI_SPIN_DELAY_MS));  // Short anti-spin pause
                    empty_hits = 0;
                }
            } else {
                empty_hits = 0;
            }
 
        } // Unlock socketsMutex
    } // while (_isRunning)
}

void TCP::closeSocket(int sock) {
    // Assumes _socketsMutex is held by caller if necessary, or called during stop()
    // For runTcpTask, it should be called within the _socketsMutex lock.
    Modbus::Debug::LOG_MSGF("Closing socket %d.", sock);
    ::close(sock);
    if (_isServer) {
        for (size_t i = 0; i < _activeSocketCount; ++i) {
            if (_activeSockets[i] == sock) {
                // Shift remaining sockets left
                for (size_t j = i; j + 1 < _activeSocketCount; ++j) {
                    _activeSockets[j] = _activeSockets[j + 1];
                }
                --_activeSocketCount;
                Modbus::Debug::LOG_MSGF("Removed socket %d from active server clients. Count: %d", sock, static_cast<int>(_activeSocketCount));
                break;
            }
        }
    } else { // Client mode
        if (sock == _clientSocket) {
            _clientSocket = -1;
            // Optionally, attempt reconnect or notify higher layer
            Modbus::Debug::LOG_MSGF("Client socket %d disconnected.", sock);
        }
    }
}

bool TCP::sendMsg(const uint8_t* payload, const size_t len, const int destSocket, int* actualSocket) {
    if (!_isRunning) return false;

    /* 1. Check reconnection needed */
    bool needReconnect = false;
    {
        Lock guard(_socketsMutex);
        needReconnect = (!_isServer && destSocket == -1 && _clientSocket == -1);
    }

    /* 2. Reconnect OUTSIDE mutex if needed */
    if (needReconnect) {
        Modbus::Debug::LOG_MSGF("Client disconnected, attempting reconnect to %s:%u", _cfgIP, _cfgPort);
        if (!setupClientSocket(_cfgIP, _cfgPort)) {
            Modbus::Debug::LOG_MSGF("Auto-reconnection failed");
            return false;
        }
        Modbus::Debug::LOG_MSGF("Auto-reconnection successful");
        
        if (!_isRunning) return false;
    }

    /* 3. Send phase - re-lock and re-read socket */
    Lock guard(_socketsMutex);
    
    int targetSocket = -1;
    if (destSocket != -1) {
        targetSocket = destSocket;
    } else if (!_isServer) {
        targetSocket = _clientSocket; // Re-read after potential reconnection
    } else {
        Modbus::Debug::LOG_MSGF("sendMsg: Server mode with default destSocket (-1) not supported");
        return false;
    }

    if (actualSocket) {
        *actualSocket = targetSocket;
    }

    if (targetSocket == -1) {
        Modbus::Debug::LOG_MSGF("sendMsg: No valid target socket");
        return false;
    }

    if (len == 0 || len > MAX_MODBUS_FRAME_SIZE) {
        Modbus::Debug::LOG_MSGF("sendMsg: Invalid message length %d.", len);
        return false;
    }

    /* 4. Protected send */
    Modbus::Debug::LOG_MSGF("Sending %zu bytes to socket %d", len, targetSocket);
    ssize_t sent_bytes = ::send(targetSocket, payload, len, 0);

    if (sent_bytes < 0) {
        Modbus::Debug::LOG_MSGF("send() failed, errno: %d", errno);
        if (errno == EPIPE || errno == ECONNRESET || errno == ENOTCONN || errno == EBADF) {
            closeSocket(targetSocket);
        }
        return false;
    }

    if ((size_t)sent_bytes != len) {
        Modbus::Debug::LOG_MSGF("send() incomplete: sent %zd of %zu bytes", sent_bytes, len);
        return false;
    }

    Modbus::Debug::LOG_MSGF("Successfully sent %d bytes to socket %d", sent_bytes, targetSocket);
    return true;
}

size_t TCP::getActiveSocketCount() {
    Lock guard(_socketsMutex);
    if (_isServer) {
        return _activeSocketCount;
    } else {
        return (_clientSocket != -1) ? 1 : 0;
    }
}

bool TCP::isServerRunning() const {
    return _isRunning && _isServer && _listenSocket != -1;
}

bool TCP::isClientConnected() {
    Lock guard(_socketsMutex);
    return _isRunning && !_isServer && _clientSocket != -1;
}

bool TCP::isReady() {
    if (!_isRunning) return false;
    if (_isServer) {
        return _listenSocket != -1;  // Server ready if listening
    } else {
        return _cfgMode == CfgMode::CLIENT;  // Client ready if configured
    }
}

size_t TCP::readSocketData(int socketNum, uint8_t* dst, size_t maxLen) {
    if (!dst || maxLen == 0) return SIZE_MAX;

    ssize_t r = ::recv(socketNum, dst, maxLen, 0); // Non-bloquant (socket configuré O_NONBLOCK)

    if (r > 0) {
        return static_cast<size_t>(r);
    }

    if (r == 0) {
        Modbus::Debug::LOG_MSGF("readSocketData: socket %d closed by peer", socketNum);
        return SIZE_MAX;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0; // Pas de data prête.
    }

    Modbus::Debug::LOG_MSGF("readSocketData: recv error on socket %d, errno %d", socketNum, errno);
    return SIZE_MAX;
}

void TCP::cleanupDeadSockets() {
    Lock guard(_socketsMutex);
    
    // Test listen socket (server)
    if (_isServer && _listenSocket != -1) {
        int error = 0;
        socklen_t len = sizeof(error);
        if (getsockopt(_listenSocket, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
            Modbus::Debug::LOG_MSGF("Listen socket %d is dead, closing", _listenSocket);
            ::close(_listenSocket);
            _listenSocket = -1;
        }
    }
    
    // Test client socket
    if (!_isServer && _clientSocket != -1) {
        int error = 0;
        socklen_t len = sizeof(error);
        if (getsockopt(_clientSocket, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
            Modbus::Debug::LOG_MSGF("Client socket %d is dead, closing", _clientSocket);
            ::close(_clientSocket);
            _clientSocket = -1;
        }
    }
    
    // Test active sockets (server clients)
    for (size_t i = 0; i < _activeSocketCount; ) {
        int sock = _activeSockets[i];
        int error = 0;
        socklen_t len = sizeof(error);
        
        if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
            Modbus::Debug::LOG_MSGF("Active socket %d is dead, removing", sock);
            closeSocket(sock); // closeSocket handles array shifting
        } else {
            i++;
        }
    }
}

} // namespace ModbusHAL