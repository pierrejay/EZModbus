/**
 * @file ModbusTCP.h
 * @brief Modbus TCP class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusCodec.hpp"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"
#include "drivers/ModbusHAL_TCP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifndef EZMODBUS_TCP_TXN_SAFETY_TIMEOUT // TCP transaction safety timeout (ms)
    #define EZMODBUS_TCP_TXN_SAFETY_TIMEOUT 5000
#endif

#ifndef EZMODBUS_TCP_RXTX_TASK_STACK_SIZE // TCP RX/TX task stack size (bytes)
    #ifdef EZMODBUS_DEBUG
        #define EZMODBUS_TCP_RXTX_TASK_STACK_SIZE 6144
    #else
        #define EZMODBUS_TCP_RXTX_TASK_STACK_SIZE 4096
    #endif
#endif

namespace ModbusInterface {

class TCP : public IInterface {
public:

    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t RX_ASSEMBLY_TIMEOUT_MS = 50;
    static constexpr uint32_t RXTX_QUEUE_CHECK_TIMEOUT_MS = 100; // Added for task management
    // Safety timeout - much longer than client timeout for emergency cleanup
    static constexpr uint32_t TCP_TRANSACTION_SAFETY_TIMEOUT_MS = (uint32_t)EZMODBUS_TCP_TXN_SAFETY_TIMEOUT;

    // Tasks stack sizes (higher for debug to let room for the printf/hexdump buffers)
    static constexpr uint32_t RXTX_TASK_STACK_SIZE = (uint32_t)EZMODBUS_TCP_RXTX_TASK_STACK_SIZE;

    // ===================================================================================
    // DATA STRUCTURES
    // ===================================================================================

    /* @brief Stores the current transaction context */
    struct TransactionCtx {
        volatile bool active = false;
        int startMs = 0;
        int socketNum = -1;
        uint16_t tid = 0;

        void set(int socketNum, uint16_t tid) { active=true; startMs=TIME_MS(); this->socketNum=socketNum; this->tid=tid; }
        void clear() { socketNum=-1; startMs=0; tid=0; active=false; }
    };

    /* @brief Stores the current transmission data context
    * @note This structs only holds the metadata of the current transmission, not the data itself
    *       which is stored in _txBuffer
    */
    struct TxCtx {
        uint16_t tid = 0;
        int destSock = -1;
        bool isBroadcast = false;
        TxResultCallback txCallback = nullptr;
        void* ctx = nullptr;

        void set(uint16_t tid, int socketNum, bool isBroadcast = false, TxResultCallback txCallback = nullptr, void* ctx = nullptr) { 
            this->tid = tid; 
            this->destSock = socketNum; 
            this->isBroadcast = isBroadcast; 
            this->txCallback = txCallback;
            this->ctx = ctx;
        }
        void clear() { tid=0; destSock=-1; isBroadcast=false; txCallback=nullptr; ctx=nullptr; }
    };

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    explicit TCP(ModbusHAL::TCP& hal, Modbus::Role role);
    virtual ~TCP();

    Result begin() override;
    Result sendFrame(const Modbus::Frame &frame, TxResultCallback txCallback, void* ctx) override;
    bool isReady() override;
    void abortCurrentTransaction() override;
    TaskHandle_t getRxTxTaskHandle();

private:
    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // TCP communication
    ModbusHAL::TCP& _tcpHAL;
    bool _isInitialized = false; // Added initialization flag

    // Data buffers & protection
    std::array<uint8_t, ModbusCodec::TCP::MAX_FRAME_SIZE> _rxBuf;
    ByteBuffer _rxBuffer; // Linked to _rxBuf
    std::array<uint8_t, ModbusCodec::TCP::MAX_FRAME_SIZE> _txBuf;
    ByteBuffer _txBuffer; // Linked to _txBuf
    TxCtx _txCtx; // Stores the current TX data context
    Mutex _txMutex; // Protects access to _txBuffer & _txCtx

    // Transaction management
    TransactionCtx _currentTransaction; // IMPORTANT: must ONLY be written from rxTxTask context!

    // Data processing
    // _rxEventQueue: receives RX events from HAL (only the handle, belongs to HAL layer)
    QueueHandle_t _rxEventQueue = nullptr;
    // _txRequestQueue: just a dummy signaling queue so that we can use xQueueSet 
    // to wait for both RX and TX without wasting CPU
    StaticQueue_t _txRequestQueueBuffer;
    alignas(4) uint8_t _txRequestQueueStorage[1 * sizeof(void*)];
    QueueHandle_t _txRequestQueue = nullptr;
    // _txnControlQueue: only rxTxTask terminates transaction through this queue
    StaticQueue_t _txnControlQueueBuffer;
    alignas(4) uint8_t _txnControlQueueStorage[1 * sizeof(void*)];
    QueueHandle_t _txnControlQueue = nullptr;
    // _eventQueueSet: Combines TX event queue + HAL event queue + Txn control queue
    QueueSetHandle_t _eventQueueSet = nullptr;

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    // Lifecycle management
    void beginCleanup();
    void killRxTxTask();
    
    // RX frame processing
    Result processReceivedFrame(const ByteBuffer& frameBytes, int socketNum);
    Result handleDecodedFrame(const Modbus::Frame& frame, int socketNum, uint16_t transactionId);
    Result fetchSocketData(int socketNum, ByteBuffer& rcvBuf, uint16_t& rcvTid);

    // TX frame processing
    Result handleTxRequest();

    // Transaction management
    uint16_t getNextOutgoingTransactionId();
    bool beginTransaction(int socketNum, uint16_t transactionId);
    void endTransaction();

    // ===================================================================================
    // TASKS
    // ===================================================================================

    static void rxTxTask(void* tcp);
    StaticTask_t _rxTxTaskBuffer;
    StackType_t _rxTxTaskStack[RXTX_TASK_STACK_SIZE];
    TaskHandle_t _rxTxTaskHandle;

};

} // namespace ModbusInterface