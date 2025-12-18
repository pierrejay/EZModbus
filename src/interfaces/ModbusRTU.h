/* @file ModbusRTU.h
 * @brief ModbusRTU class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusCodec.hpp"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"
#include "drivers/ModbusHAL_UART.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifndef EZMODBUS_RTU_TASK_STACK_SIZE // RTU RX/TX task stack size (bytes)
    #ifdef EZMODBUS_DEBUG
        #define EZMODBUS_RTU_TASK_STACK_SIZE 4096
    #else
        #define EZMODBUS_RTU_TASK_STACK_SIZE 2048
    #endif
#endif


namespace ModbusInterface {

class RTU : public IInterface {
public:
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t RXTX_QUEUE_CHECK_TIMEOUT_MS = 100;

    // Tasks stack sizes
    static constexpr uint32_t RXTX_TASK_STACK_SIZE = (uint32_t)EZMODBUS_RTU_TASK_STACK_SIZE;

    // ===================================================================================
    // DATA STRUCTURES
    // ===================================================================================

    /* @brief Dedicated struct to measure the round trip time of a transaction (inbound or outbound) */
    struct RoundTripTimer {
        uint64_t startUs = 0;
        uint64_t storeUs = 0;

    #ifdef EZMODBUS_DEBUG
        inline void store() {
            storeUs = TIME_US();
        }

        inline void start(uint64_t* storeUs = nullptr) {
            if (storeUs) {
                startUs = *storeUs;
            } else {
                startUs = TIME_US();
            }
        }

        inline void end(uint64_t* storeUs = nullptr) const {
            uint64_t endUs;
            if (storeUs) {
                endUs = *storeUs;
            } else {
                endUs = TIME_US();
            }
            uint64_t deltaUs = (endUs - startUs);
            Modbus::Debug::LOG_MSGF("RTT: %d us", (uint32_t)deltaUs);
        }
    #else // Do not use the RTT if debug is disabled
        void start(uint64_t* storeUs = nullptr) {}
        void end(uint64_t* storeUs = nullptr) const {}
        void store() {}
    #endif
    };

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    explicit RTU(ModbusHAL::UART& uart, Modbus::Role role = Modbus::MASTER);
    virtual ~RTU();

    Result begin() override;
    Result setSilenceTimeMs(uint32_t silenceTimeMs);
    Result setSilenceTimeBaud(); 
    Result sendFrame(const Modbus::Frame& frame, TxResultCallback txCallback, void* ctx) override;
    bool isReady() override;
    TaskHandle_t getRxTxTaskHandle();

private:
    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // Serial communication
    ModbusHAL::UART& _uartHAL;
    uint64_t _silenceTimeUs = 0;   // Sets the silence time to observe between consecutive frames (RX + TX)
    uint64_t _lastTxTimeUs = 0; // Last time a frame was sent (used to enforce the silence time for TX)
    bool _isInitialized = false;

    // Data buffers & protection
    std::array<uint8_t, ModbusCodec::RTU::MAX_FRAME_SIZE> _rxBuf;
    ByteBuffer _rxBuffer; // Linked to _rxBuf
    std::array<uint8_t, ModbusCodec::RTU::MAX_FRAME_SIZE> _txBuf;
    ByteBuffer _txBuffer; // Linked to _txBuf
    Mutex _txMutex; // Protects access to _txBuffer
    TxResultCallback _txResultCallback; // Stores callback to be called with TX result
    void* _txCallbackCtx; // Stores context for TX callback
    
    // Data processing
    // _rxEventQueue: receives RX events from HAL (only the handle, belongs to HAL layer)
    QueueHandle_t _rxEventQueue = nullptr;
    // _txRequestQueue: just a dummy signaling queue so that we can use xQueueSet 
    // to wait for both RX and TX without wasting CPU
    StaticQueue_t _txRequestQueueBuffer;
    alignas(4) uint8_t _txRequestQueueStorage[1 * sizeof(void*)];
    QueueHandle_t _txRequestQueue = nullptr; 
    // _eventQueueSet: Combines RX event queue + UART event queue
    QueueSetHandle_t _eventQueueSet = nullptr;

    // Miscellaneous
    RoundTripTimer _rtt{};

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    void beginCleanup();
    void killRxTxTask();
    Result processReceivedFrame(const ByteBuffer& frameBytes);
    Result updateUartIdleDetection();
    inline void notifyTxResult(ModbusInterface::IInterface::Result res);

    // Utility methods for rxTxTask
    Result handleUartEvent(const uart_event_t& event);
    Result handleTxRequest();

    // ===================================================================================
    // TASKS
    // ===================================================================================

    static void rxTxTask(void* rtu);
    StaticTask_t _rxTxTaskBuffer;
    StackType_t _rxTxTaskStack[RXTX_TASK_STACK_SIZE];
    TaskHandle_t _rxTxTaskHandle = nullptr;

    // ===================================================================================
    // HELPER METHODS
    // ===================================================================================

    // Conversion of UART event types to strings
    static constexpr const char* toString(uart_event_type_t event_type) {
        switch (event_type) {
            case UART_DATA: return "data received";
            case UART_BREAK: return "break detected";
            case UART_BUFFER_FULL: return "buffer full";
            case UART_FIFO_OVF: return "FIFO overflow";
            case UART_FRAME_ERR: return "frame error";
            case UART_PARITY_ERR: return "parity error";
            case UART_DATA_BREAK: return "data and break";
            case UART_PATTERN_DET: return "pattern detected";
            default: return "unknown event type";
        }
    }
};

} // namespace ModbusInterface