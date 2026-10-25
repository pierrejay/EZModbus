/**
 * @file ModbusBridge.hpp
 * @brief Modbus bridge implementation
 */

#pragma once

#include "core/ModbusCore.h"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"
#include "freertos/timers.h"

#ifndef EZMODBUS_BRIDGE_TIMEOUT_MS // Bridge response timeout (ms)
    #define EZMODBUS_BRIDGE_TIMEOUT_MS 1000
#endif

namespace Modbus {

class Bridge {
public:
// ===================================================================================
// CONSTANTS
// ===================================================================================

    // Max time for the device behind the bridge to answer, before replying a
    // GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND exception to the requester
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = (uint32_t)EZMODBUS_BRIDGE_TIMEOUT_MS;
    static constexpr TickType_t TIMER_CMD_TOUT_TICKS = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1; // Bounded wait for FreeRTOS timer commands

// ===================================================================================
// RESULT TYPES
// ===================================================================================
   
    enum Result {
        SUCCESS,
        ERR_INIT_FAILED
    };
    static constexpr const char* toString(Result result) {
        switch (result) {
            case SUCCESS: return "success";
            case ERR_INIT_FAILED: return "init failed";
            default: return "unknown";
        }
    }

    // Helper to cast an error
    // - Returns a Result
    // - Captures point of call context & prints a log message when debug 
    // is enabled. No overhead when debug is disabled (except for
    // the desc string, if any)
    static inline Result Error(Result res, const char* desc = nullptr
                        #ifdef EZMODBUS_DEBUG
                        , Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()
                        #endif
                        ) {
        #ifdef EZMODBUS_DEBUG
            if (desc && *desc != '\0') {
                Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s (%s)", toString(res), desc);
            } else {
                Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s", toString(res));
            }
        #endif
        return res;
    }

    // Helper to cast a success
    // - Returns Result::SUCCESS
    // - Captures point of call context & prints a log message when debug 
    // is enabled. No overhead when debug is disabled (except for
    // the desc string, if any)
    static inline Result Success(const char* desc = nullptr
                          #ifdef EZMODBUS_DEBUG
                          , Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()
                          #endif
                          ) {
        #ifdef EZMODBUS_DEBUG
            if (desc && *desc != '\0') {
                Modbus::Debug::LOG_MSGF_CTX(ctx, "Success: %s", desc);
            }
        #endif
        return SUCCESS;
    }

// ===================================================================================
// CONSTRUCTOR & PUBLIC METHODS
// ===================================================================================
    
    /* @brief Constructor taking the two interfaces to link
     * @note The bridge recognizes the role of each interface
     *       (they must be different otherwise init will fail)
     * @param Interface1 The first interface
     * @param Interface2 The second interface
     * @param timeoutMs Max time for the device behind the bridge to answer a request
     */
    Bridge(ModbusInterface::IInterface& Interface1, 
           ModbusInterface::IInterface& Interface2,
           uint32_t timeoutMs = DEFAULT_TIMEOUT_MS)
        : interface1(Interface1)
        , interface2(Interface2)
        , _timeoutMs(timeoutMs) {}

    /* @brief Destructor
     * @note Waits until the timeout timer is deleted and its callback can't run anymore
     * @note Detaches from the interfaces, which must outlive the bridge
     */
    ~Bridge() {
        // The interfaces must not call this object anymore: a request still held by the master
        // interface is aborted (best effort), and its TX result waited for
        interface1.removeRcvCallbacks(this);
        interface2.removeRcvCallbacks(this);
        if (masterInterface) masterInterface->releaseTransaction(this, _pendingActive);
        interface1.waitTxCallbackDone(this);
        interface2.waitTxCallbackDone(this);
        if (!_timer) return;
        xTimerDelete(_timer, portMAX_DELAY);
        // Fence: runs in the timer daemon after the delete command, once no callback can run
        if (xTaskGetCurrentTaskHandle() != xTimerGetTimerDaemonTaskHandle()) {
            while (_timerFence.tryTake()) {} // Purge the semaphore
            xTimerPendFunctionCall([](void* ctx, uint32_t) {
                static_cast<Bridge*>(ctx)->_timerFence.giveForce();
            }, this, 0, portMAX_DELAY);
            _timerFence.take();
        }
        _timer = nullptr;
    }

    Bridge(const Bridge&) = delete;
    Bridge& operator=(const Bridge&) = delete;

    /* @brief Initialize the bridge
     * @return The result of the initialization
     */
    Result begin() {
        // Both interfaces must have a different role
        if (interface1.getRole() == interface2.getRole()) {
            return Error(ERR_INIT_FAILED, "both interfaces must not have the same role");
        }

        // Determine which interface is the master and which is the slave
        if (interface1.getRole() == Modbus::MASTER) {
            masterInterface = &interface1;
            slaveInterface = &interface2;
        } else {
            masterInterface = &interface2;
            slaveInterface = &interface1;
        }

        // Response timeout timer (period rounded up: it must not fire before the timeout)
        if (!_timer) {
            TickType_t periodTicks = (_timeoutMs + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
            if (periodTicks == 0) periodTicks = 1;
            _timer = xTimerCreateStatic("ModbusBridge", periodTicks, pdFALSE, this, timeoutCallback, &_timerBuf);
            if (!_timer) {
                return Error(ERR_INIT_FAILED, "failed to create timeout timer");
            }
        }

        // Both interfaces must be initialized
        auto resInitMaster = masterInterface->begin();
        if (resInitMaster != ModbusInterface::IInterface::SUCCESS) {
            return Error(ERR_INIT_FAILED, "failed to initialize master interface");
        }   
        auto resInitSlave = slaveInterface->begin();
        if (resInitSlave != ModbusInterface::IInterface::SUCCESS) {
            return Error(ERR_INIT_FAILED, "failed to initialize slave interface");
        }
        
        // Registered once whatever the number of calls, and none left if begin() fails
        interface1.removeRcvCallbacks(this);
        interface2.removeRcvCallbacks(this);

        // Configure the master interface callback to redirect to the slave
        auto resSetCbMaster = masterInterface->setRcvCallback([](const Modbus::Frame& frame, void* ctx) {
            auto* self = static_cast<Bridge*>(ctx);
            if (frame.type == Modbus::RESPONSE) { // The master interface only sends responses back to the slave
                self->forwardResponse(frame);
            }
        }, this);
        if (resSetCbMaster != ModbusInterface::IInterface::SUCCESS) {
            return Error(ERR_INIT_FAILED, "failed to set master interface callback");
        }

        // Configure the slave interface callback to redirect to the master
        auto resSetCbSlave = slaveInterface->setRcvCallback([](const Modbus::Frame& frame, void* ctx) {
            auto* self = static_cast<Bridge*>(ctx);
            if (frame.type == Modbus::REQUEST) { // The slave interface only forwards requests to the master
                self->forwardRequest(frame);
            }
        }, this);
        if (resSetCbSlave != ModbusInterface::IInterface::SUCCESS) {
            masterInterface->removeRcvCallbacks(this);
            return Error(ERR_INIT_FAILED, "failed to set slave interface callback");
        }

        return Success();
    }

private:
// ===================================================================================
// PRIVATE MEMBERS
// ===================================================================================
    
    // References to the two "real" interfaces
    ModbusInterface::IInterface& interface1;
    ModbusInterface::IInterface& interface2;
    // Aliases depending on their respective roles
    ModbusInterface::IInterface* masterInterface = nullptr;
    ModbusInterface::IInterface* slaveInterface = nullptr;

    // Request forwarded to the master side, waiting for its response
    uint32_t _timeoutMs;
    Mutex _pendingMutex;           // Protects the pending request (accessed by both interfaces' tasks & the timer)
    bool _pendingActive = false;
    bool _txPending = false;       // TX result of the last forwarded request not back yet
    Modbus::FrameMeta _pending;
    uint32_t _pendingStartMs = 0;
    TimerHandle_t _timer = nullptr;
    StaticTimer_t _timerBuf;
    BinarySemaphore _timerFence;   // Signals that the timer daemon processed the timer deletion

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

    /* @brief Forward a request from the slave side to the master side
     * @note Replies GATEWAY_PATH_UNAVAILABLE if the request can't be forwarded
     */
    void forwardRequest(const Modbus::Frame& request) {
        // The slave interface couldn't decode the request: answer its exception directly
        if (request.exceptionCode != Modbus::NULL_EXCEPTION) {
            if (!Modbus::isBroadcastId(request.slaveId)) {
                Modbus::Frame exception;
                Modbus::makeException(request, exception, request.exceptionCode);
                slaveInterface->sendFrame(exception, nullptr, nullptr);
            }
            return;
        }

        // No response expected for broadcasts: nothing to track
        if (Modbus::isBroadcastId(request.slaveId)) {
            masterInterface->sendFrame(request, nullptr, nullptr);
            return;
        }

        // One request at a time: none while a request waits for its response, nor until the TX
        // result of the previous one came back (a late TX failure would close the newer request)
        bool admitted = false;
        {
            Lock guard(_pendingMutex);
            if (!_pendingActive && !_txPending) {
                _pending = Modbus::FrameMeta(request);
                _pendingStartMs = TIME_MS();
                _pendingActive = true;
                _txPending = true;
                admitted = true;
            }
        }
        if (!admitted) {
            Modbus::Frame exception;
            Modbus::makeException(request, exception, Modbus::SLAVE_DEVICE_BUSY);
            slaveInterface->sendFrame(exception, nullptr, nullptr);
            return;
        }

        if (xTimerReset(_timer, TIMER_CMD_TOUT_TICKS) != pdPASS) {
            // No timeout guarantee: don't forward, the requester can retry
            onTxResult(ModbusInterface::IInterface::ERR_BUSY, Modbus::SLAVE_DEVICE_BUSY);
            return;
        }

        // The TX result comes through the callback, once per sendFrame() (synchronously or not)
        masterInterface->sendFrame(request, [](ModbusInterface::IInterface::Result result, void* ctx) {
            static_cast<Bridge*>(ctx)->onTxResult(result, Modbus::GATEWAY_PATH_UNAVAILABLE);
        }, this);
    }

    /* @brief TX result of the forwarded request: a failure closes it with an exception
     * @note No request is admitted until it came back: the pending request, if any, is its own
     */
    void onTxResult(ModbusInterface::IInterface::Result result, Modbus::ExceptionCode ec) {
        Modbus::Frame exception;
        {
            Lock guard(_pendingMutex);
            _txPending = false;
            if (!_pendingActive) {
                // Closed before its TX result (e.g. timeout before the frame was queued): the
                // transaction its frame claimed meanwhile is released here
                masterInterface->releaseLateClaim(this);
                return;
            }
            if (result == ModbusInterface::IInterface::SUCCESS || !closePendingLocked(ec, exception, false)) return;
        }
        Modbus::Debug::LOG_MSGF("Bridge: replying exception %s", Modbus::toString(ec));
        slaveInterface->sendFrame(exception, nullptr, nullptr);
    }

    /* @brief Forward a response from the master side to the slave side
     * @note Matched to the pending request by slave ID & FC only (RTU has no transaction ID):
     *       a late or unsolicited response is dropped, unless a request to the same slave & FC
     *       is pending, which takes it as its answer
     * @note Not before the TX result of the pending request was notified: until then, the frame
     *       answers an earlier one
     */
    void forwardResponse(const Modbus::Frame& response) {
        {
            Lock guard(_pendingMutex);
            if (!_pendingActive || _txPending || response.slaveId != _pending.slaveId || response.fc != _pending.fc) {
                Modbus::Debug::LOG_MSG("Bridge: dropping unexpected or late response");
                return;
            }
            _pendingActive = false;
            masterInterface->releaseTransaction(this, false);
        }
        // The timer isn't stopped: its callback finds no expired request (stopping it here could
        // cancel the timer of a request forwarded meanwhile)
        slaveInterface->sendFrame(response, nullptr, nullptr);
    }

    /* @brief Timer callback: the device behind the bridge didn't answer in time */
    static void timeoutCallback(TimerHandle_t timer) {
        auto* self = static_cast<Bridge*>(pvTimerGetTimerID(timer));
        self->closePendingWithException(Modbus::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND, true);
    }

    /* @brief Close the pending request (if any) by replying an exception to the slave side
     * @param onTimeout Called by the timer: only close the request if its timeout has elapsed (a
     *        late timer event must not expire a newer request)
     * @return True if a pending request was closed
     * @note The master side is released (what's left aborted) under the mutex: before a newer
     *       request can be forwarded
     */
    bool closePendingWithException(Modbus::ExceptionCode ec, bool onTimeout = false) {
        Modbus::Frame exception;
        {
            Lock guard(_pendingMutex);
            if (!_pendingActive) return false; // Already answered
            if (onTimeout && TIME_MS() - _pendingStartMs < _timeoutMs) return false;
            closePendingLocked(ec, exception, onTimeout); // Releases the transaction (aborted on timeout)
        }
        Modbus::Debug::LOG_MSGF("Bridge: replying exception %s", Modbus::toString(ec));
        slaveInterface->sendFrame(exception, nullptr, nullptr);
        return true;
    }

    /* @brief Close the pending request (if any) & build its exception response
     * @param abort True if the request is given up (timeout): what's left of it is aborted
     * @note Caller holds _pendingMutex, and sends the exception after releasing it
     * @return True if a pending request was closed
     */
    bool closePendingLocked(Modbus::ExceptionCode ec, Modbus::Frame& exception, bool abort) {
        if (!_pendingActive) return false;
        _pendingActive = false;
        masterInterface->releaseTransaction(this, abort); // Non-blocking
        exception.clear();
        exception.type = Modbus::RESPONSE;
        exception.fc = _pending.fc;
        exception.slaveId = _pending.slaveId;
        exception.regAddress = _pending.regAddress;
        exception.regCount = _pending.regCount;
        exception.exceptionCode = ec;
        return true;
    }

}; // class Bridge

} // namespace Modbus
