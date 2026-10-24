/**
 * @file ModbusInterface.hpp
 * @brief Modbus interface abstract class
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusCodec.hpp"
#include "utils/ModbusDebug.hpp"

namespace ModbusInterface {

    class IInterface;

    /* @brief Receive callback type (function ptr + context to avoid dynamic allocations)
     * @param frame The frame to process
     * @param ctx   User context pointer (passed back as-is)
     * @note Validity of the frame is guaranteed ONLY until the callback returns
     */
    using RcvCallbackFn = void (*)(const Modbus::Frame& frame, void* ctx);
    struct RcvCallback {
        RcvCallbackFn fn = nullptr;
        void* ctx = nullptr;
    };

    class IInterface {
    public:

        // Define max number of registered callbacks stored
        // (number of instances of applications like ModbusClient & ModbusServer
        // that can share the same interface)
        static constexpr uint8_t MAX_RCV_CALLBACKS = 5; 

        // Result of a Modbus operation
        enum Result {
            SUCCESS,
            NODATA,
            ERR_INIT_FAILED,
            ERR_INVALID_FRAME,
            ERR_BUSY,
            ERR_RX_FAILED,
            ERR_SEND_FAILED,
            ERR_INVALID_MSG_TYPE,
            ERR_INVALID_TRANSACTION_ID,
            ERR_TIMEOUT,
            ERR_INVALID_ROLE,
            ERR_ADD_CALLBACK_BUSY,
            ERR_TOO_MANY_CALLBACKS,
            ERR_NO_CALLBACKS,
            ERR_NOT_INITIALIZED,
            ERR_CONNECTION_FAILED,
            ERR_CONFIG_FAILED
        };
        static constexpr const char* toString(Result result) {
            switch (result) {
                case SUCCESS: return "success";
                case NODATA: return "no data to process";
                case ERR_INIT_FAILED: return "init failed";
                case ERR_INVALID_FRAME: return "invalid frame";
                case ERR_BUSY: return "busy";
                case ERR_RX_FAILED: return "RX error";
                case ERR_SEND_FAILED: return "send failed";
                case ERR_INVALID_MSG_TYPE: return "invalid message type";
                case ERR_INVALID_TRANSACTION_ID: return "transaction id mismatch";
                case ERR_TIMEOUT: return "timeout";
                case ERR_INVALID_ROLE: return "invalid role";
                case ERR_ADD_CALLBACK_BUSY: return "callback store is busy";
                case ERR_TOO_MANY_CALLBACKS: return "too many callbacks stored";
                case ERR_NO_CALLBACKS: return "no callbacks stored";
                case ERR_NOT_INITIALIZED: return "interface not initialized";
                case ERR_CONNECTION_FAILED: return "connection failed";
                case ERR_CONFIG_FAILED: return "configuration failed";
                default: return "unknown error";
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

        /* @brief TX result callback type
         * @param result The result of the TX operation
         * @param ctx Internal context pointer
         * @note Called from rxTxTask context when TX operation completes
         */
        using TxResultCallback = void (*)(Result result, void* ctx);

        virtual ~IInterface() = default;
        
        /* @brief Initialize the interface
         * @return The result of the operation
         */
        virtual Result begin() = 0;

        /* @brief Send a frame with TX callback
         * @param frame The frame to send
         * @param txCallback Callback to call when TX completes (optional)
         * @param ctx Internal context pointer passed to callback (optional)
         * @return The result of the operation
         */
        virtual Result sendFrame(const Modbus::Frame& frame, 
                                 TxResultCallback txCallback = nullptr, void* ctx = nullptr) = 0;

        /* @brief Check if the interface is ready to transmit a frame
         * @return true if the interface is ready, false otherwise
         */
        virtual bool isReady() = 0;
        
        /* @brief Abort current transaction (cleanup hint from client)
         * @note Called when client times out to allow transport cleanup
         * @note Default implementation is no-op for stateless transports
         */
        virtual void abortCurrentTransaction() {};

        /* @brief Get the role of the interface
         * @return The role of the interface
         */
        Modbus::Role getRole() const {
            return _role;
        }

        /* @brief Set a callback for spontaneous messages
         * @param callback The callback to call when a message is received
         * @return The result of the operation
         */
        Result setRcvCallback(RcvCallbackFn fn, void* ctx = nullptr) {
            if (!fn) return Error(ERR_INVALID_FRAME, "null callback");
            Lock guard(_callbackMutex, 0);  // Try-lock sans attente
            if (!guard.isLocked()) return Error(ERR_ADD_CALLBACK_BUSY);

            for (auto& cb : _rcvCallbacks) {
                if (!cb.fn) {
                    cb.fn = fn;
                    cb.ctx = ctx;
                    return Success();
                }
            }
            return Error(ERR_TOO_MANY_CALLBACKS);
        }

        /* @brief Remove the callbacks registered with a context (application destruction)
         * @param ctx The context the callbacks were registered with
         * @note Waits for a callback being called to return: must not be called from a callback
         */
        void removeRcvCallbacks(void* ctx) {
            Lock guard(_callbackMutex);
            for (auto& cb : _rcvCallbacks) {
                if (cb.fn && cb.ctx == ctx) cb = RcvCallback{};
            }
        }

        /* @brief Notify all callbacks for a spontaneous message
         * @param frame The frame to notify
         * @return The result of the operation
         */
        Result notifyCallbacks(const Modbus::Frame& frame) {
            Lock guard(_callbackMutex);  // Lock with max timeout
            bool hasCallback = false;
            for (const auto& cb : _rcvCallbacks) {
                if (cb.fn) {
                    hasCallback = true;
                    cb.fn(frame, cb.ctx);
                    Modbus::Debug::LOG_MSG("Callback notified");
                }
            }
            return hasCallback ? Success() : Error(ERR_NO_CALLBACKS);
        }

        /* @brief Get the catch all slave IDs flag
         * @return true if the catch all slave IDs flag is set, false otherwise
         */
        inline bool checkCatchAllSlaveIds() const {
            return _catchAllSlaveIds;
        }

        /* @brief Get the extended slave IDs flag
         * @return true if slave IDs above the RTU limit (up to 255) are valid on this interface
         */
        inline bool checkExtendedSlaveIds() const {
            return _extendedSlaveIds;
        }

    protected:

        // Callback for spontaneous messages & lock to protect against concurrent access
        // (derived interfaces only call them through notifyCallbacks(): removeRcvCallbacks() relies on it)
        std::array<RcvCallback, MAX_RCV_CALLBACKS> _rcvCallbacks;
        Mutex _callbackMutex; 
        Modbus::Role _role;
        bool _catchAllSlaveIds = false; // Can be overridden by the derived class to ignore Slave ID field (especially TCP server)
        bool _extendedSlaveIds = false; // Can be overridden by the derived class to accept slave IDs up to 255 (TCP Unit IDs)

    }; // class IInterface

} // namespace ModbusInterface
