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
         * @note Called from rxTxTask context when TX operation completes, or from the caller of
         *       sendFrame() when it fails at once
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
         * @note Contract: each call invokes txCallback exactly once, whatever the result (before
         *       returning when it fails at once, from the interface task otherwise)
         * @note Contract: txCallback returns before the response to this frame is delivered to the
         *       receive callbacks (Client & Bridge drop a response whose request's TX result isn't back)
         * @note Client side, interfaces tracking the owner (TCP, RTU): a request sent with a ctx makes
         *       it the owner of the transaction until it calls releaseTransaction(ctx); meanwhile,
         *       other requesters get ERR_BUSY and don't receive its response
         */
        virtual Result sendFrame(const Modbus::Frame& frame, 
                                 TxResultCallback txCallback = nullptr, void* ctx = nullptr) = 0;

        /* @brief Check if the interface is ready to transmit a frame
         * @return true if the interface is ready, false otherwise
         */
        virtual bool isReady() = 0;
        
        /* @brief Abort current transaction (transport cleanup, called by releaseTransaction())
         * @note Called when a request is given up (timeout, destruction while active)
         * @note Default implementation is no-op for stateless transports
         * @note Contract: short & non-blocking, never calls the application back nor takes its
         *       locks (the Client releases under its own mutex)
         */
        virtual void abortCurrentTransaction() {};

        /* @brief Client side: end of the requester's request
         * @param ctx The requester (ctx passed to sendFrame())
         * @param abort True if the request is given up (timeout, destruction while active): what's
         *        left of it (frame not sent yet, response awaited) is aborted before another
         *        requester can claim the transaction. False once it completed (response, TX failure)
         * @note Interfaces tracking the owner: no effect unless ctx owns the transaction (a requester
         *       can't release nor abort another's)
         * @note Interfaces without owners (custom ones not calling claimTransaction()): only the
         *       abort, if requested
         */
        void releaseTransaction(void* ctx, bool abort) {
            if (!_tracksOwner) {
                if (abort) abortCurrentTransaction();
                return;
            }
            releaseOwned(ctx, abort);
        }

        /* @brief Client side: TX result of a request that already ended (e.g. timed out before its
         *        frame was queued)
         * @param ctx The requester (ctx passed to sendFrame())
         * @note If its frame claimed the transaction meanwhile, releases it & aborts what's left (e.g.
         *       a TCP transaction awaiting the response). No effect otherwise, nor on interfaces without
         *       owners (their abort came with the end of the request)
         */
        void releaseLateClaim(void* ctx) {
            if (_tracksOwner) releaseOwned(ctx, true);
        }

        /* @brief Wait until no TX result callback registered with a context is pending or running
         * @param ctx The context passed to sendFrame() (application destruction)
         * @note Default implementation for interfaces calling TX callbacks from sendFrame() only
         */
        virtual void waitTxCallbackDone(void* ctx) {};

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
         * @note Client side: only the owner of the transaction gets the response
         */
        Result notifyCallbacks(const Modbus::Frame& frame) {
            Lock guard(_callbackMutex);  // Lock with max timeout
            void* owner = (_role == Modbus::MASTER) ? currentOwner() : nullptr;
            bool hasCallback = false;
            for (const auto& cb : _rcvCallbacks) {
                if (cb.fn && (!owner || cb.ctx == owner)) {
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

        // Client side transaction owner, for the derived classes that track it (_tracksOwner set in
        // their constructor). Called from sendFrame() under their TX lock: no other requester can
        // claim meanwhile, the owner can only release. Lock order: TX lock, owner, abort's own locks.

        /* @brief Check that no other requester owns the transaction
         * @note A frame sent without ctx (e.g. a forwarded broadcast) needs a free transaction
         */
        bool ownedByOther(void* ctx) {
            Lock guard(_ownerMutex);
            return _owner && _owner != ctx;
        }

        /* @brief Claim the transaction for the requester, before queuing its frame
         * @return True if newly claimed: undo with unclaimTransaction() if the frame isn't queued
         */
        bool claimTransaction(void* ctx) {
            Lock guard(_ownerMutex);
            if (!ctx || _owner) return false;
            _owner = ctx;
            return true;
        }

        void unclaimTransaction(void* ctx) {
            Lock guard(_ownerMutex);
            if (_owner == ctx) _owner = nullptr;
        }

        bool isOwned() {
            Lock guard(_ownerMutex);
            return _owner != nullptr;
        }

        void* currentOwner() {
            if (!_tracksOwner) return nullptr;
            Lock guard(_ownerMutex);
            return _owner;
        }

        void releaseOwned(void* ctx, bool abort) {
            Lock guard(_ownerMutex);
            if (!ctx || _owner != ctx) return;
            if (abort) abortCurrentTransaction(); // Before another requester can claim
            _owner = nullptr;
        }

        bool _tracksOwner = false;
        void* _owner = nullptr;
        Mutex _ownerMutex;

        // Callback for spontaneous messages & lock to protect against concurrent access
        // (derived interfaces only call them through notifyCallbacks(): removeRcvCallbacks() relies on it)
        std::array<RcvCallback, MAX_RCV_CALLBACKS> _rcvCallbacks;
        Mutex _callbackMutex; 
        Modbus::Role _role;
        bool _catchAllSlaveIds = false; // Can be overridden by the derived class to ignore Slave ID field (especially TCP server)
        bool _extendedSlaveIds = false; // Can be overridden by the derived class to accept slave IDs up to 255 (TCP Unit IDs)

    }; // class IInterface

} // namespace ModbusInterface
