/**
 * @file ModbusBridge.hpp
 * @brief Modbus bridge implementation
 */

#pragma once

#include "core/ModbusCore.h"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"

namespace Modbus {

class Bridge {
public:
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
     */
    Bridge(ModbusInterface::IInterface& Interface1, 
           ModbusInterface::IInterface& Interface2)
        : interface1(Interface1)
        , interface2(Interface2) {}

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

        // Both interfaces must be initialized
        auto resInitMaster = masterInterface->begin();
        if (resInitMaster != ModbusInterface::IInterface::SUCCESS) {
            return Error(ERR_INIT_FAILED, "failed to initialize master interface");
        }   
        auto resInitSlave = slaveInterface->begin();
        if (resInitSlave != ModbusInterface::IInterface::SUCCESS) {
            return Error(ERR_INIT_FAILED, "failed to initialize slave interface");
        }
        
        // Configure the master interface callback to redirect to the slave
        auto resSetCbMaster = masterInterface->setRcvCallback([](const Modbus::Frame& frame, void* ctx) {
            auto* self = static_cast<Bridge*>(ctx);
            if (frame.type == Modbus::RESPONSE) { // The master interface only sends responses back to the slave
                self->slaveInterface->sendFrame(frame, nullptr, nullptr);
            }
        }, this);
        if (resSetCbMaster != ModbusInterface::IInterface::SUCCESS) {
            return Error(ERR_INIT_FAILED, "failed to set master interface callback");
        }

        // Configure the slave interface callback to redirect to the master
        auto resSetCbSlave = slaveInterface->setRcvCallback([](const Modbus::Frame& frame, void* ctx) {
            auto* self = static_cast<Bridge*>(ctx);
            if (frame.type == Modbus::REQUEST) { // The slave interface only forwards requests to the master
                self->masterInterface->sendFrame(frame, nullptr, nullptr);
            }
        }, this);
        if (resSetCbSlave != ModbusInterface::IInterface::SUCCESS) {
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
    ModbusInterface::IInterface* masterInterface;
    ModbusInterface::IInterface* slaveInterface;

}; // class Bridge

} // namespace Modbus
