/**
 * @file ModbusCore.h
 * @brief Modbus core definitions
 */

#pragma once

#include "core/ModbusTypes.hpp"
// EventBus moved to ModbusDebug.hpp to break circular dependency


namespace Modbus {

// ===================================================================================
// MODBUS CONSTANTS
// ===================================================================================

    static constexpr size_t MIN_PDU_SIZE = 1;      // FC only
    static constexpr size_t MAX_PDU_SIZE = 253;    // Excludes Slave ID & CRC (not in PDU)
    static constexpr uint16_t MAX_SLAVE_ID = 247;    // We accept up to 247 for regular requests
    static constexpr std::array<uint16_t, 1> BROADCAST_SLAVE_IDS = {0x00}; // 0 only by default (can be extended if needed)
    static constexpr size_t MAX_REGISTERS_READ = 125;   // Maximum registers per request
    static constexpr size_t MAX_REGISTERS_WRITE = 123;   // Maximum registers per request
    static constexpr size_t MAX_COILS_READ = 2000;      // Maximum coils per request
    static constexpr size_t MAX_COILS_WRITE = 1968;      // Maximum coils per request
    static constexpr uint16_t MAX_REG_ADDR = 0xFFFF; // Maximum register address
    static constexpr size_t FRAME_DATASIZE = 125; // Maximum size (registers or packed coils) of Modbus::Frame data field

// ===================================================================================
// MODBUS REGISTER TYPES
// ===================================================================================

    /* @brief The type of a register.
     * @note Representation of the register types used in the Modbus protocol.
     */
    enum RegisterType {
        NULL_RT = 0,
        COIL,
        DISCRETE_INPUT,
        HOLDING_REGISTER,
        INPUT_REGISTER
    };
    static constexpr const char* toString(const RegisterType type) {
        switch (type) {
            case DISCRETE_INPUT: return "Discrete Input";
            case COIL: return "Coil";
            case HOLDING_REGISTER: return "Holding Register";
            case INPUT_REGISTER: return "Input Register";
            default: return "Invalid register type";
        }
    }
    static constexpr bool isValid(const RegisterType type) {
        return type > NULL_RT && type <= INPUT_REGISTER;
    }

// ===================================================================================
// MODBUS FUNCTION CODES
// ===================================================================================

    /* @brief The type of a function code.
     * @note Representation of the function codes used in the Modbus protocol.
     */
    enum FunctionCode {
        NULL_FC = 0x00,
        READ_COILS = 0x01,
        READ_DISCRETE_INPUTS = 0x02,
        READ_HOLDING_REGISTERS = 0x03,
        READ_INPUT_REGISTERS = 0x04,
        WRITE_COIL = 0x05,
        WRITE_REGISTER = 0x06,
        WRITE_MULTIPLE_COILS = 0x0F,
        WRITE_MULTIPLE_REGISTERS = 0x10,
        NB_FC
    };
    static constexpr const char* toString(FunctionCode fc) {
        switch (fc) {
            case NULL_FC: return "Null function code";
            case READ_COILS: return "Read coils";
            case READ_DISCRETE_INPUTS: return "Read discrete inputs";
            case READ_HOLDING_REGISTERS: return "Read holding registers";
            case READ_INPUT_REGISTERS: return "Read input registers";
            case WRITE_COIL: return "Write single coil";
            case WRITE_REGISTER: return "Write single register";
            case WRITE_MULTIPLE_COILS: return "Write multiple coils";
            case WRITE_MULTIPLE_REGISTERS: return "Write multiple registers";
            default: return "Invalid function code";
        }
    }
    static constexpr bool isValid(const FunctionCode fc) {
        return fc >= NULL_FC && fc < NB_FC;
    }
    
    /* @brief Convert a function code to a register type
     * @param fc The function code to convert
     * @return The register type
     */
    static constexpr RegisterType toRegisterType(const FunctionCode fc) {
        switch (fc) {
            case READ_COILS: 
            case WRITE_COIL:
            case WRITE_MULTIPLE_COILS:
                return RegisterType::COIL;
            case READ_DISCRETE_INPUTS:
                return RegisterType::DISCRETE_INPUT;
            case READ_HOLDING_REGISTERS:
            case WRITE_REGISTER:
            case WRITE_MULTIPLE_REGISTERS:
                return RegisterType::HOLDING_REGISTER;
            case READ_INPUT_REGISTERS:
                return RegisterType::INPUT_REGISTER;
            default:
                return RegisterType::NULL_RT; // TODO: add a NULL_TYPE
        }
    }

// ===================================================================================
// MODBUS EXCEPTION CODES
// ===================================================================================

    /* @brief The type of an exception code.
     * @note This is a representation of the exception codes used in the Modbus protocol.
     */
    enum ExceptionCode {
        NULL_EXCEPTION = 0x00,
        ILLEGAL_FUNCTION = 0x01,
        ILLEGAL_DATA_ADDRESS = 0x02,
        ILLEGAL_DATA_VALUE = 0x03,
        SLAVE_DEVICE_FAILURE = 0x04,
        ACKNOWLEDGE = 0x05,
        SLAVE_DEVICE_BUSY = 0x06,
        NEGATIVE_ACKNOWLEDGE = 0x07,
        MEMORY_PARITY_ERROR = 0x08,
        NB_EC
    };
    static constexpr const char* toString(ExceptionCode ec) {
        switch (ec) {
            case NULL_EXCEPTION: return "No exception";
            case ILLEGAL_FUNCTION: return "Illegal function";
            case ILLEGAL_DATA_ADDRESS: return "Illegal data address";
            case ILLEGAL_DATA_VALUE: return "Illegal data value";
            case SLAVE_DEVICE_FAILURE: return "Slave device failure";
            case ACKNOWLEDGE: return "Acknowledge";
            case SLAVE_DEVICE_BUSY: return "Slave device busy";
            case NEGATIVE_ACKNOWLEDGE: return "Negative acknowledge";
            case MEMORY_PARITY_ERROR: return "Memory parity error";
            default: return "Invalid exception code";
        }
    }
    static constexpr bool isValid(const ExceptionCode ec) {
        return ec >= NULL_EXCEPTION && ec < NB_EC;
    }

// ===================================================================================
// MODBUS MESSAGE TYPES
// ===================================================================================

    /* @brief The type of a Modbus message.
     * @note Representation of the message types used in the Modbus protocol.
     */
    enum MsgType {
        NULL_MSG,
        REQUEST,
        RESPONSE
    };
    static constexpr const char* toString(MsgType type) {
        switch (type) {
            case NULL_MSG: return "Undefined message type";
            case REQUEST: return "Request";
            case RESPONSE: return "Response";
            default: return "Invalid message type";
        }
    }
    static constexpr bool isValid(const MsgType type) {
        return type == REQUEST || type == RESPONSE;
    }

// ===================================================================================
// MODBUS ROLES
// ===================================================================================

    /* @brief The role of a Modbus device.
     * @note Representation of the roles used in the Modbus protocol.
     * @note Legacy & modern semantics are mixed in this enum.
     */
    enum Role {
        SLAVE = 0,
        SERVER = SLAVE,  // Alias for SLAVE
        MASTER = 1,
        CLIENT = MASTER  // Alias for MASTER
    };
    static constexpr bool isValid(const Role role) {
        return role == SLAVE || role == SERVER || role == MASTER || role == CLIENT;
    }


// ===================================================================================
// HELPER FUNCTIONS
// ===================================================================================

    /* @brief Check if a slave ID is a broadcast address
     * @brief ATTENTION: this function checks ONLY the Slave ID, not the message type (req/resp) or the FC (read/write).
     * @param slaveId The slave ID to check
     * @return true if it's a broadcast address, false otherwise
     */
    inline bool isBroadcastId(const uint8_t slaveId) {
        for (auto id : BROADCAST_SLAVE_IDS) {
            if (slaveId == id) return true;
        }
        return false;
    }

} // namespace Modbus