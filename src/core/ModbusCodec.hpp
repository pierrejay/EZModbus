/**
 * @file ModbusCodec.hpp
 * @brief Modbus codecs
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusFrame.hpp"

#ifndef NATIVE_TEST // Do NOT include additional headers for native tests (standalone compilation)
    #include "utils/ModbusDebug.hpp"
#endif

namespace ModbusCodec {

    enum Result {
        SUCCESS,
        // General errors
        ERR_INVALID_TYPE,
        ERR_INVALID_LEN,
        ERR_BUFFER_OVERFLOW,
        // PDU errors
        ERR_INVALID_FC,
        ERR_INVALID_REG_ADDR,
        ERR_INVALID_REG_COUNT,
        ERR_INVALID_BYTE_COUNT,
        ERR_INVALID_DATA,
        ERR_INVALID_EXCEPTION, // Incorrect exception code or type mismatch (request with non-null exception code)
        // RTU errors
        ERR_INVALID_SLAVEID,
        ERR_INVALID_CRC,
        // TCP errors
        ERR_INVALID_MBAP_LEN,
        ERR_INVALID_MBAP_PROTOCOL_ID
    };
    static constexpr const char* toString(const Result result) {
        switch (result) {
            case SUCCESS: return "success";
            case ERR_INVALID_TYPE: return "invalid message type";
            case ERR_INVALID_LEN: return "invalid length";
            case ERR_BUFFER_OVERFLOW: return "buffer overflow";
            case ERR_INVALID_FC: return "invalid function code";
            case ERR_INVALID_REG_ADDR: return "invalid register address";
            case ERR_INVALID_REG_COUNT: return "invalid register count";
            case ERR_INVALID_BYTE_COUNT: return "invalid byte count";
            case ERR_INVALID_DATA: return "invalid data";
            case ERR_INVALID_EXCEPTION: return "invalid exception";
            case ERR_INVALID_SLAVEID: return "invalid slave ID";
            case ERR_INVALID_CRC: return "invalid CRC";
            case ERR_INVALID_MBAP_LEN: return "invalid MBAP length";
            case ERR_INVALID_MBAP_PROTOCOL_ID: return "invalid MBAP protocol ID";
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


    // "Raw" validation methods - work both for encoding and decoding using 
    // the raw values of the frame fields (allows on-the-fly validation for decoding)
    // except for MsgType (never part of the raw frame)

    /* @brief Check if a message type is valid
     * @param type The message type to check
     * @return true if the message type is valid, false otherwise
     */
    inline bool isValidMessageType(const Modbus::MsgType type) {
        return Modbus::isValid(type);
    }

    /* @brief Check if a slave ID is valid
     * @param slaveId The slave ID to check
     * @param fc The function code
     * @param type The message type
     * @param extendedSlaveId Whether the slave ID is extended
     * @return true if the slave ID is valid, false otherwise
     */
    inline bool isValidSlaveId(const uint8_t slaveId, const uint8_t fc, const Modbus::MsgType type, bool extendedSlaveId = false) {
        // All regular slave IDs are valid
        if (slaveId >= 1 && slaveId <= Modbus::MAX_SLAVE_ID) return true;

        // For responses, only regular slaves IDs are valid
        if (type == Modbus::RESPONSE) return false;

        // If extendedSlaveId is true, we won't check further
        if (extendedSlaveId) return true;

        // For requests:
        switch ((Modbus::FunctionCode)fc) {
            // READ requests cannot have a broadcast ID
            case Modbus::READ_COILS:
            case Modbus::READ_DISCRETE_INPUTS:
            case Modbus::READ_HOLDING_REGISTERS:
            case Modbus::READ_INPUT_REGISTERS:
                return false;
            // WRITE requests can have a broadcast ID
            case Modbus::WRITE_COIL:
            case Modbus::WRITE_REGISTER:
            case Modbus::WRITE_MULTIPLE_COILS:
            case Modbus::WRITE_MULTIPLE_REGISTERS:
                for (auto id : Modbus::BROADCAST_SLAVE_IDS) {
                    if (slaveId == id) return true;
                }
                return false;
            default:
                return false;
        }
    }

    /* @brief Check if a register address is valid
     * @param address The register address to check
     * @return true if the register address is valid, false otherwise
     */
    inline bool isValidRegisterAddress(const uint16_t address) {
        return address <= Modbus::MAX_REG_ADDR;
    }
    
    /* @brief Check if a register count is valid
     * @param regCount The register count to check
     * @param fc The function code
     * @param type The message type
     * @return true if the register count is valid, false otherwise
     */
    inline bool isValidRegisterCount(const uint16_t regCount, const uint8_t fc, const Modbus::MsgType type) {
        switch ((Modbus::FunctionCode)fc) {
            case Modbus::READ_COILS:
            case Modbus::READ_DISCRETE_INPUTS:
                return regCount >= 1 && regCount <= (uint32_t)Modbus::MAX_COILS_READ;
            case Modbus::WRITE_COIL:
            case Modbus::WRITE_REGISTER:
                return regCount == 1;
            case Modbus::WRITE_MULTIPLE_COILS:
                return regCount >= 1 && regCount <= (uint32_t)Modbus::MAX_COILS_WRITE;
            case Modbus::READ_HOLDING_REGISTERS:
            case Modbus::READ_INPUT_REGISTERS:
                return regCount >= 1 && regCount <= (uint32_t)Modbus::MAX_REGISTERS_READ;
            case Modbus::WRITE_MULTIPLE_REGISTERS:
                return regCount >= 1 && regCount <= (uint32_t)Modbus::MAX_REGISTERS_WRITE;
            default:
                return false;
        }
    }

    /* @brief Check if a function code is valid
     * @param fc The function code to check
     * @return true if the function code is valid, false otherwise
     */
    inline bool isValidFunctionCode(const uint8_t fc) {
        return Modbus::isValid(static_cast<Modbus::FunctionCode>(fc));
    }

    /* @brief Check if an exception code is valid
     * @param ec The exception code to check
     * @param type The message type
     * @return true if the exception code is valid, false otherwise
     */
    inline bool isValidExceptionCode(const uint8_t ec, const Modbus::MsgType type = Modbus::RESPONSE) {
        if (type == Modbus::REQUEST && ec != 0x00) return false;
        return Modbus::isValid(static_cast<Modbus::ExceptionCode>(ec));
    }

    /* @brief Check if a frame is valid
     * @param frame The frame to check
     * @param extendedSlaveId Whether the slave ID is extended (for TCP frames: Unit ID 1...255 accepted, 0 still broadcast)
     * @return true if the frame is valid, false otherwise
     */
    inline Result isValidFrame(const Modbus::Frame& frame, bool extendedSlaveId = false) {
        if (!isValidMessageType(frame.type)) {
            return Error(ERR_INVALID_TYPE);
        }
        if (!isValidFunctionCode((uint8_t)frame.fc)) {
            return Error(ERR_INVALID_FC);
        }
        if (!isValidExceptionCode((uint8_t)frame.exceptionCode, frame.type)) {
            return Error(ERR_INVALID_EXCEPTION);
        }
        if (!isValidSlaveId((uint8_t)frame.slaveId, (uint8_t)frame.fc, frame.type, extendedSlaveId)) {
            return Error(ERR_INVALID_SLAVEID);
        }
        if (!isValidRegisterAddress((uint16_t)frame.regAddress)) {
            return Error(ERR_INVALID_REG_ADDR);
        }
        if (!isValidRegisterCount((uint16_t)frame.regCount, (uint8_t)frame.fc, frame.type)) {
                return Error(ERR_INVALID_REG_COUNT);
        }
        // isValidRegisterCount() already checks data size
        return Success();
    }

/* @brief The Modbus PDU codec.
 * @note This class is responsible for generating & extracting the PDU from 
 *       Modbus::Frame to raw data and vice versa.
 */
class PDU {

    public:
    
    /* @brief Set the PDU of a Modbus::Frame from a byte buffer.
     * @note PDU = FC + data only (no CRC, no slave/Unit ID)
     * @note If an error occurs, the whole frame is cleared.
     * @param bytes The byte buffer to set the PDU from
     * @param pdu The Modbus::Frame to set the PDU to
     * @param type The message type
     * @return The result of the operation
     */
    static Result setFromBytes(const ByteBuffer& bytes, Modbus::Frame& pdu, 
                               const Modbus::MsgType type) {
        // Check buffer & size are valid
        if (bytes.empty()) return HandleError(pdu, ERR_INVALID_LEN, "empty buffer");

        // Extract and validate function & exception code
        pdu.fc = (Modbus::FunctionCode)bytes[0];
        pdu.exceptionCode = Modbus::NULL_EXCEPTION;
        
        // Flip FC bit 7 if exception code is not NULL
        bool isException = ((uint8_t)pdu.fc & 0x80) != 0;
        if (isException) pdu.fc = (Modbus::FunctionCode)((uint8_t)pdu.fc & 0x7F);
        if (!isValidFunctionCode(pdu.fc)) return HandleError(pdu, ERR_INVALID_FC);

        // Validate exception code
        if (isException) {
            if (bytes.size() != 2) return HandleError(pdu, ERR_INVALID_LEN);
            uint8_t ec = bytes.data()[1];
            if (!isValidExceptionCode(ec, type)) return HandleError(pdu, ERR_INVALID_EXCEPTION);
            pdu.exceptionCode = (Modbus::ExceptionCode)ec;
            return SUCCESS;
        }

        // Validate data structure depending on the function code
        switch (pdu.fc) {
            case Modbus::READ_COILS:
            case Modbus::READ_DISCRETE_INPUTS: {
                if (type == Modbus::REQUEST) {
                    if (bytes.size() != 5) return HandleError(pdu, ERR_INVALID_LEN);
                    pdu.regAddress = (bytes[1] << 8) | bytes[2];
                    pdu.regCount = (bytes[3] << 8) | bytes[4];
                    if (!isValidRegisterCount((uint16_t)pdu.regCount, (uint8_t)pdu.fc, type)) return HandleError(pdu, ERR_INVALID_REG_COUNT);
                } else {
                    if (bytes.size() < 3) return HandleError(pdu, ERR_INVALID_LEN);
                    uint8_t byteCount = bytes[1];
                    if (bytes.size() != byteCount + 2) return HandleError(pdu, ERR_INVALID_LEN);
                    
                    size_t totalCoils = 0;
                    for (uint8_t i = 0; i < byteCount; i++) {
                        size_t coilsExtracted = extractCoils(bytes[2 + i], pdu.data.data(), totalCoils);
                        totalCoils += coilsExtracted;
                    }
                    pdu.regCount = totalCoils;  // regCount = nombre exact de coils
                }
                break;
            }

            case Modbus::READ_HOLDING_REGISTERS:
            case Modbus::READ_INPUT_REGISTERS: {
                if (type == Modbus::REQUEST) {
                    if (bytes.size() != 5) return HandleError(pdu, ERR_INVALID_LEN);
                    pdu.regAddress = (bytes[1] << 8) | bytes[2];
                    pdu.regCount = (bytes[3] << 8) | bytes[4];
                    if (!isValidRegisterCount((uint16_t)pdu.regCount, (uint8_t)pdu.fc, type)) {
                        return HandleError(pdu, ERR_INVALID_REG_COUNT);
                    }
                } else {
                    if (bytes.size() < 2) return HandleError(pdu, ERR_INVALID_LEN);
                    uint8_t byteCount = bytes[1];
                    if (bytes.size() != byteCount + 2) return HandleError(pdu, ERR_INVALID_LEN);
                    if (byteCount % 2 != 0) return HandleError(pdu, ERR_INVALID_BYTE_COUNT);
                    
                    pdu.clearData(false);
                    for (uint8_t i = 0; i < byteCount; i += 2) {
                        uint16_t value = (bytes[2 + i] << 8) | bytes[2 + i + 1];
                        pdu.data[i/2] = value;  // Diviser par 2 pour avoir le bon index
                    }
                    pdu.regCount = byteCount / 2;
                }
                break;
            }

            case Modbus::WRITE_COIL: {
                if (bytes.size() != 5) return HandleError(pdu, ERR_INVALID_LEN);
                pdu.regAddress = (bytes[1] << 8) | bytes[2];
                uint16_t value = (bytes[3] << 8) | bytes[4];
                if (value != 0x0000 && value != 0xFF00) return HandleError(pdu, ERR_INVALID_DATA, "invalid coil value");
                uint16_t val = static_cast<uint16_t>(value == 0xFF00 ? 1u : 0u);
                pdu.data[0] = val;
                pdu.regCount = 1;
                break;
            }

            case Modbus::WRITE_REGISTER: {
                if (bytes.size() != 5) return HandleError(pdu, ERR_INVALID_LEN);
                pdu.regAddress = (bytes[1] << 8) | bytes[2];
                uint16_t val = (uint16_t)((bytes[3] << 8) | bytes[4]);
                pdu.data[0] = val;
                pdu.regCount = 1;
                break;
            }

            case Modbus::WRITE_MULTIPLE_COILS: {
                if (type == Modbus::REQUEST) {
                    if (bytes.size() < 6) return HandleError(pdu, ERR_INVALID_LEN);
                    pdu.regAddress = (bytes[1] << 8) | bytes[2];
                    pdu.regCount = (bytes[3] << 8) | bytes[4];
                    uint8_t byteCount = bytes[5];
                    if (bytes.size() != byteCount + 6) return HandleError(pdu, ERR_INVALID_LEN);
                    if (byteCount != (pdu.regCount + 7) / 8) return HandleError(pdu, ERR_INVALID_BYTE_COUNT);
                    
                    size_t totalCoils = 0;
                    for (uint8_t i = 0; i < byteCount; i++) {
                        size_t coilsToExtract = std::min(pdu.regCount - totalCoils, size_t(8));
                        size_t extracted = extractCoils(bytes[6 + i], pdu.data.data(), totalCoils, coilsToExtract);
                        totalCoils += extracted;
                    }
                } else {
                    if (bytes.size() != 5) return HandleError(pdu, ERR_INVALID_LEN);
                    pdu.regAddress = (bytes[1] << 8) | bytes[2];
                    pdu.regCount = (bytes[3] << 8) | bytes[4];
                }
                break;
            }

            case Modbus::WRITE_MULTIPLE_REGISTERS: {
                if (type == Modbus::REQUEST) {
                    if (bytes.size() < 6) return HandleError(pdu, ERR_INVALID_LEN);
                    pdu.regAddress = (bytes[1] << 8) | bytes[2];
                    pdu.regCount = (bytes[3] << 8) | bytes[4];
                    uint8_t byteCount = bytes[5];
                    if (bytes.size() != byteCount + 6) return HandleError(pdu, ERR_INVALID_LEN);
                    if (byteCount != pdu.regCount * 2) return HandleError(pdu, ERR_INVALID_BYTE_COUNT);
                    
                    pdu.clearData(false);
                    for (uint8_t i = 0; i < byteCount; i += 2) {
                        uint16_t value = (bytes[6 + i] << 8) | bytes[6 + i + 1];
                        pdu.data[i/2] = value;
                    }
                } else {
                    if (bytes.size() != 5) return HandleError(pdu, ERR_INVALID_LEN);
                    pdu.regAddress = (bytes[1] << 8) | bytes[2];
                    pdu.regCount = (bytes[3] << 8) | bytes[4];
                }
                break;
            }

            default:
                return HandleError(pdu, ERR_INVALID_FC);
        }

        return Success();
    }

    /* @brief Append the PDU of a Modbus::Frame to a byte vector.
     * @note The bytes are cleared if an error occurs.
     * @note If a position is provided, the PDU is appended at the given index,
     *       otherwise the PDU is appended at the end of the buffer.
     * @param pdu The Modbus::Frame to append
     * @param type The message type
     * @param bytes The byte vector to append to
     * @param pos The position (index) to append the PDU to, or none/SIZE_MAX to append at the end
     * @return The result of the operation
     */
    static Result appendToBytes(const Modbus::Frame& pdu, const Modbus::MsgType type, 
                                ByteBuffer& bytes, size_t pos = SIZE_MAX) {
        // Resize the buffer if needed to append at the given position
        if (pos != SIZE_MAX) {
            if (!bytes.resize(pos)) return Error(ERR_BUFFER_OVERFLOW);
        }

        // Flip FC bit 7 if exception code is not NULL
        uint8_t fc = (uint8_t)pdu.fc;
        if (pdu.exceptionCode > 0x00) fc |= 0x80;

        // Start building the frame
        bytes.push_back(fc);
        
        // If it's an exception, add the exception code and skip filling the rest of bytes
        if (pdu.exceptionCode != Modbus::NULL_EXCEPTION) {
            bytes.push_back((uint8_t)pdu.exceptionCode);
        } 

        // Otherwise, build the rest of the frame, doing further checks on register count & data size
        else {
            switch (pdu.fc) {
                case Modbus::READ_COILS:
                case Modbus::READ_DISCRETE_INPUTS:
                case Modbus::READ_HOLDING_REGISTERS:
                case Modbus::READ_INPUT_REGISTERS: {
                    if (type == Modbus::REQUEST) {
                        bytes.push_back((pdu.regAddress >> 8) & 0xFF);
                        bytes.push_back(pdu.regAddress & 0xFF);
                        bytes.push_back((pdu.regCount >> 8) & 0xFF);
                        bytes.push_back(pdu.regCount & 0xFF);
                    } else {
                        bool isRegister = (pdu.fc == Modbus::READ_HOLDING_REGISTERS || pdu.fc == Modbus::READ_INPUT_REGISTERS);
                        uint8_t byteCount;
                        if (isRegister) {
                            byteCount = pdu.regCount * 2;
                            bytes.push_back(byteCount);
                            for (uint16_t i = 0; i < pdu.regCount; i++) {
                                uint16_t value = pdu.data[i];
                                bytes.push_back((value >> 8) & 0xFF);
                                bytes.push_back(value & 0xFF);
                            }
                        } else {
                            byteCount = (pdu.regCount + 7) / 8;  // regCount = nombre de coils
                            if (byteCount == 0) return HandleError(bytes, ERR_INVALID_DATA, "null byte count");
                            bytes.push_back(byteCount);
                            
                            // Nouveau: utiliser formatCoils
                            for (size_t coilIndex = 0; coilIndex < pdu.regCount; coilIndex += 8) {
                                uint8_t byte;
                                size_t coilsInByte = std::min(size_t(8), pdu.regCount - coilIndex);
                                if (!formatCoils(pdu.data.data(), coilIndex, byte, coilsInByte)) {
                                    return HandleError(bytes, ERR_INVALID_DATA, "invalid coil formatting");
                                }
                                bytes.push_back(byte);
                            }
                        }
                    }
                    break;
                }

                case Modbus::WRITE_COIL: {
                    bytes.push_back((pdu.regAddress >> 8) & 0xFF);
                    bytes.push_back(pdu.regAddress & 0xFF);
                    bool coil = (pdu.data[0] & 0x0001) != 0;
                    // If other bits are set, it's an error
                    if (pdu.data[0] & ~0x0001)
                        return HandleError(bytes, ERR_INVALID_DATA, "invalid coil data");
                    // OK, we encode
                    uint16_t value = coil ? 0xFF00 : 0x0000;
                    bytes.push_back((value >> 8) & 0xFF);
                    bytes.push_back(value & 0xFF);
                    break;
                }

                case Modbus::WRITE_REGISTER: {
                    bytes.push_back((pdu.regAddress >> 8) & 0xFF);
                    bytes.push_back(pdu.regAddress & 0xFF);
                    bytes.push_back((pdu.data[0] >> 8) & 0xFF);
                    bytes.push_back(pdu.data[0] & 0xFF);
                    break;
                }

                case Modbus::WRITE_MULTIPLE_COILS: {
                    if (type == Modbus::REQUEST) {
                        if (pdu.regCount == 0) {
                            return HandleError(bytes, ERR_INVALID_DATA, "zero coil count");
                        }
                        bytes.push_back((pdu.regAddress >> 8) & 0xFF);
                        bytes.push_back(pdu.regAddress & 0xFF);
                        bytes.push_back((pdu.regCount >> 8) & 0xFF);
                        bytes.push_back(pdu.regCount & 0xFF);

                        uint8_t byteCount = (pdu.regCount + 7) / 8;
                        bytes.push_back(byteCount);
                        
                        // Nouveau: utiliser formatCoils directement
                        for (size_t coilIndex = 0; coilIndex < pdu.regCount; coilIndex += 8) {
                            uint8_t byte;
                            size_t coilsInByte = std::min(size_t(8), pdu.regCount - coilIndex);
                            if (!formatCoils(pdu.data.data(), coilIndex, byte, coilsInByte)) {
                                return HandleError(bytes, ERR_INVALID_DATA, "invalid coil formatting");
                            }
                            bytes.push_back(byte);
                        }
                    } else {
                        bytes.push_back((pdu.regAddress >> 8) & 0xFF);
                        bytes.push_back(pdu.regAddress & 0xFF);
                        bytes.push_back((pdu.regCount >> 8) & 0xFF);
                        bytes.push_back(pdu.regCount & 0xFF);
                    }
                    break;
                }

                case Modbus::WRITE_MULTIPLE_REGISTERS: {
                    if (type == Modbus::REQUEST) {
                        bytes.push_back((pdu.regAddress >> 8) & 0xFF);
                        bytes.push_back(pdu.regAddress & 0xFF);
                        bytes.push_back((pdu.regCount >> 8) & 0xFF);
                        bytes.push_back(pdu.regCount & 0xFF);
                        
                        uint8_t byteCount = pdu.regCount * 2;
                        bytes.push_back(byteCount);
                        
                        for (int i=0; i < pdu.regCount; i++) {
                            bytes.push_back((pdu.data[i] >> 8) & 0xFF);  // Écrit pdu.data[i]
                            bytes.push_back(pdu.data[i] & 0xFF);
                        }
                    } else {
                        bytes.push_back((pdu.regAddress >> 8) & 0xFF);
                        bytes.push_back(pdu.regAddress & 0xFF);
                        bytes.push_back((pdu.regCount >> 8) & 0xFF);
                        bytes.push_back(pdu.regCount & 0xFF);
                    }
                    break;
                }

                default:
                    return HandleError(bytes, ERR_INVALID_FC);
            }
        }

        return Success();
    }

    private:

    /* @brief Compute the length of a Modbus PDU.
     * @param pdu The Modbus PDU to compute the length of.
     * @param type The message type (request or response).
     * @return The length of the PDU.
     */
    static size_t computePduLength(const Modbus::Frame& pdu, Modbus::MsgType type) {
        // Always 1 byte for the function code
        // If exception, add 1 byte for the exception and stop.
        if (pdu.exceptionCode != Modbus::NULL_EXCEPTION) {
            return /*FC*/1 + /*exceptionCode*/1;
        }

        switch (pdu.fc) {
            case Modbus::READ_COILS:
            case Modbus::READ_DISCRETE_INPUTS: {
                if (type == Modbus::REQUEST) {
                    // FC + addrHi + addrLo + countHi + countLo
                    return 1 + 2 + 2;
                } else {
                    // FC + byteCount + N bytes of coils
                    size_t byteCount = (pdu.regCount + 7) / 8;
                    return 1 + 1 + byteCount;
                }
            }

            case Modbus::READ_HOLDING_REGISTERS:
            case Modbus::READ_INPUT_REGISTERS: {
                if (type == Modbus::REQUEST) {
                    // FC + addrHi + addrLo + countHi + countLo
                    return 1 + 2 + 2;
                } else {
                    // FC + byteCount + 2*regCount bytes
                    size_t byteCount = pdu.regCount * 2;
                    return 1 + 1 + byteCount;
                }
            }

            case Modbus::WRITE_COIL:
            case Modbus::WRITE_REGISTER:
                // FC + addrHi + addrLo + valueHi + valueLo
                return 1 + 2 + 2;

            case Modbus::WRITE_MULTIPLE_COILS: {
                if (type == Modbus::REQUEST) {
                    // FC + addrHi + addrLo + countHi + countLo + byteCount + N bytes
                    size_t byteCount = (pdu.regCount + 7) / 8;
                    return 1 + 2 + 2 + 1 + byteCount;
                } else {
                    // FC + addrHi + addrLo + countHi + countLo
                    return 1 + 2 + 2;
                }
            }

            case Modbus::WRITE_MULTIPLE_REGISTERS: {
                if (type == Modbus::REQUEST) {
                    // FC + addrHi + addrLo + countHi + countLo + byteCount + 2*regCount bytes
                    size_t byteCount = pdu.regCount * 2;
                    return 1 + 2 + 2 + 1 + byteCount;
                } else {
                    // FC + addrHi + addrLo + countHi + countLo
                    return 1 + 2 + 2;
                }
            }

            default:
                // At least the FC
                return 1;
        }
    }


    // Template function to clear an object + cast an error (for appendToBytes & setFromBytes)
    template<typename T>
    static Result HandleError(T& objectToClear, Result errorCode, const char* desc = nullptr
                    #ifdef EZMODBUS_DEBUG
                    , Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()
                    #endif
                    ) {
        objectToClear.clear();
        #ifdef EZMODBUS_DEBUG
            return Error(errorCode, desc, ctx);
        #else
            return Error(errorCode, desc);
        #endif
    }


    /* @brief Extract coils from a source byte and store them in a destination array.
     * @param sourceByte The source byte to extract coils from.
     * @param destArray The destination array to store the coils.
     * @param firstCoilIndex The index of the first coil to extract.
     * @param maxCoils The maximum number of coils to extract.
     * @return The number of coils extracted.
     */
    static size_t extractCoils(const uint8_t  sourceByte,
                            uint16_t*      destArray,
                            size_t         firstCoilIndex,
                            size_t         maxCoils = 8)
    {
        for (size_t i = 0; i < maxCoils; ++i) {
            size_t bitPos   = firstCoilIndex + i;   // index global de la bobine
            size_t wordIdx  = bitPos / 16;          // mot de destination
            size_t bitInWord= bitPos % 16;          // bit à modifier

            bool bit = (sourceByte >> i) & 0x01;
            if (bit)  destArray[wordIdx] |=  (1u << bitInWord);
            else      destArray[wordIdx] &= ~(1u << bitInWord);
        }
        return maxCoils;
    }

    /* @brief Format coils from a source array into a destination byte.
     * @param sourceArray The source array to format.
     * @param firstCoilIndex The index of the first coil to format.
     * @param destByte The destination byte.
     * @param coilsInByte The number of coils to format.
     * @return true if the coils were formatted successfully, false otherwise.
     */
    static bool formatCoils(const uint16_t* sourceArray,
                            size_t          firstCoilIndex,
                            uint8_t&        destByte,
                            size_t          coilsInByte = 8)
    {
        if (coilsInByte == 0 || coilsInByte > 8) return false;

        destByte = 0;
        for (size_t i = 0; i < coilsInByte; ++i) {
            size_t bitPos    = firstCoilIndex + i;
            size_t wordIdx   = bitPos / 16;
            size_t bitInWord = bitPos % 16;

            bool bit = (sourceArray[wordIdx] >> bitInWord) & 0x01;
            if (bit) destByte |= (1u << i);         // LSB-first, Modbus style
        }
        return true;
    }

}; // class PDU

/* @brief The Modbus RTU codec.
 * @note This class is responsible for encoding and decoding Modbus RTU frames.
 */
class RTU {
    
public:

    static constexpr uint16_t MIN_FRAME_SIZE = Modbus::MIN_PDU_SIZE + 3;
    static constexpr uint16_t MAX_FRAME_SIZE = Modbus::MAX_PDU_SIZE + 3;

    /* @brief Decode a Modbus frame from a byte buffer.
     * @note To decode, we need to know the frame type (request or response)
     *       as they are ambiguous cases in the Modbus protocol where we couldn't
     *       decide whether it's a request or a response, and the same frame could
     *       be parsed with 2 valid interpretations.
     * @param bytes The byte buffer to decode.
     * @param frame The Modbus frame to decode into.
     * @param type The message type (request or response).
     * @return The result of the operation.
     */
    static Result decode(const ByteBuffer& bytes, Modbus::Frame& frame, const Modbus::MsgType type) {

        // Check if the message type is valid first.
        // The user doesn't have to specify it in the struct, he can just pass
        // an empty frame and the function will fill it with the decoded data.
        if (type != Modbus::REQUEST && type != Modbus::RESPONSE) {
            return Error(ERR_INVALID_TYPE);
        }

        // Validate CRC
        if (!validateCRC(bytes)) return ERR_INVALID_CRC;

        // Validate frame size
        if (bytes.size() < MIN_FRAME_SIZE || bytes.size() > MAX_FRAME_SIZE) {
            return Error(ERR_INVALID_LEN);
        }

        // Extract and validate slave ID (needs the function code to know if it's a broadcast)
        uint8_t rawSlaveId = bytes[0];
        uint8_t rawFc = bytes[1];
        if (!isValidSlaveId(rawSlaveId, rawFc, type)) return Error(ERR_INVALID_SLAVEID);

        // Extract the PDU (without the slave ID and CRC)
        size_t pduStartIdx = 1; // Start after the slave ID
        size_t pduLen = bytes.size() - 3; // -2 for CRC, -1 for slave ID
        ByteBuffer pduSlice = bytes.slice(pduStartIdx, pduLen);
        // Try to decode PDU (will clear the frame if error)
        Result pduResult = PDU::setFromBytes(pduSlice, frame, type);
        if (pduResult != SUCCESS) return Error(pduResult);

        // If all is ok, set the slave ID and the message type
        frame.slaveId = rawSlaveId;
        frame.type = type;

        return Success();

    }

    /* @brief Encode a Modbus frame into a byte buffer.
     * @param frame The Modbus frame to encode.
     * @param bytes The byte buffer to encode into.
     * @return The result of the operation.
     */
    static Result encode(const Modbus::Frame& frame, ByteBuffer& bytes) {
        bytes.clear();

        // Validate frame format
        Result frameResult = isValidFrame(frame);
        if (frameResult != SUCCESS) return Error(frameResult);

        // Add the slave ID
        if (bytes.free_space() < 1) return Error(ERR_BUFFER_OVERFLOW, "buffer too small for encoding");
        bytes.push_back(frame.slaveId);

        // Try to encode PDU (appendToBytes will clear the bytes if error)
        Result pduResult = PDU::appendToBytes(frame, frame.type, bytes);
        if (pduResult != SUCCESS) return Error(pduResult);
        size_t sizeBeforeCRC = bytes.size();

        // Add the CRC at the end
        appendCRC(bytes);
        if (bytes.size() != sizeBeforeCRC + 2) return Error(ERR_BUFFER_OVERFLOW, "buffer too small for CRC");

        return Success();
    }

    // CRC16 Look-Up Table
    static constexpr uint16_t CRC16_TABLE[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    /* @brief Calculate the CRC of a byte buffer.
     * @param bytes The byte buffer to calculate the CRC of.
     * @return The CRC of the byte buffer.
     */
    static uint16_t calculateCRC(const ByteBuffer& bytes) {
        uint16_t crc = 0xFFFF;
        
        for (size_t i = 0; i < bytes.size(); i++) {
            uint8_t pos = (crc ^ bytes[i]) & 0xFF;
            crc = (crc >> 8) ^ CRC16_TABLE[pos];
        }
        
        return crc;
    }

    /* @brief Validate the CRC of a byte buffer.
     * @param bytes The byte buffer to validate the CRC of.
     * @return true if the CRC is valid, false otherwise.
     */
    static bool validateCRC(const ByteBuffer& bytes) {
        if (bytes.size() < 2) return false;
        
        uint16_t receivedCRC = (bytes[bytes.size()-1] << 8) | bytes[bytes.size()-2];
        uint16_t calculatedCRC = calculateCRC(bytes.slice(0, bytes.size()-2));
        
        return receivedCRC == calculatedCRC;
    }

    /* @brief Append the CRC to a frame.
     * @param frameWithoutCRC The frame without the CRC.
     * @note The buffer must have at least 2 free bytes to append the CRC, otherwise
     *       the function will silently fail.
     */
    static void appendCRC(ByteBuffer& frameWithoutCRC) {
        if (frameWithoutCRC.free_space() < 2) return;
        uint16_t crc = calculateCRC(frameWithoutCRC);
        frameWithoutCRC.push_back(crc & 0xFF);         // CRC Low byte first
        frameWithoutCRC.push_back((crc >> 8) & 0xFF);  // CRC High byte second
    }

    /* @brief Build a minimal Modbus-RTU exception frame (5 bytes) into a ByteBuffer.
     * @param slaveId  The slave address.
     * @param fc       The original function code.
     * @param ec       The exception code (e.g. Modbus::SLAVE_DEVICE_BUSY).
     * @param bytes    Destination buffer (capacity ≥ EXCEPTION_FRAME_SIZE).
     * @return true on success, false if capacity is insufficient.
     */
    static inline bool buildException(uint8_t slaveId,
                                      Modbus::FunctionCode fc,
                                      Modbus::ExceptionCode ec,
                                      ByteBuffer& bytes)
    {
        if (bytes.capacity() < EXCEPTION_FRAME_SIZE) return false;

        bytes.clear();
        bytes.push_back(slaveId);
        bytes.push_back(static_cast<uint8_t>(fc) | 0x80); // FC | 0x80
        bytes.push_back(static_cast<uint8_t>(ec));

        // Append CRC (adds two bytes)
        appendCRC(bytes);

        return bytes.size() == EXCEPTION_FRAME_SIZE;
    }

    static constexpr size_t EXCEPTION_FRAME_SIZE = 5;
    
}; // class RTU

class TCP {

public:

    /* @brief The Modbus Application Layer Header.
     * @note This is a representation of the header that is used in the Modbus TCP protocol.
     */
    struct MBAP {
        uint16_t transactionId;
        uint16_t protocolId = 0;
        uint16_t length;
        uint8_t unitId = 1;

        // Write the MBAP to a byte buffer at a given offset from the start
        void writeToBytes(ByteBuffer& bytes, size_t offset) {
            if (bytes.capacity() < offset + MBAP_SIZE) return;
            bytes.write_at(offset + 0, (transactionId >> 8));
            bytes.write_at(offset + 1, (transactionId & 0xFF));
            bytes.write_at(offset + 2, (protocolId >> 8));
            bytes.write_at(offset + 3, (protocolId & 0xFF));
            bytes.write_at(offset + 4, (length >> 8));
            bytes.write_at(offset + 5, (length & 0xFF));
            bytes.write_at(offset + 6, unitId);
        }
    };

    static constexpr uint16_t MBAP_SIZE = 7;
    static constexpr uint16_t MIN_FRAME_SIZE = MBAP_SIZE + Modbus::MIN_PDU_SIZE;
    static constexpr uint16_t MAX_FRAME_SIZE = MBAP_SIZE + Modbus::MAX_PDU_SIZE;

    /* @brief Decode a Modbus frame from a byte buffer.
     * @param bytes The byte buffer to decode.
     * @param frame The Modbus frame to decode into.
     * @param type The message type (request or response).
     * @return The result of the operation.
     */
    static Result decode(const ByteBuffer& bytes, Modbus::Frame& frame, const Modbus::MsgType type) {
        // Check if the message type is valid first.
        // The user doesn't have to specify it in the struct, he can just pass
        // an empty frame and the function will fill it with the decoded data.
        if (type != Modbus::REQUEST && type != Modbus::RESPONSE) {
            return Error(ERR_INVALID_TYPE);
        }

        // Validate frame size
        if (bytes.size() < MIN_FRAME_SIZE || bytes.size() > MAX_FRAME_SIZE) {
            return Error(ERR_INVALID_LEN);
        }

        // Extract & validate MBAP
        MBAP mbap = {
            .transactionId = (uint16_t)((bytes[0] << 8) | bytes[1]),
            .protocolId = (uint16_t)((bytes[2] << 8) | bytes[3]),
            .length = (uint16_t)((bytes[4] << 8) | bytes[5]),
            .unitId = bytes[6]
        };

        // Validate MBAP
        uint16_t pduLength = bytes.size() - MBAP_SIZE;
        if (mbap.length != pduLength + 1) return Error(ERR_INVALID_MBAP_LEN);
        if (mbap.protocolId != 0) return Error(ERR_INVALID_MBAP_PROTOCOL_ID);

        // Extract frame w/o MBAP
        size_t pduStartOffset = MBAP_SIZE;
        size_t pduLen = bytes.size() - pduStartOffset; // Full length - MBAP size
        ByteBuffer pduSlice = bytes.slice(pduStartOffset, pduLen);

        // Try to decode PDU (will clear the frame if error)
        Result pduResult = PDU::setFromBytes(pduSlice, frame, type);
        if (pduResult != SUCCESS) return Error(pduResult);

        // If all is ok, set the slave ID and the message type
        frame.type = type;
        frame.slaveId = mbap.unitId;

        return Success();
    }

    /* @brief Encode a Modbus frame into a byte buffer.
     * @note The transactionID of a response should be the same as the request's.
     * @param frame The Modbus frame to encode.
     * @param bytes The byte buffer to encode into.
     * @param transactionId The transaction ID to use for the frame.
     * @return The result of the operation.
     */
    static Result encode(const Modbus::Frame& frame, ByteBuffer& bytes, const uint16_t transactionId) {
        bytes.clear();

        // Validate frame
        Result frameResult = isValidFrame(frame, true); // We allow extended slave ID for TCP frames
        if (frameResult != SUCCESS) return Error(frameResult);

        // Try to encode PDU (appendToBytes will clear the bytes if error)
        Result pduResult = PDU::appendToBytes(frame, frame.type, bytes, MBAP_SIZE);
        if (pduResult != SUCCESS) return Error(pduResult);

        // Calculate PDU size
        size_t pduSize = bytes.size() - MBAP_SIZE;

        // Create the MBAP
        MBAP mbap = {
            .transactionId = transactionId,
            .protocolId = 0,
            .length = (uint16_t)(pduSize + 1), // +1 for the unit ID
            .unitId = frame.slaveId
        };

        // Add the MBAP to the bytes
        mbap.writeToBytes(bytes, 0);

        return Success();
    }

    /* @brief Build a minimal Modbus-TCP exception frame (9 bytes) into a ByteBuffer.
     * @param transactionId Original transaction ID of the request.
     * @param unitId        Unit ID (Slave ID).
     * @param fc            Original function code.
     * @param ec            Exception code.
     * @param bytes         Destination buffer (capacity ≥ EXCEPTION_FRAME_SIZE).
     * @return true on success, false if capacity is insufficient.
     */
    static inline bool buildException(uint16_t transactionId,
                                      uint8_t  unitId,
                                      Modbus::FunctionCode  fc,
                                      Modbus::ExceptionCode  ec,
                                      ByteBuffer& bytes)
    {
        if (bytes.capacity() < EXCEPTION_FRAME_SIZE) return false;

        bytes.clear();
        bytes.resize(EXCEPTION_FRAME_SIZE); // ensure space and size

        static constexpr uint16_t PROTO_ID = 0;
        static constexpr uint16_t LEN_PDU  = 3; // UnitID + FC + EC

        // MBAP
        bytes.write_at(0, static_cast<uint8_t>((transactionId >> 8) & 0xFF));
        bytes.write_at(1, static_cast<uint8_t>(transactionId & 0xFF));
        bytes.write_at(2, static_cast<uint8_t>((PROTO_ID >> 8) & 0xFF));
        bytes.write_at(3, static_cast<uint8_t>(PROTO_ID & 0xFF));
        bytes.write_at(4, static_cast<uint8_t>((LEN_PDU >> 8) & 0xFF));
        bytes.write_at(5, static_cast<uint8_t>(LEN_PDU & 0xFF));
        bytes.write_at(6, unitId);

        // PDU – exception
        bytes.write_at(7, static_cast<uint8_t>(fc) | 0x80);
        bytes.write_at(8, static_cast<uint8_t>(ec));

        return true;
    }

    static constexpr size_t EXCEPTION_FRAME_SIZE = 9;

}; // class TCP

// ===================================================================================
// DATA TYPE CONVERSION UTILITIES
// ===================================================================================

/* @brief Convert a float to two 16-bit registers (IEEE 754 format)
 * @param value The float value to convert
 * @param registers Output array of 2 registers (must be pre-allocated)
 * @note Stores the float as: registers[0] = upper 16 bits, registers[1] = lower 16 bits
 */
inline void floatToRegisters(float value, uint16_t* registers) {
    uint32_t float_bits;
    memcpy(&float_bits, &value, sizeof(float));
    registers[0] = static_cast<uint16_t>(float_bits >> 16);    // Upper 16 bits
    registers[1] = static_cast<uint16_t>(float_bits & 0xFFFF); // Lower 16 bits
}

/* @brief Convert two 16-bit registers to a float (IEEE 754 format)
 * @param registers Input array of 2 registers
 * @return The reconstructed float value
 * @note Expects: registers[0] = upper 16 bits, registers[1] = lower 16 bits
 */
inline float registersToFloat(const uint16_t* registers) {
    uint32_t float_bits = (static_cast<uint32_t>(registers[0]) << 16) | registers[1];
    float value;
    memcpy(&value, &float_bits, sizeof(float));
    return value;
}

} // namespace ModbusCodec