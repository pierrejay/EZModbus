/**
 * @file ModbusFrame.hpp
 * @brief Modbus::Frame class def/impl & helper functions
 */

#pragma once

#include "core/ModbusCore.h"

namespace Modbus {

// ===================================================================================
// MODBUS FRAME HEADER
// ===================================================================================

/* @brief Contains the detail of a Modbus request without protocol implementation details
* (byte count, crc & frame length are computed by codec functions)
*/
struct Frame {
    // Data members
    Modbus::MsgType type = Modbus::NULL_MSG;
    Modbus::FunctionCode fc = Modbus::NULL_FC;
    uint8_t slaveId = 0;
    uint16_t regAddress = 0;
    uint16_t regCount = 0;
    std::array<uint16_t, FRAME_DATASIZE> data = {};
    Modbus::ExceptionCode exceptionCode = Modbus::NULL_EXCEPTION;

    // Clear functions
    void clear();
    void clearData();

    // Getter functions for frame data
    uint16_t getRegister(size_t index) const;
    std::vector<uint16_t> getRegisters() const;
    size_t getRegisters(uint16_t* dst, size_t dstSize) const;
    bool getCoil(size_t index) const;
    std::vector<bool> getCoils() const;
    size_t getCoils(bool* dst, size_t dstSize) const;

    // Setter functions for frame data
    bool setRegisters(const std::vector<uint16_t>& src);
    bool setRegisters(const std::initializer_list<uint16_t>& src);
    bool setRegisters(const uint16_t* src, size_t len);     
    bool setCoils(const std::vector<bool>& src);
    bool setCoils(const std::vector<uint16_t>& src);
    bool setCoils(const std::initializer_list<bool>& src);
    bool setCoils(const std::initializer_list<uint16_t>& src);
    bool setCoils(const bool* src, size_t len);
    bool setCoils(const uint16_t* src, size_t len);
    
    // Positioned setter functions for streaming operations (NO dynamic allocation)
    bool setRegisters(const uint16_t* src, size_t len, size_t startRegIndex);
    bool setCoils(const std::vector<bool>& src, size_t startCoilIndex);
    bool setCoils(const bool* src, size_t len, size_t startCoilIndex);
};

/* @brief Clear the frame.
    * @note This function clears the frame to its default state.
    */
inline void Frame::clear() {
    type = NULL_MSG;
    fc = NULL_FC;
    slaveId = 0;
    regAddress = 0;
    regCount = 0;
    data.fill(0);
    exceptionCode = NULL_EXCEPTION;
}

// ===================================================================================
// INLINE FUNCTIONS - IN-PLACE DATA PACKING/UNPACKING IN MODBUS::FRAME
// ===================================================================================

/* @brief Pack register data from a vector into data array format.
    * @param src The source vector to pack the registers from.
    * @note Returns an empty array if the source vector is empty.
    * @return The array of registers.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packRegisters(const std::vector<uint16_t>& src) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    size_t count = std::min(src.size(), size_t(FRAME_DATASIZE));
    memcpy(result.data(), src.data(), count * sizeof(uint16_t));
    return result;
}

/* @brief Pack register data from an initializer list into data array format.
    * @param src The source initializer list to pack the registers from.
    * @note Returns an empty array if the source initializer list is empty.
    * @return The array of registers.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packRegisters(std::initializer_list<uint16_t> src) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    size_t i = 0;
    for (auto val : src) {
        if (i < FRAME_DATASIZE) result[i++] = val;
        else break;
    }
    return result;
}

/* @brief Pack register data from a buffer into data array format.
    * @param src The source buffer to pack the registers from.
    * @param len The length of the source buffer.
    * @note Returns an empty array if the source buffer is nullptr or len is 0.
    * @return The array of registers.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packRegisters(const uint16_t* src, size_t len) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    if (!src || len == 0) return result;
    size_t count = std::min(len, size_t(FRAME_DATASIZE));
    memcpy(result.data(), src, count * sizeof(uint16_t));
    return result;
}

/* @brief Pack coil data from a bool vector into data array format.
    * @param src The source vector to pack the coils from.
    * @note Returns an empty array if the source vector is empty.
    * @return The array of registers with packed coils.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const std::vector<bool>& src) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    
    // Pack 16 bits at once to optimize
    for (size_t i = 0; i < src.size() && i < (FRAME_DATASIZE * 16); i += 16) {
        uint16_t packed = 0;
        for (size_t j = 0; j < 16 && (i + j) < src.size(); j++) {
            if (src[i + j]) {
                packed |= (1u << j);
            }
        }
        result[i / 16] = packed;
    }
    return result;
}

/* @brief Pack coil data from a uint16_t vector into data array format (0/non-zero -> bool).
    * @param src The source vector to pack the coils from.
    * @note Returns an empty array if the source vector is empty. Any non-zero value is considered true.
    * @return The array of registers with packed coils.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const std::vector<uint16_t>& src) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    
    // Pack 16 bits at once to optimize
    for (size_t i = 0; i < src.size() && i < (FRAME_DATASIZE * 16); i += 16) {
        uint16_t packed = 0;
        for (size_t j = 0; j < 16 && (i + j) < src.size(); j++) {
            if (src[i + j] != 0) {
                packed |= (1u << j);
            }
        }
        result[i / 16] = packed;
    }
    return result;
}

/* @brief Pack coil data from a bool initializer list into data array format.
    * @param src The source initializer list to pack the coils from.
    * @note Returns an empty array if the source initializer list is empty.
    * @return The array of registers with packed coils.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packCoils(std::initializer_list<bool> src) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    
    size_t bitIndex = 0;
    for (bool bit : src) {
        if (bitIndex >= FRAME_DATASIZE * 16) break;
        
        size_t wordIdx = bitIndex / 16;
        size_t bitPos = bitIndex % 16;
        if (bit) {
            result[wordIdx] |= (1u << bitPos);
        }
        bitIndex++;
    }
    return result;
}

/* @brief Pack coil data from a uint16_t initializer list into data array format (0/non-zero -> bool).
    * @param src The source initializer list to pack the coils from.
    * @note Returns an empty array if the source initializer list is empty. Any non-zero value is considered true.
    * @return The array of registers with packed coils.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packCoils(std::initializer_list<uint16_t> src) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    
    size_t bitIndex = 0;
    for (uint16_t val : src) {
        if (bitIndex >= FRAME_DATASIZE * 16) break;
        
        size_t wordIdx = bitIndex / 16;
        size_t bitPos = bitIndex % 16;
        if (val != 0) {
            result[wordIdx] |= (1u << bitPos);
        }
        bitIndex++;
    }
    return result;
}

/* @brief Pack coil data from a bool buffer into data array format.
    * @param src The source buffer to pack the coils from.
    * @param len The length of the source buffer.
    * @note Returns an empty array if the source buffer is nullptr or len is 0.
    * @return The array of registers with packed coils.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const bool* src, size_t len) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    if (!src || len == 0) return result;
    
    // Pack 16 bits at once to optimize
    for (size_t i = 0; i < len && i < (FRAME_DATASIZE * 16); i += 16) {
        uint16_t packed = 0;
        for (size_t j = 0; j < 16 && (i + j) < len; j++) {
            if (src[i + j]) {
                packed |= (1u << j);
            }
        }
        result[i / 16] = packed;
    }
    return result;
}

/* @brief Pack coil data from a uint16_t buffer into data array format (0/non-zero -> bool).
    * @param src The source buffer to pack the coils from.
    * @param len The length of the source buffer.
    * @note Returns an empty array if the source buffer is nullptr or len is 0. Any non-zero value is considered true.
    * @return The array of registers with packed coils.
    */
inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const uint16_t* src, size_t len) {
    std::array<uint16_t, FRAME_DATASIZE> result{};
    if (!src || len == 0) return result;
    
    // Pack 16 bits at once to optimize
    for (size_t i = 0; i < len && i < (FRAME_DATASIZE * 16); i += 16) {
        uint16_t packed = 0;
        for (size_t j = 0; j < 16 && (i + j) < len; j++) {
            if (src[i + j] != 0) {
                packed |= (1u << j);
            }
        }
        result[i / 16] = packed;
    }
    return result;
}

// ===================================================================================
// MODBUS FRAME IMPLEMENTATION
// ===================================================================================

/* @brief Clear the data of the frame.
    * @note This function clears the data of the frame to its default state.
    */
inline void Frame::clearData() {
    data.fill(0);
}

/* @brief Get a register value from frame data.
    * @param index The index of the register to get.
    * @note Returns 0 if the index is out of bounds. Check size first with regCount.
    * @return The register at the given index.
    */
inline uint16_t Modbus::Frame::getRegister(size_t index) const {
    return (index < regCount && index < FRAME_DATASIZE) ? data[index] : 0;
}

/* @brief Get all registers from frame data.
    * @note Returns an empty vector if the frame has no registers.
    * @return A vector of all registers.
    */
inline std::vector<uint16_t> Modbus::Frame::getRegisters() const {
    if (regCount == 0) return {};
    
    std::vector<uint16_t> result;
    size_t count = std::min((size_t)regCount, FRAME_DATASIZE);
    result.resize(count);
    memcpy(result.data(), data.data(), count * sizeof(uint16_t));
    return result;
}

/* @brief Get all registers from frame data.
    * @param dst The destination array to store the registers.
    * @param dstLen The length of the destination array.
    * @note Returns 0 if the frame has no registers or if the destination array is nullptr.
    * @note Data is converted to registers regardless of the function code.
    * @return The number of registers copied to the destination array.
    */
inline size_t Modbus::Frame::getRegisters(uint16_t* dst, size_t dstLen) const {
    if (regCount == 0 || dst == nullptr || dstLen == 0) return 0;
    
    size_t count = std::min(regCount, (uint16_t)dstLen);
    count = std::min(count, FRAME_DATASIZE);
    
    memcpy(dst, data.data(), count * sizeof(uint16_t));
    return count;
}

/* @brief Get a coil value from frame data.
    * @param index The index of the coil to get.
    * @note Returns false if the index is out of bounds. Check size first with regCount.
    * @return The coil at the given index.
    */
inline bool Modbus::Frame::getCoil(size_t index) const {
    if (index >= regCount || index >= FRAME_DATASIZE * 16) return false;
    size_t wordIdx = index / 16;
    size_t bitIdx = index % 16;
    return (data[wordIdx] >> bitIdx) & 1;
}

/* @brief Get all coils from frame data.
    * @note Returns an empty vector if the frame has no coils.
    * @return A vector of all coils.
    */
inline std::vector<bool> Modbus::Frame::getCoils() const {
    if (regCount == 0) return {};
    
    std::vector<bool> result;
    size_t count = std::min((size_t)regCount,(FRAME_DATASIZE * 16));
    result.reserve(count);
    
    for (size_t i = 0; i < count; i++) {
        size_t wordIdx = i / 16;
        size_t bitIdx = i % 16;
        result.push_back((data[wordIdx] >> bitIdx) & 1);
    }
    return result;
}

/* @brief Get all coils from frame data.
    * @param dst The destination array to store the coils.
    * @param dstLen The length of the destination array.
    * @note Returns 0 if the frame has no coils or if the destination array is nullptr.
    * @note Data is converted to coils regardless of the function code.
    * @return The number of coils copied to the destination array.
    */
inline size_t Modbus::Frame::getCoils(bool* dst, size_t dstLen) const {
    if (regCount == 0 || dst == nullptr || dstLen == 0) return 0;
    
    size_t count = std::min(regCount, (uint16_t)(dstLen));
    count = std::min(count, (size_t)(FRAME_DATASIZE * 16));
    
    for (size_t i = 0; i < count; i++) {
        size_t wordIdx = i / 16;
        size_t bitIdx = i % 16;
        dst[i] = (data[wordIdx] >> bitIdx) & 1;
    }
    return count;
}

/* @brief Set register data from a vector.
    * @param src The source vector to set the registers from.
    * @note Updates both data array and regCount.
    * @return true if successful, false if the source vector is too large.
    */
inline bool Modbus::Frame::setRegisters(const std::vector<uint16_t>& src) {
    if (src.size() > FRAME_DATASIZE) return false;
    
    data = packRegisters(src);
    regCount = std::min(src.size(), size_t(FRAME_DATASIZE));
    return true;
}

/* @brief Set register data from an initializer list.
    * @param src The source initializer list to set the registers from.
    * @note Updates both data array and regCount.
    * @return true if successful, false if the source list is too large.
    */
inline bool Modbus::Frame::setRegisters(const std::initializer_list<uint16_t>& src) {
    if (src.size() > FRAME_DATASIZE) return false;
    
    data = packRegisters(src);
    regCount = std::min(src.size(), size_t(FRAME_DATASIZE));
    return true;
}

/* @brief Set register data from a buffer.
    * @param src The source buffer to set the registers from.
    * @param len The length of the source buffer.
    * @note Updates both data array and regCount.
    * @return true if successful, false if the source buffer is invalid or too large.
    */
inline bool Modbus::Frame::setRegisters(const uint16_t* src, size_t len) {
    if (!src || len == 0 || len > FRAME_DATASIZE) return false;
    
    data = packRegisters(src, len);
    regCount = std::min(len, size_t(FRAME_DATASIZE));
    return true;
}

/* @brief Set coil data from a bool vector.
    * @param src The source vector to set the coils from.
    * @note Updates both data array and regCount.
    * @return true if successful, false if the source vector is too large.
    */
inline bool Modbus::Frame::setCoils(const std::vector<bool>& src) {
    if (src.size() > FRAME_DATASIZE * 16) return false;
    
    data = packCoils(src);
    regCount = src.size();
    return true;
}

/* @brief Set coil data from a uint16_t vector (0/non-zero -> bool).
    * @param src The source vector to set the coils from.
    * @note Updates both data array and regCount. Any non-zero value is considered true.
    * @return true if successful, false if the source vector is too large.
    */
inline bool Modbus::Frame::setCoils(const std::vector<uint16_t>& src) {
    if (src.size() > FRAME_DATASIZE * 16) return false;
    
    data = packCoils(src);
    regCount = src.size();
    return true;
}

/* @brief Set coil data from a bool initializer list.
    * @param src The source initializer list to set the coils from.
    * @note Updates both data array and regCount.
    * @return true if successful, false if the source list is too large.
    */
inline bool Modbus::Frame::setCoils(const std::initializer_list<bool>& src) {
    if (src.size() > FRAME_DATASIZE * 16) return false;
    
    data = packCoils(src);
    regCount = src.size();
    return true;
}

/* @brief Set coil data from a uint16_t initializer list (0/non-zero -> bool).
    * @param src The source initializer list to set the coils from.
    * @note Updates both data array and regCount. Any non-zero value is considered true.
    * @return true if successful, false if the source list is too large.
    */
inline bool Modbus::Frame::setCoils(const std::initializer_list<uint16_t>& src) {
    if (src.size() > FRAME_DATASIZE * 16) return false;
    
    data = packCoils(src);
    regCount = src.size();
    return true;
}

/* @brief Set coil data from a bool buffer.
    * @param src The source buffer to set the coils from.
    * @param len The length of the source buffer.
    * @note Updates both data array and regCount.
    * @return true if successful, false if the source buffer is invalid or too large.
    */
inline bool Modbus::Frame::setCoils(const bool* src, size_t len) {
    if (!src || len == 0 || len > FRAME_DATASIZE * 16) return false;
    
    data = packCoils(src, len);
    regCount = len;
    return true;
}

/* @brief Set coil data from a uint16_t buffer (0/non-zero -> bool).
    * @param src The source buffer to set the coils from.
    * @param len The length of the source buffer.
    * @note Updates both data array and regCount. Any non-zero value is considered true.
    * @return true if successful, false if the source buffer is invalid or too large.
    */
inline bool Modbus::Frame::setCoils(const uint16_t* src, size_t len) {
    if (!src || len == 0 || len > FRAME_DATASIZE * 16) return false;
    
    data = packCoils(src, len);
    regCount = len;
    return true;
}

/* @brief Set registers at specific position in frame data (for streaming operations)
    * @param src Source register values
    * @param len Number of registers to set
    * @param startRegIndex Starting register position in frame data (0-based)
    * @note Updates frame data at specified position, does NOT update regCount
    * @return true if successful, false if position is out of bounds
    */
inline bool Modbus::Frame::setRegisters(const uint16_t* src, size_t len, size_t startRegIndex) {
    if (!src || len == 0 || startRegIndex + len > FRAME_DATASIZE) return false;
    
    // Copy registers directly to specified position
    memcpy(&data[startRegIndex], src, len * sizeof(uint16_t));
    return true;
}

/* @brief Set coils at specific bit position in frame data (for streaming operations)
    * @param src Source coil values (vector<bool>)
    * @param startCoilIndex Starting bit position in frame data (0-based)
    * @note Updates frame data at specified bit position, does NOT update regCount
    * @return true if successful, false if position is out of bounds
    */
inline bool Modbus::Frame::setCoils(const std::vector<bool>& src, size_t startCoilIndex) {
    if (src.empty() || startCoilIndex + src.size() > FRAME_DATASIZE * 16) return false;
    
    // Set each coil bit individually at specified position
    for (size_t i = 0; i < src.size(); ++i) {
        size_t bitIndex = startCoilIndex + i;
        size_t wordIdx = bitIndex / 16;
        size_t bitPos = bitIndex % 16;
        
        if (src[i]) {
            data[wordIdx] |= (1u << bitPos);   // Set bit
        } else {
            data[wordIdx] &= ~(1u << bitPos);  // Clear bit
        }
    }
    return true;
}

/* @brief Set coils at specific bit position in frame data (for streaming operations)
    * @param src Source coil values (bool buffer)
    * @param len Number of coils to set
    * @param startCoilIndex Starting bit position in frame data (0-based)
    * @note Updates frame data at specified bit position, does NOT update regCount
    * @return true if successful, false if position is out of bounds
    */
inline bool Modbus::Frame::setCoils(const bool* src, size_t len, size_t startCoilIndex) {
    if (!src || len == 0 || startCoilIndex + len > FRAME_DATASIZE * 16) return false;
    
    // Set each coil bit individually at specified position
    for (size_t i = 0; i < len; ++i) {
        size_t bitIndex = startCoilIndex + i;
        size_t wordIdx = bitIndex / 16;
        size_t bitPos = bitIndex % 16;
        
        if (src[i]) {
            data[wordIdx] |= (1u << bitPos);   // Set bit
        } else {
            data[wordIdx] &= ~(1u << bitPos);  // Clear bit
        }
    }
    return true;
}





// ===================================================================================
// HELPER FUNCTIONS
// ===================================================================================

/* @brief Create an ILLEGAL_FUNCTION exception response from a request frame
    * @note Copies the original request frame passed as argument
    * @param request The initial request frame
    * @return The response frame with the exception
    */
inline Frame setIllegalFunction(const Frame& request) {
    return Frame {
        .type = RESPONSE,
        .fc = request.fc,
        .slaveId = request.slaveId,
        .regAddress = request.regAddress,
        .regCount = request.regCount,
        .data = {},
        .exceptionCode = ILLEGAL_FUNCTION
    };
}

/* @brief Create a SLAVE_DEVICE_BUSY exception response from a request frame
    * @note Copies the original request frame passed as argument
    * @param request The initial request frame
    * @return The response frame with the exception
    */
inline Frame setSlaveBusy(const Frame& request) {
    return Frame {
        .type = RESPONSE,
        .fc = request.fc,
        .slaveId = request.slaveId,
        .regAddress = request.regAddress,
        .regCount = request.regCount,
        .data = {},
        .exceptionCode = SLAVE_DEVICE_BUSY
    };
}

/* @brief Create a SLAVE_DEVICE_FAILURE exception response from a request frame
    * @note Copies the original request frame passed as argument
    * @param request The initial request frame
    * @return The response frame with the exception
    */
inline Frame setSlaveDeviceFailure(const Frame& request) {
    return Frame {
        .type = RESPONSE,
        .fc = request.fc,
        .slaveId = request.slaveId,
        .regAddress = request.regAddress,
        .regCount = request.regCount,
        .data = {},
        .exceptionCode = SLAVE_DEVICE_FAILURE
    };
}


} // namespace Modbus