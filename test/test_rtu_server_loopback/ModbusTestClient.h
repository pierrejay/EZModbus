#pragma once

#include "ModbusTestAgent.h"
#include "EZModbus.h"
#include <string>

// ModbusTestClient allows to emulate a Modbus RTU client using
// EZModbus RTU codec & the ModbusTestAgent class for transport.
// The original tests used ArduinoModbus (Freemodbus), this class
// keeps the same API while using the tested EZModbus codec to
// encode/decode Modbus frames acting as a client.

class ModbusTestClient {
public:

    ModbusTestClient(ModbusTestAgent& agent) : _agent(agent), _lastError(""), _rxBuffer(_rxBuf, sizeof(_rxBuf)) {}

    // Single register operations
    int holdingRegisterRead(uint8_t slaveId, uint16_t address) {
        // Create the request frame
        Modbus::Frame request {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_HOLDING_REGISTERS,
            .slaveId = slaveId,
            .regAddress = address,
            .regCount = 1,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(request, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to encode request";
            return -1;
        }

        // Send the frame
        _agent.send(requestBytes);

        // Wait and read the response
        if (!waitForResponse()) {
            // //_lastError = "No response received";
            return -1;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to decode response";
            return -1;
        }

        // Check if it's an exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            _lastError.clear();
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = toupper(s[0]);
            _lastError = s;
            return -1;
        }

        // Return the value
        if (response.regCount < 1) return -1;
        return response.getRegister(0);
    }

    int inputRegisterRead(uint8_t slaveId, uint16_t address) {
        // Create the request frame
        Modbus::Frame request {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_INPUT_REGISTERS,
            .slaveId = slaveId,
            .regAddress = address,
            .regCount = 1,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(request, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to encode request";
            return -1;
        }

        // Send the frame
        _agent.send(requestBytes);

        // Wait and read the response
        if (!waitForResponse()) {
            // //_lastError = "No response received";
            return -1;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to decode response";
            return -1;
        }

        // Check if it's an exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            _lastError.clear();
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = toupper(s[0]);
            _lastError = s;
            return -1;
        }

        // Return the value
        if (response.regCount < 1) {
            _lastError = "Illegal data address";
            return -1;
        }
        return response.getRegister(0);
    }

    int coilRead(uint8_t slaveId, uint16_t address) {
        // Create the request frame
        Modbus::Frame request {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_COILS,
            .slaveId = slaveId,
            .regAddress = address,
            .regCount = 1,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(request, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to encode request";
            return -1;
        }

        // Send the frame
        _agent.send(requestBytes);

        // Wait and read the response
        if (!waitForResponse()) {
            // //_lastError = "No response received";
            return -1;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to decode response";
            return -1;
        }

        // Check if it's an exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            _lastError.clear();
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = toupper(s[0]);
            _lastError = s;
            return -1;
        }

        // Return the value
        if (response.regCount < 1) {
            // //_lastError = "No data in response";
            return -1;
        }
        return response.getCoil(0);
    }

    int discreteInputRead(uint8_t slaveId, uint16_t address) {
        // Create the request frame
        Modbus::Frame request {
            .type = Modbus::REQUEST,
            .fc = Modbus::READ_DISCRETE_INPUTS,
            .slaveId = slaveId,
            .regAddress = address,
            .regCount = 1,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(request, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to encode request";
            return -1;
        }

        // Send the frame
        _agent.send(requestBytes);

        // Wait and read the response
        if (!waitForResponse()) {
            // //_lastError = "No response received";
            return -1;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            // //_lastError = "Failed to decode response";
            return -1;
        }

        // Check if it's an exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            _lastError.clear();
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = toupper(s[0]);
            _lastError = s;
            return -1;
        }

        // Return the value
        if (response.regCount < 1) {
            // //_lastError = "No data in response";
            return -1;
        }
        return response.getCoil(0);
    }

    int holdingRegisterWrite(uint8_t slaveId, uint16_t address, uint16_t value) {
        // Create the request frame
        Modbus::Frame request {
            .type = Modbus::REQUEST,
            .fc = Modbus::WRITE_REGISTER,
            .slaveId = slaveId,
            .regAddress = address,
            .regCount = 1,
            .data = Modbus::packRegisters({value}),
            .exceptionCode = Modbus::NULL_EXCEPTION
        };

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(request, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            //_lastError = "Failed to encode request";
            return 0;
        }

        // Send the frame
        _agent.send(requestBytes);

        // If it's a broadcast, we don't need to wait for a response
        if (Modbus::isBroadcastId(slaveId)) {
            return 1;
        }

        // Wait and read the response
        if (!waitForResponse()) {
            //_lastError = "No response received";
            return 0;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            //_lastError = "Failed to decode response";
            return 0;
        }

        // Check if it's an exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            _lastError.clear();
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = toupper(s[0]);
            _lastError = s;
            return 0;
        }

        return 1;
    }

    int coilWrite(uint8_t slaveId, uint16_t address, uint16_t value) {
        // Create the request frame
        Modbus::Frame request {
            .type = Modbus::REQUEST,
            .fc = Modbus::WRITE_COIL,
            .slaveId = slaveId,
            .regAddress = address,
            .regCount = 1,
            .data = Modbus::packCoils({ value?true:false }),
            .exceptionCode = Modbus::NULL_EXCEPTION
        };

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(request, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            //_lastError = "Failed to encode request";
            return 0;
        }

        // Send the frame
        _agent.send(requestBytes);

        // If it's a broadcast, we don't need to wait for a response
        if (Modbus::isBroadcastId(slaveId)) {
            return 1;
        }

        // Wait and read the response
        if (!waitForResponse()) {
            //_lastError = "No response received";
            return 0;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            //_lastError = "Failed to decode response";
            return 0;
        }

        // Check if it's an exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            _lastError.clear();
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = toupper(s[0]);
            _lastError = s;
            return 0;
        }

        return 1;
    }

    // Multiple register operations
    int requestFrom(uint8_t slaveId, int type, uint16_t address, uint16_t quantity) {
        // Convert the original ArduinoModbus "type" to the Modbus::FunctionCode
        Modbus::FunctionCode fc;
        switch (type) {
            case 0: fc = Modbus::READ_COILS; break;
            case 1: fc = Modbus::READ_DISCRETE_INPUTS; break;
            case 3: fc = Modbus::READ_HOLDING_REGISTERS; break;
            case 4: fc = Modbus::READ_INPUT_REGISTERS; break;
            default:
                //_lastError = "Invalid register type";
                return 0;
        }

        // Create the request frame
        Modbus::Frame request {
            .type = Modbus::REQUEST,
            .fc = fc,
            .slaveId = slaveId,
            .regAddress = address,
            .regCount = quantity,
            .data = {},
            .exceptionCode = Modbus::NULL_EXCEPTION
        };

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(request, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            //_lastError = "Failed to encode request";
            return 0;
        }

        // Send the frame
        _agent.send(requestBytes);

        // Always reset the error and the old buffer
        _lastError.clear();
        _rxData.clear();
        _rxDataIndex = 0;

        // Wait and read the response
        if (!waitForResponse()) {
            _lastError = "No response received";
            return 0;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            _lastError = "Failed to decode response";
            return 0;
        }

        // If it's a Modbus exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            // Store the code (and we could do an ucfirst if we want)
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
            _lastError = s;
            return 0;
        }

            // OK, on transforme en vector<uint16_t>
            std::vector<uint16_t> vec;
            if (fc == Modbus::READ_COILS || fc == Modbus::READ_DISCRETE_INPUTS) {

                auto bits = response.getCoils();
                if (bits.size() > quantity) {
                    bits.resize(quantity);
                }

                vec.reserve(bits.size());
                for (bool b : bits) {
                    vec.push_back(b ? 1u : 0u);
                }
            }
            else {
                // récupère un vector<uint16_t>
                vec = response.getRegisters();
            }

            _rxData = std::move(vec);
            _rxDataIndex = 0;
            return static_cast<int>(_rxData.size());
    }

    int beginTransmission(uint8_t slaveId, int type, uint16_t address, uint16_t quantity) {
        // Convert the original ArduinoModbus "type" to the Modbus::FunctionCode
        switch (type) {
            case 0: _txFrame.fc = Modbus::WRITE_MULTIPLE_COILS; break;
            case 3: _txFrame.fc = Modbus::WRITE_MULTIPLE_REGISTERS; break;
            default:
                //_lastError = "Invalid register type";
                return 0;
        }

        // Initialize the frame
        _txFrame.type = Modbus::REQUEST;
        _txFrame.slaveId = slaveId;
        _txFrame.regAddress = address;
        _txFrame.regCount = quantity;
        _txFrame.clearData(false);
        _txFrame.exceptionCode = Modbus::NULL_EXCEPTION;
        _txValues.clear();

        return 1;
    }

    int write(uint16_t value) {
        _txValues.push_back(value);
        return 1;
    }

    int endTransmission() {
        // Check that the number of values corresponds to the regCount
        if (_txValues.size() != _txFrame.regCount) {
            //_lastError = "Invalid number of values";
            #ifdef EZMODBUS_DEBUG
                Modbus::Debug::LOG_MSGF("Invalid number of values: %d != %d\n", _txValues.size(), _txFrame.regCount);
            #endif
            return 0;
        }

        _txFrame.data = (_txFrame.fc == Modbus::WRITE_MULTIPLE_COILS)
                        ? Modbus::packCoils(_txValues)
                        : Modbus::packRegisters(_txValues);

        // Encode the frame
        uint8_t _raw[256];
        ByteBuffer requestBytes(_raw, sizeof(_raw));
        auto encodeResult = ModbusCodec::RTU::encode(_txFrame, requestBytes);
        if (encodeResult != ModbusCodec::SUCCESS) {
            //_lastError = "Failed to encode request";
            #ifdef EZMODBUS_DEBUG
                Modbus::Debug::LOG_MSGF("Failed to encode request: %s\n", ModbusCodec::toString(encodeResult));
            #endif
            return 0;
        }

        // Send the frame
        _agent.send(requestBytes);

        // If it's a broadcast, we don't need to wait for a response
        if (Modbus::isBroadcastId(_txFrame.slaveId)) {
            return 1;
        }

        // Wait and read the response
        if (!waitForResponse()) {
            //_lastError = "No response received";
            #ifdef EZMODBUS_DEBUG
                Modbus::Debug::LOG_MSGF("No response received");
            #endif
            return 0;
        }

        // Decode the response
        Modbus::Frame response;
        auto decodeResult = ModbusCodec::RTU::decode(_rxBuffer, response, Modbus::RESPONSE);
        if (decodeResult != ModbusCodec::SUCCESS) {
            //_lastError = "Failed to decode response";
            #ifdef EZMODBUS_DEBUG
                Modbus::Debug::LOG_MSGF("Failed to decode response");
            #endif
            return 0;
        }

        // Check if it's an exception
        if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
            _lastError.clear();
            std::string s = Modbus::toString(response.exceptionCode);
            if (!s.empty()) s[0] = toupper(s[0]);
            _lastError = s;
            #ifdef EZMODBUS_DEBUG
                Modbus::Debug::LOG_MSGF("Response is an exception");
            #endif
            return 0;
        }

        return 1;
    }

    // Buffer management
    int available() {
        return _rxData.size() - _rxDataIndex;
    }

    uint16_t read() {
        if (_rxDataIndex >= _rxData.size()) {
            return 0;
        }
        return _rxData[_rxDataIndex++];
    }

    const char* lastError() {
        return _lastError.c_str();
    }

private:
    ModbusTestAgent& _agent;
    std::string _lastError;
    uint8_t _rxBuf[256];
    ByteBuffer _rxBuffer;
    std::vector<uint16_t> _rxData;
    size_t _rxDataIndex;
    Modbus::Frame _txFrame;
    std::vector<uint16_t> _txValues;

    bool waitForResponse(uint32_t timeoutMs = 1000) {
        uint32_t startTime = millis();
        while (millis() - startTime < timeoutMs) {
            _agent.poll();
            if (_agent.hasData()) {
                Bytes bytes = _agent.fetch();
                _rxBuffer.clear();
                _rxBuffer.write_at(0, bytes.data(), bytes.size());
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return false;
    }
}; 