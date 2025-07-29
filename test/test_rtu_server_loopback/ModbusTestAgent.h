#pragma once

#include <vector>
#include <queue>
#include <cstring>
#include <Arduino.h>
#include "EZModbus.h"

// ModbusTestAgent sends & receive data to/from serial UART port.
// It is used to test the Modbus library by emulating a Modbus RTU client
// when plugged to the ModbusTestClient class, handling reading and writing
// to the test serial port.

using Bytes = std::vector<uint8_t>;
using namespace ModbusTypeDef;

class ModbusTestAgent {
public:
    ModbusTestAgent(HardwareSerial& serial, uint32_t silenceTimeUs = 20000, Stream* logStream = nullptr) 
        : _serial(serial), _silenceTimeUs(silenceTimeUs), _lastRxTimeUs(0), _logStream(logStream) {
        _rxBuffer.reserve(256);
        flushRxBuffer();
    }

    // Push data to be "received" by the Modbus library
    void send(const ByteBuffer& data) {
        // Send data to the UART that will be read by the Modbus library
        _serial.write(data.data(), data.size());
        _serial.flush();
        
        #ifdef EZMODBUS_DEBUG
            Bytes tempBytesForLog(data.data(), data.data() + data.size());
            String hexString = toHexStr(tempBytesForLog);
            Modbus::Debug::LOG_MSGF("Pushed data: %s\n", hexString.c_str());
        #endif
    }

    // Push hexadecimal string (easier for test writing)
    void send(const char* hexStr) {
        Bytes data = toBytes(hexStr);
        ByteBuffer buffer(data.data(), data.size());
        send(buffer);
    }

    // Process received data from the Modbus library
    void poll() {
        // Read available data from UART
        while (_serial.available()) {
            uint8_t byte = _serial.read();
            _rxBuffer.push_back(byte);
            _lastRxTimeUs = micros();
        }

        // Check if we need to finalize a frame after silence
        if (!_rxBuffer.empty() && (micros() - _lastRxTimeUs >= _silenceTimeUs)) {
            #ifdef EZMODBUS_DEBUG
                Modbus::Debug::LOG_MSGF("Received frame: %s", toHexStr(_rxBuffer).c_str());
            #endif
            _rxQueue.push(_rxBuffer);
            _rxBuffer.clear();
        }
    }

    // Get the next frame received from the Modbus library
    Bytes fetch() {
        if (_rxQueue.empty()) {
            return {};  // Return empty vector if queue is empty
        }
        Bytes data = _rxQueue.front();
        _rxQueue.pop();

        #ifdef EZMODBUS_DEBUG
        if (_logStream) {
            _logStream->print("Popped data: ");
            _logStream->println(toHexStr(data).c_str());
        }
        #endif
        
        return data;
    }

    // Helper function to peek at next frame without removing it
    Bytes peek() const {
        if (_rxQueue.empty()) {
            return {};
        }
        return _rxQueue.front();
    }

    // Clear the queue of received data
    void clear() {
        if (_rxQueue.empty()) {
            return;
        }
        while (!_rxQueue.empty()) {
            _rxQueue.pop();
        }
    }

    // Helper function to check if queue has data
    bool hasData() const {
        return !_rxQueue.empty();
    }

    // Static helper to compare bytes with hex string
    static bool compare(const Bytes& bytes, const char* hexStr) {
        Bytes expected = toBytes(hexStr);
        
        if (expected.size() != bytes.size()) {
            return false;
        }
        
        for (size_t i = 0; i < expected.size(); i++) {
            if (expected[i] != bytes[i]) {
                return false;
            }
        }
        return true;
    }
    
    // Convert bytes to hex string for debugging
    static String toHexStr(const Bytes& bytes) {
        String result;
        result.reserve(bytes.size() * 3); // Each byte = 2 hex chars + 1 space
        
        for (size_t i = 0; i < bytes.size(); i++) {
            char hexByte[4];
            snprintf(hexByte, sizeof(hexByte), "%02X ", bytes[i]);
            result += hexByte;
        }
        
        // Remove trailing space if any
        if (result.length() > 0) {
            result.remove(result.length() - 1);
        }
        
        return result;
    }


    // Convert hex string to bytes
    static Bytes toBytes(const char* hexStr) {
        Bytes bytes;
        size_t len = strlen(hexStr);
        
        for (size_t i = 0; i < len; i += 2) {
            // Skip spaces and other separators
            if (hexStr[i] == ' ' || hexStr[i] == ':' || hexStr[i] == '-') {
                i--;
                continue;
            }
            
            if (i + 1 >= len) break; // Avoid odd number of chars
            
            char byteStr[3] = { hexStr[i], hexStr[i+1], 0 };
            bytes.push_back(strtol(byteStr, nullptr, 16));
        }
        
        return bytes;
    }

private:
    HardwareSerial& _serial;
    uint32_t _silenceTimeUs;
    uint32_t _lastRxTimeUs;
    Bytes _rxBuffer;
    std::queue<Bytes> _rxQueue;
    Stream* _logStream;

    void flushRxBuffer() {
        while (_serial.available()) {
            _serial.read();
        }
    }
};