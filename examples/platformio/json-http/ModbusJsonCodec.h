/**
 * @file ModbusJsonCodec.h
 * @brief Add-on to ModbusCodec.hpp implementing JSON encoding/decoding
 * @note Depends on the ArduinoJson library
 */

#pragma once

// Global
#include <core/ModbusCodec.hpp>
#include <ArduinoJson.h>

namespace ModbusCodec {

class JSON {

public:

    static Result decode(const JsonObject& json, Modbus::Frame& frame) {
        Modbus::Frame tmpFrame;

        // Validate JSON format (fields type & sizes)
        Result formatResult = validateJsonFormat(json);
        if (formatResult != SUCCESS) return formatResult;

        // Validate type
        String typeStr = json["type"].as<String>();
        if (typeStr == "request") tmpFrame.type = Modbus::REQUEST;
        else if (typeStr == "response") tmpFrame.type = Modbus::RESPONSE;
        else return ERR_INVALID_TYPE;

        // Copy JSON fields to temporary frame
        tmpFrame.fc = (Modbus::FunctionCode)(json["fc"].as<uint8_t>());
        tmpFrame.slaveId = json["slaveId"].as<uint8_t>();
        tmpFrame.regAddress = json["regAddress"].as<uint16_t>() | 0;
        tmpFrame.regCount = json["regCount"].as<uint16_t>() | 0;
        tmpFrame.data.clear();
        if (json["data"]) {
            JsonArray data = json["data"];
            for (JsonVariant v : data) {
                tmpFrame.data.push_back(v.as<uint16_t>());
            }
        }
        tmpFrame.exceptionCode = (Modbus::ExceptionCode)(json["exceptionCode"].as<uint8_t>() | 0);

        // Validate temporary frame
        Result frameResult = isValidFrame(tmpFrame);
        if (frameResult != SUCCESS) return frameResult;

        // Convert data coils to 0/1
        if (tmpFrame.fc == Modbus::READ_COILS 
            || tmpFrame.fc == Modbus::READ_DISCRETE_INPUTS
            || tmpFrame.fc == Modbus::WRITE_COIL
            || tmpFrame.fc == Modbus::WRITE_MULTIPLE_COILS) {
            for (size_t i = 0; i < tmpFrame.data.size(); i++) {
                tmpFrame.data[i] = tmpFrame.data[i] ? 1 : 0;
            }
        }

        // Copy temporary frame to output frame
        frame = tmpFrame;
        return SUCCESS;
    }

    static Result encode(const Modbus::Frame& frame, JsonObject& json) {

        // Validate frame
        Result frameResult = isValidFrame(frame);
        if (frameResult != SUCCESS) return frameResult;
        
        // Encode JSON fields
        json["type"] = frame.type == Modbus::REQUEST ? "request" : "response";
        json["fc"] = (uint8_t)frame.fc;
        json["slaveId"] = frame.slaveId;
        json["regAddress"] = frame.regAddress;
        json["regCount"] = frame.regCount;
        json.createNestedArray("data");
        auto data = json["data"];
        for (auto value : frame.data) {
            data.add(value);
        }
        json["exceptionCode"] = (uint8_t)frame.exceptionCode;

        // Encode data coils to 0/1
        if (frame.fc == Modbus::READ_COILS 
            || frame.fc == Modbus::READ_DISCRETE_INPUTS
            || frame.fc == Modbus::WRITE_COIL
            || frame.fc == Modbus::WRITE_MULTIPLE_COILS) {
            for (size_t i = 0; i < json["data"].size(); i++) {
                json["data"][i] = json["data"][i] ? 1 : 0;
            }
        }

        return SUCCESS;
    }

private:

    static Result validateJsonFormat(const JsonObject& json) {
        // Required fields
        if (!json["type"].is<String>()) return ERR_INVALID_TYPE;
        if (!json["fc"].is<uint8_t>()) return ERR_INVALID_FC;
        if (!json["slaveId"].is<uint8_t>()) return ERR_INVALID_SLAVEID;
        // Optional fields
        if (json["regAddress"].as<uint32_t>() > UINT16_MAX) return ERR_INVALID_REG_ADDR;
        if (json["regCount"].as<uint32_t>() > UINT16_MAX) return ERR_INVALID_REG_COUNT;
        if (json["exceptionCode"].as<uint32_t>() > UINT8_MAX) return ERR_INVALID_EXCEPTION;
        JsonArray data = json["data"];
        for (JsonVariant v : data) {
            if (v.as<uint32_t>() > UINT16_MAX) return ERR_INVALID_DATA;
        }
        return SUCCESS;
    }

}; // class JSON

} // namespace ModbusCodec