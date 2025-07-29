/**
 * @file ModbusDMX.h
 * @brief Example using EZModbus for a DMX application
 * @brief (demonstration of Modbus DMX usage)
 * @note Depends on the EZDMX library (https://github.com/pierrejay/esp32-EZDMX)
 */

#pragma once

#include "interfaces/ModbusInterface.hpp"
#include "core/ModbusCore.h"
#include <EZDMX.h>
#include <vector>
#include <map>

/**
 * @brief ModbusDMX Interface transforming Modbus holding register requests
 *        into DMX master commands. Inherits ModbusInterface::IInterface, and
 *        relies on EZDMX library for DMX communication.
 *
 * Mapping:
 *  - Modbus registers [1..512] ↔ DMX channels [1..512]
 *  - Register value [0..65535] scaled to DMX [0..255] on write
 *  - DMX value [0..255] scaled to register [0..65535] on read
 */
class ModbusDMX : public ModbusInterface::IInterface {
public:
    using Result = IInterface::Result;

    /**
     * @brief Construct with Arduino Serial port and pins for DMX master
     */
    ModbusDMX(Stream* serial = &Serial1,
              int txPin = 17,
              int rxPin = 16,
              int dePin = -1)
        : _dmx(EZDMX::Mode::MASTER, serial, txPin, rxPin, dePin) {
        _role = Modbus::MASTER;
    }

    /**
     * @brief Initialize DMX hardware and start transmission
     */
    Result begin() override {
        auto r = _dmx.begin();
        if (r != EZDMX::SUCCESS) {
            return Error(ERR_CONNECTION_FAILED, EZDMX::toString(r.status));
        }
        r = _dmx.start();
        if (r != EZDMX::SUCCESS) {
            return Error(ERR_CONNECTION_FAILED, EZDMX::toString(r.status));
        }
        return Success();
    }

    /**
     * @brief Process incoming Modbus request frames
     */
    Result sendFrame(const Modbus::Frame& frame, TaskHandle_t notifyTask = nullptr) override {
        // Only handle requests
        if (frame.type != Modbus::REQUEST) {
            return Success();
        }
        bool isBroadcast = Modbus::isBroadcastId(frame.slaveId);

        // Validate address range
        if (frame.regAddress < 1 || frame.regAddress > EZDMX::DMX_UNIVERSE_SIZE) {
            return Error(ERR_INVALID_FRAME, "illegal data address");
        }

        switch (frame.fc) {
            case Modbus::WRITE_SINGLE_REGISTER: {
                if (frame.data.size() < 1) {
                    return Error(ERR_INVALID_FRAME, "missing register value");
                }
                uint16_t reg = frame.regAddress;
                uint16_t regVal = frame.data[0];
                uint8_t dmxVal = scaleToDMX(regVal);
                auto r = _dmx.set(reg, dmxVal);
                if (r != EZDMX::SUCCESS) {
                    return Error(ERR_SEND_FAILED, EZDMX::toString(r.status));
                }
                if (!isBroadcast) {
                    Modbus::Frame resp{ Modbus::RESPONSE, frame.slaveId, frame.fc,
                                        reg, 1, { regVal }, Modbus::NULL_EXCEPTION };
                    notifyCallbacks(resp);
                }
                break;
            }
            case Modbus::WRITE_MULTIPLE_REGISTERS: {
                if (frame.data.size() < frame.regCount) {
                    return Error(ERR_INVALID_FRAME, "data length mismatch");
                }
                uint16_t start = frame.regAddress;
                std::map<uint16_t,uint8_t> m;
                for (uint16_t i = 0; i < frame.regCount; ++i) {
                    m[start + i] = scaleToDMX(frame.data[i]);
                }
                auto r = _dmx.set(m);
                if (r != EZDMX::SUCCESS) {
                    return Error(ERR_SEND_FAILED, EZDMX::toString(r.status));
                }
                if (!isBroadcast) {
                    Modbus::Frame resp{ Modbus::RESPONSE, frame.slaveId, frame.fc,
                                        start, frame.regCount, {}, Modbus::NULL_EXCEPTION };
                    notifyCallbacks(resp);
                }
                break;
            }
            case Modbus::READ_HOLDING_REGISTERS: {
                if (frame.regCount < 1 || frame.regAddress + frame.regCount - 1 > EZDMX::DMX_UNIVERSE_SIZE) {
                    return Error(ERR_INVALID_FRAME, "illegal data address/count");
                }
                std::vector<uint16_t> vals;
                vals.reserve(frame.regCount);
                for (uint16_t i = 0; i < frame.regCount; ++i) {
                    uint8_t dv;
                    _dmx.get(frame.regAddress + i, dv);
                    vals.push_back(scaleToModbus(dv));
                }
                Modbus::Frame resp{ Modbus::RESPONSE, frame.slaveId, frame.fc,
                                    frame.regAddress, frame.regCount, vals, Modbus::NULL_EXCEPTION };
                notifyCallbacks(resp);
                break;
            }
            default:
                return Error(ERR_INVALID_MSG_TYPE);
        }
        return Success();
    }

    bool isReady() override {
        return true;
    }

private:
    EZDMX _dmx;

    static uint8_t scaleToDMX(uint16_t v) {
        // round nearest
        return uint8_t((uint32_t(v) * 255 + 32767) / 65535);
    }

    static uint16_t scaleToModbus(uint8_t v) {
        return uint16_t((uint32_t(v) * 65535 + 127) / 255);
    }
};
