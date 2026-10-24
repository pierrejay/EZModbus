#pragma once

#include <core/ModbusCore.h>
#include <core/ModbusCodec.hpp>
#include <interfaces/ModbusInterface.hpp>
#include <utils/ModbusDebug.hpp>
#include <core/ModbusTypes.hpp>
#include <functional>
#include <memory>
#include <queue>

namespace ModbusInterface {

class Dummy : public IInterface {
public:

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    Dummy() { _role = Modbus::SLAVE; };
    virtual ~Dummy() = default;

    Result begin() override { return Result::SUCCESS; }
    
    // Method from the abstract class
    Result sendFrame(const Modbus::Frame& frame, TxResultCallback txCallback, void* ctx) override { 
        Modbus::Debug::LOG_FRAME(frame, "Received frame from bridge");
        rcvFrames.push(frame);
        if (txCallback) txCallback(Result::SUCCESS, ctx);
        return Result::SUCCESS; 
    }
    
    // Method to inject a frame in the bridge
    void transmitFrame(const Modbus::Frame& frame) { 
        Modbus::Debug::LOG_FRAME(frame, "Transmitting frame to bridge"); 
        notifyCallbacks(frame); 
    }
    
    bool isReady() override { return true; }

    // Number of receive callbacks registered by applications
    size_t callbackCount() const {
        size_t n = 0;
        for (const auto& cb : _rcvCallbacks) if (cb.fn) n++;
        return n;
    }

    // Method to fetch the last received frame
    bool getReceivedFrame(Modbus::Frame& frame) {
        if (rcvFrames.empty()) {
            return false;
        }
        frame = rcvFrames.front();
        rcvFrames.pop();
        return true;
    }

private:
    std::queue<Modbus::Frame> rcvFrames; // Queue to store received frames

};

} // namespace ModbusInterface 