#pragma once

#include <atomic>
#include <vector>
#include <unity.h>
#include "EZModbus.h"
#include <utils/ModbusLogger.hpp>

// Give some time for the application logs to be printed before asserting
#ifdef EZMODBUS_DEBUG
    #define TEST_ASSERT_START() { Modbus::Logger::waitQueueFlushed(); }
#else
    #define TEST_ASSERT_START() { vTaskDelay(pdMS_TO_TICKS(50)); }
#endif

// ===================================================================================
// WORD API HELPERS (New)
// ===================================================================================

// Pour simplifier les noms
using ServerWord = Modbus::Word;
using RegType = Modbus::RegisterType;

// Single-register Word using direct pointer access (like old Register)
struct DummyWord1Reg {
    volatile uint16_t value;
    ServerWord word;
    
    // Compatibility structure to emulate old Register API
    struct {
        uint16_t address;
        RegType type;
        std::function<uint16_t(const ServerWord&)> readCb;
        std::function<bool(uint16_t, const ServerWord&)> writeCb;
    } reg;

    DummyWord1Reg(RegType RT, uint16_t addr, bool useDirectPointer = true) {
        // Define the value as address of the register (or 1 if it's a coil or discrete input)
        switch (RT) {
            case Modbus::DISCRETE_INPUT:
            case Modbus::COIL:
                value = 1;
                break;
            case Modbus::HOLDING_REGISTER:
            case Modbus::INPUT_REGISTER:
                value = addr;
                break;
        }
        
        word.type = RT;
        word.startAddr = addr;
        word.nbRegs = 1;
        
        if (useDirectPointer) {
            // Use direct pointer access for performance
            word.value = &value;
            word.readHandler = nullptr;
            word.writeHandler = nullptr;
        } else {
            // Use handlers (like old callback system)
            word.value = nullptr;
            word.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
                auto* self = static_cast<DummyWord1Reg*>(userCtx);
                outVals[0] = static_cast<uint16_t>(self->value);
                return Modbus::NULL_EXCEPTION;
            };
            word.userCtx = (void*)this;

            if (RT == Modbus::HOLDING_REGISTER || RT == Modbus::COIL) {
                word.writeHandler = [](const uint16_t* writeVals, const ServerWord& w, void* userCtx) -> Modbus::ExceptionCode {
                    auto* self = static_cast<DummyWord1Reg*>(userCtx);
                    self->value = writeVals[0];
                    return Modbus::NULL_EXCEPTION;
                };
            } else {
                word.writeHandler = nullptr;
            }
            word.userCtx = (void*)this;
        }
        
        // Initialize compatibility structure
        reg.address = addr;
        reg.type = RT;
        reg.readCb = [this](const auto& r) -> uint16_t { 
            return static_cast<uint16_t>(value); 
        };
        
        if (RT == Modbus::HOLDING_REGISTER || RT == Modbus::COIL) {
            reg.writeCb = [this](uint16_t v, const auto& w) -> bool {
                value = v;
                return true;
            };
        } else {
            reg.writeCb = nullptr;
        }
    }
};

// Generic Word helper (like DummyWord from test_helpers_current.h)
struct DummyWord {
    std::vector<uint16_t> values;
    ServerWord word;

    DummyWord(RegType RT, uint16_t startAddr, uint16_t nbRegs, uint16_t baseValue = 0) {
        values.resize(nbRegs);
        
        // Initialize values (for testing purposes)
        for (uint16_t i = 0; i < nbRegs; ++i) {
            switch (RT) {
                case Modbus::DISCRETE_INPUT:
                case Modbus::COIL:
                    values[i] = (i % 2); // Alternating 0/1 pattern
                    break;
                case Modbus::HOLDING_REGISTER:
                case Modbus::INPUT_REGISTER:
                    values[i] = baseValue + i;
                    break;
            }
        }
        
        word.type = RT;
        word.startAddr = startAddr;
        word.nbRegs = nbRegs;
        word.value = nullptr;  // Use handlers by default
        
        // Read handler
        word.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            auto* self = static_cast<DummyWord*>(userCtx);
            for (uint16_t i = 0; i < w.nbRegs; ++i) {
                outVals[i] = self->values[i];
            }
            return Modbus::NULL_EXCEPTION;
        };
        word.userCtx = (void*)this;
        
        // Write handler (only for writable types)
        if (RT == Modbus::HOLDING_REGISTER || RT == Modbus::COIL) {
            word.writeHandler = [](const uint16_t* writeVals, const ServerWord& w, void* userCtx) -> Modbus::ExceptionCode {
                auto* self = static_cast<DummyWord*>(userCtx);
                for (uint16_t i = 0; i < w.nbRegs; ++i) {
                    self->values[i] = writeVals[i];
                }
                return Modbus::NULL_EXCEPTION;
            };
            word.userCtx = (void*)this;
        } else {
            word.writeHandler = nullptr;
        }
    }
};

// Multi-register Word suite (must use handlers)
struct DummyWordMultiReg {
    std::vector<uint16_t> values;
    ServerWord word;

    DummyWordMultiReg(RegType RT, uint16_t startAddr, uint16_t nbRegs, uint16_t baseValue = 0) {
        values.resize(nbRegs);
        
        // Initialize values (for testing purposes)
        for (uint16_t i = 0; i < nbRegs; ++i) {
            switch (RT) {
                case Modbus::DISCRETE_INPUT:
                case Modbus::COIL:
                    values[i] = (i % 2); // Alternating 0/1 pattern
                    break;
                case Modbus::HOLDING_REGISTER:
                case Modbus::INPUT_REGISTER:
                    values[i] = baseValue + i;
                    break;
            }
        }
        
        word.type = RT;
        word.startAddr = startAddr;
        word.nbRegs = nbRegs;
        word.value = nullptr;  // Multi-reg cannot use direct pointer
        
        // Read handler
        word.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            auto* self = static_cast<DummyWordMultiReg*>(userCtx);
            for (uint16_t i = 0; i < w.nbRegs; ++i) {
                outVals[i] = self->values[i];
            }
            return Modbus::NULL_EXCEPTION;
        };
        word.userCtx = (void*)this;
        
        // Write handler (only for writable types)
        if (RT == Modbus::HOLDING_REGISTER || RT == Modbus::COIL) {
            word.writeHandler = [](const uint16_t* writeVals, const ServerWord& w, void* userCtx) -> Modbus::ExceptionCode {
                auto* self = static_cast<DummyWordMultiReg*>(userCtx);
                for (uint16_t i = 0; i < w.nbRegs; ++i) {
                    self->values[i] = writeVals[i];
                }
                return Modbus::NULL_EXCEPTION;
            };
            word.userCtx = (void*)this;
        } else {
            word.writeHandler = nullptr;
        }
    }
};

// Word suite for testing multiple single-register Words (Register migration)
template<RegType RT>
struct DummyWordSuite1Reg {
    std::vector<DummyWord1Reg> storage;      // garde en vie tous les Words
    std::vector<ServerWord>    handle;       // copie des Words pour addWords()

    DummyWordSuite1Reg(uint16_t start, size_t N, bool useDirectPointer = true) {
        storage.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            storage.emplace_back(RT, uint16_t(start + i), useDirectPointer);
        }
        update();
    }

    void update() {
        handle.clear();
        handle.reserve(storage.size());
        for (auto& tw : storage) {
            handle.push_back(tw.word);
        }
    }
};

// Word avec handler de lecture qui échoue
struct FailingReadWord {
    Modbus::Word word;
    
    FailingReadWord(Modbus::RegisterType type, uint16_t addr, uint16_t nbRegs) {
        word.type = type;
        word.startAddr = addr;
        word.nbRegs = nbRegs;
        word.userCtx = nullptr;
        
        // Handler qui retourne toujours SLAVE_DEVICE_FAILURE
        word.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            return Modbus::SLAVE_DEVICE_FAILURE;
        };
        
        // Si ce n'est pas un type read-only, il faut un write handler
        if (type == Modbus::HOLDING_REGISTER || type == Modbus::COIL) {
            word.writeHandler = [](const uint16_t* writeVals, const ServerWord& w, void* userCtx) -> Modbus::ExceptionCode {
                return Modbus::NULL_EXCEPTION;  // Write accepté mais ne fait rien
            };
        } else {
            word.writeHandler = nullptr;
        }
    }
};

// Word avec handler d'écriture qui échoue
struct FailingWriteWord {
    Modbus::Word word;
    uint16_t values[10];
    
    FailingWriteWord(uint16_t addr, uint16_t nbRegs, uint16_t startVal) {
        word.type = Modbus::HOLDING_REGISTER;
        word.startAddr = addr;
        word.nbRegs = nbRegs;
        word.userCtx = values;
        
        // Init values
        for (uint16_t i = 0; i < nbRegs; i++) {
            values[i] = startVal + i;
        }
        
        // Handler de lecture normal
        word.readHandler = [](const ServerWord& w, uint16_t* outVals, void* userCtx) -> Modbus::ExceptionCode {
            uint16_t* vals = (uint16_t*)userCtx;
            for (uint16_t i = 0; i < w.nbRegs; i++) {
                outVals[i] = vals[i];
            }
            return Modbus::NULL_EXCEPTION;
        };
        
        // Handler d'écriture qui échoue toujours
        word.writeHandler = [](const uint16_t* writeVals, const ServerWord& w, void* userCtx) -> Modbus::ExceptionCode {
            return Modbus::SLAVE_DEVICE_FAILURE;
        };
    }
};

// Helper functions for Word API
inline void do_test_add_word(Modbus::Server& server,
                            const ServerWord& word,
                            bool expectSuccess,
                            const char* desc)
{
    auto res = server.addWord(word);
    TEST_ASSERT_START();
    if (expectSuccess)
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
    else
        TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
}

inline void do_test_add_words(Modbus::Server& server,
                             const std::vector<ServerWord>& words,
                             bool expectSuccess,
                             const char* desc)
{
    auto res = server.addWords(words);
    TEST_ASSERT_START();
    if (expectSuccess)
        TEST_ASSERT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
    else
        TEST_ASSERT_NOT_EQUAL_MESSAGE(Modbus::Server::SUCCESS, res, desc);
}

// ===================================================================================
// LEGACY COMPATIBILITY LAYER (Register API emulation using Words)
// ===================================================================================

// Compatibility aliases for migration
using DummyRegister = DummyWord1Reg;
template<RegType RT>
using DummyRegSuite = DummyWordSuite1Reg<RT>;

// Legacy helper functions that map to Word API
inline void do_test_add_register(Modbus::Server& server,
                                 const ServerWord& word,  // Note: was ServerReg
                                 bool expectSuccess,
                                 const char* desc)
{
    return do_test_add_word(server, word, expectSuccess, desc);
}

inline void do_test_add_registers(Modbus::Server& server,
                                  const std::vector<ServerWord>& words,  // Note: was vector<ServerReg>
                                  bool expectSuccess,
                                  const char* desc)
{
    return do_test_add_words(server, words, expectSuccess, desc);
}