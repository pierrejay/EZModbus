#pragma once

// === Dynamic parameters ===
#define TEST_SLAVE_ID           1

// Test addresses
#define READ_COIL_ADDR          2
#define READ_DISCRETE_INPUT_ADDR 3
#define READ_HOLDING_ADDR       4
#define READ_INPUT_ADDR         5

// For multiple reads
#define MULTI_START_ADDR        10
#define MULTI_COUNT             50

// For broadcast writes
#define BROADCAST_SLAVE_ID      0x00
#define BROADCAST_START_ADDR    100
#define BROADCAST_COUNT         50

// ModbusTestServer initialization functions
#define MBT_INIT_START_REG                  0x00         // Default start register address
#ifndef MBT_INIT_REG_COUNT
    #define MBT_INIT_REG_COUNT              2500           // Increased to cover MAX_COILS_READ (2000) from MULTI_START_ADDR (10). Overridable per-env (shrink on RAM-constrained chips w/o PSRAM).
#endif
#define MBT_INIT_COIL_VALUE(x)              true         // Default coil value : true
#define MBT_INIT_DISCRETE_INPUT_VALUE(x)    true         // Default discrete input value : true
#define MBT_INIT_HOLDING_REGISTER_VALUE(x)  (10 + x)     // Default holding register value : 10 + register index
#define MBT_INIT_INPUT_REGISTER_VALUE(x)    (20 + x)     // Default input register value : 20 + register index

// === Table of single + multiple reads ===
//   X(ShortName, SingleMethod, MultipleMethod, AddressMacro, ExpectedMacro, FunctionCode)
#define READ_TESTS                                                                                                  \
  X(Coil,           readCoil,             readCoils,             READ_COIL_ADDR,            MBT_INIT_COIL_VALUE,            Modbus::READ_COILS) \
  X(DiscreteInput,  readDiscreteInput,    readDiscreteInputs,    READ_DISCRETE_INPUT_ADDR,  MBT_INIT_DISCRETE_INPUT_VALUE,  Modbus::READ_DISCRETE_INPUTS) \
  X(HoldingReg,     readHoldingRegister,  readHoldingRegisters,  READ_HOLDING_ADDR,         MBT_INIT_HOLDING_REGISTER_VALUE, Modbus::READ_HOLDING_REGISTERS) \
  X(InputReg,       readInputRegister,    readInputRegisters,    READ_INPUT_ADDR,           MBT_INIT_INPUT_REGISTER_VALUE,   Modbus::READ_INPUT_REGISTERS)

// Test addresses for write tests
#define WRITE_COIL_ADDR         30
#define WRITE_HOLDING_ADDR      31

// Test values for write tests
#define TEST_COIL_VALUE         false
#define TEST_HOLDING_VALUE      42

// Definition of write tests
#define WRITE_TESTS \
    X(Coil,       writeCoil,            writeCoils,           WRITE_COIL_ADDR,    TEST_COIL_VALUE,    Modbus::WRITE_COIL,    Modbus::WRITE_MULTIPLE_COILS) \
    X(HoldingReg, writeHoldingRegister, writeHoldingRegisters, WRITE_HOLDING_ADDR, TEST_HOLDING_VALUE, Modbus::WRITE_REGISTER, Modbus::WRITE_MULTIPLE_REGISTERS) 


    