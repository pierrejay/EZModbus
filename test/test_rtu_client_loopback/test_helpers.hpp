/**
 * @file test_helpers.hpp
 * @brief Test suite for Client::read<T>() and Client::write<T>() template helpers
 * @note These tests are interface-agnostic and can be run with RTU or TCP
 */

#pragma once

#include <unity.h>
#include "test_params.h"

// External declarations from test_main.cpp
extern Modbus::Client client;
extern uint16_t serverDiscreteInputs[];
extern uint16_t serverCoils[];
extern uint16_t serverHoldingRegisters[];
extern uint16_t serverInputRegisters[];

// ===================================================================================
// HELPER TESTS - BASELINE (uint16_t)
// ===================================================================================

void test_helper_read_uint16_holding_registers() {
    uint16_t buf[5];
    Modbus::ExceptionCode excep;

    // Prepare server data
    for (int i = 0; i < 5; i++) {
        serverHoldingRegisters[100 + i] = 1000 + i;
    }

    auto res = client.read(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 100, 5, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL_UINT16(1000 + i, buf[i]);
    }
}

void test_helper_read_uint16_coils() {
    uint16_t buf[8];
    Modbus::ExceptionCode excep;

    // Prepare server data
    serverCoils[50] = 1;
    serverCoils[51] = 0;
    serverCoils[52] = 1;
    serverCoils[53] = 1;
    serverCoils[54] = 0;
    serverCoils[55] = 0;
    serverCoils[56] = 1;
    serverCoils[57] = 0;

    auto res = client.read(TEST_SLAVE_ID, Modbus::COIL, 50, 8, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(1, buf[0]);
    TEST_ASSERT_EQUAL_UINT16(0, buf[1]);
    TEST_ASSERT_EQUAL_UINT16(1, buf[2]);
    TEST_ASSERT_EQUAL_UINT16(1, buf[3]);
    TEST_ASSERT_EQUAL_UINT16(0, buf[4]);
    TEST_ASSERT_EQUAL_UINT16(0, buf[5]);
    TEST_ASSERT_EQUAL_UINT16(1, buf[6]);
    TEST_ASSERT_EQUAL_UINT16(0, buf[7]);
}

void test_helper_write_uint16_holding_registers() {
    uint16_t buf[3] = {5000, 6000, 7000};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 200, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(5000, serverHoldingRegisters[200]);
    TEST_ASSERT_EQUAL_UINT16(6000, serverHoldingRegisters[201]);
    TEST_ASSERT_EQUAL_UINT16(7000, serverHoldingRegisters[202]);
}

void test_helper_write_uint16_coils() {
    uint16_t buf[4] = {1, 0, 1, 0};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::COIL, 60, 4, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(1, serverCoils[60]);
    TEST_ASSERT_EQUAL_UINT16(0, serverCoils[61]);
    TEST_ASSERT_EQUAL_UINT16(1, serverCoils[62]);
    TEST_ASSERT_EQUAL_UINT16(0, serverCoils[63]);
}

// ===================================================================================
// HELPER TESTS - TYPE CONVERSION & CLAMPING (< 16 bits)
// ===================================================================================

void test_helper_read_uint8_clamp() {
    uint8_t buf[3];
    Modbus::ExceptionCode excep;

    // Prepare server data with values > 255
    serverHoldingRegisters[300] = 65535;  // Should clamp to 255
    serverHoldingRegisters[301] = 200;    // Should stay 200
    serverHoldingRegisters[302] = 1000;   // Should clamp to 255

    auto res = client.read(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 300, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT8(255, buf[0]);  // Clamped
    TEST_ASSERT_EQUAL_UINT8(200, buf[1]);  // No clamp
    TEST_ASSERT_EQUAL_UINT8(255, buf[2]);  // Clamped
}

void test_helper_read_int8_clamp() {
    int8_t buf[3];
    Modbus::ExceptionCode excep;

    // Prepare server data
    serverHoldingRegisters[310] = 200;    // Should clamp to 127
    serverHoldingRegisters[311] = 50;     // Should stay 50
    serverHoldingRegisters[312] = 65535;  // Should clamp to 127

    auto res = client.read(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 310, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_INT8(127, buf[0]);   // Clamped
    TEST_ASSERT_EQUAL_INT8(50, buf[1]);    // No clamp
    TEST_ASSERT_EQUAL_INT8(127, buf[2]);   // Clamped
}

void test_helper_write_uint8() {
    uint8_t buf[3] = {255, 100, 0};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 400, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(255, serverHoldingRegisters[400]);
    TEST_ASSERT_EQUAL_UINT16(100, serverHoldingRegisters[401]);
    TEST_ASSERT_EQUAL_UINT16(0, serverHoldingRegisters[402]);
}

void test_helper_write_int8_negative_clamp() {
    int8_t buf[4] = {-10, 50, -100, 127};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 410, 4, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(0, serverHoldingRegisters[410]);    // Negative clamped to 0
    TEST_ASSERT_EQUAL_UINT16(50, serverHoldingRegisters[411]);   // Positive ok
    TEST_ASSERT_EQUAL_UINT16(0, serverHoldingRegisters[412]);    // Negative clamped to 0
    TEST_ASSERT_EQUAL_UINT16(127, serverHoldingRegisters[413]);  // Max int8_t ok
}

// ===================================================================================
// HELPER TESTS - TYPE CONVERSION & CLAMPING (> 16 bits)
// ===================================================================================

void test_helper_write_uint32_clamp() {
    uint32_t buf[3] = {70000, 50000, 65535};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 500, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(65535, serverHoldingRegisters[500]);  // Clamped to UINT16_MAX
    TEST_ASSERT_EQUAL_UINT16(50000, serverHoldingRegisters[501]);  // No clamp
    TEST_ASSERT_EQUAL_UINT16(65535, serverHoldingRegisters[502]);  // No clamp
}

void test_helper_write_int32_clamp() {
    int32_t buf[3] = {70000, 30000, 65535};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 510, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(65535, serverHoldingRegisters[510]);  // Clamped to UINT16_MAX
    TEST_ASSERT_EQUAL_UINT16(30000, serverHoldingRegisters[511]);  // No clamp
    TEST_ASSERT_EQUAL_UINT16(65535, serverHoldingRegisters[512]);  // No clamp
}

void test_helper_write_int32_negative_clamp() {
    int32_t buf[3] = {-1000, 5000, -50};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 520, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(0, serverHoldingRegisters[520]);      // Negative clamped to 0
    TEST_ASSERT_EQUAL_UINT16(5000, serverHoldingRegisters[521]);   // Positive ok
    TEST_ASSERT_EQUAL_UINT16(0, serverHoldingRegisters[522]);      // Negative clamped to 0
}

// ===================================================================================
// HELPER TESTS - BOOL (special case)
// ===================================================================================

void test_helper_read_bool_coils() {
    bool buf[6];
    Modbus::ExceptionCode excep;

    // Prepare server data
    serverCoils[70] = 1;
    serverCoils[71] = 0;
    serverCoils[72] = 1;
    serverCoils[73] = 0;
    serverCoils[74] = 0;
    serverCoils[75] = 1;

    auto res = client.read(TEST_SLAVE_ID, Modbus::COIL, 70, 6, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_TRUE(buf[0]);
    TEST_ASSERT_FALSE(buf[1]);
    TEST_ASSERT_TRUE(buf[2]);
    TEST_ASSERT_FALSE(buf[3]);
    TEST_ASSERT_FALSE(buf[4]);
    TEST_ASSERT_TRUE(buf[5]);
}

void test_helper_write_bool_coils() {
    bool buf[4] = {true, false, true, false};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::COIL, 80, 4, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(1, serverCoils[80]);
    TEST_ASSERT_EQUAL_UINT16(0, serverCoils[81]);
    TEST_ASSERT_EQUAL_UINT16(1, serverCoils[82]);
    TEST_ASSERT_EQUAL_UINT16(0, serverCoils[83]);
}

void test_helper_write_bool_registers() {
    bool buf[3] = {true, false, true};
    Modbus::ExceptionCode excep;

    auto res = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 600, 3, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(1, serverHoldingRegisters[600]);  // true = 1
    TEST_ASSERT_EQUAL_UINT16(0, serverHoldingRegisters[601]);  // false = 0
    TEST_ASSERT_EQUAL_UINT16(1, serverHoldingRegisters[602]);  // true = 1
}

// ===================================================================================
// HELPER TESTS - MODBUS EXCEPTIONS
// ===================================================================================

void test_helper_exception_illegal_address() {
    uint16_t buf[5];
    Modbus::ExceptionCode excep = Modbus::NULL_EXCEPTION;

    // Read from invalid address (server should return exception)
    auto res = client.read(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 9999, 5, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res);  // Transport OK
    TEST_ASSERT_EQUAL(Modbus::ILLEGAL_DATA_ADDRESS, excep);  // Modbus exception
}

void test_helper_exception_vs_transport_error() {
    uint16_t buf[5];
    Modbus::ExceptionCode excep = Modbus::NULL_EXCEPTION;

    // Valid request - should succeed
    serverHoldingRegisters[100] = 1234;
    auto res1 = client.read(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 100, 1, buf, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::SUCCESS, res1);
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);
    TEST_ASSERT_EQUAL_UINT16(1234, buf[0]);

    // Invalid slave ID - should timeout (transport error, not Modbus exception)
    auto res2 = client.read(99, Modbus::HOLDING_REGISTER, 100, 1, buf, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_TIMEOUT, res2);  // Transport error
    TEST_ASSERT_EQUAL(Modbus::NULL_EXCEPTION, excep);      // No Modbus exception
}

// ===================================================================================
// HELPER TESTS - PARAMETER VALIDATION
// ===================================================================================

void test_helper_invalid_regtype() {
    uint16_t buf[5];
    Modbus::ExceptionCode excep;

    // Invalid register type
    auto res = client.read(TEST_SLAVE_ID, Modbus::NULL_RT, 100, 5, buf, &excep);

    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, res);
}

void test_helper_null_buffer() {
    Modbus::ExceptionCode excep;

    // Null read buffer
    auto res1 = client.read(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 100, 5, (uint16_t*)nullptr, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, res1);

    // Null write buffer
    auto res2 = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 100, 5, (const uint16_t*)nullptr, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, res2);
}

void test_helper_qty_zero() {
    uint16_t buf[5];
    Modbus::ExceptionCode excep;

    // Zero quantity
    auto res1 = client.read(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 100, 0, buf, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, res1);

    auto res2 = client.write(TEST_SLAVE_ID, Modbus::HOLDING_REGISTER, 100, 0, buf, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, res2);
}

void test_helper_write_readonly_regtype() {
    uint16_t buf[5] = {1, 2, 3, 4, 5};
    Modbus::ExceptionCode excep;

    // Try to write to INPUT_REGISTER (read-only)
    auto res = client.write(TEST_SLAVE_ID, Modbus::INPUT_REGISTER, 100, 5, buf, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, res);

    // Try to write to DISCRETE_INPUT (read-only)
    auto res2 = client.write(TEST_SLAVE_ID, Modbus::DISCRETE_INPUT, 100, 5, buf, &excep);
    TEST_ASSERT_EQUAL(Modbus::Client::ERR_INVALID_FRAME, res2);
}
