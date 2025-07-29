#!/usr/bin/env python3
"""
Modbus TCP Client Tester for testing Pico server
Sends various requests to verify server functionality
"""

import logging
import time
import sys
from pymodbus.client import ModbusTcpClient
from pymodbus import __version__ as pymodbus_version

# Configure logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s [%(name)s] %(message)s',
    level=logging.INFO
)
log = logging.getLogger("ModbusClientTester")

# Server configuration - Pico running the server
PICO_IP = "192.168.0.124"
PICO_PORT = 5020  # Match the test server port
SLAVE_ID = 1

# Test configuration matching tcp_server_main.cpp
SERVER_REGISTER_START = 100    # Start address of server's register bank
SERVER_REGISTER_COUNT = 10     # Number of registers (100-109)

class TestResult:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []
    
    def success(self, test_name):
        self.passed += 1
        log.info(f"✅ {test_name}")
    
    def failure(self, test_name, reason):
        self.failed += 1
        self.errors.append(f"{test_name}: {reason}")
        log.error(f"❌ {test_name} - {reason}")
    
    def summary(self):
        total = self.passed + self.failed
        log.info("=" * 60)
        log.info("TEST SUMMARY")
        log.info("=" * 60)
        log.info(f"Total tests: {total}")
        log.info(f"Passed: {self.passed}")
        log.info(f"Failed: {self.failed}")
        if self.failed > 0:
            log.info("\nFailures:")
            for error in self.errors:
                log.info(f"  - {error}")
        log.info("=" * 60)
        return self.failed == 0


def test_connection(client):
    """Test basic connection to the server"""
    try:
        result = client.connect()
        if result:
            return True
        else:
            return False
    except Exception as e:
        log.error(f"Connection error: {e}")
        return False


def test_single_register_read(client, test_result):
    """Test reading a single register"""
    try:
        result = client.read_holding_registers(address=SERVER_REGISTER_START, count=1, slave=SLAVE_ID)
        if result.isError():
            test_result.failure("Single Register Read", f"Modbus error: {result}")
        else:
            value = result.registers[0]
            test_result.success(f"Single Register Read - Addr {SERVER_REGISTER_START}: {value}")
            return value
    except Exception as e:
        test_result.failure("Single Register Read", f"Exception: {e}")
    return None


def test_single_register_write(client, test_result, address, value):
    """Test writing a single register"""
    try:
        result = client.write_register(address, value, slave=SLAVE_ID)
        if result.isError():
            test_result.failure("Single Register Write", f"Modbus error: {result}")
            return False
        else:
            test_result.success(f"Single Register Write - Addr {address}: {value}")
            return True
    except Exception as e:
        test_result.failure("Single Register Write", f"Exception: {e}")
        return False


def test_multiple_register_read(client, test_result, start_addr, count):
    """Test reading multiple registers"""
    try:
        result = client.read_holding_registers(address=start_addr, count=count, slave=SLAVE_ID)
        if result.isError():
            test_result.failure("Multiple Register Read", f"Modbus error: {result}")
        else:
            values = result.registers
            test_result.success(f"Multiple Register Read - Addr {start_addr}-{start_addr+count-1}: {values}")
            return values
    except Exception as e:
        test_result.failure("Multiple Register Read", f"Exception: {e}")
    return None


def test_multiple_register_write(client, test_result, start_addr, values):
    """Test writing multiple registers"""
    try:
        result = client.write_registers(start_addr, values, slave=SLAVE_ID)
        if result.isError():
            test_result.failure("Multiple Register Write", f"Modbus error: {result}")
            return False
        else:
            test_result.success(f"Multiple Register Write - Addr {start_addr}: {values}")
            return True
    except Exception as e:
        test_result.failure("Multiple Register Write", f"Exception: {e}")
        return False


def test_write_read_verify(client, test_result):
    """Test write-read-verify cycle"""
    test_addr = SERVER_REGISTER_START + 2  # Use register 102
    test_value = 9999
    
    # Write
    if not test_single_register_write(client, test_result, test_addr, test_value):
        return
    
    # Small delay
    time.sleep(0.1)
    
    # Read back
    try:
        result = client.read_holding_registers(address=test_addr, count=1, slave=SLAVE_ID)
        if result.isError():
            test_result.failure("Write-Read-Verify", f"Read error: {result}")
        else:
            read_value = result.registers[0]
            if read_value == test_value:
                test_result.success(f"Write-Read-Verify - Value {test_value} verified")
            else:
                test_result.failure("Write-Read-Verify", f"Value mismatch: wrote {test_value}, read {read_value}")
    except Exception as e:
        test_result.failure("Write-Read-Verify", f"Exception: {e}")


def test_boundary_addresses(client, test_result):
    """Test reading at boundary addresses"""
    # Test first register
    try:
        result = client.read_holding_registers(address=SERVER_REGISTER_START, count=1, slave=SLAVE_ID)
        if result.isError():
            test_result.failure("Boundary Test - First Register", f"Error: {result}")
        else:
            test_result.success(f"Boundary Test - First Register ({SERVER_REGISTER_START}): {result.registers[0]}")
    except Exception as e:
        test_result.failure("Boundary Test - First Register", f"Exception: {e}")
    
    # Test last register
    last_addr = SERVER_REGISTER_START + SERVER_REGISTER_COUNT - 1
    try:
        result = client.read_holding_registers(address=last_addr, count=1, slave=SLAVE_ID)
        if result.isError():
            test_result.failure("Boundary Test - Last Register", f"Error: {result}")
        else:
            test_result.success(f"Boundary Test - Last Register ({last_addr}): {result.registers[0]}")
    except Exception as e:
        test_result.failure("Boundary Test - Last Register", f"Exception: {e}")


def test_invalid_address(client, test_result):
    """Test reading invalid address (should return exception)"""
    invalid_addr = 999  # Outside server's register range
    try:
        result = client.read_holding_registers(address=invalid_addr, count=1, slave=SLAVE_ID)
        if result.isError():
            test_result.success(f"Invalid Address Test - Correctly rejected address {invalid_addr}")
        else:
            test_result.failure("Invalid Address Test", f"Should have failed but got: {result.registers}")
    except Exception as e:
        test_result.success(f"Invalid Address Test - Exception as expected: {e}")


def run_tests():
    """Run all tests"""
    
    log.info("=" * 60)
    log.info("Modbus TCP Client Tester")
    log.info(f"pymodbus version: {pymodbus_version}")
    log.info("=" * 60)
    log.info(f"Target: {PICO_IP}:{PICO_PORT}")
    log.info(f"Slave ID: {SLAVE_ID}")
    log.info(f"Expected server registers: {SERVER_REGISTER_START}-{SERVER_REGISTER_START + SERVER_REGISTER_COUNT - 1}")
    log.info("")
    
    test_result = TestResult()
    
    # Create client
    client = ModbusTcpClient(PICO_IP, port=PICO_PORT)
    
    # Test connection
    log.info("Testing connection...")
    if not test_connection(client):
        log.error("❌ Cannot connect to server. Ensure Pico server is running!")
        return False
    
    log.info("✅ Connected to Pico server")
    log.info("")
    
    try:
        # Test suite
        log.info("Running test suite...")
        log.info("")
        
        # Basic read tests
        original_value = test_single_register_read(client, test_result)
        test_multiple_register_read(client, test_result, SERVER_REGISTER_START, 5)
        
        # Write tests
        if original_value is not None:
            test_single_register_write(client, test_result, SERVER_REGISTER_START, original_value + 100)
        
        test_multiple_register_write(client, test_result, SERVER_REGISTER_START + 3, [2000, 2001, 2002])
        
        # Verification test
        test_write_read_verify(client, test_result)
        
        # Boundary tests
        test_boundary_addresses(client, test_result)
        
        # Error condition tests
        test_invalid_address(client, test_result)
        
        # Wait a bit before final summary
        time.sleep(1)
        
    except KeyboardInterrupt:
        log.info("\nTest interrupted by user")
        return False
    
    finally:
        client.close()
    
    # Print summary
    return test_result.summary()


def main():
    """Main entry point"""
    try:
        success = run_tests()
        sys.exit(0 if success else 1)
    except Exception as e:
        log.error(f"Test runner error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()