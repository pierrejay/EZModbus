#!/usr/bin/env python3
"""
Modbus TCP Echo Server for testing Pico client
Implements a simple server that echoes/mirrors register operations
"""

import logging
import asyncio
from pymodbus import __version__ as pymodbus_version
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.server import StartAsyncTcpServer
from pymodbus.device import ModbusDeviceIdentification

# Configure logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s [%(name)s] %(message)s',
    level=logging.INFO
)
log = logging.getLogger("ModbusEchoServer")

# Server configuration matching tcp_client_main.cpp
SERVER_PORT = 5020  # Use non-privileged port (avoid sudo requirement)
SLAVE_ID = 1

# Register configuration
REGISTER_START = 0     # Start address for our register bank
REGISTER_COUNT = 200   # Total registers (covers addresses 0-199)
INITIAL_VALUE = 1000   # Initial value for registers


class LoggingDataBlock(ModbusSequentialDataBlock):
    """Custom data block that logs all operations"""
    
    def setValues(self, address, values):
        """Override to log write operations"""
        if isinstance(values, list):
            log.info(f"WRITE: Address={address}, Count={len(values)}, Values={values}")
        else:
            log.info(f"WRITE: Address={address}, Value={values}")
        return super().setValues(address, values)
    
    def getValues(self, address, count=1):
        """Override to log read operations"""
        values = super().getValues(address, count)
        log.info(f"READ: Address={address}, Count={count}, Values={values}")
        return values


async def run_server():
    """Run the Modbus TCP server"""
    
    log.info("=" * 60)
    log.info("Modbus TCP Echo Server")
    log.info(f"pymodbus version: {pymodbus_version}")
    log.info("=" * 60)
    
    # Initialize data store with logging
    # Create blocks for different register types
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [0] * REGISTER_COUNT),  # Discrete Inputs
        co=ModbusSequentialDataBlock(0, [0] * REGISTER_COUNT),  # Coils
        hr=LoggingDataBlock(0, [INITIAL_VALUE + i for i in range(REGISTER_COUNT)]),  # Holding Registers
        ir=ModbusSequentialDataBlock(0, [0] * REGISTER_COUNT),  # Input Registers
    )
    
    # Create server context
    context = ModbusServerContext(slaves=store, single=True)
    
    # Server identity (optional but nice to have)
    identity = ModbusDeviceIdentification()
    identity.VendorName = "PyModbus"
    identity.ProductCode = "ECHO"
    identity.VendorUrl = "https://github.com/pymodbus-dev/pymodbus"
    identity.ProductName = "Modbus Echo Server"
    identity.ModelName = "Echo Server Example"
    identity.MajorMinorRevision = pymodbus_version
    
    # Log initial register values for addresses used by client
    log.info("\nInitial register values:")
    log.info(f"  Address 10: {store.getValues(3, 10, 1)[0]}")  # fc=3 for holding registers
    log.info(f"  Addresses 10-19: {store.getValues(3, 10, 10)}")
    log.info("")
    
    # Start the server
    log.info(f"Starting Modbus TCP server on 192.168.0.234:{SERVER_PORT}")
    log.info(f"Slave ID: {SLAVE_ID}")
    log.info("Client (Pico) should connect from 192.168.0.124")
    log.info("NOTE: Using port 5020 instead of 502 to avoid sudo requirement")
    log.info("Waiting for client connections...")
    log.info("")
    
    await StartAsyncTcpServer(
        context=context,
        identity=identity,
        address=("192.168.0.234", SERVER_PORT),  # Bind to Mac's IP
    )


def main():
    """Main entry point"""
    try:
        # Python 3.7+
        asyncio.run(run_server())
    except AttributeError:
        # Python 3.6
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run_server())
        loop.run_forever()
    except KeyboardInterrupt:
        log.info("\nServer stopped by user")
    except Exception as e:
        log.error(f"Server error: {e}")


if __name__ == "__main__":
    main()