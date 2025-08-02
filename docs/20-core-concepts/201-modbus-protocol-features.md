# Modbus protocol features

## Registers type & Function codes support

### Register types

Both client & server supports all four Modbus register types with appropriate read/write permissions:

* `COIL` (digital inputs, read-write)
* `DISCRETE_INPUT` (digital outputs, read-only)
* `HOLDING_REGISTER` (analog inputs, read-write)
* `INPUT_REGISTER` (analog outputs, read-only)

### Function codes

The function codes supported are the following:

* `READ_COILS` (FC01)
* `READ_DISCRETE_INPUTS` (FC02)
* `READ_HOLDING_REGISTERS`(FC03)
* `READ_INPUT_REGISTERS` (FC04)
* `WRITE_COIL` (FC05)
* `WRITE_REGISTER` (FC06, for holding registers)
* `WRITE_MULTIPLE_COILS` (FC15)
* `WRITE_MULTIPLE_REGISTERS` (FC16, for holding registers)

They are described in the `Modbus::FunctionCode` enum and can be handled in plaintext in user code. Inconsistent data trigger explicit errors, such as `ERR_INVALID_REG_COUNT` when trying to use a “single write” function code while supplying several register values.

!!! note
    Out of simplicity, the function code for “Diagnostics” and “Read/Write multiple registers” have not been implemented, the current features covering most common use cases for interfacing with industrial systems.

## Modbus Role semantics

Since Modbus is a request/response protocol, the `Modbus::Role` defines whether a device is acting as a consumer (the one sending requests) or a provider (the one responding to requests) of Modbus data. Normally, this type is passed as argument when initializing an interface and does not need to be used later on.

Historically, Modbus RTU systems used traditional `MASTER` / `SLAVE` semantics while more recent Modbus TCP devices opted for `CLIENT`/`SERVER` , closest to networking logic. In EZModbus, both work :

* `MASTER` and `CLIENT` are aliases pointing to the same value
* `SLAVE` and `SERVER` as well
