## Description

This example demonstrates how to use the `EZModbus` library to communicate locally between a Modbus RTU Client & Modbus RTU Server using a local UART loopback connection.

## How to run

The example uses `UART_NUM_1` and `UART_NUM_2` and builds on ESP32, ESP32-S3 and
ESP32-P4. Before running it, connect the two UARTs with the matching jumpers:

| Target | Server RX ↔ Client TX | Server TX ↔ Client RX |
| --- | --- | --- |
| ESP32 | GPIO16 ↔ GPIO19 | GPIO17 ↔ GPIO18 |
| ESP32-S3 / ESP32-P4 | GPIO44 ↔ GPIO43 | GPIO7 ↔ GPIO6 |

ESP32-C6 has no `UART_NUM_2`; its LP-UART needs a different configuration, so this
example is skipped for that target by the compile checks.
