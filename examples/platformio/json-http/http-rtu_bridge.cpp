/**
 * @file http-rtu_bridge.cpp
 * @brief Example using EZModbus for a Modbus Bridge application
 * @brief (demonstration of bridging between RTU Client and JSON HTTP Server)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <EZModbus.h>
#include "ModbusHTTP.h"
#include "ModbusJsonCodec.h"

// Wi-Fi credentials
const char* ssid = "your_ssid_here";
const char* password = "your_password_here";

// Serial pins for RTU port
const int RXD_PIN = 44;
const int TXD_PIN = 43;

// Aliases for convenience
using ModbusRTU = ModbusInterface::RTU;
using ModbusBridge = Modbus::Bridge;

// HTTP server instance (in WS mode to allow spontaneous messages)
ModbusHTTP modbusHTTP(80);

// RTU Master interface (using Serial1, no flow control)
ModbusRTU modbusRTU(Serial1, Modbus::CLIENT);

// Bridge between HTTP (master) and RTU (slave)
// The request from HTTP interface is directly forwarded to RTU,
// and the response from RTU interface is sent back to HTTP.
ModbusBridge bridge(modbusHTTP, modbusRTU);

void setup() {
    Serial.begin(115200);
    // Initialize serial port for RTU communication (e.g. RS485)
    Serial1.begin(9600, SERIAL_8N1, RXD_PIN, TXD_PIN);
    // Adjust timing if needed
    modbusRTU.setCharTimeMs(50);

    delay(1000);
    Serial.println("Starting Modbus RTU/HTTP bridge...");

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected, IP : ");
    Serial.println(WiFi.localIP());

    // Start the bridge which will create the tasks
    if (bridge.begin() != ModbusBridge::SUCCESS) {
        Serial.println("Failed to initialize Modbus bridge");
        while (1) { delay(1000); } // Halt
    }
    
    Serial.println("Modbus bridge initialized");
}

void loop() {
    bridge.poll();
    vTaskDelay(pdMS_TO_TICKS(1));
}
