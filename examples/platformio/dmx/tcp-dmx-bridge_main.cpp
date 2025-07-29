/**
 * @file ModbusTCP_to_DMX_bridge.ino
 * @brief Example bridge from Modbus TCP Server to DMX device
 * @note Uses EZModbus ModbusBridge component with a TCP server interface 
 *       and a ModbusDMX interface
 */

/*
 * USAGE NOTES:
 * 
 * 1. Configure your Modbus TCP client to connect to the ESP32's IP address on port 502
 * 
 * 2. Use Modbus function codes:
 *    - 0x06 (Write Single Register): Set one DMX channel
 *    - 0x10 (Write Multiple Registers): Set multiple DMX channels
 *    - 0x03 (Read Holding Registers): Read current DMX channel values
 * 
 * 3. Register addressing:
 *    - Modbus register 1 = DMX channel 1
 *    - Modbus register 512 = DMX channel 512
 * 
 * 4. Value scaling:
 *    - Modbus value 0 = DMX value 0 (0%)
 *    - Modbus value 32767 = DMX value 127 (50%)
 *    - Modbus value 65535 = DMX value 255 (100%)
 * 
 * 5. Example Modbus commands:
 *    - Set DMX channel 1 to 50%: Write register 1 with value 32767
 *    - Set DMX channels 1-4 to full: Write registers 1-4 with value 65535
 *    - Read DMX channels 10-20: Read holding registers 10-20
 * 
 * 6. Broadcast writes (slave ID 0) are supported and will not generate responses
 */ 

#include <WiFi.h>
#include <EZModbus.h>
#include "ModbusDMX.h"

// Wi-Fi credentials
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

// Network settings for Modbus TCP
const IPAddress serverIP(192, 168, 1, 100);
const uint16_t serverPort = 502;

// DMX hardware configuration
const int dmxTxPin = 17;
const int dmxRxPin = 16;
const int dmxDePin = 4;  // RS485 direction control pin

// Modbus slave ID for DMX device
const uint8_t dmxSlaveId = 1;

// Global objects
ModbusHAL::TCP tcpServer(serverPort);
ModbusInterface::TCP modbusTcp(tcpServer, Modbus::SERVER);
ModbusDMX dmxInterface(&Serial1, dmxTxPin, dmxRxPin, dmxDePin);
Modbus::Bridge bridge(modbusTcp, dmxInterface);

void setup() {
    Serial.begin(115200);
    Serial.println("=== Modbus TCP to DMX Bridge Example ===");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());

    // Initialize and start the bridge
    auto result = bridge.begin();
    if (result != Modbus::Bridge::SUCCESS) {
        Serial.printf("Failed to initialize bridge: %s\n", 
                     Modbus::Bridge::toString(result));
        while (true) delay(1000);
    }

    Serial.println("Bridge initialized successfully!");
    Serial.printf("Modbus TCP Server listening on %s:%d\n", 
                 WiFi.localIP().toString().c_str(), serverPort);
    Serial.printf("DMX Master on UART%d (TX:%d, RX:%d, DE:%d)\n", 
                 dmxUart, dmxTxPin, dmxRxPin, dmxDePin);
    Serial.println();
    Serial.println("Register mapping: Modbus [1..512] <-> DMX channels [1..512]");
    Serial.println("Value scaling: Modbus [0..65535] <-> DMX [0..255]");
    Serial.println();
    Serial.println("Bridge ready - waiting for Modbus TCP connections...");
}

void loop() {
    // The bridge handles all communication automatically
    // Just keep the watchdog happy
    delay(100);
    
    // Optional: Add status monitoring
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 10000) {  // Every 10 seconds
        Serial.printf("Status: WiFi %s, Bridge active\n", 
                     WiFi.isConnected() ? "connected" : "disconnected");
        lastStatus = millis();
    }
}

