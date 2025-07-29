/**
 * @file ModbusHTTP.h
 * @brief Example using EZModbus for a Modbus HTTP Server application
 * @brief (implementation of the ModbusInterface layer for HTTP REST)
 * @note Depends on the ESPAsyncWebServer & ArduinoJson libraries
 */

#pragma once

#include <Arduino.h>
#include <functional>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <core/ModbusCore.h>
#include <core/ModbusCodec.hpp>
#include <interfaces/ModbusInterface.hpp>
#include "ModbusJsonCodec.h"

class ModbusHTTP : public ModbusInterface::IInterface {
public:

    ModbusHTTP(uint16_t port = 80) 
        : _server(port)
    {
        // Create the HTTP route for Modbus requests
        auto handler = new AsyncCallbackJsonWebHandler("/modbus", [this](AsyncWebServerRequest *request, JsonVariant &json) {
            handlePOSTRequest(request, json);
        });

        handler->setMethod(HTTP_POST);
        _server.addHandler(handler);

        // Add the GET route to retrieve the pending request
        _server.on("/modbus", HTTP_GET, [this](AsyncWebServerRequest *request) {
            handleGETRequest(request);
        });

    }

    void begin() override {
        _server.begin();
    }

    Result sendFrame(const Modbus::Frame& frame) override {
        // If an incoming HTTP request is active
        if (_pendingIncomingRequest.active) {

            // If a message is already being sent back, return a busy error
            if (_pendingIncomingRequest.responseReady) {
                Serial.println("[HTTP] Message en cours de traitement, réessayer plus tard");
                return Result::ERR_BUSY;
            }

            Serial.println("[HTTP] Requête entrante active, envoi réponse");
            printFrame("[HTTP] Réponse à envoyer", frame);

            // It must be a response
            if (frame.type != Modbus::RESPONSE) {
                Serial.println("[HTTP] Requête en cours, attendu: RESPONSE");
                return Result::ERR_BUSY;
            }

            // Clear the JSON document before encoding
            _pendingIncomingRequest.responseJson.clear();
            
            // Encode the response frame to JSON
            JsonObject respObj = _pendingIncomingRequest.responseJson.to<JsonObject>();
            auto encodeResult = ModbusCodec::JSON::encode(frame, respObj);
            if (encodeResult != ModbusCodec::Result::SUCCESS) {
                Serial.printf("[HTTP] Erreur d'encodage: %s\n", ModbusCodec::toString(encodeResult));
                return Result::ERR_INVALID_FRAME;
            }

            // Marquer la réponse comme prête
            _pendingIncomingRequest.responseReady = true;
            Serial.println("[HTTP] Requête sortante disponible");
            
            return Result::SUCCESS;
        }
        
        // If it's a spontaneous message : first try to encode it in JSON
        Serial.println("[HTTP] Tentative d'encodage du message spontané");
        StaticJsonDocument<JSON_SIZE> doc;
        JsonObject jsonObj = doc.to<JsonObject>();
        auto encodeResult = ModbusCodec::JSON::encode(frame, jsonObj);
        if (encodeResult != ModbusCodec::Result::SUCCESS) {
            Serial.printf("[HTTP] Erreur d'encodage: %s\n", ModbusCodec::toString(encodeResult));
            return Result::ERR_INVALID_FRAME;
        }

        // Copy the outgoing frame to the pending outgoing message and set it to active status
        _pendingOutgoingMsg.set(jsonObj);

        return Result::SUCCESS;
    }

    // Check for timeouts
    void poll() override {
        checkTimeouts();
    }

    bool isReady() override {
        return true; // The HTTP interface is always ready (messages are handled by ESPAsyncWebServer)
    }

private:
    static constexpr uint32_t OUTGOING_MESSAGE_TIMEOUT = 10000;  // 10 seconds for outgoing messages
    static constexpr uint32_t INCOMING_REQUEST_TIMEOUT = 3000;   // 3 seconds for incoming requests
    static constexpr size_t JSON_SIZE = 512;

    // Specific structure for an incoming HTTP request waiting for a response
    struct IncomingHTTPRequest {
        uint32_t timestamp = 0;                      // Timestamp of the request
        bool active = false;                         // Indicate if the request is active
        bool timeout = false;                       // Indicate if the request has timed out
        StaticJsonDocument<JSON_SIZE> responseJson;  // JSON object to store the response
        bool responseReady = false;                  // Indicate if the response is ready

        void clear() {
            timestamp = 0;
            active = false;
            responseJson.clear();
            responseReady = false;
        }

        void set() {
            timestamp = millis();
            active = true;
        }

    };

    // Specific structure to store an outgoing frame waiting to be polled by the client
    struct OutgoingHTTPMessage {
        StaticJsonDocument<JSON_SIZE> json;  // JSON object to store the outgoing frame
        uint32_t timestamp;                  // Timestamp of the outgoing frame
        bool active;                         // Indicate if the outgoing frame is active

        void clear() {
            json.clear();
            timestamp = 0;
            active = false;
        }

        void set(const JsonObject& json) {
            this->json = json;
            timestamp = millis();
            active = true;
        }
    };

    AsyncWebServer _server;
    IncomingHTTPRequest _pendingIncomingRequest;
    OutgoingHTTPMessage _pendingOutgoingMsg;

    void handlePOSTRequest(AsyncWebServerRequest *request, JsonVariant &json) {
        Serial.println("[HTTP] Réception requête POST");
        
        // Log the raw JSON request
        String rawJson;
        serializeJsonPretty(json, rawJson);
        Serial.printf("[HTTP] Requête JSON reçue: %s\n", rawJson.c_str());

        // Decode the incoming Modbus message
        JsonObject reqObj = json.as<JsonObject>();
        Modbus::Frame tmpFrame;
        auto decodeResult = ModbusCodec::JSON::decode(reqObj, tmpFrame);
        if (decodeResult != ModbusCodec::Result::SUCCESS) {
            Serial.printf("[HTTP] Erreur décodage: %s\n", ModbusCodec::toString(decodeResult));
            const char* errorMsg = ModbusCodec::toString(decodeResult);
            request->send(400, "application/json",
                        "{\"error\":\"" + String(errorMsg) + "\"}");
            return;
        }
        Serial.println("[HTTP] Frame décodée avec succès:");
        printFrame("[HTTP] Requête", tmpFrame);

        // If an incoming request is already active, return a busy error
        if (_pendingIncomingRequest.active) {
            Serial.println("[HTTP] Incoming request active, send busy error");
            tmpFrame = Modbus::setSlaveBusy(tmpFrame);
            String jsonStr;
            serializeFrame(tmpFrame, jsonStr);
            request->send(503, "application/json", jsonStr);
            return;
        }

        // If it's a request, create a chunked response
        if (tmpFrame.type == Modbus::REQUEST) {
            AsyncWebServerResponse *response = request->beginChunkedResponse(
                "application/json",
                [this](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {

                    // If the response is not ready
                    if (_pendingIncomingRequest.active && !_pendingIncomingRequest.responseReady) {
                        // Timeout
                        if (millis() - _pendingIncomingRequest.timestamp >= INCOMING_REQUEST_TIMEOUT) {
                            Modbus::Frame timeoutFrame;
                            Modbus::setSlaveDeviceFailure(timeoutFrame);
                            String jsonStr;
                            serializeFrame(timeoutFrame, jsonStr);
                            size_t written = snprintf((char*)buffer, maxLen, "%s", jsonStr.c_str());
                            _pendingIncomingRequest.clear();
                            return written;
                        }
                        // Delay
                        return RESPONSE_TRY_AGAIN;
                    }

                    // If a response is ready, send it
                    if (_pendingIncomingRequest.active && _pendingIncomingRequest.responseReady) {
                        // Send to buffer
                        String jsonStr;
                        serializeJsonPretty(_pendingIncomingRequest.responseJson, jsonStr);
                        size_t written = snprintf((char*)buffer, maxLen, "%s", jsonStr.c_str());
                        _pendingIncomingRequest.clear();
                        return written;
                    }

                    // If the request is already inactive (or any other case), return 0
                    // (terminates a chunked response by sending EOF characters)
                    return 0;
                    
                }
            );

            // Enable incoming request & send the chunked response immediately
            _pendingIncomingRequest.set();
            request->send(response);
        }

        // If it's a response, send a 200 OK
        if (tmpFrame.type == Modbus::RESPONSE) {
            request->send(200, "application/json", "{\"success\":\"true\"}");
        }

        // Notify the receive callback in any case
        if (_rcvCallback) {
            Serial.println("[HTTP] Call receive callback");
            _rcvCallback(tmpFrame);
        }
    }

    void handleGETRequest(AsyncWebServerRequest *request) {
        // If there's an outgoing message waiting to be polled,
        // push it to the client, otherwise return a 404 error
        if (_pendingOutgoingMsg.active) {
            String response;
            serializeJsonPretty(_pendingOutgoingMsg.json, response);
            request->send(200, "application/json", response);
            // The outgoing message stays active after polling until timeout expiration
        } else {
            request->send(404, "application/json", 
                        "{\"error\":\"void\"}");
        }
    }

    void checkTimeouts() {
        uint32_t currentTime = millis();

        // Check outgoing message timeout
        // (if the message isn't polled by the client with GET, 
        // it's considered as lost after timeout expiration)
        if ((_pendingOutgoingMsg.active) && 
            (currentTime - _pendingOutgoingMsg.timestamp >= OUTGOING_MESSAGE_TIMEOUT)) {
            _pendingOutgoingMsg.clear();
        }

        // Check incoming request timeout
        // (a timeout error must be returned to the client if 
        // the response timeout expires)
        // DONE IN CHUNKED RESPONSE HANDLER
    }

    // Fonction utilitaire pour afficher une frame
    void printFrame(const char* label, const Modbus::Frame& frame) {
        Serial.printf("\n=== %s ===\n", label);
        Serial.printf("Type            : %s\n", frame.type == Modbus::REQUEST ? "REQUEST" : "RESPONSE");
        Serial.printf("Function code   : 0x%02X\n", frame.fc);
        Serial.printf("Slave ID       : %d\n", frame.slaveId);
        Serial.printf("Register Addr  : %d\n", frame.regAddress);
        Serial.printf("Register Count : %d\n", frame.regCount);
        Serial.print("Data           : ");
        for (auto d : frame.data) {
            Serial.printf("0x%04X ", d);
        }
        Serial.println();
        if (frame.exceptionCode != Modbus::NULL_EXCEPTION) {
            Serial.printf("Exception     : 0x%02X\n", frame.exceptionCode);
        }
        Serial.println("================\n");
    }

    // Transform a Modbus frame into a String JSON and return its size (0 if encoding failed)
    size_t serializeFrame(const Modbus::Frame& frame, String& buffer) {
        StaticJsonDocument<JSON_SIZE> doc;  
        JsonObject jsonObj = doc.to<JsonObject>();
        auto encodeResult = ModbusCodec::JSON::encode(frame, jsonObj);
        if (encodeResult != ModbusCodec::SUCCESS) return 0;
        serializeJsonPretty(doc, buffer);
        return buffer.length();
    }
};