#include <unity.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstring>
#include <cfloat>
#include <climits>
#include <cstdint>

#include "core/ModbusCodec.hpp"

// Helper function to compare two Modbus frames
bool compareFrames(const Modbus::Frame& f1, const Modbus::Frame& f2) {
    using namespace Modbus;

    // 1) RESPONSES READ_COILS & READ_DISCRETE_INPUTS (bit-mask + padding)
    if (f1.type==RESPONSE &&
       (f1.fc==READ_COILS || f1.fc==READ_DISCRETE_INPUTS)) 
    {
        // On compare uniquement jusqu'à regCount bits
        for (size_t i=0; i<f1.regCount; ++i) 
            if (f1.getCoil(i) != f2.getCoil(i)) return false;
        return f1.type==f2.type && f1.fc==f2.fc && f1.slaveId==f2.slaveId;
    }

    // 2) RESPONSES READ_HOLDING_REGISTERS & READ_INPUT_REGISTERS (juste data)
    if (f1.type==RESPONSE &&
       (f1.fc==READ_HOLDING_REGISTERS || f1.fc==READ_INPUT_REGISTERS))
    {
        if (f1.type!=f2.type || f1.fc!=f2.fc || f1.slaveId!=f2.slaveId) return false;
        if (f1.regCount!=f2.regCount) return false;
        for (size_t i=0; i<f1.regCount; ++i)
            if (f1.getRegister(i)!=f2.getRegister(i)) return false;
        return true;
    }

    // 3) SPECIAL CASE WRITE_SINGLE (coil & register), request *and* response
    if ((f1.fc==WRITE_COIL || f1.fc==WRITE_REGISTER) &&
        (f1.type==REQUEST || f1.type==RESPONSE))
    {
        if (f1.type    != f2.type)          return false;
        if (f1.fc      != f2.fc)            return false;
        if (f1.slaveId != f2.slaveId)       return false;
        if (f1.regAddress!= f2.regAddress)  return false;
        if (f1.exceptionCode!=f2.exceptionCode) return false;
        if (f1.regCount!= f2.regCount)    return false;
        // Pour un write single, on ne compare que la première valeur
        if (f1.getRegister(0)!=f2.getRegister(0))     return false;
        return true;
    }

    // 4) CAS GÉNÉRIQUE (tout le reste : read-requests, exceptions, multi-write, etc.)
    if (f1.type          != f2.type)           return false;
    if (f1.fc            != f2.fc)             return false;
    if (f1.slaveId       != f2.slaveId)        return false;
    if (f1.regAddress    != f2.regAddress)     return false;
    if (f1.regCount      != f2.regCount)       return false;
    if (f1.exceptionCode != f2.exceptionCode)  return false;
    
    // On compare les données jusqu'à regCount
    for (size_t i=0; i<f1.regCount; ++i)
        if (f1.getRegister(i)!=f2.getRegister(i))      return false;

    return true;
}





// Helper function to print frame details for debugging
void printFrame(const Modbus::Frame& frame, const char* prefix = "") {
    std::cout << prefix << "Frame details:" << std::endl;
    std::cout << prefix << "  Type: " << Modbus::toString(frame.type) << std::endl;
    std::cout << prefix << "  FC: " << Modbus::toString(frame.fc) << std::endl;
    std::cout << prefix << "  SlaveID: " << (int)frame.slaveId << std::endl;
    std::cout << prefix << "  RegAddr: " << frame.regAddress << std::endl;
    std::cout << prefix << "  RegCount: " << frame.regCount << std::endl;
    std::cout << prefix << "  Data: [";
    for (size_t i = 0; i < frame.regCount && i < frame.data.size(); i++) {  // ✅ CORRIGÉ
        if (i > 0) std::cout << ", ";
        std::cout << frame.data[i];
    }
    std::cout << "]" << std::endl;
    std::cout << prefix << "  Exception: " << Modbus::toString(frame.exceptionCode) << std::endl;
}

// Structure de description d'un cas
struct Case {
    Modbus::MsgType     type;
    Modbus::FunctionCode fc;
    uint8_t             slaveId;
    uint16_t            regAddress;
    uint16_t            regCount;
    bool                isException;    // true => response with exception
};

// Génère dynamiquement le contenu de .data et .exceptionCode
Modbus::Frame makeFrame(const Case& C) {
    using namespace Modbus;
    Frame F;
    F.type        = C.type;
    F.fc          = C.fc;
    F.slaveId     = C.slaveId;
    F.regAddress  = C.regAddress;
    F.regCount    = C.regCount;
    if (C.type == RESPONSE && C.isException) {
        F.exceptionCode = ILLEGAL_FUNCTION; // tu peux varier selon C.fc
        return F;
    }
    F.exceptionCode = NULL_EXCEPTION;
    F.clearData(false);  // On s'assure que data est propre

    // Remplissage des data
    if (C.type == REQUEST) {
        switch (C.fc) {
          case WRITE_COIL:
            F.data = Modbus::packCoils({true});  // ON value for coil
            break;
          case WRITE_REGISTER:
            F.data[0] = C.regAddress;  // Echo value
            break;
          case WRITE_MULTIPLE_COILS:
            {
                std::vector<bool> coils(C.regCount);
                for (int i = 0; i < C.regCount; ++i) coils[i] = (i%2) != 0;
                F.data = Modbus::packCoils(coils);
            }
            break;
          case WRITE_MULTIPLE_REGISTERS:
            for (int i = 0; i < C.regCount; ++i)
                F.data[i] = C.regAddress + i;
            break;
          default: /* read requests no data */ break;
        }
    } else { // RESPONSE non-exception
        switch (C.fc) {
          case READ_COILS:
          case READ_DISCRETE_INPUTS:
            {
                std::vector<bool> coils(C.regCount);
                for (int i = 0; i < C.regCount; ++i) coils[i] = (i%2) != 0;
                F.data = Modbus::packCoils(coils);
            }
            break;
          case READ_HOLDING_REGISTERS:
          case READ_INPUT_REGISTERS:
            for (int i = 0; i < C.regCount; ++i)
                F.data[i] = C.regAddress + i;
            break;
          case WRITE_COIL:
            F.data = Modbus::packCoils({true});  // ON value for coil
            break;
          case WRITE_REGISTER:
            F.data[0] = F.regAddress;  // Echo value
            break;
          case WRITE_MULTIPLE_COILS:
          case WRITE_MULTIPLE_REGISTERS:
            // response only echo addr+count
            break;
          default: break;
        }
    }
    return F;
}

void test_codec_rtu() {
    using namespace Modbus;
    using namespace ModbusCodec;  // Ajout du namespace ModbusCodec
    // Liste brute de quelques combinaisons (à compléter)
    std::vector<Case> cases;
    auto addCases = [&](MsgType t, FunctionCode fc, std::initializer_list<uint16_t> counts){
        for (uint8_t sid : {1, 0, 255}) {
          for (uint16_t addr : {0, 1, 100}) {
            for (auto cnt : counts) {
                // Validité
                bool validSid = ModbusCodec::isValidSlaveId(sid, fc, t);
                bool validCnt = ModbusCodec::isValidRegisterCount(cnt, (uint8_t)fc, t);
                
                if (t == REQUEST) {
                    // Request valide
                    if (validSid && validCnt)
                        cases.push_back({t,fc,sid,addr,cnt,false});
                    // Request invalide avec ID invalide
                    if (!validSid)
                        cases.push_back({t,fc,sid,addr,cnt,false});
                } else { // RESPONSE
                    // Pour les réponses, on n'ajoute que les cas avec ID valide
                    if (validSid && validCnt) {
                        cases.push_back({t,fc,sid,addr,cnt,false});
                        // Response exception (seulement pour les IDs valides)
                        cases.push_back({t,fc,sid,addr,cnt,true});
                    }
                }
            }
          }
        }
    };

    // Pour chaque FC on choisit des regCount pertinents
    addCases(REQUEST,  READ_COILS,            {1,5,10});
    addCases(RESPONSE, READ_COILS,            {1,5,10});
    addCases(REQUEST,  READ_DISCRETE_INPUTS,  {1,5,10});
    addCases(RESPONSE, READ_DISCRETE_INPUTS,  {1,5,10});
    addCases(REQUEST,  READ_HOLDING_REGISTERS,{1,5,10});
    addCases(RESPONSE, READ_HOLDING_REGISTERS,{1,5,10});
    addCases(REQUEST,  READ_INPUT_REGISTERS,  {1,5,10});
    addCases(RESPONSE, READ_INPUT_REGISTERS,  {1,5,10});
    addCases(REQUEST,  WRITE_COIL,           {1});
    addCases(RESPONSE, WRITE_COIL,           {1});
    addCases(REQUEST,  WRITE_REGISTER,       {1});
    addCases(RESPONSE, WRITE_REGISTER,       {1});
    addCases(REQUEST,  WRITE_MULTIPLE_COILS, {1,5,10});
    addCases(RESPONSE, WRITE_MULTIPLE_COILS, {1,5,10});
    addCases(REQUEST,  WRITE_MULTIPLE_REGISTERS,{1,5,10});
    addCases(RESPONSE, WRITE_MULTIPLE_REGISTERS,{1,5,10});

    // ==== 1) CAS LIMITES RTU ====
    // regCount = 0 (invalid)
    cases.push_back({REQUEST, READ_COILS, 1, 0, 0, false});
    // regCount = MAX_COILS_READ (valid) / +1 (invalid)
    cases.push_back({REQUEST, READ_COILS, 1, 0, MAX_COILS_READ, false});
    cases.push_back({REQUEST, READ_COILS, 1, 0, uint16_t(MAX_COILS_READ+1), false});
    // regCount = MAX_REGISTERS_READ (valid) / +1 (invalid)
    cases.push_back({REQUEST, READ_HOLDING_REGISTERS, 1, 0, MAX_REGISTERS_READ, false});
    cases.push_back({REQUEST, READ_HOLDING_REGISTERS, 1, 0, uint16_t(MAX_REGISTERS_READ+1), false});
    // regAddress = 0xFFFF (borne haute)
    cases.push_back({REQUEST, READ_INPUT_REGISTERS, 1, 0xFFFF, 1, false});

    // ==== 2) BROADCAST EN RÉPONSE (doit échouer) ====
    cases.push_back({RESPONSE, READ_HOLDING_REGISTERS, 0, 0, 1, false});
    cases.push_back({RESPONSE, WRITE_COIL,             255, 0, 1, false});

    // ==== 3) EXCEPTIONS VARIÉES ====
    {
        // ILLEGAL_DATA_ADDRESS
        Frame F;
        F.type          = RESPONSE;
        F.fc            = READ_HOLDING_REGISTERS;
        F.slaveId       = 1;
        F.regAddress    = 0;
        F.regCount      = 5;
        F.exceptionCode = ILLEGAL_DATA_ADDRESS;
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        // encode
        auto r1 = ModbusCodec::RTU::encode(F, raw);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r1, "encode exception ILLEGAL_DATA_ADDRESS");
        // decode
        Frame D;
        auto r2 = ModbusCodec::RTU::decode(raw, D, RESPONSE);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r2, "decode exception ILLEGAL_DATA_ADDRESS");
        TEST_ASSERT_EQUAL_MESSAGE(F.exceptionCode, D.exceptionCode, "exceptionCode preserved");
    }
    {
        // SLAVE_DEVICE_BUSY
        Frame F = makeFrame(Case{RESPONSE, WRITE_REGISTER, 1, 1, 1, true});
        F.exceptionCode = SLAVE_DEVICE_BUSY;
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,
            ModbusCodec::RTU::encode(F, raw),
            "encode exception SLAVE_DEVICE_BUSY");
    }

    // ==== 4) FUNCTION CODE INVALIDE ====
    {
        Frame F;
        F.type       = REQUEST;
        F.fc         = static_cast<Modbus::FunctionCode>(0x99);
        F.slaveId    = 1;
        F.regAddress = 0;
        F.regCount   = 1;
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        auto r = ModbusCodec::RTU::encode(F, raw);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, "encode invalid function code");
    }

    // ==== 5) BORNES MAXIMALES MULTI-WRITE ====
    {
        // Test MAX_COILS_WRITE
        cases.push_back({REQUEST, WRITE_MULTIPLE_COILS, 1, 0, MAX_COILS_WRITE, false});
        cases.push_back({REQUEST, WRITE_MULTIPLE_COILS, 1, 0, uint16_t(MAX_COILS_WRITE+1), false});
        
        // Test MAX_REGISTERS_WRITE
        cases.push_back({REQUEST, WRITE_MULTIPLE_REGISTERS, 1, 0, MAX_REGISTERS_WRITE, false});
        cases.push_back({REQUEST, WRITE_MULTIPLE_REGISTERS, 1, 0, uint16_t(MAX_REGISTERS_WRITE+1), false});
        
        // Test regCount = 0 en multi-write
        cases.push_back({REQUEST, WRITE_MULTIPLE_COILS, 1, 0, 0, false});
        cases.push_back({REQUEST, WRITE_MULTIPLE_REGISTERS, 1, 0, 0, false});
    }

    // ==== 6) PDU MAL FORMÉES ====
    {
        // Test byteCount incorrect pour WRITE_MULTIPLE_COILS
        Frame F = makeFrame(Case{REQUEST, WRITE_MULTIPLE_COILS, 1, 0, 16, false});
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        ModbusCodec::RTU::encode(F, raw);
        // Corrompre le byteCount (trop petit)
        raw.write_at(6, raw[6]-1); // Le byteCount est à l'index 6 dans la trame RTU
        raw.resize(raw.size()-2);
        ModbusCodec::RTU::appendCRC(raw);
        Frame D;
        auto r = ModbusCodec::RTU::decode(raw, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r, "should fail on invalid byte count (too small)");

        // Corrompre le byteCount (trop grand)
        raw.write_at(6, raw[6]+2);
        raw.resize(raw.size()-2);
        ModbusCodec::RTU::appendCRC(raw);
        r = ModbusCodec::RTU::decode(raw, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r, "should fail on invalid byte count (too large)");

        // Tronquer le payload d'un write-register
        F = makeFrame(Case{REQUEST, WRITE_REGISTER, 1, 0, 1, false});
        raw.clear();
        ModbusCodec::RTU::encode(F, raw);
        raw.resize(raw.size()-3); // Enlever le CRC + 1 octet
        ModbusCodec::RTU::appendCRC(raw); // Remettre le CRC
        r = ModbusCodec::RTU::decode(raw, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r, "should fail on truncated write-register payload");
    }

    // ==== 7) RÉPONSES MULTI-WRITE ====
    {
        // Vérifier que les réponses WRITE_MULTIPLE ne contiennent que address+count
        Frame F = makeFrame(Case{RESPONSE, WRITE_MULTIPLE_COILS, 1, 0x1234, 5, false});
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        ModbusCodec::RTU::encode(F, raw);
        // La trame RTU doit faire exactement 8 octets:
        // slaveId(1) + fc(1) + addr(2) + count(2) + crc(2)
        TEST_ASSERT_EQUAL_MESSAGE(8, raw.size(), "WRITE_MULTIPLE_COILS response size check");

        F = makeFrame(Case{RESPONSE, WRITE_MULTIPLE_REGISTERS, 1, 0x1234, 5, false});
        raw.clear();
        ModbusCodec::RTU::encode(F, raw);
        TEST_ASSERT_EQUAL_MESSAGE(8, raw.size(), "WRITE_MULTIPLE_REGISTERS response size check");
    }

    // ==== 7b) RÉPONSE MULTI-WRITE REGCOUNT TROP GRAND ====
    {
        Frame F = makeFrame(Case{RESPONSE, WRITE_MULTIPLE_COILS,     1, 0, uint16_t(MAX_COILS_WRITE+1), false});
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        auto r = ModbusCodec::RTU::encode(F, raw);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r,
            "response WRITE_MULTIPLE_COILS should fail on regCount > MAX_COILS_WRITE");

        F = makeFrame(Case{RESPONSE, WRITE_MULTIPLE_REGISTERS, 1, 0, uint16_t(MAX_REGISTERS_WRITE+1), false});
        raw.clear();
        r = ModbusCodec::RTU::encode(F, raw);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r,
            "response WRITE_MULTIPLE_REGISTERS should fail on regCount > MAX_REGISTERS_WRITE");
    }

    // ==== 8) FLUX CRC CORROMPU ====
    {
        Frame F = makeFrame(Case{REQUEST, READ_HOLDING_REGISTERS, 1, 0, 1, false});
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        ModbusCodec::RTU::encode(F, raw);
        // Corrompre le CRC
        raw.write_at(raw.size()-1, raw[raw.size()-1] ^ 0xFF);
        Frame D;
        auto r = ModbusCodec::RTU::decode(raw, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_CRC, r, "should fail on invalid CRC");
    }

    // ==== 9) BROADCAST WRITE-MULTIPLES ====
    {
        // Test broadcast response pour WRITE_MULTIPLE_COILS
        cases.push_back({RESPONSE, WRITE_MULTIPLE_COILS, 0, 0, 1, false});
        cases.push_back({RESPONSE, WRITE_MULTIPLE_COILS, 255, 0, 1, false});
        
        // Test broadcast response pour WRITE_MULTIPLE_REGISTERS
        cases.push_back({RESPONSE, WRITE_MULTIPLE_REGISTERS, 0, 0, 1, false});
        cases.push_back({RESPONSE, WRITE_MULTIPLE_REGISTERS, 255, 0, 1, false});
    }

    // ==== 10) CAS LIMITES RTU SUPPLÉMENTAIRES ====
    {
        // 1) Décodage de trames trop courtes ou trop longues
        {
            // Trame trop courte (3 octets)
            uint8_t _shortFrame[3] = {0x01, 0x03, 0x00};
            ByteBuffer shortFrame(_shortFrame, sizeof(_shortFrame));
            Frame D;
            auto r = ModbusCodec::RTU::decode(shortFrame, D, REQUEST);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, "should fail on invalid frame length");

            // Trame trop longue (257 octets)
            uint8_t _longFrame[257];
            ByteBuffer longFrame(_longFrame, sizeof(_longFrame));
            r = ModbusCodec::RTU::decode(longFrame, D, REQUEST);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, "should fail on invalid frame length");
        }

        // 2) Broadcast ID invalide en lecture
        {
            Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            ModbusCodec::RTU::encode(F, raw);
            raw.write_at(0, 0); // Écraser avec broadcast ID
            // Retirer le CRC
            raw.resize(raw.size()-2);
            // Ajouter le CRC recalculé
            ModbusCodec::RTU::appendCRC(raw);
            Frame D;
            auto r = ModbusCodec::RTU::decode(raw, D, REQUEST);
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_SLAVEID, r, "should fail on broadcast read request");
        }

        // 3) Exception dans une requête
        {
            Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
            F.exceptionCode = ILLEGAL_FUNCTION;
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            auto r = ModbusCodec::RTU::encode(F, raw);
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_EXCEPTION, r, "should fail on request with exception");
        }

        // 4) Type de message invalide
        {
            Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            ModbusCodec::RTU::encode(F, raw);
            Frame D;
            auto r = ModbusCodec::RTU::decode(raw, D, NULL_MSG);
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_TYPE, r, "should fail on invalid message type");
        }

        // 5) Function code invalide en décodage
        {
            Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            ModbusCodec::RTU::encode(F, raw);
            raw.write_at(1, 0x99); // FC invalide
            // Retirer le CRC
            raw.resize(raw.size()-2);
            // Ajouter le CRC recalculé
            ModbusCodec::RTU::appendCRC(raw);
            Frame D;
            auto r = ModbusCodec::RTU::decode(raw, D, REQUEST);
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_FC, r, "should fail on invalid function code");
        }

        // 6) Slave ID hors plage
        {
            Frame F = makeFrame(Case{REQUEST, WRITE_COIL, 1, 0, 1, false});
            F.slaveId = 248; // > MAX_SLAVE_ID
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            auto r = ModbusCodec::RTU::encode(F, raw);
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_SLAVEID, r, "should fail on slave ID > 247");
        }

        // 7) Single write invalide : regCount = 0
        {
            Frame F;
            F.type = REQUEST;
            F.fc = WRITE_COIL;
            F.slaveId = 1;
            F.regAddress = 0;
            F.regCount = 0;
            F.clearData(false);  // Utilisation de clearData() au lieu de clear()
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            auto r = ModbusCodec::RTU::encode(F, raw);
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_REG_COUNT, r, "should fail on regCount=0 for WRITE_COIL");
        }

        // 7bis) Single write invalide : regCount = 2
        {
            Frame F;
            F.type = REQUEST;
            F.fc = WRITE_COIL;
            F.slaveId = 1;
            F.regAddress = 0;
            F.regCount = 2;
            F.clearData(false);  // Utilisation de clearData() au lieu de clear()
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            auto r = ModbusCodec::RTU::encode(F, raw);
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_REG_COUNT, r, "should fail on regCount>1 for WRITE_COIL");
        }

        // 8) Alignement pile/bit-packing
        {
            // Test avec 8 coils (1 octet plein)
            Frame F = makeFrame(Case{RESPONSE, READ_COILS, 1, 0, 8, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            ModbusCodec::RTU::encode(F, raw);
            Frame D8;
            ModbusCodec::RTU::decode(raw, D8, RESPONSE);
            TEST_ASSERT_TRUE_MESSAGE(compareFrames(F, D8), "round-trip 8 coils");

            // Test avec 9 coils (1 octet + 1 bit)
            F = makeFrame(Case{RESPONSE, READ_COILS, 1, 0, 9, false});
            raw.clear();
            ModbusCodec::RTU::encode(F, raw);
            Frame D9;
            ModbusCodec::RTU::decode(raw, D9, RESPONSE);
            TEST_ASSERT_TRUE_MESSAGE(compareFrames(F, D9), "round-trip 9 coils");
            
            // Vérifier qu'on obtient exactement 16 coils (2 octets)
            std::vector<bool> coils = D9.getCoils();
            TEST_ASSERT_EQUAL_MESSAGE(16, coils.size(), "should have exactly 16 coils");
            // Vérifier que les bits au-delà de regCount sont à 0
            for (size_t i = 9; i < 16; i++) {
                TEST_ASSERT_FALSE_MESSAGE(D9.getCoil(i), "bits beyond regCount should be 0");
            }
        }
    }

    // Itération
    for (auto& C : cases) {
        Frame A = makeFrame(C);
        // RTU
        {
          uint8_t _raw[256];
          ByteBuffer raw(_raw, sizeof(_raw));
          // Ajout des logs détaillés avant l'encodage
          std::cout << "\n=== Test RTU pour Frame ===" << std::endl;
          std::cout << "Type: " << Modbus::toString(C.type) << std::endl;
          std::cout << "FC: " << Modbus::toString(C.fc) << " (0x" << std::hex << (int)C.fc << ")" << std::endl;
          std::cout << "SlaveID: " << std::dec << (int)C.slaveId << std::endl;
          std::cout << "RegAddr: " << C.regAddress << std::endl;
          std::cout << "RegCount: " << C.regCount << std::endl;
          std::cout << "IsException: " << (C.isException ? "true" : "false") << std::endl;
          
          ModbusCodec::Result r = ModbusCodec::RTU::encode(A, raw);
          if (r != ModbusCodec::SUCCESS) {
              std::cout << "RTU encode failed with result: " << r << std::endl;
          }
          
          if (C.type==REQUEST && !ModbusCodec::isValidSlaveId(C.slaveId,(uint8_t)C.fc,C.type))
              TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"should fail bad slaveId");
          else if (C.type==REQUEST && !ModbusCodec::isValidRegisterCount(C.regCount,(uint8_t)C.fc,C.type))
              TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"should fail bad regCount");
          else if (C.type==RESPONSE && (C.slaveId == 0 || C.slaveId == 255))
              TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"should fail broadcast response");
          else {
              TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"RTU encode");
              Frame B; auto rd = ModbusCodec::RTU::decode(raw,B,C.type);
              // exception vs normal
              if (C.isException) {
                  TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, rd,"exception responses decode OK");
                  TEST_ASSERT_EQUAL_MESSAGE(A.exceptionCode,B.exceptionCode,"exceptionCode");
              } else {
                  TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, rd,"RTU decode");
                  if (rd == SUCCESS && !C.isException && !compareFrames(A,B)) {
                        std::cout << "== Échec round-trip RTU pour FC=" 
                                << Modbus::toString(A.fc) 
                                << " Type=" << Modbus::toString(A.type)
                                << " SlaveID=" << int(A.slaveId) 
                                << " RegCount=" << A.regCount 
                                << " RegAddr=" << A.regAddress 
                                << " ===\n";
                        printFrame(A, " A: ");
                        printFrame(B, " B: ");
                        std::cout << " raw: ";
                        for (auto b: raw) printf("%02X ", b);
                        std::cout << "\n\n";
                  }
                  TEST_ASSERT_TRUE_MESSAGE(compareFrames(A,B),"round-trip RTU");
              }
          }
        }
    }
}

void test_codec_tcp() {
    using namespace Modbus;
    using namespace ModbusCodec;

    std::cout << "\n=== DÉBUT DES TESTS TCP ===\n" << std::endl;

    // Liste des cas de test (réutilisation de la structure Case)
    std::vector<Case> cases;
    auto addCases = [&](MsgType t, FunctionCode fc, std::initializer_list<uint16_t> counts){
        for (uint8_t sid : {1, 0, 255}) {
            for (uint16_t addr : {0, 1, 100}) {
                for (auto cnt : counts) {
                    // Validité
                    bool validSid = ModbusCodec::isValidSlaveId(sid, fc, t);
                    bool validCnt = ModbusCodec::isValidRegisterCount(cnt, (uint8_t)fc, t);
                    
                    if (t == REQUEST) {
                        // Request valide
                        if (validSid && validCnt)
                            cases.push_back({t,fc,sid,addr,cnt,false});
                        // Request invalide avec ID invalide
                        if (!validSid)
                            cases.push_back({t,fc,sid,addr,cnt,false});
                    } else { // RESPONSE
                        // Pour les réponses, on n'ajoute que les cas avec ID valide
                        if (validSid && validCnt) {
                            cases.push_back({t,fc,sid,addr,cnt,false});
                            // Response exception (seulement pour les IDs valides)
                            cases.push_back({t,fc,sid,addr,cnt,true});
                        }
                    }
                }
            }
        }
    };

    // Ajout des cas de test (même que RTU)
    addCases(REQUEST,  READ_COILS,            {1,5,10});
    addCases(RESPONSE, READ_COILS,            {1,5,10});
    addCases(REQUEST,  READ_DISCRETE_INPUTS,  {1,5,10});
    addCases(RESPONSE, READ_DISCRETE_INPUTS,  {1,5,10});
    addCases(REQUEST,  READ_HOLDING_REGISTERS,{1,5,10});
    addCases(RESPONSE, READ_HOLDING_REGISTERS,{1,5,10});
    addCases(REQUEST,  READ_INPUT_REGISTERS,  {1,5,10});
    addCases(RESPONSE, READ_INPUT_REGISTERS,  {1,5,10});
    addCases(REQUEST,  WRITE_COIL,           {1});
    addCases(RESPONSE, WRITE_COIL,           {1});
    addCases(REQUEST,  WRITE_REGISTER,       {1});
    addCases(RESPONSE, WRITE_REGISTER,       {1});
    addCases(REQUEST,  WRITE_MULTIPLE_COILS, {1,5,10});
    addCases(RESPONSE, WRITE_MULTIPLE_COILS, {1,5,10});
    addCases(REQUEST,  WRITE_MULTIPLE_REGISTERS,{1,5,10});
    addCases(RESPONSE, WRITE_MULTIPLE_REGISTERS,{1,5,10});

    // ==== 1) CAS LIMITES MBAP ====
    {
        // Test avec protocol ID invalide
        Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        uint16_t txnId = 0x1234;
        ModbusCodec::TCP::encode(F, raw, txnId);
        raw.write_at(2, 0x12); // Corrompre le protocol ID
        raw.write_at(3, 0x34);
        Frame D;
        auto r = ModbusCodec::TCP::decode(raw, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_MBAP_PROTOCOL_ID, r, "should fail on invalid protocol ID");

        // Test avec length invalide
        F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
        raw.clear();
        ModbusCodec::TCP::encode(F, raw, txnId);
        raw.write_at(4, 0xFF); // Corrompre la longueur
        raw.write_at(5, 0xFF);
        r = ModbusCodec::TCP::decode(raw, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_MBAP_LEN, r, "should fail on invalid MBAP length");
    }

    // ==== 2) TRANSACTION ID ====
    {
        // Test de préservation du transaction ID
        Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        uint16_t txnId = 0x1234;
        ModbusCodec::TCP::encode(F, raw, txnId);
        uint16_t receivedTxnId = (raw[0] << 8) | raw[1];
        TEST_ASSERT_EQUAL_MESSAGE(txnId, receivedTxnId, "transaction ID should be preserved");
    }

    // ==== 3) TAILLE MINIMALE/MAXIMALE ====
    {
        // Test trame trop courte
        uint8_t _shortFrame[ModbusCodec::TCP::MIN_FRAME_SIZE - 1];
        ByteBuffer shortFrame(_shortFrame, sizeof(_shortFrame));
        Frame D;
        auto r = ModbusCodec::TCP::decode(shortFrame, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r, "should fail on too short frame");

        // Test trame trop longue
        uint8_t _longFrame[ModbusCodec::TCP::MAX_FRAME_SIZE + 1];
        ByteBuffer longFrame(_longFrame, sizeof(_longFrame));
        r = ModbusCodec::TCP::decode(longFrame, D, REQUEST);
        TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r, "should fail on too long frame");
    }

    // // ==== 4) UNIT ID ====
    // {
    //     // Test avec unit ID invalide pour une lecture
    //     Frame F = makeFrame(Case{REQUEST, READ_COILS, 0, 0, 1, false});
    //     std::vector<uint8_t> raw;
    //     uint16_t txnId = 0x1234;
    //     auto r = ModbusCodec::TCP::encode(F, raw, txnId);
    //     TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_SLAVEID, r, "should fail on invalid unit ID for read request");
    // }

    // ==== 5) BROADCAST RESPONSES ====
    {
        std::cout << "\n=== TEST BROADCAST RESPONSES ===" << std::endl;
        
        // Test avec broadcast ID en réponse
        for (uint8_t broadcastId : {0}) {
            std::cout << "\nTest avec broadcast ID: " << (int)broadcastId << std::endl;
            
            Frame F = makeFrame(Case{RESPONSE, READ_COILS, broadcastId, 0, 1, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            uint16_t txnId = 0x1234;
            
            std::cout << "Frame à encoder:" << std::endl;
            printFrame(F, "  ");
            
            auto r = ModbusCodec::TCP::encode(F, raw, txnId);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec encode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_SLAVEID, r,
                "should reject broadcast response at encode");
        }
    }

    // ==== 6) EXCEPTIONS VARIÉES ====
    {
        std::cout << "\n=== TEST EXCEPTIONS VARIÉES ===" << std::endl;
        
        std::array<ExceptionCode, 3> exceptions = {
            ILLEGAL_DATA_ADDRESS,
            ILLEGAL_DATA_VALUE,
            SLAVE_DEVICE_FAILURE
        };
        
        for (auto ec : exceptions) {
            std::cout << "\nTest avec exception: " << toString(ec) << std::endl;
            
            Frame F = makeFrame(Case{RESPONSE, READ_HOLDING_REGISTERS, 1, 0, 1, true});
            F.exceptionCode = ec;
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            uint16_t txnId = 0x1234;
            
            std::cout << "Frame à encoder:" << std::endl;
            printFrame(F, "  ");
            
            auto r = ModbusCodec::TCP::encode(F, raw, txnId);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec encode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, "should encode exception response");
            
            std::cout << "Trame encodée (" << raw.size() << " octets):" << std::endl;
            std::cout << "MBAP: ";
            for (size_t i = 0; i < ModbusCodec::TCP::MBAP_SIZE; i++) {
                printf("%02X ", raw[i]);
            }
            std::cout << "\nPDU:  ";
            for (size_t i = ModbusCodec::TCP::MBAP_SIZE; i < raw.size(); i++) {
                printf("%02X ", raw[i]);
            }
            std::cout << std::endl;
            
            Frame D;
            r = ModbusCodec::TCP::decode(raw, D, RESPONSE);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec decode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, "should decode exception response");
            
            if (D.exceptionCode != ec) {
                std::cout << "Exception code mismatch!" << std::endl;
                std::cout << "Expected: " << toString(ec) << std::endl;
                std::cout << "Got: " << toString(D.exceptionCode) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ec, D.exceptionCode, "should preserve exception code");
            
            uint8_t fc = raw[ModbusCodec::TCP::MBAP_SIZE];
            std::cout << "Function code in PDU: 0x" << std::hex << (int)fc << std::dec << std::endl;
            TEST_ASSERT_TRUE_MESSAGE((fc & 0x80) != 0, "should set FC bit 7 for exception");
        }
    }

    // ==== 7) BORNES REGCOUNT ====
    {
        std::cout << "\n=== TEST BORNES REGCOUNT ===" << std::endl;
        
        struct TestCase {
            FunctionCode fc;
            uint16_t regCount;
            bool shouldSucceed;
            const char* desc;
        };
        
        std::vector<TestCase> tests = {
            {READ_COILS, 0, false, "regCount = 0 (invalide)"},
            {READ_COILS, MAX_COILS_READ, true, "regCount = MAX_COILS_READ"},
            {READ_COILS, uint16_t(MAX_COILS_READ + 1), false, "regCount > MAX_COILS_READ"},
            {READ_HOLDING_REGISTERS, MAX_REGISTERS_READ, true, "regCount = MAX_REGISTERS_READ"},
            {READ_HOLDING_REGISTERS, uint16_t(MAX_REGISTERS_READ + 1), false, "regCount > MAX_REGISTERS_READ"}
        };
        
        for (const auto& test : tests) {
            std::cout << "\nTest " << test.desc << std::endl;
            
            Frame F = makeFrame(Case{REQUEST, test.fc, 1, 0, test.regCount, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            uint16_t txnId = 0x1234;
            
            std::cout << "Frame à encoder:" << std::endl;
            printFrame(F, "  ");
            
            auto r = ModbusCodec::TCP::encode(F, raw, txnId);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec encode(): " << ModbusCodec::toString(r) << std::endl;
            }
            if (test.shouldSucceed) {
                TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, test.desc);
            } else {
                TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, test.desc);
            }
        }
    }

    // ==== 8) MBAP ROUND-TRIP ====
    {
        // Test que le champ length est correctement calculé pour différents types de PDU
        struct TestCase {
            FunctionCode fc;
            uint16_t regCount;
            const char* desc;
        };
        
        std::vector<TestCase> mbapTests = {
            {READ_COILS, 1, "read single coil"},
            {READ_COILS, 10, "read multiple coils"},
            {READ_HOLDING_REGISTERS, 1, "read single register"},
            {READ_HOLDING_REGISTERS, 10, "read multiple registers"},
            {WRITE_MULTIPLE_REGISTERS, 5, "write multiple registers"}
        };

        for (const auto& test : mbapTests) {
            Frame F = makeFrame(Case{REQUEST, test.fc, 1, 0, test.regCount, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            uint16_t txnId = 0x1234;
            
            std::cout << "\n=== Test MBAP pour " << test.desc << " ===" << std::endl;
            std::cout << "Frame à encoder:" << std::endl;
            printFrame(F, "  ");
            
            auto r = ModbusCodec::TCP::encode(F, raw, txnId);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec encode(): " << ModbusCodec::toString(r) << std::endl;
                continue;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, test.desc);
            
            // Afficher la trame complète
            std::cout << "Trame encodée (" << raw.size() << " octets):" << std::endl;
            std::cout << "MBAP: ";
            for (size_t i = 0; i < ModbusCodec::TCP::MBAP_SIZE; i++) {
                printf("%02X ", raw[i]);
            }
            std::cout << "\nPDU:  ";
            for (size_t i = ModbusCodec::TCP::MBAP_SIZE; i < raw.size(); i++) {
                printf("%02X ", raw[i]);
            }
            std::cout << std::endl;
            
            // Vérifier le champ length du MBAP
            uint16_t mbapLength = (raw[4] << 8) | raw[5];
            size_t pduSize = raw.size() - ModbusCodec::TCP::MBAP_SIZE;
            std::cout << "MBAP length: " << mbapLength << std::endl;
            std::cout << "PDU size: " << pduSize << std::endl;
            std::cout << "Expected length: " << (pduSize + 1) << std::endl;
            std::cout << "Raw size: " << raw.size() << std::endl;
            std::cout << "MBAP_SIZE: " << ModbusCodec::TCP::MBAP_SIZE << std::endl;
            
            TEST_ASSERT_EQUAL_MESSAGE(pduSize + 1, mbapLength, 
                "MBAP length should be PDU size + 1 for unit ID");

            // Vérifier que decode() accepte cette longueur
            Frame D;
            std::cout << "\nDécodage de la trame..." << std::endl;
            r = ModbusCodec::TCP::decode(raw, D, REQUEST);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec decode(): " << ModbusCodec::toString(r) << std::endl;
                std::cout << "Taille totale: " << raw.size() << " octets" << std::endl;
                std::cout << "Taille PDU attendue: " << (mbapLength - 1) << " octets" << std::endl;
                std::cout << "Taille PDU réelle: " << pduSize << " octets" << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, r, 
                "should accept correct MBAP length");
            
            // Vérifier que la frame décodée correspond
            if (!compareFrames(F, D)) {
                std::cout << "Frame originale:" << std::endl;
                printFrame(F, "  ");
                std::cout << "Frame décodée:" << std::endl;
                printFrame(D, "  ");
            }
            TEST_ASSERT_TRUE_MESSAGE(compareFrames(F, D), "round-trip frame comparison");
        }
    }

    // ==== 9) BROADCAST EXCEPTIONS ====
    {
        std::cout << "\n=== TEST BROADCAST EXCEPTIONS ===" << std::endl;
        
        // Test qu'une réponse exception en broadcast est bien rejetée
        for (uint8_t broadcastId : {0, 255}) {
            std::cout << "\nTest avec broadcast ID: " << (int)broadcastId << std::endl;
            
            Frame F = makeFrame(Case{RESPONSE, READ_COILS, broadcastId, 0, 1, true});
            F.exceptionCode = ILLEGAL_DATA_ADDRESS;
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            uint16_t txnId = 0x1234;
            
            std::cout << "Frame à encoder:" << std::endl;
            printFrame(F, "  ");
            
            auto r = ModbusCodec::TCP::encode(F, raw, txnId);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec encode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_SLAVEID, r,
                "should reject broadcast response at encode");

            // On ne teste pas le decode() car si l'encode() échoue correctement,
            // il n'y a pas de raison de tester le décodage d'une trame invalide
        }
    }

    // ==== 10) TRAMES TCP MALFORMÉES ====
    {
        std::cout << "\n=== TEST TRAMES TCP MALFORMÉES ===" << std::endl;
        
        // MBAP header incomplet
        {
            std::cout << "\nTest MBAP header incomplet" << std::endl;
            uint8_t _raw[ModbusCodec::TCP::MBAP_SIZE - 1];
            ByteBuffer raw(_raw, sizeof(_raw));
            std::cout << "Taille trame: " << raw.size() << " octets (MBAP_SIZE-1)" << std::endl;
            
            Frame D;
            auto r = ModbusCodec::TCP::decode(raw, D, REQUEST);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec decode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r,
                "should reject incomplete MBAP header");
        }

        // PDU tronquée après le function code
        {
            std::cout << "\nTest PDU tronquée après FC" << std::endl;
            Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            ModbusCodec::TCP::encode(F, raw, 0x1234);
            
            std::cout << "Trame originale (" << raw.size() << " octets):" << std::endl;
            for (auto b: raw) printf("%02X ", b);
            std::cout << std::endl;
            
            raw.resize(ModbusCodec::TCP::MBAP_SIZE + 1);
            // Corriger la longueur dans le MBAP header pour qu'elle corresponde à la nouvelle taille
            raw.write_at(4, 0x00);  // Length high byte
            raw.write_at(5, 0x02);  // Length low byte (unit ID + FC = 2 bytes)
            
            std::cout << "Trame tronquée (" << raw.size() << " octets):" << std::endl;
            for (auto b: raw) printf("%02X ", b);
            std::cout << std::endl;
            
            Frame D;
            auto r = ModbusCodec::TCP::decode(raw, D, REQUEST);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec decode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r,
                "should reject truncated PDU");
        }

        // PDU tronquée au milieu des données
        {
            std::cout << "\nTest PDU tronquée au milieu des données" << std::endl;
            Frame F = makeFrame(Case{REQUEST, WRITE_MULTIPLE_REGISTERS, 1, 0, 5, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            ModbusCodec::TCP::encode(F, raw, 0x1234);
            
            std::cout << "Trame originale (" << raw.size() << " octets):" << std::endl;
            for (auto b: raw) printf("%02X ", b);
            std::cout << std::endl;
            
            raw.resize(raw.size() - 3);
            // Corriger la longueur dans le MBAP header pour qu'elle corresponde à la nouvelle taille
            uint16_t newLength = raw.size() - ModbusCodec::TCP::MBAP_SIZE + 1; // +1 pour unit ID
            raw.write_at(4, (newLength >> 8) & 0xFF);    // Length high byte
            raw.write_at(5, newLength & 0xFF);           // Length low byte
            
            std::cout << "Trame tronquée (" << raw.size() << " octets):" << std::endl;
            for (auto b: raw) printf("%02X ", b);
            std::cout << std::endl;
            
            Frame D;
            auto r = ModbusCodec::TCP::decode(raw, D, REQUEST);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec decode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_LEN, r,
                "should reject truncated data");
        }

        // Length MBAP incohérente
        {
            std::cout << "\nTest length MBAP incohérente" << std::endl;
            Frame F = makeFrame(Case{REQUEST, READ_COILS, 1, 0, 1, false});
            uint8_t _raw[256];
            ByteBuffer raw(_raw, sizeof(_raw));
            ModbusCodec::TCP::encode(F, raw, 0x1234);
            
            std::cout << "Trame originale (" << raw.size() << " octets):" << std::endl;
            for (auto b: raw) printf("%02X ", b);
            std::cout << std::endl;
            
            raw.write_at(4, 0xFF);
            raw.write_at(5, 0xFF);
            std::cout << "Trame corrompue (length = 0xFFFF):" << std::endl;
            for (auto b: raw) printf("%02X ", b);
            std::cout << std::endl;
            
            Frame D;
            auto r = ModbusCodec::TCP::decode(raw, D, REQUEST);
            if (r != ModbusCodec::SUCCESS) {
                std::cout << "Échec decode(): " << ModbusCodec::toString(r) << std::endl;
            }
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::ERR_INVALID_MBAP_LEN, r,
                "should reject inconsistent MBAP length");
        }
    }

    // Itération sur tous les cas de test
    std::cout << "\n=== TESTS GÉNÉRIQUES ===" << std::endl;
    
    for (auto& C : cases) {
        Frame A = makeFrame(C);
        std::cout << "\nTest générique pour:" << std::endl;
        std::cout << "Type: " << toString(C.type) << std::endl;
        std::cout << "FC: " << toString(C.fc) << " (0x" << std::hex << (int)C.fc << ")" << std::dec << std::endl;
        std::cout << "UnitID: " << (int)C.slaveId << std::endl;
        std::cout << "RegAddr: " << C.regAddress << std::endl;
        std::cout << "RegCount: " << C.regCount << std::endl;
        std::cout << "IsException: " << (C.isException ? "true" : "false") << std::endl;
        
        uint8_t _raw[256];
        ByteBuffer raw(_raw, sizeof(_raw));
        uint16_t txnId = 0x1234;
        
        auto r = ModbusCodec::TCP::encode(A, raw, txnId);
        if (r != ModbusCodec::SUCCESS) {
            std::cout << "Échec encode(): " << ModbusCodec::toString(r) << std::endl;
        }
        
        if (C.type==REQUEST && !ModbusCodec::isValidSlaveId(C.slaveId,(uint8_t)C.fc,C.type,true))
            TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"should fail bad slaveId");
        else if (C.type==REQUEST && !ModbusCodec::isValidRegisterCount(C.regCount,(uint8_t)C.fc,C.type))
            TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"should fail bad regCount");
        else if (C.type==RESPONSE && (C.slaveId == 0 || C.slaveId == 255))
            TEST_ASSERT_NOT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"should fail broadcast response");
        else {
            TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS,r,"TCP encode");
            
            if (r == SUCCESS) {
                std::cout << "Trame encodée (" << raw.size() << " octets):" << std::endl;
                std::cout << "MBAP: ";
                for (size_t i = 0; i < ModbusCodec::TCP::MBAP_SIZE; i++) {
                    printf("%02X ", raw[i]);
                }
                std::cout << "\nPDU:  ";
                for (size_t i = ModbusCodec::TCP::MBAP_SIZE; i < raw.size(); i++) {
                    printf("%02X ", raw[i]);
                }
                std::cout << std::endl;
            }
            
            Frame B;
            auto rd = ModbusCodec::TCP::decode(raw,B,C.type);
            if (rd != ModbusCodec::SUCCESS) {
                std::cout << "Échec decode(): " << ModbusCodec::toString(rd) << std::endl;
            }
            
            // exception vs normal
            if (C.isException) {
                TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, rd,"exception responses decode OK");
                TEST_ASSERT_EQUAL_MESSAGE(A.exceptionCode,B.exceptionCode,"exceptionCode");
            } else {
                TEST_ASSERT_EQUAL_MESSAGE(ModbusCodec::SUCCESS, rd,"TCP decode");
                if (rd == SUCCESS && !C.isException && !compareFrames(A,B)) {
                    std::cout << "Échec round-trip!" << std::endl;
                    std::cout << "Frame originale:" << std::endl;
                    printFrame(A, "  ");
                    std::cout << "Frame décodée:" << std::endl;
                    printFrame(B, "  ");
                }
                TEST_ASSERT_TRUE_MESSAGE(compareFrames(A,B),"round-trip TCP");
            }
        }
    }
    
    std::cout << "\n=== FIN DES TESTS TCP ===\n" << std::endl;
}

// ===================================================================================
// DATA TYPE CONVERSION TESTS
// ===================================================================================

void test_conversion_float_operations() {
    using namespace Modbus;
    
    Frame frame;
    float testValue = 123.456f;
    float result = 0.0f;
    
    // Test ABCD (Big Endian)
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(testValue, 0, ByteOrder::ABCD), "setFloat ABCD should return 2");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.regCount, "regCount should be auto-incremented to 2");
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(result, 0, ByteOrder::ABCD), "getFloat ABCD should succeed");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, testValue, result, "Float round-trip ABCD");
    
    // Test CDAB (Word Swap - very common in Modbus)
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(testValue, 0, ByteOrder::CDAB), "setFloat CDAB should return 2");
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(result, 0, ByteOrder::CDAB), "getFloat CDAB should succeed");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, testValue, result, "Float round-trip CDAB");
    
    // Test BADC (Byte + Word Swap)
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(testValue, 0, ByteOrder::BADC), "setFloat BADC should return 2");
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(result, 0, ByteOrder::BADC), "getFloat BADC should succeed");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, testValue, result, "Float round-trip BADC");
    
    // Test DCBA (Little Endian)
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(testValue, 0, ByteOrder::DCBA), "setFloat DCBA should return 2");
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(result, 0, ByteOrder::DCBA), "getFloat DCBA should succeed");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, testValue, result, "Float round-trip DCBA");
}

void test_conversion_uint32_operations() {
    using namespace Modbus;
    
    Frame frame;
    uint32_t testValue = 0x12345678;
    uint32_t result = 0;
    
    // Test ABCD
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(testValue, 0, ByteOrder::ABCD), "setUint32 ABCD should return 2");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(result, 0, ByteOrder::ABCD), "getUint32 ABCD should succeed");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(testValue, result, "Uint32 round-trip ABCD");
    
    // Test CDAB - verify raw data is swapped correctly
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(testValue, 0, ByteOrder::CDAB), "setUint32 CDAB should return 2");
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0x5678, frame.data[0], "CDAB word1 should be low word");
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0x1234, frame.data[1], "CDAB word2 should be high word");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(result, 0, ByteOrder::CDAB), "getUint32 CDAB should succeed");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(testValue, result, "Uint32 round-trip CDAB");
    
    // Test BADC
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(testValue, 0, ByteOrder::BADC), "setUint32 BADC should return 2");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(result, 0, ByteOrder::BADC), "getUint32 BADC should succeed");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(testValue, result, "Uint32 round-trip BADC");
    
    // Test DCBA
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(testValue, 0, ByteOrder::DCBA), "setUint32 DCBA should return 2");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(result, 0, ByteOrder::DCBA), "getUint32 DCBA should succeed");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(testValue, result, "Uint32 round-trip DCBA");
}

void test_conversion_int32_operations() {
    using namespace Modbus;
    
    Frame frame;
    int32_t testValue = -123456789;
    int32_t result = 0;
    
    // Test all byte orders
    ByteOrder orders[] = {ByteOrder::ABCD, ByteOrder::CDAB, ByteOrder::BADC, ByteOrder::DCBA};
    
    for (size_t i = 0; i < 4; ++i) {
        frame.clearData();
        TEST_ASSERT_EQUAL_MESSAGE(2, frame.setInt32(testValue, 0, orders[i]), "setInt32 should return 2");
        TEST_ASSERT_TRUE_MESSAGE(frame.getInt32(result, 0, orders[i]), "getInt32 should succeed");
        TEST_ASSERT_EQUAL_INT32_MESSAGE(testValue, result, "Int32 round-trip");
    }
}

void test_conversion_uint16_operations() {
    using namespace Modbus;
    
    Frame frame;
    uint16_t testValue = 0xABCD;
    uint16_t result = 0;
    
    // Test AB (Big Endian)
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setUint16(testValue, 0, ByteOrder::AB), "setUint16 AB should return 1");
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.regCount, "regCount should be auto-incremented to 1");
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0xABCD, frame.data[0], "AB should preserve byte order");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint16(result, 0, ByteOrder::AB), "getUint16 AB should succeed");
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(testValue, result, "Uint16 round-trip AB");
    
    // Test BA (Little Endian)
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setUint16(testValue, 0, ByteOrder::BA), "setUint16 BA should return 1");
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0xCDAB, frame.data[0], "BA should swap bytes");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint16(result, 0, ByteOrder::BA), "getUint16 BA should succeed");
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(testValue, result, "Uint16 round-trip BA");
}

void test_conversion_int16_operations() {
    using namespace Modbus;
    
    Frame frame;
    int16_t testValue = -12345;
    int16_t result = 0;
    
    // Test AB
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setInt16(testValue, 0, ByteOrder::AB), "setInt16 AB should return 1");
    TEST_ASSERT_TRUE_MESSAGE(frame.getInt16(result, 0, ByteOrder::AB), "getInt16 AB should succeed");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(testValue, result, "Int16 round-trip AB");
    
    // Test BA
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setInt16(testValue, 0, ByteOrder::BA), "setInt16 BA should return 1");
    TEST_ASSERT_TRUE_MESSAGE(frame.getInt16(result, 0, ByteOrder::BA), "getInt16 BA should succeed");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(testValue, result, "Int16 round-trip BA");
}

void test_conversion_regcount_auto_increment() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    
    // Initially regCount should be 0
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.regCount, "Initial regCount should be 0");
    
    // Set float at index 2 - should update regCount to 4
    frame.setFloat(123.45f, 2, ByteOrder::ABCD);
    TEST_ASSERT_EQUAL_MESSAGE(4, frame.regCount, "regCount should be 4 after setFloat at index 2");
    
    // Set uint16 at index 5 - should update regCount to 6
    frame.setUint16(0x1234, 5, ByteOrder::AB);
    TEST_ASSERT_EQUAL_MESSAGE(6, frame.regCount, "regCount should be 6 after setUint16 at index 5");
    
    // Set uint32 at index 0 - should not change regCount (still 6)
    frame.setUint32(0x12345678, 0, ByteOrder::ABCD);
    TEST_ASSERT_EQUAL_MESSAGE(6, frame.regCount, "regCount should remain 6 when setting at lower index");
}

void test_conversion_boundary_conditions() {
    using namespace Modbus;
    
    Frame frame;
    
    // Test writing at the edge of the buffer
    frame.clearData();
    
    // Should succeed - last 2 registers
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(0x12345678, FRAME_DATASIZE - 2), "Should succeed at buffer edge");
    
    // Should fail - would exceed buffer
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setUint32(0x12345678, FRAME_DATASIZE - 1), "Should fail when exceeding buffer");
    
    // Should fail - would exceed buffer
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setFloat(123.45f, FRAME_DATASIZE), "Should fail when at buffer limit");
    
    // Test reading beyond regCount
    frame.clearData();
    frame.setUint32(0x12345678, 0);  // Sets regCount to 2
    
    uint32_t result;
    // Should fail - trying to read beyond regCount
    TEST_ASSERT_FALSE_MESSAGE(frame.getUint32(result, 2), "Should fail when reading beyond regCount");
}

void test_conversion_mixed_data_types() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    
    // Build a frame with mixed data types
    float floatVal = 98.765f;
    uint32_t uint32Val = 0xDEADBEEF;
    int16_t int16Val = -999;
    
    // Set values at different positions (no order constraint!)
    size_t totalRegs = 0;
    totalRegs += frame.setFloat(floatVal, 0, ByteOrder::CDAB);      // Registers 0-1
    totalRegs += frame.setUint32(uint32Val, 2, ByteOrder::CDAB);    // Registers 2-3
    totalRegs += frame.setInt16(int16Val, 4, ByteOrder::AB);        // Register 4
    
    TEST_ASSERT_EQUAL_MESSAGE(5, totalRegs, "Total registers set should be 5");
    TEST_ASSERT_EQUAL_MESSAGE(5, frame.regCount, "regCount should be 5");
    
    // Read values back
    float readFloat;
    uint32_t readUint32;
    int16_t readInt16;
    
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(readFloat, 0, ByteOrder::CDAB), "getFloat should succeed");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(readUint32, 2, ByteOrder::CDAB), "getUint32 should succeed");
    TEST_ASSERT_TRUE_MESSAGE(frame.getInt16(readInt16, 4, ByteOrder::AB), "getInt16 should succeed");
    
    // Verify values
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, floatVal, readFloat, "Float value should match");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(uint32Val, readUint32, "Uint32 value should match");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(int16Val, readInt16, "Int16 value should match");
}

void test_conversion_overwrite_scenarios() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    
    // Test that we can overwrite values (no anti-overwrite protection)
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(0x11111111, 0, ByteOrder::ABCD), "First setUint32 should succeed");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(0x22222222, 0, ByteOrder::ABCD), "Second setUint32 should succeed (overwrite)");
    
    uint32_t result;
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(result, 0, ByteOrder::ABCD), "getUint32 should succeed");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(0x22222222, result, "Should get overwritten value");
    
    // Test partial overlap
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(0x33333333, 1, ByteOrder::ABCD), "Partial overlap should succeed");
    
    // Test that we can set values in any order
    frame.clearData();
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setUint16(0x1234, 5, ByteOrder::AB), "Set at index 5 first");
    TEST_ASSERT_EQUAL_MESSAGE(6, frame.regCount, "regCount should be 6");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(123.45f, 2, ByteOrder::ABCD), "Set at index 2 second");
    TEST_ASSERT_EQUAL_MESSAGE(6, frame.regCount, "regCount should remain 6");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(0xABCDEF01, 0, ByteOrder::CDAB), "Set at index 0 last");
    TEST_ASSERT_EQUAL_MESSAGE(6, frame.regCount, "regCount should remain 6");
}

void test_conversion_extreme_values() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    
    // Float extremes
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(FLT_MAX, 0), "Should handle FLT_MAX");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(FLT_MIN, 2), "Should handle FLT_MIN");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(-FLT_MAX, 4), "Should handle -FLT_MAX");
    
    // NaN and infinity
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(NAN, 6), "Should handle NaN");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(INFINITY, 8), "Should handle INFINITY");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(-INFINITY, 10), "Should handle -INFINITY");
    
    // Integer extremes
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setUint32(UINT32_MAX, 12), "Should handle UINT32_MAX");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setInt32(INT32_MIN, 14), "Should handle INT32_MIN");
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setInt32(INT32_MAX, 16), "Should handle INT32_MAX");
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setUint16(UINT16_MAX, 18), "Should handle UINT16_MAX");
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setInt16(INT16_MIN, 19), "Should handle INT16_MIN");
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setInt16(INT16_MAX, 20), "Should handle INT16_MAX");
    
    // Verify we can read back the values
    float floatResult;
    uint32_t uint32Result;
    int32_t int32Result;
    uint16_t uint16Result;
    int16_t int16Result;
    
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(floatResult, 0), "Should read FLT_MAX");
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(FLT_MAX, floatResult, "FLT_MAX should match");
    
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(uint32Result, 12), "Should read UINT32_MAX");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(UINT32_MAX, uint32Result, "UINT32_MAX should match");
    
    TEST_ASSERT_TRUE_MESSAGE(frame.getInt16(int16Result, 20), "Should read INT16_MAX");
    TEST_ASSERT_EQUAL_MESSAGE(INT16_MAX, int16Result, "INT16_MAX should match");
}

void test_conversion_invalid_parameters() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    
    // Invalid ByteOrder values (cast from invalid integers)
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setFloat(123.45f, 0, static_cast<ByteOrder>(99)), "Should reject invalid ByteOrder");
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setUint32(12345, 0, static_cast<ByteOrder>(-1)), "Should reject negative ByteOrder");
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setUint16(100, 0, static_cast<ByteOrder>(50)), "Should reject invalid 16-bit ByteOrder");
    
    // Invalid reading with bad ByteOrder
    float floatResult;
    uint32_t uint32Result;
    uint16_t uint16Result;
    
    TEST_ASSERT_FALSE_MESSAGE(frame.getFloat(floatResult, 0, static_cast<ByteOrder>(99)), "Should reject invalid ByteOrder for getFloat");
    TEST_ASSERT_FALSE_MESSAGE(frame.getUint32(uint32Result, 0, static_cast<ByteOrder>(-1)), "Should reject invalid ByteOrder for getUint32");
    TEST_ASSERT_FALSE_MESSAGE(frame.getUint16(uint16Result, 0, static_cast<ByteOrder>(50)), "Should reject invalid ByteOrder for getUint16");
}

void test_conversion_endianness_consistency() {
    using namespace Modbus;
    
    // Test that set + get with same ByteOrder is consistent
    Frame frame;
    
    // Test all valid ByteOrder values for 32-bit types
    ByteOrder orders32[] = {ByteOrder::ABCD, ByteOrder::CDAB, ByteOrder::BADC, ByteOrder::DCBA};
    for (auto order : orders32) {
        frame.clearData();
        
        // Float consistency
        float originalFloat = 123.456789f;
        if (frame.setFloat(originalFloat, 0, order) == 2) {
            float readFloat;
            TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(readFloat, 0, order), "Should read back float with same endianness");
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, originalFloat, readFloat, "Float roundtrip should be accurate");
        }
        
        // uint32_t consistency
        uint32_t originalUint32 = 0x12345678;
        if (frame.setUint32(originalUint32, 2, order) == 2) {
            uint32_t readUint32;
            TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(readUint32, 2, order), "Should read back uint32 with same endianness");
            TEST_ASSERT_EQUAL_HEX32_MESSAGE(originalUint32, readUint32, "uint32 roundtrip should be exact");
        }
        
        // int32_t consistency
        int32_t originalInt32 = -0x12345678;
        if (frame.setInt32(originalInt32, 4, order) == 2) {
            int32_t readInt32;
            TEST_ASSERT_TRUE_MESSAGE(frame.getInt32(readInt32, 4, order), "Should read back int32 with same endianness");
            TEST_ASSERT_EQUAL_MESSAGE(originalInt32, readInt32, "int32 roundtrip should be exact");
        }
    }
    
    // Test 16-bit types
    ByteOrder orders16[] = {ByteOrder::AB, ByteOrder::BA};
    for (auto order : orders16) {
        frame.clearData();
        
        uint16_t originalUint16 = 0x1234;
        if (frame.setUint16(originalUint16, 0, order) == 1) {
            uint16_t readUint16;
            TEST_ASSERT_TRUE_MESSAGE(frame.getUint16(readUint16, 0, order), "Should read back uint16 with same endianness");
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(originalUint16, readUint16, "uint16 roundtrip should be exact");
        }
        
        int16_t originalInt16 = -0x1234;
        if (frame.setInt16(originalInt16, 1, order) == 1) {
            int16_t readInt16;
            TEST_ASSERT_TRUE_MESSAGE(frame.getInt16(readInt16, 1, order), "Should read back int16 with same endianness");
            TEST_ASSERT_EQUAL_MESSAGE(originalInt16, readInt16, "int16 roundtrip should be exact");
        }
    }
}

void test_conversion_capacity_limits() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    
    const size_t maxRegs = 125; // FRAME_DATASIZE
    
    // Test at exact limit
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(1.0f, maxRegs-2), "Should set float at position 123-124");
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setFloat(2.0f, maxRegs-1), "Should reject float at position 124-125 (overflows)");
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setUint32(123, maxRegs), "Should reject uint32 at position 125+ (out of bounds)");
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setUint32(456, maxRegs+10), "Should reject uint32 way out of bounds");
    
    // Test 16-bit types at limit
    TEST_ASSERT_EQUAL_MESSAGE(1, frame.setUint16(100, maxRegs-1), "Should set uint16 at last position");
    TEST_ASSERT_EQUAL_MESSAGE(0, frame.setUint16(200, maxRegs), "Should reject uint16 past end");
    
    // Verify reading at limits
    float floatResult;
    uint16_t uint16Result;
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(floatResult, maxRegs-2), "Should read float at limit");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint16(uint16Result, maxRegs-1), "Should read uint16 at limit");
    TEST_ASSERT_FALSE_MESSAGE(frame.getFloat(floatResult, maxRegs-1), "Should reject reading float past limit");
    TEST_ASSERT_FALSE_MESSAGE(frame.getUint16(uint16Result, maxRegs), "Should reject reading uint16 past limit");
}

void test_conversion_insufficient_data() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    frame.regCount = 3; // Only 3 registers of data
    
    float floatResult;
    uint32_t uint32Result;
    uint16_t uint16Result;
    
    // Attempts to read beyond available data
    TEST_ASSERT_FALSE_MESSAGE(frame.getFloat(floatResult, 2), "Should reject float at reg 2-3 with regCount=3");
    TEST_ASSERT_FALSE_MESSAGE(frame.getUint32(uint32Result, 2), "Should reject uint32 at reg 2-3 with regCount=3");
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint16(uint16Result, 2), "Should accept uint16 at reg 2 with regCount=3");
    TEST_ASSERT_FALSE_MESSAGE(frame.getUint16(uint16Result, 3), "Should reject uint16 at reg 3 with regCount=3");
    
    // Test with regCount = 0
    frame.regCount = 0;
    TEST_ASSERT_FALSE_MESSAGE(frame.getFloat(floatResult, 0), "Should reject any read with regCount=0");
    TEST_ASSERT_FALSE_MESSAGE(frame.getUint16(uint16Result, 0), "Should reject any read with regCount=0");
}

void test_conversion_mixed_with_raw_api() {
    using namespace Modbus;
    
    Frame frame;
    frame.clearData();
    
    // Set raw registers first
    std::vector<uint16_t> rawData = {0x1234, 0x5678, 0xABCD, 0xEF01};
    frame.setRegisters(rawData);
    
    // Read with conversion API
    uint32_t value;
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(value, 0, ByteOrder::ABCD), "Should read raw data with conversion API");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(0x12345678, value, "Should get correct ABCD conversion");
    
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(value, 0, ByteOrder::CDAB), "Should read raw data with word swap");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(0x56781234, value, "Should get correct CDAB conversion");
    
    // Overwrite with conversion API
    TEST_ASSERT_EQUAL_MESSAGE(2, frame.setFloat(999.0f, 2, ByteOrder::CDAB), "Should overwrite with conversion API");
    
    // Verify that first two registers are intact
    TEST_ASSERT_TRUE_MESSAGE(frame.getUint32(value, 0, ByteOrder::ABCD), "Should still read first registers");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(0x12345678, value, "First registers should be unchanged");
    
    // Verify mixed access works
    uint16_t rawReg = frame.getRegister(0);
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0x1234, rawReg, "Raw register access should work after conversion operations");
    
    float floatResult;
    TEST_ASSERT_TRUE_MESSAGE(frame.getFloat(floatResult, 2, ByteOrder::CDAB), "Should read back converted float");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1f, 999.0f, floatResult, "Float should be accurate");
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_codec_rtu);
    RUN_TEST(test_codec_tcp);
    RUN_TEST(test_conversion_float_operations);
    RUN_TEST(test_conversion_uint32_operations);
    RUN_TEST(test_conversion_int32_operations);
    RUN_TEST(test_conversion_uint16_operations);
    RUN_TEST(test_conversion_int16_operations);
    RUN_TEST(test_conversion_regcount_auto_increment);
    RUN_TEST(test_conversion_boundary_conditions);
    RUN_TEST(test_conversion_mixed_data_types);
    RUN_TEST(test_conversion_overwrite_scenarios);
    RUN_TEST(test_conversion_extreme_values);
    RUN_TEST(test_conversion_invalid_parameters);
    RUN_TEST(test_conversion_endianness_consistency);
    RUN_TEST(test_conversion_capacity_limits);
    RUN_TEST(test_conversion_insufficient_data);
    RUN_TEST(test_conversion_mixed_with_raw_api);
    UNITY_END();
}