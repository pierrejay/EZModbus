#include <unity.h>
#include <iostream>
#include <string>

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
    F.clearData();  // On s'assure que data est propre

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
            F.clearData();  // Utilisation de clearData() au lieu de clear()
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
            F.clearData();  // Utilisation de clearData() au lieu de clear()
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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_codec_rtu);
    RUN_TEST(test_codec_tcp);
    UNITY_END();
}