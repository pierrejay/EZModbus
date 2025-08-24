# Guide d'intégration - Utilitaires Multi-Word avec Endianness

## Objectif
Ajouter des utilitaires pour manipuler des types multi-word (float, uint32, int32, etc.) avec gestion de l'endianness dans `ModbusFrame.hpp`, sans impacter l'API existante.

## Spécifications

### Types à supporter (Tier 1)
- **32-bit** : `float`, `uint32_t`, `int32_t` (2 registres)  
- **16-bit** : `uint16_t`, `int16_t` (1 registre avec endianness)

### Ordres d'octets supportés
```cpp
namespace Modbus {
    enum class ByteOrder {
        // 16-bit (1 registre)
        AB,          // Big endian (défaut)
        BA,          // Little endian
        
        // 32-bit (2 registres)  
        ABCD,        // Big endian (défaut)
        CDAB,        // Word swap (très courant)
        BADC,        // Byte + word swap
        DCBA         // Little endian
    };
}
```

## API à ajouter dans `struct Frame`

### Setters (retournent nombre de registres settés, 0 = erreur)
```cpp
// 32-bit types (2 registres)
size_t setFloat(float value, size_t regIndex, ByteOrder order = ByteOrder::ABCD);
size_t setUint32(uint32_t value, size_t regIndex, ByteOrder order = ByteOrder::ABCD);  
size_t setInt32(int32_t value, size_t regIndex, ByteOrder order = ByteOrder::ABCD);

// 16-bit types (1 registre avec endianness)
size_t setUint16(uint16_t value, size_t regIndex, ByteOrder order = ByteOrder::AB);
size_t setInt16(int16_t value, size_t regIndex, ByteOrder order = ByteOrder::AB);
```

### Getters (retournent bool success, valeur par référence)
```cpp
// 32-bit types
bool getFloat(float& result, size_t regIndex, ByteOrder order = ByteOrder::ABCD) const;
bool getUint32(uint32_t& result, size_t regIndex, ByteOrder order = ByteOrder::ABCD) const;
bool getInt32(int32_t& result, size_t regIndex, ByteOrder order = ByteOrder::ABCD) const;

// 16-bit types  
bool getUint16(uint16_t& result, size_t regIndex, ByteOrder order = ByteOrder::AB) const;
bool getInt16(int16_t& result, size_t regIndex, ByteOrder order = ByteOrder::AB) const;
```

## Comportement des setters

### Auto-incrémentation de regCount
- Chaque setter met à jour automatiquement `regCount = max(regCount, regIndex + nbRegistres)`
- `setFloat(value, 2)` → `regCount` devient au minimum 4
- `setUint16(value, 5)` → `regCount` devient au minimum 6

### Validation anti-écrasement (supprimé)
- **IMPORTANT** : Vérifier qu'aucun registre dans la plage n'est déjà utilisé (≠ 0)
- Retourner 0 si tentative d'écrasement détectée
- Toujours faire `clearData()` avant construction d'une nouvelle frame
=> Exigence supprimée car impossible de savoir si un registre individuel est set/unset. Si on se base sur le regCount au moment de l'appel, on risque d'obliger l'utilisateur à setter les registres dans le bon ordre. Conclusion : ajouter à la documentation qu'il y a un risque d'écrasement si on exécute les setters sur des registres déjà settés.

### Implémentation helper
```cpp
private:
    bool checkDataBounds(size_t startIdx, size_t regCount) const;
```

## Exemple d'implémentation (setUint32)

```cpp
inline size_t Frame::setUint32(uint32_t value, size_t regIndex, ByteOrder order) {
    // Vérification limites
    if (regIndex + 2 > FRAME_DATASIZE) return 0;
    
    // Anti-écrasement (supprimé)
    // if (!checkDataBounds(regIndex, 2)) return 0;
    
    // Conversion endianness
    uint16_t word1, word2;
    switch (order) {
        case ByteOrder::ABCD:
            word1 = (value >> 16) & 0xFFFF;
            word2 = value & 0xFFFF;
            break;
        case ByteOrder::CDAB:
            word1 = value & 0xFFFF;
            word2 = (value >> 16) & 0xFFFF;
            break;
        case ByteOrder::BADC:
            word1 = __builtin_bswap16((value >> 16) & 0xFFFF);
            word2 = __builtin_bswap16(value & 0xFFFF);
            break;
        case ByteOrder::DCBA:
            word1 = __builtin_bswap16(value & 0xFFFF);
            word2 = __builtin_bswap16((value >> 16) & 0xFFFF);
            break;
        default: return 0;
    }
    
    // Stockage
    data[regIndex] = word1;
    data[regIndex + 1] = word2;
    
    // Auto-incrémentation regCount
    regCount = std::max(regCount, uint16_t(regIndex + 2));
    
    return 2; // 2 registres settés
}
```

## Usage côté développeur

### Construction sécurisée
```cpp
Modbus::Frame request;
request.clearData(); // ⚠️ OBLIGATOIRE avant construction

size_t totalRegs = 0;
totalRegs += request.setFloat(123.45f, 0, Modbus::ByteOrder::CDAB);    // regCount → 2
totalRegs += request.setUint32(67890, 2, Modbus::ByteOrder::CDAB);     // regCount → 4  
totalRegs += request.setInt16(999, 4, Modbus::ByteOrder::AB);          // regCount → 5

if (totalRegs != 5) {
    // Erreur construction
    return ERR_INVALID_FRAME;
}
```

### Lecture sécurisée  
```cpp
float voltage;
uint32_t current;
int16_t status;

if (response.getFloat(voltage, 0, Modbus::ByteOrder::CDAB) &&
    response.getUint32(current, 2, Modbus::ByteOrder::CDAB) &&
    response.getInt16(status, 4, Modbus::ByteOrder::AB)) {
    
    // Toutes les valeurs sont valides
    processData(voltage, current, status);
} else {
    // Erreur parsing - données insuffisantes ou index invalide
    return ERR_INVALID_RESPONSE;
}
```

## Points de validation

1. **Garder l'API existante** intacte (backward compatibility)
2. **Placer le ByteOrder enum** dans le namespace `Modbus`
3. **Implémenter checkDataBounds()** pour la sécurité
4. **Utiliser __builtin_bswap16()** pour les swaps d'octets (GCC/Clang)
5. **Tester avec différents cas** : limites, écrasement, endianness
6. **Documentation** : Mentionner l'obligation de `clearData()`

## 🔮 Extensions futures (pas pour cette version)
- Tier 2 : `double`, `uint64_t`, `int64_t` (4 registres)
- Tier 3 : `uint8_t`, `int8_t` (parties de registre)

---

**L'objectif est de créer une API intuitive et sécurisée qui simplifie la vie des développeurs travaillant avec des devices Modbus aux endianness variées, tout en conservant la philosophie KISS d'EZModbus.**