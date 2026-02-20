/**
 * @file ModbusWord.hpp
 * @brief Modbus::Word type def & WordStore classes def/impl
 */

#pragma once

#include "core/ModbusCore.h"

namespace Modbus {

// ===================================================================================
// MODBUS WORD TYPE DEFINITION
// ===================================================================================

struct Word; // Forward declaration

// WORD HANDLERS - Function pointer types for maximum performance (no std::function overhead)
// READ: takes the word context, output array, and optional user context, returns exception code (NULL_EXCEPTION = success)
using ReadWordHandler = Modbus::ExceptionCode (*)(const Word&, uint16_t* outVals, void* userCtx);
// WRITE: takes the input array, word context, and optional user context, returns exception code (NULL_EXCEPTION = success)
using WriteWordHandler = Modbus::ExceptionCode (*)(const uint16_t* writeVals, const Word&, void* userCtx);


/* @brief Public API structure to define a word (group of registers treated atomically)
    * @note Words are processed atomically - either the entire range is read/written or the operation fails
    * @note Words have priority over individual registers when address ranges overlap
    * @note For single-register Words (nbRegs = 1): can use direct pointer access OR handlers
    * @note For multi-register Words (nbRegs > 1): must use handlers only (thread safety requirement)
    */
struct Word {
    // Word metadata
    Modbus::RegisterType type = Modbus::NULL_RT;   // Type of registers in this word
    uint16_t startAddr = 0;                        // Starting address of the word
    uint16_t nbRegs = 0;                           // Number of registers in this word

    // Word value access (for single-register Words only)
    volatile uint16_t* value = nullptr;            // Direct pointer access for performance (nbRegs = 1 only)

    // Word handlers (atomic read/write operations)
    ReadWordHandler readHandler = nullptr;         // Read handler for the entire word
    WriteWordHandler writeHandler = nullptr;       // Write handler for the entire word
    void* userCtx = nullptr;                       // Optional user context passed to handlers

    operator bool() const {
        return Modbus::isValid(type) && nbRegs > 0;  // A word is valid if its type is valid and has registers
    }
};


// ===================================================================================
// MODBUS WORDSTORE ABSTRACT BASE CLASS
// ===================================================================================

class Server; // Forward declaration

/**
 * @brief Abstract interface for unified Word storage
 * @note Manages Words for all 4 RegisterTypes in a single store
 */
class IWordStore {
public:
    virtual ~IWordStore() = default;

    // Only Server may invoke the mutating/query interface
    friend class Server;

protected:
    // Memory management
    virtual void reserve(Modbus::RegisterType type, size_t count) = 0;
    virtual void clear(Modbus::RegisterType type) = 0;
    virtual void clearAll() = 0;

    // Word search & access  
    virtual Word* findExact(Modbus::RegisterType type, uint16_t address) = 0;
    virtual Word* findNext(Modbus::RegisterType type, uint16_t address) = 0;
    virtual Word* findContaining(Modbus::RegisterType type, uint16_t address) = 0;

    // Word management
    virtual bool overlaps(const Word& word) const = 0;
    virtual bool insert(const Word& word) = 0;  // Returns success/failure instead of void

    // Utility
    virtual size_t size(Modbus::RegisterType type) const = 0;
    virtual bool hasCapacity(Modbus::RegisterType type) const = 0;
    // Global capacity helpers (for atomic multi-insert)
    virtual size_t totalSize() const = 0;
    virtual size_t totalCapacity() const = 0;
    
    // Performance optimization - sort all stores once after bulk insertion
    virtual void sortAll() = 0;
};


// ===================================================================================
// MODBUS STATIC & DYNAMIC WORDSTORE CLASSES
// ===================================================================================

/**
 * @brief Static Word storage
 * @tparam N Maximum total number of Words (all types combined)
 * @note Usage: StaticWordStore<200> store;
 */
template<size_t N>
class StaticWordStore : public IWordStore {
public:
    StaticWordStore() {
        static_assert(N > 0, "StaticWordStore capacity must be > 0");
        // _count=0 and _needsSorting=true by default (bulk mode).
        // No clearAll() call — matches DynamicWordStore behavior.
    }

protected:
    void reserve(Modbus::RegisterType type, size_t count) override {
        (void)type; (void)count; /* no-op: capacity fixed at construction */
    }

    void clear(Modbus::RegisterType type) override {
        // Remove every Word whose type matches
        size_t dst = 0;
        for (size_t src = 0; src < _count; ++src) {
            if (_words[src].type != type) {
                _words[dst++] = _words[src];
            }
        }
        _count = dst;
    }

    void clearAll() override {
        _count = 0;
        _needsSorting = false; // Empty store is trivially sorted
    }

    Word* findExact(Modbus::RegisterType type, uint16_t address) override {
        if (_count == 0) return nullptr;
        auto begin = _words.begin();
        auto end   = begin + _count;
        auto it = std::lower_bound(begin, end, address, [](const Word& w, uint16_t addr){
            return w.startAddr < addr;
        });
        // Scan forward for possible candidates having same address
        for (; it != end && it->startAddr == address; ++it) {
            if (it->type == type) return &(*it);
        }
        return nullptr;
    }

    Word* findNext(Modbus::RegisterType type, uint16_t address) override {
        if (_count == 0) return nullptr;
        auto begin = _words.begin();
        auto end   = begin + _count;
        auto it = std::upper_bound(begin, end, address, [](uint16_t addr, const Word& w){
            return addr < w.startAddr;
        });
        for (; it != end; ++it) {
            if (it->type == type) return &(*it);
        }
        return nullptr;
    }

    Word* findContaining(Modbus::RegisterType type, uint16_t address) override {
        if (_count == 0) return nullptr;
        auto begin = _words.begin();
        auto end   = begin + _count;
        auto it = std::upper_bound(begin, end, address, [](uint16_t addr, const Word& w){
            return addr < w.startAddr;
        });
        // Candidate may be just before upper_bound
        while (it != begin) {
            --it;
            if (it->type == type && address >= it->startAddr && address < (it->startAddr + it->nbRegs)) {
                return &(*it);
            }
            // If startAddr differs we can break (because sorted) when address >= it->startAddr + nbRegs
            if (address >= it->startAddr + it->nbRegs) break;
        }
        return nullptr;
    }

    bool overlaps(const Word& word) const override {
        uint32_t wordEnd = word.startAddr + word.nbRegs;
        for (size_t i = 0; i < _count; ++i) {
            const auto& existing = _words[i];
            if (existing.type != word.type) continue;
            uint32_t existingEnd = existing.startAddr + existing.nbRegs;
            if (word.startAddr < existingEnd && existing.startAddr < wordEnd) {
                return true;
            }
        }
        return false;
    }

    bool insert(const Word& word) override {
        if (_count >= N) return false; // capacity exceeded
        if (!_needsSorting && overlaps(word)) return false; // Reject overlaps when sorted

        if (!_needsSorting) {
            // maintain sorted order by startAddr
            auto pos = std::lower_bound(_words.begin(), _words.begin() + _count, word.startAddr,
                                        [](const Word& w, uint16_t addr){ return w.startAddr < addr; });
            // shift right
            for (size_t idx = _count; idx > static_cast<size_t>(pos - _words.begin()); --idx) {
                _words[idx] = _words[idx - 1];
            }
            *pos = word;
        } else {
            // Bulk mode: fast append, defer sort+overlap check to sortAll()
            _words[_count] = word;
        }
        ++_count;
        return true;
    }

    size_t size(Modbus::RegisterType type) const override {
        size_t cnt = 0;
        for (size_t i = 0; i < _count; ++i) if (_words[i].type == type) ++cnt;
        return cnt;
    }

    bool hasCapacity(Modbus::RegisterType) const override { return _count < N; }

    size_t totalSize() const override { return _count; }
    size_t totalCapacity() const override { return N; }

    void sortAll() override {
        if (!_needsSorting) return;
        std::sort(_words.begin(), _words.begin() + _count, [](const Word& a, const Word& b){
            return a.startAddr < b.startAddr;
        });
        _needsSorting = false;
    }

private:
    std::array<Word, N> _words{};
    size_t _count = 0;
    bool _needsSorting = true; // Start in bulk mode (matches DynamicWordStore)
};

/**
 * @brief Dynamic Word storage
 * @note Usage: DynamicWordStore store(10000);
 * @note Allocated on heap, hides std::vector containers
 */
class DynamicWordStore : public IWordStore {
public:
    explicit DynamicWordStore(size_t totalCapacity = 100)
        : _totalCapacity(totalCapacity > 0 ? totalCapacity : 100), _needsSorting(true) {
        _words.reserve(_totalCapacity);
    }

protected:
    void reserve(Modbus::RegisterType type, size_t count) override {
        (void)type; (void)count; /* no-op: capacity fixed at construction */
    }

    void clear(Modbus::RegisterType type) override {
        // Remove all words with the given type
        auto it = _words.begin();
        while (it != _words.end()) {
            if (it->type == type) {
                it = _words.erase(it);
            } else {
                ++it;
            }
        }
    }

    void clearAll() override {
        _words.clear();
        _needsSorting = false; // Empty vector is sorted
    }

    Word* findExact(Modbus::RegisterType type, uint16_t address) override {
        if (_words.empty()) return nullptr;
        auto it = std::lower_bound(_words.begin(), _words.end(), address,
            [](const Word& w, uint16_t addr){ return w.startAddr < addr; });
        for (; it != _words.end() && it->startAddr == address; ++it) {
            if (it->type == type) return &(*it);
        }
        return nullptr;
    }

    Word* findNext(Modbus::RegisterType type, uint16_t address) override {
        if (_words.empty()) return nullptr;
        auto it = std::upper_bound(_words.begin(), _words.end(), address,
            [](uint16_t addr, const Word& w){ return addr < w.startAddr; });
        for (; it != _words.end(); ++it) {
            if (it->type == type) return &(*it);
        }
        return nullptr;
    }

    Word* findContaining(Modbus::RegisterType type, uint16_t address) override {
        if (_words.empty()) return nullptr;
        auto it = std::upper_bound(_words.begin(), _words.end(), address,
            [](uint16_t addr, const Word& w){ return addr < w.startAddr; });
        while (it != _words.begin()) {
            --it;
            if (it->type == type && address >= it->startAddr && address < (it->startAddr + it->nbRegs)) {
                return &(*it);
            }
            if (address >= it->startAddr + it->nbRegs) break;
        }
        return nullptr;
    }

    bool overlaps(const Word& word) const override {
        uint32_t wordEnd = word.startAddr + word.nbRegs;
        for (const auto& existing : _words) {
            if (existing.type != word.type) continue;
            uint32_t existingEnd = existing.startAddr + existing.nbRegs;
            if (word.startAddr < existingEnd && existing.startAddr < wordEnd) return true;
        }
        return false;
    }

    bool insert(const Word& word) override {
        if (_words.size() >= _totalCapacity) return false;
        if (!_needsSorting && overlaps(word)) return false; // Skip overlap scan in bulk mode

        if (!_needsSorting) {
            // Insert in sorted order immediately
            auto it = std::lower_bound(_words.begin(), _words.end(), word.startAddr,
                [](const Word& w, uint16_t addr){ return w.startAddr < addr; });
            _words.insert(it, word);
        } else {
            _words.push_back(word);
        }
        return true;
    }

    size_t size(Modbus::RegisterType type) const override {
        size_t cnt = 0;
        for (const auto& w : _words) if (w.type == type) ++cnt;
        return cnt;
    }

    bool hasCapacity(Modbus::RegisterType) const override {
        return _words.size() < _totalCapacity;
    }

    size_t totalSize() const override { return _words.size(); }
    size_t totalCapacity() const override { return _totalCapacity; }

    void sortAll() override {
        if (_needsSorting) {
            std::sort(_words.begin(), _words.end(), [](const Word& a, const Word& b){
                return a.startAddr < b.startAddr;
            });
            _needsSorting = false;
        }
    }

private:
    std::vector<Word> _words;
    const size_t _totalCapacity;
    bool _needsSorting;
};


} // namespace Modbus