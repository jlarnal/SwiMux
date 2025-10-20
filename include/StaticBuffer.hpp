#ifndef H_STATIC_BUFFER_H
#define H_STATIC_BUFFER_H

#include <stddef.h>
#include <stdint.h>

template <size_t BUFFER_CAPACITY> class StaticBuffer_t {
  public:
    StaticBuffer_t() : _count(0) {}
    inline void clear() { _count = 0; }
    bool append(const uint8_t value)
    {
        if (_count >= (BUFFER_CAPACITY - 1))
            return false;
        _pool[_count++] = value;
        return true;
    }

    inline size_t count() { return _count; }
    operator uint8_t*() { return _pool; }
    uint8_t& operator[](int index) { return _pool[index]; }


  private:
    uint8_t _pool[BUFFER_CAPACITY] = { 0 };
    size_t _count;
};


#endif //!H_STATIC_BUFFER_H
