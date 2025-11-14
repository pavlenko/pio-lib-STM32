#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace STM32
{
    template <size_t max>
    class Buffer
    {
    private:
        uint8_t _buf[max];
        uint32_t _pos{ 0 };

    public:
        Buffer() = default;

        void seek(size_t pos)
        {
            _pos = pos;
        }

        size_t write(uint8_t data)
        {
            if (_pos >= max) return 0;
            _buf[_pos++] = data;
            return 1;
        }

        size_t write(const uint8_t* data, size_t size)
        {
            size_t n{ 0 };
            for (; n < size; n++) {
                if (!write(data[n])) break;
            }
            return n;
        }

        size_t write(const char* str)
        {
            if (str == NULL) return 0;
            return write(reinterpret_cast<const uint8_t*>(str), strlen(str));
        }

        uint8_t* data() { return _buf; }
        size_t size() { return _pos; }
    };
}
