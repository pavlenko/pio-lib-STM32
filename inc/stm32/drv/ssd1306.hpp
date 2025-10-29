#pragma once

#include <stm32/dev/i2c.hpp>

template <typename tBus, uint16_t tWidth = 128u, uint16_t tHeight = 64u>
class SSD1306
{
private:
    static const uint8_t _address = 0x78 >> 1;

    static uint8_t _buffer[tWidth * tHeight / 8];
    static uint16_t _x;
    static uint16_t _y;

public:
    enum CMD : uint8_t {
        SET_MEMORY_MODE = 0x20,    // 0x20,mode(0x0 = horizontal, 0x1 = vertical, 0x2 = page, 0x3 = invalid)
        SET_COLUMN_ADDRESS = 0x21, // 0x21,min,max
        SET_PAGE_ADDRESS = 0x22,   // 0x22,min,max
        SET_START_LINE = 0x40,     // 0x40 | value
        SET_CHARGE_PUMP = 0x8D,    // 0x8D,(0x14 = enable || 0x10 = disable)
        SEGMENT_REMAP_OFF = 0xA0,
        SEGMENT_REMAP_ON = 0xA1,
        SET_DISPLAY_NORMAL = 0xA6,
        ENTIRE_DISPLAY_RAM = 0xA4,
        ENTIRE_DISPLAY_ON = 0xA5,
        SET_DISPLAY_INVERSE = 0xA7,
        SET_MULTIPLEX_RATIO = 0xA8,
        OFF = 0xAE,
        ON = 0xAF,
        SET_SCAN_NORMAL = 0xC0,
        SET_SCAN_REVERSE = 0xC8,
        SET_COM_PINS_CONFIG = 0xDA,
    };

    static void init()
    {
        constexpr uint8_t data[] = {
            CMD::OFF,
            CMD::SET_MEMORY_MODE,
            0x00, // memory mode = horizontal
            CMD::SET_COLUMN_ADDRESS,
            0x00, // min column = 0
            0x7F, // max column = 127
            CMD::SET_PAGE_ADDRESS,
            0x00, // min page = 0
            0x07, // max page = 7
            CMD::SET_START_LINE | 0x00,
            CMD::SEGMENT_REMAP_ON,
            CMD::SET_SCAN_REVERSE,
            CMD::SET_COM_PINS_CONFIG,
            0x12, // disable remap
            CMD::SET_DISPLAY_NORMAL,
            CMD::ENTIRE_DISPLAY_RAM,
            CMD::SET_CHARGE_PUMP,
            0x14, // enable charge pump
            CMD::ON,
        };

        tBus::select(_address, 400000);
        tBus::send(data, sizeof(data)); // TODO send in sync mode

        // reset
        memset(_buffer, 0x00, sizeof(_buffer));
        _x = 0;
        _y = 0;
    }

    static inline void update()
    {
        tBus::send(_buffer, tWidth * tHeight / 8); // send in async mode
    }

    // TODO GFX methods...
};
