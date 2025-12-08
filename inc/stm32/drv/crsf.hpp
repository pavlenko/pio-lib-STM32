#ifndef __CRSF_H__
#define __CRSF_H__

/*
# CRSF Broadcast Frame (sync bytes <= 0x27 )
# Offset |        Usage        | CRC | Comment
#   0    |      sync byte      | No  | uint8, e.g. 0xEE -> to TX module, 0xC8 -> to FC, 0xEC -> FC to RX
#   1    |    frame length     | No  | uint8, entire frame size -2 (allowed values: 2-62)
#   2    |         type        | Yes | uint8, e.g. 0x02 -> GPS data
#   3    |       payload       | Yes | payload with length n bytes
#  n+4   |      checksum       | No  | uint8, basic CRC8 using polynomial 0xD5 (-> DVB-S2)

# CRSF Extended Frame (sync bytes >= 0x28 )
# Offset |        Usage        | CRC | Comment
#   0    |      sync byte      | No  | uint8, e.g. 0xEE -> to TX module, 0xC8 -> to FC, 0xEC -> FC to RX
#   1    |    frame length     | No  | uint8, entire frame size -2 (allowed values: 2-62)
#   2    |         type        | Yes | uint8, e.g. 0x02 -> GPS data
#   3    | destination address | Yes | uint8, e.g. 0xC8 -> Flight Controller
#   4    |    origin address   | Yes | uint8, e.g. 0xEA -> Remote Control
#   5    |       payload       | Yes | payload with length n bytes
#  n+6   |      checksum       | No  | uint8, basic CRC8 using polynomial 0xD5 (-> DVB-S2)
*/

#include <stm32/dev/uart.hpp>

namespace CRSF
{
    enum Address : uint8_t {
        BROADCAST = 0x00,
        CLOUD = 0x0E,
        USBD = 0x10,
        WIFI_BLUETOOTH = 0x12,
        WIFI_RX = 0x13,
        VRX = 0x14,
        TBS_CORE_PNP_PRO = 0x80,
        ESC1 = 0x90,
        ESC2 = 0x91,
        ESC3 = 0x92,
        ESC4 = 0x93,
        ESC5 = 0x94,
        ESC6 = 0x95,
        ESC7 = 0x96,
        ESC8 = 0x97,
        CURRENT_SENSOR = 0xC0,
        GPS = 0xC2,
        TBS_BLACKBOX = 0xC4,
        FC = 0xC8,
        RACE_TAG = 0xCC,
        VTX = 0xCE,
        REMOTE_CONTROL = 0xEA,
        CRSF_RX = 0xEC,
        CRSF_TX = 0xEE,
    };
}

using namespace STM32;

static uint8_t rxBuf[64];

static inline void configureTX() { UART1::configure<400000u, UART::Config::HALF_DUPLEX>(); }

static inline void configureRX() { UART1::configure<416666u, UART::Config::FULL_DUPLEX>(); }

static inline void onUARTRx(DMA::Event e, uint16_t n);

static inline void startRX()
{
    // Read data from ELRS RX:
    // gaps each 10-12ms -> can use to idle
    UART1::rxDMA(rxBuf, 64, onUARTRx);
}

static inline void onUARTRx(DMA::Event e, uint16_t nReceived)
{
    // min length = 4
    // max length = 64
    if (nReceived < 4) return; // Invalid length - skip
    if (rxBuf[1] != (nReceived - 2)) return; // Bad frame length
    // TODO check CRC8

    if (rxBuf[2] < 0x27) {
        // Broadcast frame
    } else {
        // Extended frame
    }

}

#endif // __CRSF_H__