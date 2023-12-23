

#ifndef __PN532_INTERFACE_H__
#define __PN532_INTERFACE_H__

#include <stdint.h>

#define PN532_PREAMBLE                (0x00)
#define PN532_STARTCODE1              (0x00)
#define PN532_STARTCODE2              (0xFF)
#define PN532_POSTAMBLE               (0x00)

#define PN532_HOSTTOPN532             (0xD4)
#define PN532_PN532TOHOST             (0xD5)

#define PN532_ACK_WAIT_TIME           (10)  // ms, timeout of waiting for ACK

#define PN532_INVALID_ACK             (-1)
#define PN532_TIMEOUT                 (-2)
#define PN532_INVALID_FRAME           (-3)
#define PN532_NO_SPACE                (-4)

#define REVERSE_BITS_ORDER(b)         b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; \
                                      b = (b & 0xCC) >> 2 | (b & 0x33) << 2; \
                                      b = (b & 0xAA) >> 1 | (b & 0x55) << 1
typedef struct {
    void (*begin)(void);
    void (*wakeup)(void);
    int8_t (*writeCommand)(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen);
    int16_t (*readResponse)(uint8_t buf[], uint8_t len, uint16_t timeout);
}PN532Interface;

#endif

