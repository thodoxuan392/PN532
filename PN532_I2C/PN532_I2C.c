/**
 * @modified picospuch
 */

#include "PN532_I2C.h"
#include "PN532_debug.h"

#include <string.h>

#include "main.h"
#include "Hal/i2c.h"

#define PN532_I2C_ADDRESS (0x48 >> 1)
#define PN532_I2C_TRANSMIT_BUFFER_MAX_LENGTH 512
#define PN532_I2C_RECEIVE_BUFFER_MAX_LENGTH 512

static uint8_t command = 0;
static uint8_t txBuffer[PN532_I2C_TRANSMIT_BUFFER_MAX_LENGTH];
static uint8_t rxBuffer[PN532_I2C_TRANSMIT_BUFFER_MAX_LENGTH];

static int8_t PN532_I2C_readAckFrame();
static int16_t PN532_I2C_getResponseLength(uint8_t buf[], uint8_t len, uint16_t timeout);

static void PN532_I2C_begin();
static void PN532_I2C_wakeup();
static int8_t PN532_I2C_writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen);
static int16_t PN532_I2C_readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);


static PN532Interface PN532_I2C_interface = {
    .begin = PN532_I2C_begin,
    .wakeup = PN532_I2C_wakeup,
    .writeCommand = PN532_I2C_writeCommand,
    .readResponse = PN532_I2C_readResponse
};

PN532Interface* PN532_I2C_getInterface()
{
    return &PN532_I2C_interface;
}

void PN532_I2C_begin()
{
    
}

void PN532_I2C_wakeup()
{
    
}

int8_t PN532_I2C_writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    command = header[0];

    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    uint32_t txBufferLen = 0;
    txBuffer[txBufferLen++] = PN532_PREAMBLE;
    txBuffer[txBufferLen++] = PN532_STARTCODE1;
    txBuffer[txBufferLen++] = PN532_STARTCODE2;
    txBuffer[txBufferLen++] = length;
    txBuffer[txBufferLen++] = ~length + 1;
    txBuffer[txBufferLen++] = PN532_HOSTTOPN532;

    DMSG("write: ");

    for (uint8_t i = 0; i < hlen; i++)
    {
        txBuffer[txBufferLen++] = header[i];
        sum += header[i];
        DMSG_HEX(header[i]);
    }

    for (uint8_t i = 0; i < blen; i++)
    {
        txBuffer[txBufferLen++] = body[i];
        sum += body[i];
        DMSG_HEX(body[i]);
    }

    uint8_t checksum = ~sum + 1; // checksum of TFI + DATA
    txBuffer[txBufferLen++] = checksum;
    txBuffer[txBufferLen++] = PN532_POSTAMBLE;

    I2C_write(PN532_I2C_ADDRESS, txBuffer, txBufferLen);

    DMSG_CHAR('\n');

    return PN532_I2C_readAckFrame();
}

int16_t PN532_I2C_getResponseLength(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    const uint8_t PN532_NACK[] = {0, 0, 0xFF, 0xFF, 0, 0};
    uint16_t time = 0;
    uint32_t rxBufferIndex = 0;
    do
    {
        if (I2C_read(PN532_I2C_ADDRESS, rxBuffer, 6))
        {
            if (rxBuffer[rxBufferIndex++] & 1)
            {          // check first byte --- status
                break; // PN532 is ready
            }
        }
        HAL_Delay(1);
        time++;
        if ((0 != timeout) && (time > timeout))
        {
            return -1;
        }
    } while (1);

    if (0x00 != rxBuffer[rxBufferIndex++] || // PREAMBLE
        0x00 != rxBuffer[rxBufferIndex++] || // STARTCODE1
        0xFF != rxBuffer[rxBufferIndex++]    // STARTCODE2
    )
    {

        return PN532_INVALID_FRAME;
    }

    uint8_t length = rxBuffer[4];

    // request for last respond msg again
    I2C_write(PN532_I2C_ADDRESS, PN532_NACK, sizeof(PN532_NACK));

    return length;
}

int16_t PN532_I2C_readResponse(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    uint16_t time = 0;
    uint8_t length;

    length = PN532_I2C_getResponseLength(buf, len, timeout);
    uint32_t rxBufferIndex = 0;

    // [RDY] 00 00 FF LEN LCS (TFI PD0 ... PDn) DCS 00
    do
    {
        if (I2C_read(PN532_I2C_ADDRESS,rxBuffer, 6 + length + 2))
        {
            if (rxBuffer[rxBufferIndex++] & 1)
            {          // check first byte --- status
                break; // PN532 is ready
            }
        }
        HAL_Delay(1);
        time++;
        if ((0 != timeout) && (time > timeout))
        {
            return -1;
        }
    } while (1);

    if (0x00 != rxBuffer[rxBufferIndex++] || // PREAMBLE
        0x00 != rxBuffer[rxBufferIndex++] || // STARTCODE1
        0xFF != rxBuffer[rxBufferIndex++]    // STARTCODE2
    )
    {

        return PN532_INVALID_FRAME;
    }

    length = rxBuffer[rxBufferIndex++];

    if (0 != (uint8_t)(length + rxBuffer[rxBufferIndex++]))
    { // checksum of length
        return PN532_INVALID_FRAME;
    }

    uint8_t cmd = command + 1; // response command
    if (PN532_PN532TOHOST != rxBuffer[rxBufferIndex++] || (cmd) != rxBuffer[rxBufferIndex++])
    {
        return PN532_INVALID_FRAME;
    }

    length -= 2;
    if (length > len)
    {
        return PN532_NO_SPACE; // not enough space
    }

    DMSG("read:  ");
    DMSG_HEX(cmd);

    uint8_t sum = PN532_PN532TOHOST + cmd;
    for (uint8_t i = 0; i < length; i++)
    {
        buf[i] = rxBuffer[rxBufferIndex++];
        sum += buf[i];

        DMSG_HEX(buf[i]);
    }
    DMSG_CHAR('\n');

    uint8_t checksum = rxBuffer[rxBufferIndex++];
    if (0 != (uint8_t)(sum + checksum))
    {
        DMSG("checksum is not ok\n");
        return PN532_INVALID_FRAME;
    }

    return length;
}

int8_t PN532_I2C_readAckFrame()
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
    uint8_t ackBuf[sizeof(PN532_ACK)];

    DMSG("wait for ack at : ");
    DMSG_INT(HAL_GetTick());
    DMSG_CHAR('\n');

    uint32_t rxBufferIndex = 0;
    uint16_t time = 0;
    do
    {
        if (I2C_read(PN532_I2C_ADDRESS, rxBuffer, sizeof(PN532_ACK) + 1))
        {
            if (rxBuffer[rxBufferIndex++] & 1)
            {          // check first byte --- status
                break; // PN532 is ready
            }
        }
        HAL_Delay(1);
        time++;
        if (time > PN532_ACK_WAIT_TIME)
        {
            DMSG("Time out when waiting for ACK\n");
            return PN532_TIMEOUT;
        }
    } while (1);

    DMSG("ready at : ");
    DMSG_INT(HAL_GetTick());
    DMSG_CHAR('\n');

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        ackBuf[i] = rxBuffer[rxBufferIndex++];
    }

    if (memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK)))
    {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }

    return 0;
}
