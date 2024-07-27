#ifndef CRC16_H
#define CRC16_H

#include <stddef.h>
#include <stdint.h>

uint16_t crc16_bit(uint16_t crc, uint8_t const *mem, size_t len);

#endif
