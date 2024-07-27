#ifndef ROIC_EEPROM_H
#define ROIC_EEPROM_H

#include <stdbool.h>
#include <stdint.h>

uint8_t roic_eeprom_checksum8(uint8_t *buf, uint16_t len);

/*** Version number ***/
#define ROIC_EEPROM__FORMAT_START (0)
struct roic_eeprom__format
{
	uint8_t lsb;
	uint8_t msb;
};

uint8_t roic_eeprom_read_format(struct roic_eeprom__format *format);
uint8_t roic_eeprom_write_format(struct roic_eeprom__format *format);

/*** Manufacturing part numbers, work orders, and serial numbers ***/
struct roic_eeprom__mfg_numbers
{
	char part_number[16];
	char work_order[16];
	char serial_number[24];
};

enum roic_eeprom__mfg_sections
{
	// These values are the byte offset each section
	// starts at
	ROIC_EEPROM__MFG_PREROIC     = 2,
	ROIC_EEPROM__MFG_POSTROIC    = 58,
	ROIC_EEPROM__MFG_RECEIVER    = 114,
	ROIC_EEPROM__MFG_TRANSCEIVER = 170,
};

uint8_t roic_eeprom_read_mfg_numbers(enum roic_eeprom__mfg_sections section, struct roic_eeprom__mfg_numbers *numbers);
uint8_t roic_eeprom_write_mfg_numbers(enum roic_eeprom__mfg_sections section, struct roic_eeprom__mfg_numbers *numbers);
#define ROIC_EEPROM__MFG_END (255)

/*** ROIC offsets and thresholds ***/
#define ROIC_EEPROM_NUM_OFFSETS (23)
struct roic_eeprom__offsets
{
	uint8_t checksum; // checksum is first so the rest of the section can grow
	uint8_t offsets[ROIC_EEPROM_NUM_OFFSETS];
};

enum roic_eeprom__offsets_pixels
{
	// These values are the byte offset each section
	// starts at
	ROIC_EEPROM__CALIBRATION_PIXEL0 = 256,
	ROIC_EEPROM__CALIBRATION_PIXEL1 = 288,
};

uint8_t roic_eeprom_read_offsets(enum roic_eeprom__offsets_pixels pixel, struct roic_eeprom__offsets *offsets);
uint8_t roic_eeprom_write_offsets(enum roic_eeprom__offsets_pixels pixel, struct roic_eeprom__offsets *offsets);

/*** ROIC Registers ***/
#define ROIC_EEPROM_NUM_REGISTERS (352)
#define ROIC_EEPROM__REGISTERS_START (320)
struct roic_eeprom__registers
{
	uint8_t checksum; // checksum is first so the rest of the section can grow
	uint8_t registers[ROIC_EEPROM_NUM_REGISTERS];
};

uint8_t roic_eeprom_read_registers(struct roic_eeprom__registers *registers);
uint8_t roic_eeprom_write_registers(struct roic_eeprom__registers *registers);

#define ROIC_EEPROM__NUM_BYTES (1024)

#endif
