#ifndef __ROIC_COMPRESSION_CONFIG_H
#define __ROIC_COMPRESSION_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NUM_SYNC_WORDS (1)
#define TDC_AXIS_NUM_TDCS_PER_WORD (2)

#define COMPRESSION_TABLE__NUM_ENTRIES (127)
#define TOTAL_NUM_THRESHOLDS (23)

#define NUM_GAIN_CHANNELS (3)
#define NUM_PIXELS (2)

enum gain_channels
{
	HIGH_GAIN = 0,
	SUPER_LOW_GAIN,
	FAR
};

enum roic_pixel
{
	PIXEL_A = 0,
	PIXEL_B
};

struct return_address
{
	uint8_t gc;
	uint8_t comparator_idx;
	uint8_t return_idx;
};

enum input_word_type_t
{
	ADDRESS_METADATA = 0,
	ADDRESS_NON_RETURN_TDC,
	ADDRESS_RETURN_TDC,
	ADDRESS_ATD,
	ADDRESS_DROP
};

struct input_word_dest
{
	uint8_t type;
	uint8_t gc;
	uint8_t comparator_idx;
	uint8_t return_idx;
};

struct roic_comparator_settings
{
	uint8_t  offset;
	uint8_t  threshold;
	uint32_t calibrated_threshold;
	uint8_t  order;
};

extern const size_t NUM_NON_RETURN_TDCS;
extern const size_t GAIN_CHANNEL_NUM_THRESHOLDS[NUM_GAIN_CHANNELS];
extern const size_t GAIN_CHANNEL_NUM_RETURNS[NUM_GAIN_CHANNELS];
extern const size_t SUM_GC_NUM_RETURNS;
extern const size_t SUM_GC_NUM_RISE_FALL;
extern const size_t NUM_TDCS_PER_RETURN;
extern const size_t NUM_RETURN_TDCS;
extern const size_t NUM_TDCS;
extern const size_t NUM_P_BRIDGE_WORDS_PER_TABLE_ENTRY;

extern const uint16_t threshold__roic_address[NUM_PIXELS][TOTAL_NUM_THRESHOLDS];

extern const uint32_t p_bridge_compression_offset[NUM_PIXELS];

extern struct roic_comparator_settings comparator_settings[NUM_PIXELS][TOTAL_NUM_THRESHOLDS];
extern uint8_t                         comparator_multiplier[NUM_PIXELS];

extern uint16_t compression_pna;
extern uint32_t roic_prf;

void set_roic_offset(uint8_t pixel_idx, uint8_t threshold_idx, uint8_t offset);
bool default_compression_fpga_table(size_t which_pixel);
bool roic_compression_init(void);

#endif
