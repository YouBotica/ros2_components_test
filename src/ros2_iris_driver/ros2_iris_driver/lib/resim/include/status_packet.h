#ifndef STATUS_PACKET_H
#define STATUS_PACKET_H

#include "roic_compression_config.h"
#include "roic_eeprom.h"
#include <stdbool.h>
#include <stdint.h>

const size_t MAX_SUPPORTED_STATUS_PACKET_VERSION = 1;

struct status_comp_settings
{
	uint8_t offset[TOTAL_NUM_THRESHOLDS];
	uint8_t threshold[TOTAL_NUM_THRESHOLDS];
	uint8_t coarse_multiplier;
	uint8_t fine_multiplier;
};

struct StatusPacket
{
	uint32_t status_packet_version;
	// Configuration Data
	struct status_comp_settings     comp_settings[NUM_PIXELS];
	struct roic_eeprom__mfg_numbers roic_serial;
	uint32_t                        prf;
	uint16_t                        pna;
	uint16_t                        roic_id;
	uint8_t                         roic_version;
	uint8_t                         roic_config_times_set;
	uint32_t                        mipi_frame_size;
	// Health Data
	uint64_t local_timestamp_ns;
	float    roic_temperature;
	float    rcvr_temperature;
	uint32_t mipi_frames_received;
	uint32_t mipi_frames_processed;
	uint32_t eth_bytes_sent;

	// Only in version 1
	uint8_t roic_subversion;
};

#endif // STATUS_PACKET_H
