#ifndef COMPRESSED_CONTAINER__H
#define COMPRESSED_CONTAINER__H

#include <algorithm>
#include <array>
#include <cassert> // assert
#include <cstddef> // ptrdiff_t
#include <iostream>
#include <istream>
#include <iterator>    // iterator
#include <type_traits> // remove_cv
#include <utility>     // swap

#include "lum_optional.hpp"

#include <type_traits>

#include "be64toh_iterator.hpp"
#include "crc16.h"
#include "pcap_parsing_errors.hpp"
#include "time_helpers.hpp"

struct VariableMetadata
{
	lum_optional<uint32_t> az_timestamp;
	lum_optional<bool>     az_edge;
	lum_optional<uint8_t>  tab_number;
	lum_optional<uint32_t> el_angle;
	lum_optional<uint8_t>  scan_checkpoint;
	lum_optional<uint8_t>  frame_index;
	lum_optional<uint16_t> el_timestamp;
	lum_optional<uint32_t> timestamp_lsw;
	lum_optional<uint32_t> timestamp_msw;
	lum_optional<uint16_t> laser_power;
};

struct CompressedContainer
{
	CompressedContainer(const uint32_t *addr, const VariableMetadata &previous_metadata, const uint16_t packet_version,
	                    const bool check_errors = true)
	    : previous_metadata_{previous_metadata}, packet_version_{packet_version}, data{addr}
	{
		if (check_errors && !crc_valid())
		{
			throw MalformedMipiFrame{};
		}

		if (get_metadata_num_words() != get_el_msw_idx())
		{
			std::cerr << "CompressedContainer get_metadata_num_words (" << (uint16_t) get_metadata_num_words() << ") != get_el_msw_idx("
			          << (uint16_t) get_el_msw_idx() << ")! Populated fields: " << (uint16_t) get_metadata_populated_fields() << " \n";
			throw MalformedMipiFrame{};
		}
	}

	~CompressedContainer()                                = default;
	CompressedContainer(const CompressedContainer &other) = default;
	CompressedContainer &operator=(const CompressedContainer &) = default;

	static const unsigned int MAX_SEQUENCE     = (1 << 7) - 1;
	static const unsigned int NUM_HEADER_WORDS = 1;
	static const unsigned int NUM_FOOTER_WORDS = 1;

private:
	const VariableMetadata previous_metadata_;
	const uint16_t         packet_version_;
	const uint32_t *       data;

	////////////////////////////
	// Variable Metadata Helpers

	bool is_timestamp_msw_populated() const { return (get_metadata_populated_fields()) & 0x1; }
	bool is_az_populated() const { return (get_metadata_populated_fields() >> 1) & 0x1; }
	bool is_el_populated() const { return (get_metadata_populated_fields() >> 2) & 0x1; }

	size_t get_timestamp_lsw_idx() const { return 1; }
	size_t get_timestamp_msw_idx() const { return get_timestamp_lsw_idx() + (is_timestamp_msw_populated() ? 1 : 0); }
	size_t get_az_idx() const { return get_timestamp_msw_idx() + (is_az_populated() ? 1 : 0); }
	size_t get_el_lsw_idx() const { return get_az_idx() + (is_el_populated() ? 1 : 0); }
	size_t get_el_msw_idx() const { return get_el_lsw_idx() + (is_el_populated() ? 1 : 0); }

	static constexpr uint64_t NS_PER_SECOND = 1000000000;

public:
	unsigned int get_site_a_num_words() const
	{
		const uint8_t num_words = (data[0] & 0xFF);
		return num_words;
	};
	unsigned int get_site_b_num_words() const
	{
		const uint8_t num_words = ((data[0] >> 8) & 0xFF);
		return num_words;
	};
	uint8_t get_metadata_num_words() const { return ((data[0] >> 16) & 0x7); };
	uint8_t get_metadata_populated_fields() const { return (static_cast<uint8_t>(data[0] >> 28) & 0xF); };

	unsigned int get_num_words() const
	{
		// For the roic words, return the raw values since it might count the padding word
		return get_site_a_num_words() + get_site_b_num_words() + get_metadata_num_words() + NUM_HEADER_WORDS + NUM_FOOTER_WORDS;
	};

	uint16_t get_sequence_number() const { return ((data[0] >> 21) & 0x7F); };

	/**
	 * @brief  compute the full ns timestamp of the edge and return the bottom 32 bits of it.
	 */
	lum_optional<uint32_t> get_az_timestamp() const
	{
		if (!is_az_populated())
			return previous_metadata_.az_timestamp;

		if (packet_version_ == 1) // in version 1 the timestamp was in sys clock ticks.
		{
			// In version == 1, the edge timestamp is a 22-bit, 5 nsec resolution timestamp.

			// Here, we generate a fully unwrapped 32-bit 1 nsec timer. The native resolution will still be 5 nsec.
			// The 64-bit ray timer is a full 64-bit 5 nsec resolution timer.

			// We use the previous ray time as a basis for concatenating and generating a full 32 bit timer value
			// out of the 22 bit timer value.

			const auto edge_ts = (data[get_az_idx()] & 0x3FFFFFul);

			const uint64_t previous_timestamp_lsw = static_cast<uint64_t>(previous_metadata_.timestamp_lsw.value_or(0));
			const uint64_t previous_timestamp_msw = static_cast<uint64_t>(previous_metadata_.timestamp_msw.value_or(0));

			// Create edge timestamp by replacing 22 LSBs of the previous ray (or container) timestamp, convert
			// a 64 bit systems timestamp by adding the MSW.
			uint64_t edge_timestamp_tick =
			    (previous_timestamp_msw << 32) | (previous_timestamp_lsw & ~0x3FFFFFull) | static_cast<uint64_t>(edge_ts);

			// Check for a roll over. If it occurred, add 1 to the 23rd bit.  There is a 50 nsec tolerance to deal with
			// apparent MIPI frame generation latency.
			if ((previous_timestamp_lsw & 0x3FFFFFul) > static_cast<uint64_t>(edge_ts) + 10ull)
			{
				// Simple 24 bit rollover, just increment the 25th bit.
				edge_timestamp_tick += 0x400000ull;
			}

			// Convert the timestamp to a 1 nsec timestamp and return the 32 LSBs.
			return static_cast<uint32_t>(sys_time_to_ns(edge_timestamp_tick));
		}
		else if (packet_version_ == 2) // timestamp is now transmitted as ns_time >> 2
		{
			// In version==2 the msw contains the seconds and lsw nanoseconds.

			// The edge timestamp is a 22-bit timer with a 4 nsec resolution.

			// This routine attempts to reconstruct a 32-bit timestamp with 1 nsec resolution
			// using a combination of the reported ray timeestamp from the previous container
			// and the edge timestamp.

			//
			// This assumes the edge_ns was sampled at a time equal or greater to the ray timestamp
			// reported in the previous container.  The 22 bit edge timestamp is shifted to
			// create a 1 nsec counter (2 LSBs will always be zero) and replaces the 24 LSBs
			// of the ray timestamp.  A wrap of the 24 LSBs may occur between the ray timestamp
			// and edge timestamp or a wrap of the seconds counter may occur, which must be detected and corrected
			// by incrementing the 25th bit or concatenating seconds or concatenating an incremented seconds and the
			// edge timestamp, respectively.

			// We create a 64 bit counter out of the sec/nsec fields so that we will ultimately have a 32 bit counter
			// (upon taking the 32 LSBs) that doesn't not wrap until hitting 2^32.

			// Convert the edge timestamp to a 24 bit 1 nsec timer (2 LSBs always 0).
			const auto edge_ts = (data[get_az_idx()] & 0x3FFFFFul) << 2;

			const uint64_t previous_timestamp_nsec = static_cast<uint64_t>(previous_metadata_.timestamp_lsw.value_or(0));
			const uint64_t previous_timestamp_sec  = static_cast<uint64_t>(previous_metadata_.timestamp_msw.value_or(0));
			const uint64_t current_timestamp_sec   = static_cast<uint64_t>(get_timestamp_msw().value_or(0));

			const bool second_rollover_between_rays = previous_timestamp_sec < current_timestamp_sec;

			// Create edge timestamp by replacing 24 LSBs of the previous ray (or container) nsec timestamp, convert
			// a 64 bit systems timestamp by adding the seconds count.
			uint64_t edge_timestamp_nsec =
			    previous_timestamp_sec * NS_PER_SECOND + ((previous_timestamp_nsec & ~0x00FFFFFFull) | static_cast<uint64_t>(edge_ts));

			// Check for a roll over. If it occurred, add 1 to the 25th bit.  There is a 150 nsec tolerance to deal with
			// apparent MIPI frame generation latency.
			if ((previous_timestamp_nsec & 0x00FFFFFFull) > static_cast<uint64_t>(edge_ts) + 150ull)
			{
				// If it occurred because the LSBs jut rollever, increment the 25th bit.
				// If it occurred because the ns were jammed to 0 and the seconds
				// incremented, add 1 billion ns.
				if (second_rollover_between_rays)
				{
					// If we're here, it means that the seconds incremented after the last ray but before the edge timestamp was captured.
					// Expect difference between any two timestamps to be no more than 20 usec.
					// The 24 bit timestamp rolls over at ~8.4 msec.  When the seconds counter rolls over
					// we can simply replace the nsec timer with the edge_ts plus the incremented
					// seconds counter.
					//
					edge_timestamp_nsec = current_timestamp_sec * NS_PER_SECOND + static_cast<uint64_t>(edge_ts);
				}
				else
				{
					// Simple 24 bit rollover, just increment the 25th bit.
					edge_timestamp_nsec += 0x1000000ull;
				}
			}

			// Return only the 32-bit portion of 64-bit nsec timer value that has been formed.
			return static_cast<uint32_t>(edge_timestamp_nsec);
		}
		else
		{
			std::cerr << "CompressedContainer Version Invalid!\n";
			throw MalformedMipiFrame{};
		}
	};

	lum_optional<bool> get_az_edge() const
	{
		if (is_az_populated())
			return ((data[get_az_idx()] >> 22) & 0x1);
		else
			return previous_metadata_.az_edge;
	};
	lum_optional<uint8_t> get_tab_number() const
	{
		if (is_az_populated())
			return (data[get_az_idx()] >> 23) & 0xF;
		else
			return previous_metadata_.tab_number;
	};
	lum_optional<uint32_t> get_el_angle() const
	{
		if (is_el_populated())
			return (data[get_el_lsw_idx()] & 0xFFFFF);
		else
			return previous_metadata_.el_angle;
	};
	lum_optional<uint16_t> get_el_timestamp() const
	{
		if (is_el_populated())
			// el timestamp is unimplemented in firmware currently.
			return static_cast<uint16_t>(((data[get_el_msw_idx()] & 0x1) << 12) | (data[get_el_lsw_idx()] >> 20));
		else
			return previous_metadata_.el_timestamp;
	};
	lum_optional<uint8_t> get_scan_checkpoint() const
	{
		if (is_el_populated())
			return ((data[get_el_msw_idx()] >> 1) & 0xFF);
		else
			return previous_metadata_.scan_checkpoint;
	};
	lum_optional<uint8_t> get_frame_index() const
	{
		if (packet_version_ <= 1) // frame index was not included
			return 0;

		if (is_el_populated())
			return ((data[get_el_msw_idx()] >> 9) & 0xFF);
		else
			return previous_metadata_.frame_index;
	};
	// Detector
	uint16_t get_laser_power() const { return (data[get_num_words() + 1] & 0xFFF); };
	uint8_t  get_prf_stagger() const { return ((data[get_num_words() + 1] >> 12) & 0xF); };
	// Time
	lum_optional<uint32_t> get_timestamp_lsw() const { return data[get_timestamp_lsw_idx()]; };
	lum_optional<uint32_t> get_timestamp_msw() const
	{
		if (is_timestamp_msw_populated())
			return data[get_timestamp_msw_idx()];
		// In case the MSW packet was dropped, assume we haven't dropped more than 1
		// second of data and correct for it.
		else if (get_timestamp_lsw() < previous_metadata_.timestamp_lsw)
			return previous_metadata_.timestamp_msw ? (*previous_metadata_.timestamp_msw + 1) : lum_optional<uint32_t>{};
		else
			return previous_metadata_.timestamp_msw;
	};

	/**
	 * @brief return the timestamp in nanoseconds
	 * @note if the MSW has not been seen, it defaults to 0
	 */
	uint64_t get_timestamp_ns() const
	{
		if (packet_version_ <= 1) // timestamp was a 64-bit sys clock counter
		{
			const uint64_t timestamp = (static_cast<uint64_t>(get_timestamp_msw().value_or(0)) << 32) | get_timestamp_lsw().value_or(0);
			return sys_time_to_ns(timestamp);
		}
		else // timestamp is sent with seconds in the top 32 bits an ns in the bottom 32
		{
			return static_cast<uint64_t>(get_timestamp_msw().value_or(0)) * NS_PER_SECOND + get_timestamp_lsw().value_or(0);
		}
	}

	VariableMetadata get_metadata() const
	{
		VariableMetadata ret{};
		ret.az_timestamp    = get_az_timestamp();
		ret.az_edge         = get_az_edge();
		ret.tab_number      = get_tab_number();
		ret.el_angle        = get_el_angle();
		ret.scan_checkpoint = get_scan_checkpoint();
		ret.frame_index     = get_frame_index();
		ret.el_timestamp    = get_el_timestamp();
		ret.timestamp_lsw   = get_timestamp_lsw();
		ret.timestamp_msw   = get_timestamp_msw();
		return ret;
	}

	// Other
	uint16_t get_crc() const { return (data[get_num_words() - 1] >> 16) & 0xFFFF; };

	unsigned int get_num_roic_frame_words() const
	{
		return get_num_words() - NUM_HEADER_WORDS - NUM_FOOTER_WORDS - get_metadata_num_words();
	};

	const uint32_t *get_site_a_frame() const { return data + NUM_HEADER_WORDS + get_metadata_num_words(); }
	const uint32_t *get_site_b_frame() const { return data + NUM_HEADER_WORDS + get_metadata_num_words() + get_site_a_num_words(); }

	uint16_t calculate_crc() const
	{
		const uint16_t crc = crc16_bit(0, (uint8_t const *) &data[0], (get_num_words() * sizeof(uint32_t) - sizeof(uint16_t)));

		return crc;
	}

	bool crc_valid() const { return get_crc() == calculate_crc(); }
};

/**
 * @brief Build a CompressedContainer
 *
 * @note: Default to packet version 1, need to clean up includes as we don't have access to the MipiFrame::BUILDER_VERSION
 */
struct CompressedContainerBuilder
{
	CompressedContainerBuilder(const be64toh_iterator_nonconst &addr, const VariableMetadata &metadata,
	                           const uint16_t sequence_number, const kj::ArrayPtr<const capnp::byte> site_a,
	                           const kj::ArrayPtr<const capnp::byte> site_b, const uint16_t packet_version = 1)
	    : data_{addr}, wr_ptr_{addr}, packet_version_{packet_version}
	{
		wr_ptr_.set(0); // make sure this word is zero'd first, in case the buffer isn't clean
		wr_ptr_++;      // move the pointer past the site num words which may need adjusting

		set_sequence_number(sequence_number);

		// Must all be called, and in order!
		const uint8_t metadata_num_words{set_metadata(metadata)};
		const uint8_t site_a_num_words{static_cast<uint8_t>(site_a.size() / sizeof(uint32_t))};
		const uint8_t site_b_num_words{static_cast<uint8_t>(site_b.size() / sizeof(uint32_t))};

		const bool is_64_bit_aligned = ((CompressedContainer::NUM_HEADER_WORDS + metadata_num_words + //
		                                 site_a_num_words + site_b_num_words +                        //
		                                 CompressedContainer::NUM_FOOTER_WORDS)                       //
		                                % 2) == 0;

		set_site(site_a);
		set_site(site_b);

		// Containers must be a multiple of 64 bits long. If this one is not, add an
		// extra word to the second site.
		if (!is_64_bit_aligned)
		{
			wr_ptr_++;
		}
		set_site_a_num_words(site_a_num_words);
		set_site_b_num_words(static_cast<uint8_t>(site_b_num_words + (is_64_bit_aligned ? 0 : 1)));

		num_words_ = static_cast<size_t>(std::distance(data_, wr_ptr_)) + 1;

		wr_ptr_.set(0); // make sure this word is zero'd first, in case the buffer isn't clean
		set_crc();
	}

	/**
	 * @brief Get the read-only container from the builder
	 */
	std::pair<std::unique_ptr<uint32_t[]>, CompressedContainer> operator()(const VariableMetadata &previous_metadata) const
	{
		auto endian_swapped_data = std::unique_ptr<uint32_t[]>(new uint32_t[num_words_]);
		std::copy(data_, data_ + num_words_, &endian_swapped_data[0]);

		auto compressed_container = CompressedContainer{&endian_swapped_data[0], previous_metadata, packet_version_, false};

		return std::make_pair<std::unique_ptr<uint32_t[]>, CompressedContainer>(std::move(endian_swapped_data),
		                                                                        std::move(compressed_container));
	}

	/**
	 * @brief If no previous metadata is provided, default to none
	 */
	auto operator()() const { return operator()({}); }

private:
	const be64toh_iterator_nonconst data_;
	be64toh_iterator_nonconst       wr_ptr_; // ugly way of keeping track of writing into the buffer
	size_t                          num_words_;
	const uint16_t                  packet_version_;

	void set_site_a_num_words(const uint8_t num_words) { (data_ + 0).set(*(data_ + 0) | ((uint32_t) num_words & 0xFF)); }
	void set_site_b_num_words(const uint8_t num_words) { (data_ + 0).set(*(data_ + 0) | (((uint32_t) num_words & 0xFF) << 8)); }

	void set_metadata_num_words(const uint8_t num_words) { (data_ + 0).set(*(data_ + 0) | (((uint32_t) num_words & 0x7) << 16)); }
	void set_metadata_populated_fields(const VariableMetadata &metadata)
	{
		uint8_t populated_fields{0};
		if (metadata.timestamp_msw)
			populated_fields |= 0x1;
		if (metadata.az_timestamp)
			populated_fields |= 0x2;
		if (metadata.el_timestamp)
			populated_fields |= 0x4;

		(data_ + 0).set(*(data_ + 0) | (((uint32_t) populated_fields & 0xF) << 28));
	}

	void set_sequence_number(const uint16_t seq_num) { (data_ + 0).set(*(data_ + 0) | (((uint32_t) seq_num & 0x7F) << 21)); }

	uint8_t set_metadata(const VariableMetadata &metadata)
	{
		uint8_t metadata_num_words = 0;

		// Timestamp LSW
		assert(metadata.timestamp_lsw); // must be set
		wr_ptr_++.set(*metadata.timestamp_lsw);
		metadata_num_words++;

		// Timestamp MSW
		if (metadata.timestamp_msw)
		{
			wr_ptr_++.set(*metadata.timestamp_msw);
			metadata_num_words++;
		}

		// Az Timestamp & Az Edge & Az Tab number
		if (metadata.az_timestamp)
		{
			assert(metadata.az_edge && metadata.tab_number); // must all bet set as a group

			uint32_t word{0};
			word |= ((uint32_t) *metadata.az_timestamp) << 0;
			word |= ((uint32_t) *metadata.az_edge) << 22;
			word |= ((uint32_t) *metadata.tab_number) << 23;
			wr_ptr_++.set(word);

			metadata_num_words++;
		}

		// El Angle & El Timestamp & Scan Checkpoint & Frame Index
		if (metadata.el_angle)
		{
			assert(metadata.el_timestamp && metadata.scan_checkpoint && metadata.frame_index); // must all bet set as a group
			uint64_t word{0};
			word |= ((uint64_t) *metadata.el_angle) << 0;
			word |= ((uint64_t) *metadata.el_timestamp) << 20;
			word |= ((uint64_t) *metadata.scan_checkpoint) << 33;
			word |= ((uint64_t) *metadata.frame_index) << 41;

			wr_ptr_++.set(static_cast<uint32_t>(word));
			wr_ptr_++.set(static_cast<uint32_t>(word >> 32));

			metadata_num_words = static_cast<uint8_t>(metadata_num_words + 2);
		}

		set_metadata_num_words(metadata_num_words);
		set_metadata_populated_fields(metadata);

		return metadata_num_words;
	}

	void set_site(const kj::ArrayPtr<const capnp::byte> site)
	{
		const size_t num_words = site.size() / sizeof(uint32_t);
		if (num_words == 0)
			return;

		uint32_t const *const site_32 = reinterpret_cast<uint32_t const *>(&site[0]);

		for (size_t i = 0; i < num_words; ++i)
		{
			wr_ptr_++.set(site_32[i]);
		}
	}

	void set_crc()
	{
		const uint16_t crc = (*this)().second.calculate_crc();
		wr_ptr_.set(((uint32_t) crc) << 16);

		assert((*this)().second.crc_valid());
	}
};

#endif // COMPRESSED_CONTAINER__H
