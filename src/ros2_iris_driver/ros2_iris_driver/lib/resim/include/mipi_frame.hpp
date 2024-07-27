#ifndef MIPI_FRAME__H
#define MIPI_FRAME__H

#include <algorithm>
#include <array>
#include <cassert> // assert
#include <cstddef> // ptrdiff_t
#include <iostream>
#include <istream>
#include <iterator>    // iterator
#include <type_traits> // remove_cv
#include <utility>     // swap

#include "be64toh_iterator.hpp"
#include "compressed_container.hpp"
#include "crc16.h"
#include "pcap_parsing_errors.hpp"

struct MipiFrame
{
public:
	static constexpr uint16_t VERSION{2};

	/**
	 * Keep the builder outputting version 1, as support for v2 has not been implemented yet.
	 */
	static constexpr uint16_t BUILDER_VERSION{1};

	/**
	 * @param start_ptr pointer to the start of the message.
	 * @param past_ptr pointer to one word past the valid container. Will not be dereferenced.
	 * @param check_errors disables CRC etc.; set to true for better performance if data is known to be good.
	 */
	MipiFrame(uint32_t const *const start_ptr, uint32_t const *const past_ptr, const VariableMetadata &previous_metadata,
	          const bool check_errors = true)
	    : MipiFrame(start_ptr, past_ptr, check_errors)
	{
		previous_metadata_ = previous_metadata;
	}

	MipiFrame(uint32_t const *const start_ptr, uint32_t const *const past_ptr, const bool check_errors = true)
	    : num_words_{static_cast<size_t>(past_ptr - start_ptr)}
	{
		const be64toh_iterator_const start_ptr_{start_ptr};
		const be64toh_iterator_const past_ptr_{past_ptr};
		data_ = std::unique_ptr<uint32_t[]>(new uint32_t[num_words_]);
		std::copy(start_ptr_, past_ptr_, &data_[0]);

		if (!check_errors)
		{
			return;
		}

		// Error checking
		if (get_version() > VERSION)
		{
			std::cerr << "ERROR: ReSim packet version (" << get_version() << ") newer than ReSim Library's max supported version ("
			          << VERSION << ")!\n";
			throw MalformedMipiFrame{};
		}

		if ((!start_ptr || !past_ptr))
		{
			std::cerr << "Null start/past pointer in MipiFrame!\n";
			throw MalformedMipiFrame{};
		}

		if (std::distance(start_ptr, past_ptr) < (long) FOOTER__NUM_WORDS)
		{
			std::cerr << "Packet not long enough for the footer!\n";
			throw MalformedMipiFrame{};
		}

		if (!footer_crc_valid())
		{
			std::cerr << "MipiFrame CRC Invalid!\n";
			throw MalformedMipiFrame{};
		}
	}

	VariableMetadata previous_metadata_;

private:
	std::unique_ptr<uint32_t[]> data_;
	const size_t                num_words_;

	static const size_t FOOTER__NUM_WORDS{8};
	static const size_t EYE_COUNT_OFFSET{FOOTER__NUM_WORDS - 3};
	static const size_t FRAME_SEQUENCE_OFFSET{FOOTER__NUM_WORDS - 2};
	static const size_t CRC_OFFSET{FOOTER__NUM_WORDS - 1};

	friend struct MipiFrameBuilder;

public:
	uint32_t get_sequence_number() const { return data_[FRAME_SEQUENCE_OFFSET]; }
	uint16_t get_version() const { return static_cast<uint16_t>(data_[CRC_OFFSET] & 0xFFFF); }
	uint16_t get_footer_crc() const { return static_cast<uint16_t>((data_[CRC_OFFSET] >> 16) & 0xFFFF); }

	uint16_t calculate_crc() const
	{
		// Calculate the CRC of everything but the CRC field in the last 16 bits
		const uint16_t crc = crc16_bit(0, (uint8_t *) &data_[0], ((FOOTER__NUM_WORDS) * sizeof(uint32_t)) - sizeof(uint16_t));

		return crc;
	}
	bool footer_crc_valid() const { return get_footer_crc() == calculate_crc(); }

	/**
	 * @brief Iterator over compressed containers in a MipiFrame
	 *
	 * @warning methods may throw exceptions if MipiFrames are malformed
	 */
	class CompressedContainerIterator : public std::iterator<std::forward_iterator_tag, CompressedContainer, unsigned int,
	                                                         CompressedContainer *, CompressedContainer &>
	{
		uint32_t const * container_;
		VariableMetadata previous_metadata_;
		const uint16_t   packet_version_;

	public:
		CompressedContainerIterator(const uint32_t *container, const VariableMetadata &previous_metadata,
		                            const uint16_t packet_version)
		    : container_{container}, previous_metadata_{previous_metadata}, packet_version_{packet_version}
		{
		}

		CompressedContainerIterator &operator++() // Pre-increment
		{
			const auto previous_container = CompressedContainer{container_, previous_metadata_, packet_version_};
			previous_metadata_            = previous_container.get_metadata();

			container_ += previous_container.get_num_words();
			return *this;
		}

		CompressedContainerIterator operator++(int) // Post-increment
		{
			const auto previous_container = CompressedContainer{container_, previous_metadata_, packet_version_};
			previous_metadata_            = previous_container.get_metadata();

			CompressedContainerIterator tmp(*this);
			container_ += previous_container.get_num_words();
			return tmp;
		}

		bool operator==(const CompressedContainerIterator &rhs) const
		{
			// if the pointers are equal then we know (mipi_frame_ == rhs.mipi_frame_)
			return container_ == rhs.container_;
		}
		bool operator!=(const CompressedContainerIterator &rhs) const { return !(*this == rhs); }

		CompressedContainer operator*() const { return CompressedContainer{container_, previous_metadata_, packet_version_}; }

		CompressedContainer operator->() const { return CompressedContainer{container_, previous_metadata_, packet_version_}; }
	};

	CompressedContainerIterator begin() const
	{
		return CompressedContainerIterator{&data_[FOOTER__NUM_WORDS], previous_metadata_, this->get_version()};
	}
	CompressedContainerIterator end() const { return CompressedContainerIterator{&data_[num_words_], {}, this->get_version()}; }

	uint32_t get_num_containers() const
	{
		// The mipi frame is split into several
		// packets. start_ptr_[EYE_COUNT_OFFSET] will tell you how many containers
		// the original mipi frame had, but not how many made it into this packet.
		try
		{
			return std::distance(begin(), end());
		}
		catch (const MalformedMipiFrame &e)
		{
			std::cerr << "Failed getting number of containers: " << e.what() << std::endl;
			return 0;
		}
	}
};

struct MipiFrameBuilder
{
	MipiFrameBuilder(const be64toh_iterator_nonconst &addr, uint32_t seq_num) : data_start_{addr}
	{
		set_sequence_number(seq_num);
		set_version();

		set_crc();
	}
	~MipiFrameBuilder()                             = default;
	MipiFrameBuilder(const MipiFrameBuilder &other) = default;
	MipiFrameBuilder &operator=(const MipiFrameBuilder &) = default;

	MipiFrame operator()() const
	{
		uint32_t const *const start_ptr = data_start_.get_start_ptr();
		return MipiFrame{start_ptr, start_ptr + MipiFrame::FOOTER__NUM_WORDS, {}, false};
	}

	static const size_t NUM_WORDS = MipiFrame::FOOTER__NUM_WORDS;

private:
	be64toh_iterator_nonconst data_start_;

	//  	void set_num_containers(const uint32_t num_containers) { (data_start_ + 0).set(*(data_start_ + 0) | ((uint32_t) num_words
	//  & 0xFF)); }
	void set_sequence_number(const uint32_t seq_num) { (data_start_ + MipiFrame::FRAME_SEQUENCE_OFFSET).set(seq_num); }

	void set_version()
	{
		const uint32_t prev = *(data_start_ + MipiFrame::CRC_OFFSET) & 0xFFFFU << 16;
		(data_start_ + MipiFrame::CRC_OFFSET).set(prev | (uint32_t) MipiFrame::BUILDER_VERSION);
	}

	void set_crc()
	{
		const uint16_t crc  = (*this)().calculate_crc();
		const uint32_t prev = *(data_start_ + MipiFrame::CRC_OFFSET) & 0xFFFFU;
		(data_start_ + MipiFrame::CRC_OFFSET).set(prev | (((uint32_t) crc) << 16));
	}
};

#endif // MIPI_FRAME__H
