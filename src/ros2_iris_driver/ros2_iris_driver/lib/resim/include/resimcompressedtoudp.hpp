#ifndef LIBRESIM_COMPRESSEDTOUDP__H
#define LIBRESIM_COMPRESSEDTOUDP__H

#include "resim.hpp"
#include <array>
#include <map>
#include <memory>

#include "schemas/udp.capnp.h"

#include "mipi_frame.hpp"
#include "status_packet.h"

#include "lum_optional.hpp"

namespace ReSim
{

template <>
class LibReSimImpl<Schemas::Compressed, Schemas::Udp> : public std::true_type,
                                                        public LibReSimInterface<Schemas::Compressed, Schemas::Udp>
{
private:
	static constexpr uint16_t RESIM_SRC_PORT = 4369;
	static constexpr uint16_t RESIM_DST_PORT = 4370;

	static constexpr uint16_t RESIM_STATUS_SRC_PORT = RESIM_SRC_PORT + 1;
	static constexpr uint16_t RESIM_STATUS_DST_PORT = RESIM_DST_PORT;

	static constexpr uint16_t RESIM_PAYLOAD_MAX_SIZE = 1400;

	uint32_t last_mipi_frame_seq_num_{0};
	uint16_t last_container_seq_num_{0};

public:
	LibReSimImpl()  = default;
	~LibReSimImpl() = default;

	void Process(const Schemas::Compressed::Reader &compressed_frame, Schemas::Udp::Builder &packets);

private:
	using CompressedFrame = Schemas::Roic705::RoicFrame<capnp::Data>;

	VariableMetadata           last_metadata{};
	CompressedContainerBuilder CompressedFrameToCompressedContainer(const CompressedFrame::Reader    frame,
	                                                                const be64toh_iterator_nonconst &addr);

	VariableMetadata GetVariableMetadata(const Schemas::TdcMetadata::Reader metadata);
};

} // namespace ReSim

#endif // LIBRESIM_COMPRESSEDTOUDP__H
