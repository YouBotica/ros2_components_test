#ifndef LIBRESIM_UDPTOCOMPRESSED__H
#define LIBRESIM_UDPTOCOMPRESSED__H

#include "resim.hpp"
#include <array>
#include <functional>
#include <map>
#include <memory>

#include "schemas/udp.capnp.h"

#include "mipi_frame.hpp"
#include "status_packet.h"

#include "lum_optional.hpp"

#if defined(_WIN32)
// Windows' cmmdlg.h get included indirectly here and naughtily #defines this symbol, wreaking havoc for CapnProto
#undef INTERFACE
#endif

namespace ReSim
{

template <>
class LibReSimImpl<Schemas::Udp, Schemas::Compressed> : public std::true_type,
                                                        public LibReSimInterface<Schemas::Udp, Schemas::Compressed>
{
public:
	LibReSimImpl();
	~LibReSimImpl();

	static constexpr uint16_t RESIM_SRC_PORT = 4369;
	static constexpr uint16_t RESIM_DST_PORT = 4370;

	static constexpr uint16_t RESIM_STATUS_SRC_PORT = RESIM_SRC_PORT + 1;
	static constexpr uint16_t RESIM_STATUS_DST_PORT = RESIM_DST_PORT;

	/**
	 * @brief Return the version of the given packet iff it is a mipi frame.
	 * @note Status packets will return NONE
	 */
	lum_optional<uint16_t> GetVersionOfPacket(const Schemas::Udp::Packet::Reader &packet);

	std::function<void(void)> GetProcessThunk(const Schemas::Udp::Reader &udp, Schemas::Compressed::Builder &compressed);

	void Process(const Schemas::Udp::Reader &packet, Schemas::Compressed::Builder &compressed_frame);

private:
	uint32_t last_mipi_frame_seq{0};
	uint16_t last_container_seq{0};
	bool     first_mipi_frame{true};
	bool     expect_container_seq_jump{true};

	lum_optional<StatusPacket> last_status_packet_;

	VariableMetadata metadata_{};

	lum_optional<MipiFrame>    PacketToMipiFrame(const Schemas::Udp::Packet::Reader packet) const;
	lum_optional<StatusPacket> PacketToStatusPacket(const Schemas::Udp::Packet::Reader packet) const;

	using CompressedFrame     = Schemas::Roic705::RoicFrame<capnp::Data>;
	using CompressedFrameIter = capnp::List<CompressedFrame>::Builder::Iterator;

	void MipiFrameToCompressedFrames(const MipiFrame &mipi_frame, CompressedFrameIter &compressed);
};

} // namespace ReSim

#endif // LIBRESIM_UDPTOCOMPRESSED__H
