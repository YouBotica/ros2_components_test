#ifndef LIBRESIM_COMPRESSION__H
#define LIBRESIM_COMPRESSION__H

#include "resim.hpp"
#include <map>
#include <memory>

// Forward declaration of PIMPL-Hidden members
class MatlabLibReSim;
struct site_configuration_struct;

namespace ReSim
{

template <>
class LibReSimImpl<Schemas::Detector, Schemas::Compressed> : public std::true_type,
                                                             public LibReSimInterface<Schemas::Detector, Schemas::Compressed>
{

private:
	std::unique_ptr<MatlabLibReSim> simulator;

	using detector_config_container = struct
	{
		uint8_t                                           config_id = 0;
		std::unique_ptr<struct site_configuration_struct> config;
	};

	// Key: whichDetector
	std::map<uint8_t, detector_config_container> configurations;

public:
	LibReSimImpl();
	~LibReSimImpl();

	void Process(const Schemas::Detector::Reader &detector, Schemas::Compressed::Builder &compressed_builder);

	kj::ArrayPtr<capnp::byte> ProcessRoicSite(const Schemas::Roic705::Config::Reader &   config,
	                                          const Schemas::Roic705::SiteFrame::Reader &site_frame, const uint8_t which_site);

	void ProcessRoicFrame(const Schemas::Roic705::Config::Reader &                                config,
	                      const Schemas::Roic705::RoicFrame<Schemas::Roic705::SiteFrame>::Reader &roic_frame,
	                      Schemas::Roic705::RoicFrame<capnp::Data>::Builder &                     compressed_frame);
};

} // namespace ReSim

#endif // LIBRESIM_COMPRESSION__H
