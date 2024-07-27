#ifndef LIBRESIM_DECOMPRESSION__H
#define LIBRESIM_DECOMPRESSION__H

#include "resim.hpp"
#include <map>
#include <memory>

// Forward declaration of PIMPL-Hidden members
class MatlabLibReSim;
struct site_configuration_struct;

namespace ReSim
{

// Forward declaration of PIMPL-Hidden members
class ConfigParser;

template <>
class LibReSimImpl<Schemas::Compressed, Schemas::Detector> : public std::true_type,
                                                             public LibReSimInterface<Schemas::Compressed, Schemas::Detector>
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

	void Process(const Schemas::Compressed::Reader &compressed_frame, Schemas::Detector::Builder &detector_builder);

	void ProcessSiteFrame(const Schemas::Roic705::Config::Reader &config, const capnp::Data::Reader &compressed,
	                      Schemas::Roic705::SiteFrame::Builder &roic_site, const uint8_t which_detector);

	void ProcessCompressedFrame(const Schemas::Roic705::Config::Reader &                           config,
	                            const Schemas::Roic705::RoicFrame<capnp::Data>::Reader &           compressed_reader,
	                            Schemas::Roic705::RoicFrame<Schemas::Roic705::SiteFrame>::Builder &roic_frame);
};

} // namespace ReSim

#endif // LIBRESIM_DECOMPRESSION__H
