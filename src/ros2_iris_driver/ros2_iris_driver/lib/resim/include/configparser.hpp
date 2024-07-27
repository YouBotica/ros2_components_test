#ifndef CONFIG_PARSER__H
#define CONFIG_PARSER__H

#include "MatlabLibReSim.h"
#include "Simulate705ROICFrame_types.h" // for struct detector_configuration_struct
#include "resim.hpp"

namespace ReSim
{

class ConfigParser
{
public:
	// Constructor
	ConfigParser(const ReSim::Schemas::Roic705::Config::Reader &detector_config) { Parse(detector_config); }

	inline const detector_configuration_struct &getDetectorConfiguration() const { return detector_configuration; }

private:
	void Parse(const ReSim::Schemas::Roic705::Config::Reader &detector_config);

	void ParseSite(const ReSim::Schemas::Roic705::SiteConfig::Reader &detector_config, site_configuration_struct &dst);

	detector_configuration_struct detector_configuration;
};
} // namespace ReSim

#endif /*CONFIG_PARSER__H*/
