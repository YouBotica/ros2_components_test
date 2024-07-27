#ifndef LIBRESIM_DETECTORCOMPRESSION__H
#define LIBRESIM_DETECTORCOMPRESSION__H

#include "resim.hpp"
#include <memory>

// Forward declaration of PIMPL-Hidden members
class MatlabLibReSim;
struct sim_angle_state_struct;

namespace ReSim
{

// Forward declaration of PIMPL-Hidden members
class ConfigParser;

template <>
class LibReSimImpl<Schemas::PointCloud, Schemas::Compressed> : public std::true_type,
                                                               public LibReSimInterface<Schemas::PointCloud, Schemas::Compressed>
{

private:
	std::unique_ptr<MatlabLibReSim> simulator;

	std::unique_ptr<ConfigParser> config_parser;

	std::unique_ptr<struct sim_angle_state_struct> sim_angle_state;

public:
	LibReSimImpl(const Schemas::Roic705::Config::Reader &compressed_config);
	~LibReSimImpl();

	void Process(const Schemas::PointCloud::Reader &point_cloud, Schemas::Compressed::Builder &compressed_builder);

	void ProcessRay(const Schemas::PointCloud::Ray::Reader &ray_site_a, const Schemas::PointCloud::Ray::Reader &ray_site_b,
	                Schemas::Roic705::RoicFrame<capnp::Data>::Builder &compressed_frame) const;

	void ProcessRayMetadata(const Schemas::PointCloud::Ray::Reader &ray, Schemas::TdcMetadata::Builder &metadata_builder) const;
};

} // namespace ReSim

#endif // LIBRESIM_DETECTORCOMPRESSION__H
