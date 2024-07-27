#ifndef LIBRESIM_DETECTOR__H
#define LIBRESIM_DETECTOR__H

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
class LibReSimImpl<Schemas::PointCloud, Schemas::Detector> : public std::true_type,
                                                             public LibReSimInterface<Schemas::PointCloud, Schemas::Detector>
{

private:
	std::unique_ptr<MatlabLibReSim> simulator;

	std::unique_ptr<ConfigParser> config_parser;

	std::unique_ptr<struct sim_angle_state_struct> sim_angle_state;

	void ProcessRayReturns(const capnp::List<::ReSim::Schemas::PointCloud::Return, capnp::Kind::STRUCT>::Reader &rays,
	                       Schemas::Roic705::SiteFrame::Builder &                                                site_frame);

public:
	LibReSimImpl(const Schemas::Roic705::Config::Reader &detector_config);
	~LibReSimImpl();

	void Process(const Schemas::PointCloud::Reader &point_cloud, Schemas::Detector::Builder &detector_builder);

	void ProcessRay(const Schemas::PointCloud::Ray::Reader &ray_site_a, const Schemas::PointCloud::Ray::Reader &ray_site_b,
	                Schemas::Roic705::RoicFrame<Schemas::Roic705::SiteFrame>::Builder &roic_frame);

	void ProcessRayMetadata(const Schemas::PointCloud::Ray::Reader &ray, Schemas::TdcMetadata::Builder &metadata_builder);
};

} // namespace ReSim

#endif // LIBRESIM_DETECTOR__H
