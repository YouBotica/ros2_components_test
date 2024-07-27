#ifndef LIBRESIM_POINTCLOUD_H
#define LIBRESIM_POINTCLOUD_H

#include "calibrationparser.hpp"
#include "resim.hpp"
#include "schemas/detectorcalibration.capnp.h"
#include "schemas/roi_filter.capnp.h"
#include <functional>
#include <map>
#include <memory>

// Hidden Implementations from matlab
class MatlabLibReSim;
struct both_angle_states_struct;
struct angle_out_struct;
struct detector_configuration_struct;
struct site_configuration_struct;
struct detector_calibration_struct;
struct returns_struct;

namespace ReSim
{
using namespace ::ReSim::Schemas;

template <typename SourceSchema>
class LibReSimImpl<SourceSchema, Schemas::PointCloud> : public std::true_type,
                                                        public LibReSimInterface<SourceSchema, Schemas::PointCloud>
{
private:
	std::unique_ptr<MatlabLibReSim>           simulator;
	CalibrationParser                         calibration_parser;
	const std::string                         pulse_reconstruction_algorithm_;
	std::unique_ptr<both_angle_states_struct> both_angle_states;
	Schemas::RoiFilter::Reader                roi_;

	bool AngleInRoi(float az, float el);

	using config_id_t = uint8_t;
	config_id_t last_valid_config_id;
	struct roic_detector_config_containers;

	std::map<config_id_t, detector_configuration_struct> configurations;

	void EstimateAngles(const Schemas::TdcMetadata::Reader &metadata, angle_out_struct &angle_out);

	template <typename SiteDataT>
	void ProcessRay(const site_configuration_struct &config, const Schemas::TdcMetadata::Reader metadata, const uint8_t site_idx,
	                const SiteDataT site_data, const angle_out_struct &angles, Schemas::PointCloud::Ray::Builder &pc_ray) const;

	template <typename SiteDataT>
	void ComputeReturns(const site_configuration_struct &config, const detector_calibration_struct &calibration,
	                    const SiteDataT &site_data, returns_struct &returns) const;

public:
	LibReSimImpl(const Schemas::Calibration::Reader &calibration, Schemas::RoiFilter::Reader roi = {},
	             const std::string &pulse_reconstruction_algorithm = "");
	LibReSimImpl(const CalibrationParser &calibration_parser, Schemas::RoiFilter::Reader roi = {},
	             const std::string &pulse_reconstruction_algorithm = "");
	~LibReSimImpl();

	void SetCalibration(const CalibrationParser &calibration_parser);

	/**
	 * @brief Set the Region Of Interest for what should be processed
	 *
	 * @note The reader and underlying message must remain valid!
	 */
	void SetRoi(Schemas::RoiFilter::Reader roi);

	std::function<void()> GetProcessThunk(const typename SourceSchema::Reader &source, Schemas::PointCloud::Builder &pc_builder);

	/**
	 * @brief Convert the input data type to a pointcloud.
	 *
	 * @note All data from a sensor must be Processed in order. Even if the ROI
	 * configuration is used to exclude all points, the angle estimation needs to
	 * process all of the metadata.
	 */
	void Process(const typename SourceSchema::Reader &source, Schemas::PointCloud::Builder &pc_builder);
};

} // namespace ReSim

#endif // LIBRESIM_POINTCLOUD_H
