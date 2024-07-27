#ifndef LIBRESIM_POINTCLOUD_IMPL_H
#define LIBRESIM_POINTCLOUD_IMPL_H

#include "MatlabLibReSim.h"
#include "calibrationparser.hpp"
#include "configparser.hpp"
#include "detector.hpp"
#include "resimpointcloud.hpp"
#include "schemas/roi_filter.capnp.h"
#include "schemas/roic705.capnp.h"
#include "schemas/tdc_metadata.capnp.h"

#include <array>
#include <assert.h>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace ReSim
{

using namespace ::ReSim::Schemas;

template <typename SourceSchema> bool LibReSimImpl<SourceSchema, PointCloud>::AngleInRoi(float az, float el)
{
	if (az < roi_.getMinAzimuth())
		return false;
	if (az > roi_.getMaxAzimuth())
		return false;

	if (el < roi_.getMinElevation())
		return false;
	if (el > roi_.getMaxElevation())
		return false;

	return true;
}

template <typename SourceSchema>
LibReSimImpl<SourceSchema, PointCloud>::LibReSimImpl(const Calibration::Reader &calibration, RoiFilter::Reader roi,
                                                     const std::string &pulse_reconstruction_algorithm)
    : LibReSimImpl(CalibrationParser(calibration), roi, pulse_reconstruction_algorithm)
{
}

template <typename SourceSchema>
LibReSimImpl<SourceSchema, PointCloud>::LibReSimImpl(const CalibrationParser &calibration_parser, RoiFilter::Reader roi,
                                                     const std::string &pulse_reconstruction_algorithm)
    : simulator{new MatlabLibReSim()}, calibration_parser{calibration_parser},
      pulse_reconstruction_algorithm_{pulse_reconstruction_algorithm}, roi_{roi},
      both_angle_states{std::make_unique<both_angle_states_struct>()}
{
	simulator->InitAngleEstimationBoth(both_angle_states.get());
}

template <typename SourceSchema> LibReSimImpl<SourceSchema, PointCloud>::~LibReSimImpl() = default;

template <typename SourceSchema>
void LibReSimImpl<SourceSchema, PointCloud>::SetCalibration(const CalibrationParser &calibration_parser_in)
{
	calibration_parser = calibration_parser_in;
}

/**
 * @brief Set the Region Of Interest for what should be processed
 *
 * @note The reader and underlying message must remain valid!
 */
template <typename SourceSchema> void LibReSimImpl<SourceSchema, PointCloud>::SetRoi(RoiFilter::Reader roi) { roi_ = roi; }

template <typename SourceSchema>
void LibReSimImpl<SourceSchema, PointCloud>::EstimateAngles(const TdcMetadata::Reader &metadata, angle_out_struct &angle_out)
{
	const auto az_tab  = metadata.getPolygonTabNumber();
	const auto az_ts   = metadata.getPolygonEdgeTimestamp();
	const auto az_opto = metadata.getPolygonOptoInput();

	const auto el = metadata.getElevationAngle();

	const uint64_t ray_ts = metadata.getTimestamp();

	simulator->EstimateAngles(el, az_tab, az_ts, az_opto, ray_ts, both_angle_states.get(), &angle_out);
}

template <typename SourceSchema>
std::function<void()> LibReSimImpl<SourceSchema, PointCloud>::GetProcessThunk(const typename SourceSchema::Reader &source,
                                                                              PointCloud::Builder &                pc_builder)
{
	const auto &config       = source.getDetectorConfig();
	const bool  config_valid = source.hasDetectorConfig();
	// If we have not been configured yet do not output any rays
	if (configurations.empty() && !config_valid)
	{
		pc_builder.initRays(0);
		return {};
	}

	if (config_valid && configurations.find(config.getConfigId()) == configurations.end())
	{
		last_valid_config_id                 = config.getConfigId();
		const auto config_parser             = ConfigParser(config);
		configurations[last_valid_config_id] = {config_parser.getDetectorConfiguration()};

		simulator->ConfigureDetector(&configurations[last_valid_config_id]);
	}

	const auto   source_rays = source.getFrames();
	unsigned int ray_count   = 0;

	auto angles = std::vector<angle_out_struct>();
	angles.resize(source_rays.size());
	for (unsigned int i = 0; i < source_rays.size(); ++i) // TODO: make me an iterator
	{
		const auto metadata = source_rays[i].getMetadata();

		EstimateAngles(metadata, angles[i]);

		if (!AngleInRoi(static_cast<float>(angles[i].az), static_cast<float>(angles[i].el)))
			continue;

		if (source_rays[i].hasSiteA())
			ray_count += 1;

		if (source_rays[i].hasSiteB())
			ray_count += 1;
	}

	return [=, angles = std::move(angles)]() mutable {
		auto         dest_rays = pc_builder.initRays(ray_count);
		unsigned int ray_idx   = 0;
		for (unsigned int i = 0; i < source_rays.size(); ++i) // TODO: make me an iterator
		{
			if (!AngleInRoi(static_cast<float>(angles[i].az), static_cast<float>(angles[i].el)))
				continue;

			const auto metadata = source_rays[i].getMetadata();

			if (source_rays[i].hasSiteA())
			{
				auto ray_dest = dest_rays[ray_idx++];
				ProcessRay(configurations[last_valid_config_id].siteA, metadata, Roic705::SiteConfig::SITE_A, source_rays[i].getSiteA(),
				           angles[i], ray_dest);
			}

			if (source_rays[i].hasSiteB())
			{
				auto ray_dest = dest_rays[ray_idx++];
				ProcessRay(configurations[last_valid_config_id].siteB, metadata, Roic705::SiteConfig::SITE_B, source_rays[i].getSiteB(),
				           angles[i], ray_dest);
			}
		}
	};
}

/**
 * @brief Convert the input data type to a pointcloud.
 *
 * @note All data from a sensor must be Processed in order. Even if the ROI
 * configuration is used to exclude all points, the angle estimation needs to
 * process all of the metadata.
 */
template <typename SourceSchema>
void LibReSimImpl<SourceSchema, PointCloud>::Process(const typename SourceSchema::Reader &source, PointCloud::Builder &pc_builder)
{
	// make a thunk and run it right away
	auto thunk = GetProcessThunk(source, pc_builder);
	if (thunk)
	{
		thunk();
	}
}

template <typename SourceSchema>
template <typename SiteDataT>
void LibReSimImpl<SourceSchema, PointCloud>::ProcessRay(const site_configuration_struct &  config,
                                                        const Schemas::TdcMetadata::Reader metadata, const uint8_t site_idx,
                                                        const SiteDataT site_data, const angle_out_struct &angles,
                                                        Schemas::PointCloud::Ray::Builder &pc_ray) const
{
	ray_struct rayout{};

	// copy these back out for calibration
	rayout.azAngle = angles.az;
	rayout.elAngle = angles.el;
	rayout.valid   = angles.valid;

	ComputeReturns(config, calibration_parser.calibration->sites[site_idx].detector, site_data, rayout.returns);

	// Count the number of valid returns
	const size_t MAX_RETURNS = sizeof(rayout.returns.range) / sizeof(rayout.returns.range[0]);

	// Count number of returns and resort by moving all NaNs to the end.
	unsigned int num_ret = 0;
	for (unsigned int i = 0; i < MAX_RETURNS; ++i)
	{
		if (isnan(rayout.returns.range[i]))
			continue;

		rayout.returns.range[num_ret]                = rayout.returns.range[i];
		rayout.returns.reflectance[num_ret]          = rayout.returns.reflectance[i];
		rayout.returns.existenceProbability[num_ret] = rayout.returns.existenceProbability[i];
		rayout.returns.debugBits[num_ret]            = rayout.returns.debugBits[i];
		num_ret++;
	}

	simulator->GeometryCalibration(&rayout, &calibration_parser.calibration->sites[site_idx].geometry);

	pc_ray.setAzAngle(static_cast<float>(rayout.azAngle));
	pc_ray.setElAngle(static_cast<float>(rayout.elAngle));

	pc_ray.setValid(rayout.valid);

	pc_ray.setTimestamp(static_cast<uint64_t>(metadata.getTimestamp()));

	pc_ray.setCheckpoint(metadata.getScanCheckpoint());
	pc_ray.setFrameIndex(metadata.getFrameIndex());
	pc_ray.setScanProfile(0); // todo

	pc_ray.setSsi(angles.ssi);

	pc_ray.setSite(site_idx);

	pc_ray.setBlockageLevel(rayout.returns.blockageLevel);

	// Populate the returns
	auto returns = pc_ray.initReturns(num_ret);

	for (unsigned int ret_idx = 0; ret_idx < num_ret; ++ret_idx)
	{
		returns[ret_idx].setRange(static_cast<float>(rayout.returns.range[ret_idx]));
		returns[ret_idx].setReflectance(static_cast<float>(rayout.returns.reflectance[ret_idx]));
		returns[ret_idx].setExistenceProbability(static_cast<float>(rayout.returns.existenceProbability[ret_idx]));
		returns[ret_idx].setDebugBits(static_cast<uint16_t>(rayout.returns.debugBits[ret_idx]));
	}
}

template <>
template <typename SiteDataT>
void LibReSimImpl<Detector, PointCloud>::ComputeReturns(const site_configuration_struct &  config,
                                                        const detector_calibration_struct &calibration,
                                                        const SiteDataT &site_data, returns_struct &returns) const
{
	const auto tdcs = site_data.getTdcs();

	std::array<uint16_t, Roic705::SiteFrame::NUM_TDCS_PER_ROIC_FRAME> roic_frame;
	std::copy(tdcs.begin(), tdcs.end(), roic_frame.begin());

	simulator->ReconstructPulse(&returns, roic_frame.data(), &config, &calibration, pulse_reconstruction_algorithm_);
}

template <>
template <typename SiteDataT>
void LibReSimImpl<Compressed, PointCloud>::ComputeReturns(const site_configuration_struct &  config,
                                                          const detector_calibration_struct &calibration,
                                                          const SiteDataT &site_data, returns_struct &returns) const
{
	// Decompress into this array
	std::array<uint16_t, Roic705::SiteFrame::NUM_TDCS_PER_ROIC_FRAME> roic_frame;

	// Please MATLAB use unsigned integers for array sizes in the future....
	const int32_t  num_tdcs_per_roic_frame = static_cast<int32_t>(Roic705::SiteFrame::NUM_TDCS_PER_ROIC_FRAME);
	const int32_t  num_compressed_words    = static_cast<unsigned int>(site_data.size() / sizeof(uint32_t));
	const uint16_t num_compressed_bits     = num_compressed_words * sizeof(uint32_t) * 8;
	simulator->DecodeSerdesFrame(roic_frame.data(), reinterpret_cast<uint32_t const *>(site_data.begin()), &num_compressed_words,
	                             num_compressed_bits, &config);

	simulator->ReconstructPulse(&returns, roic_frame.data(), &config, &calibration, pulse_reconstruction_algorithm_);
}

} // namespace ReSim

#endif // LIBRESIM_POINTCLOUD_IMPL_H
