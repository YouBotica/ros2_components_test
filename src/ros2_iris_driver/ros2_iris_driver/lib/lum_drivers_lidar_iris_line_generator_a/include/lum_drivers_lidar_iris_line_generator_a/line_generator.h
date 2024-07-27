// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_LINE_GENERATOR_A_LINE_GENERATOR_H
#define LUM_DRIVERS_LIDAR_IRIS_LINE_GENERATOR_A_LINE_GENERATOR_H

#include <array>
#include <limits>
#include <memory>
#include <mutex>

#include <lum_common_types_observable/observable.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>
#include <lum_drivers_lidar_iris_internal_types/i_frame_boundary_detector.h>
#include <lum_drivers_lidar_iris_line_generator_a/i_point_cloud_line_generator_a.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class PointCloudLineGenerator : public IPointCloudLineGeneratorA
{
public:
  explicit PointCloudLineGenerator(const std::uint8_t sensor_id,
                                   IPointCloudLineGenerator::WithResimIndexing with_resim_indexing =
                                     IPointCloudLineGenerator::WithResimIndexing(false));

  /// @copydoc lum::drivers::lidar::iris::IPointCloudLineGeneratorA::getSensorId
  std::uint8_t getSensorId() const override;

  /// @copydoc lum::drivers::lidar::iris::IPointCloudLineGeneratorA::setSensorId
  void setSensorId(std::uint8_t sensor_id) override;

  /// @copydoc lum::drivers::lidar::iris::IPointCloudLineGeneratorA::reset
  void reset() override;

  /// @copydoc
  /// lum::drivers::lidar::iris::IPointCloudLineGeneratorA::subscribeOnScanLineSegment(const
  /// types::PointCloudLineSubscriptionCallback&)
  types::PointCloudLineSubscription
  subscribeOnScanLineSegment(const types::PointCloudLineSubscriptionCallback& callback) override;

  /// @copydoc
  /// lum::drivers::lidar::iris::IPointCloudLineGeneratorA::subscribeOnScanLineSegment(const
  /// types::PointCloudSupplementalLineSubscriptionCallback&)
  types::PointCloudSupplementalLineSubscription subscribeOnScanLineSegment(
    const types::PointCloudSupplementalLineSubscriptionCallback& callback) override;

  /// @copydoc lum::drivers::lidar::iris::IPointCloudLineGeneratorA::add
  void add(const types::LidarPacketMetadata& metadata, const types::LidarRays& rays) override;

private:
  std::uint8_t sensor_id_{0};             ///< sensor id option
  std::uint8_t ssi_delta_{NUM_DETECTORS}; ///< resim and point cloud index differently
  std::mutex mutex_;                      ///< block concurrent access to addBundle
  std::uint8_t line_sequence_{0}; ///< index lines we emit to make missed lines easier to detect
  std::unique_ptr<IFrameBoundaryDetector>
    frame_detector_; ///< detect line resets on frame boundaries

  /// @brief internal utility struct that holds state for a line
  struct LineData
  {
    types::LinePointsWithSupplementalFields points_with_metadata{};

    bool meta_written{false}; ///< whether this line has valid metadata
    std::uint32_t ssi{
      std::numeric_limits<std::uint32_t>::max()}; ///< "scan segment index", the line counter
                                                  ///< inside the sensor head
    std::uint8_t line_index{std::numeric_limits<std::uint8_t>::max()}; ///< line index
  };

  /// @brief array that maintains the current line data for each eye

  std::array<LineData, NUM_DETECTORS> current_line_data_;

  using UnstructuredPointCloudConstPtr =
    common::types::point_cloud::UnstructuredPointCloudConstPtr<types::LinePoint>;

  common::types::observable::Observable<const UnstructuredPointCloudConstPtr&,
                                        const types::PointCloudLineMeta&>
    line_observable_{}; ///< observable for new Lines

  common::types::observable::Observable<const types::LinePointsWithSupplementalFields&>
    line_with_supplemental_fields_observable_{}; ///< observable for new Lines

  void addPoint(const types::LinePoint& cloud_point,
                const types::PointCloudLineMeta& packet_meta,
                const types::SupplementalPointField supplemental_field,
                const std::uint32_t ssi);

  /// @brief utility function to publish a line, and update state tracking
  /// @param eye [in] eye index
  /// @param reset_counts [in] omit or set to true to reset frame metadata
  void publishLine(const std::uint8_t eye, const bool reset_counts = true);

  void publishAll();
  void resetOnNewFrame();
  std::uint8_t nextLineIndex();

  void clearState();
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_LINE_GENERATOR_A_LINE_GENERATOR_H
