// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_RESIM_PIPELINE_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_RESIM_PIPELINE_H

#include <memory>

#include <lum_drivers_lidar_iris_resim_internal/i_resim_processor.h>
#include <lum_drivers_lidar_iris_resim_internal/packet_batcher.h>
#include <lum_drivers_lidar_iris_resim_internal/resim_line_generator.h>
#include <lum_drivers_lidar_iris_resim_internal/types.h>
#include <lum_drivers_lidar_iris_types_resim/i_resim_scanline_processor.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

///////////////////////////////////////////////////////////////////////////////////////////////////
class ResimPipeline : public virtual IResimScanLineProcessor
{
public:
  ResimPipeline(std::unique_ptr<PacketBatcher> packet_batcher,
                std::unique_ptr<IResimProcessor> resim_processor,
                std::unique_ptr<ResimScanLineGenerator> line_generator);

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  std::uint8_t getSensorId() const override { return line_generator_->getSensorId(); }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  void setSensorId(std::uint8_t sensor_id) override { line_generator_->setSensorId(sensor_id); }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  std::pair<std::string, std::string> getNameAndVersion() const override
  {
    return {"Iris Variant A - Resim Version", "2.3.0"};
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  void reset() override
  {
    line_generator_->reset();
    resim_processor_->reset();
    packet_batcher_->reset();
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  bool isProcessingDone() const override;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  bool addPacket(const types::PooledUdpPacket& buffer) override;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  bool addPacket(const std::uint8_t* buffer,
                 std::size_t packet_size,
                 const common::types::networking::UdpHeader& header) override;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  iris::types::PointCloudLineSubscription subscribeOnScanLineSegment(
    const iris::types::PointCloudLineSubscriptionCallback& callback) override;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  iris::types::PointCloudSupplementalLineSubscription subscribeOnScanLineSegment(
    const iris::types::PointCloudSupplementalLineSubscriptionCallback& callback) override;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  iris::types::UnstructuredLayeredDataSubscription subscribeOnScanLineSegment(
    const iris::types::UnstructuredLayeredDataSubscriptionCallback&) override
  {
    return {};
  }

private:
  // Note: Order of destruction (implicit in the order of declaration) is important here
  std::unique_ptr<ResimScanLineGenerator> line_generator_;

protected:
  std::unique_ptr<IResimProcessor> resim_processor_;

private:
  std::unique_ptr<PacketBatcher> packet_batcher_;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // RESIM_PIPELINE_RESIM_PIPELINE_H
