// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_ASYNC_LINE_PROCESSOR_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_ASYNC_LINE_PROCESSOR_H

#include <atomic>
#include <future>
#include <memory>
#include <mutex>

#include <lum_common_multi_thread/jthread.h>
#include <lum_common_multi_thread/thread_pool.h>
#include <lum_common_multi_thread/work_queue.h>
#include <lum_drivers_lidar_iris_resim_internal/i_resim_processor.h>
#include <lum_drivers_lidar_iris_types_resim/region_of_interest.h>
#include <resimpointcloud.hpp>
#include <resimudptocompressed.hpp>
#include <schemas/compressed.capnp.h>
#include <schemas/roi_filter.capnp.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief class that listens for ReSim compressed messages and outputs lidar scan lines
class AsyncResimScanLineProcessor : public virtual IResimProcessor
{
public:
  AsyncResimScanLineProcessor(
    std::unique_ptr<ReSim::LibReSim<ReSim::Schemas::Compressed, ReSim::Schemas::PointCloud>> resim,
    const resim::IrisRegionOfInterest& roi_filter,
    const std::size_t num_worker_threads,
    const std::size_t internal_queue_size);

  ~AsyncResimScanLineProcessor() override;

  // Rule of 5 boilerplate
  AsyncResimScanLineProcessor(const AsyncResimScanLineProcessor&) = delete;
  AsyncResimScanLineProcessor(AsyncResimScanLineProcessor&&) = delete;
  void operator=(const AsyncResimScanLineProcessor&) & = delete;
  void operator=(AsyncResimScanLineProcessor&&) & = delete;

  bool isProcessingDone() const final;

  void printStats();

  void addMessageBatch(types::OwnedBatch msgs) final;

  void setCallback(Callback callback) final { callback_ = callback; };

  void reset() final;

protected:
  ReSim::LibReSim<ReSim::Schemas::Udp, ReSim::Schemas::Compressed>
    resim_udp_to_compressed_{}; ///< Converter for UDP packets to compressed messages

private:
  std::atomic_bool is_running_{true};
  Callback callback_;

  std::unique_ptr<ReSim::LibReSim<ReSim::Schemas::Compressed, ReSim::Schemas::PointCloud>>
    resim_decompression_point_cloud_;

  types::Message<ReSim::Schemas::RoiFilter> roi_filter_;

  using ThreadPool = lum::common::multi_thread::thread_pool::Pool;
  template <class T>
  using CapQueue = lum::common::multi_thread::ThreadSafeCapacityQueue<T>;

  std::unique_ptr<lum::common::multi_thread::JThread> stats_thread_{};

  std::unique_ptr<lum::common::multi_thread::JThread> udp_to_compressed_stage_1_thread_{};
  std::unique_ptr<lum::common::multi_thread::JThread> udp_to_compressed_stage_2_thread_{};

  struct UPDToCompressedJob
  {
    types::OwnedBatch in;
    std::function<void()> func;
    types::Message<ReSim::Schemas::Compressed> out;
  };

  /// job queue of packets to convert to compressed
  std::unique_ptr<CapQueue<types::OwnedBatch>> udp_to_compressed_stage_1_job_queue_;

  ///< job queue of packets to convert to compressed
  std::unique_ptr<CapQueue<UPDToCompressedJob>> udp_to_compressed_stage_2_job_queue_;

  /// job queue of packets to convert to compressed
  std::unique_ptr<CapQueue<types::Message<ReSim::Schemas::Compressed>>>
    decompression_pointcloud_stage_1_job_queue_;

  std::unique_ptr<lum::common::multi_thread::JThread> decompression_pointcloud_stage_1_thread_{};

  struct DecompressionPointCloudJob
  {
    types::Message<ReSim::Schemas::Compressed> in;
    std::function<void(void)> func;
    types::Message<ReSim::Schemas::PointCloud> out;
  };

  /// job queue of thunks from DecompressionPointcloud to complete
  std::unique_ptr<CapQueue<DecompressionPointCloudJob>> decompression_pointcloud_stage_2_job_queue_;

  /// thread pool with workers for running DecompressionPointcloud thunks
  ThreadPool decompression_pointcloud_stage_2_thread_pool_{};

  /// job queue of thunks from DecompressionPointcloud to complete
  std::unique_ptr<CapQueue<std::future<types::Message<ReSim::Schemas::PointCloud>>>>
    pointcloud_collate_queue_;
  std::unique_ptr<lum::common::multi_thread::JThread> pointcloud_collation_thread_;

  /// Preserves the order while transferring parallelized decompression jobs between queue
  mutable std::mutex pointcloud_collation_mutex_{};

  // Thread-main functions
  void runUdpToCompressedProcessingStage1();
  void runUdpToCompressedProcessingStage2();
  void runDecompressionStage1();
  void runDecompressionStage2Parallel();
  void runCollate();

  std::atomic<std::int64_t> job_counter_{0};         ///< count jobs in-flight
  std::atomic<std::int64_t> dropped_job_counter_{0}; ///< count dropped jobs
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
