// Copyright 2023, Indy Innovation Challenge, Inc. All rights reserved.

#ifndef IRIS_DATA_INTERFACE_HPP_
#define IRIS_DATA_INTERFACE_HPP_

#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>

#include "lum_common_memory/object_pool_heap.h"
#include "lum_drivers_data_client/data_client.h"
#include "lum_drivers_lidar_iris_data_preprocessor_a/iris_preprocessor_a.h"
#include "lum_drivers_lidar_iris_types/point_cloud_layer.h"
#include "lum_drivers_lidar_iris_types/preprocessor_config.h"
#include "lum_drivers_pcap/pcap_reader.h"
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ros2_iris_driver {
namespace interface {

class IrisDataInterface {
   public:
    IrisDataInterface(const YAML::Node& lidar_config);
    ~IrisDataInterface();

    void setPointCloudCallback(
        std::function<void(sensor_msgs::msg::PointCloud2&)>
            pointcloud_callback);
    void stop();

   private:
    bool processIncomingData(
        const lum::drivers::lidar::iris::types::point_cloud::
            UnstructuredLayeredData& frame_data);

    void assignCombinedData(
    const lum::common::types::point_cloud::UnstructuredCommonLayer& common_data,
    const lum::common::types::point_cloud::UnstructuredPolarLayer& polar_data,
    const lum::drivers::lidar::iris::types::point_cloud::UnstructuredLidarLayer&
        lidar_data,
    lum::drivers::lidar::iris::types::point_cloud::UnstructuredCombinedLayer&
        combined_data);
    
    void listenForUdpPackets();

    std::string lidar_name_;
    std::string pcap_filename_;
    uint16_t dest_port_;
    uint16_t buffer_size_;
    bool is_pcap_;
    lum::drivers::lidar::iris::types::PreprocessorConfig iris_config_;
    std::unique_ptr<lum::drivers::lidar::iris::IIrisPreprocessor>
        iris_preprocessor_;
    std::unique_ptr<lum::drivers::data_client::IDataClient> data_source_;
    lum::drivers::data_client::types::PacketSubscription packet_subscription_;
    lum::drivers::lidar::iris::types::UnstructuredLayeredDataSubscription
        frame_subscription_;
    lum::common::memory::object_pool::SmartPoolHeap<
        lum::common::types::networking::UdpPacket>
        udp_pool_;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_;
    std::thread udp_packet_thread_{};
    bool can_keep_listening_{true};
    std::function<void(sensor_msgs::msg::PointCloud2&)>
        pointcloud_callback_;
};
}  // namespace interface
}  // namespace ros2_iris_driver

#endif  // IRIS_DATA_INTERFACE_HPP_
