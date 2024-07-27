#include "ros2_iris_driver/interface/iris_data_interface.hpp"

#include <pthread.h>

#include "lum_common_ros_2_com_pointcloud_codecs/common_layer_codec.h"
#include "lum_common_ros_2_com_pointcloud_codecs/debug_layer_codec.h"
#include "lum_common_ros_2_com_pointcloud_codecs/polar_layer_codec.h"
#include "lum_drivers_lidar_iris_codecs/combined_layer_codec.h"
#include "lum_drivers_lidar_iris_codecs/iris_lidar_layer_codec.h"
#include "lum_platform_interface/lum_platform_interface.h"
#include "pcl_conversions/pcl_conversions.h"

namespace ros2_iris_driver {
namespace interface {

IrisDataInterface::IrisDataInterface(const YAML::Node& lidar_config) {
    lidar_name_ = lidar_config["lidar_name"].as<std::string>();
    dest_port_ = lidar_config["dest_port"].as<uint16_t>();
    buffer_size_ = lidar_config["buffer_size"].as<uint16_t>();

    //! handle reading in pcaps vs connecting live
    is_pcap_ = lidar_config["is_pcap"].as<bool>();
    if (is_pcap_) {
        pcap_filename_ = lidar_config["pcap_filename"].as<std::string>();
    }

    //! Iris Preprocessor configuration
    iris_config_.lines_per_frame =
        lidar_config["lines_per_frame"].as<uint16_t>();
    iris_config_.rays_per_line = lidar_config["rays_per_line"].as<uint16_t>();
    iris_config_.drop_empty_rays = lidar_config["drop_empty_rays"].as<bool>();

    pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pointcloud_msg_->header.frame_id = lidar_name_;

    auto return_type = lidar_config["return_type"].as<std::string>();
    if (return_type == "all") {
        iris_config_.return_filter =
            lum::drivers::lidar::iris::types::ReturnFilter::ALL;
    } else if (return_type == "first") {
        iris_config_.return_filter =
            lum::drivers::lidar::iris::types::ReturnFilter::FIRST;
    } else if (return_type == "last") {
        iris_config_.return_filter =
            lum::drivers::lidar::iris::types::ReturnFilter::LAST;
    } else if (return_type == "strongest") {
        iris_config_.return_filter =
            lum::drivers::lidar::iris::types::ReturnFilter::STRONGEST;
    } else {
        std::cout << "Error: filter_type " << return_type
                  << " not known. Returning STRONGEST." << std::endl;
        iris_config_.return_filter =
            lum::drivers::lidar::iris::types::ReturnFilter::STRONGEST;
    }

    iris_preprocessor_ =
        lum::drivers::lidar::iris::makeIrisPreprocessorA(iris_config_);
    if (!iris_preprocessor_) {
        throw std::runtime_error{"Make lidar data preprocessor failed!"};
    }

    udp_pool_.initialize(1);

    //! making a PCAP Data Client can throw if it can't open the file
    data_source_ =
        is_pcap_ ? lum::drivers::data_client::makePcapDataClient(
                       lum::drivers::pcap::makePcapReader(pcap_filename_), true,
                       true)
                 : lum::drivers::data_client::makeNetworkDataClient(dest_port_);

    packet_subscription_ =
        data_source_->subscribeOnPacket([this](const auto& packet) {
            this->iris_preprocessor_->addPacket(packet);
        });

    frame_subscription_ =
        iris_preprocessor_->subscribeOnFrameEnd([this](const auto& frame_data) {
            this->processIncomingData(frame_data);
        });
    
    udp_packet_thread_ = std::thread(&IrisDataInterface::listenForUdpPackets, this);
}

void IrisDataInterface::stop() {
    can_keep_listening_ = false;
    std::cout << "Waiting for UDP thread to complete" << std::endl;
    // Forcefully terminate the thread
    pthread_cancel(udp_packet_thread_.native_handle());
    
    // Join the thread to clean up
    if (udp_packet_thread_.joinable()) {
        udp_packet_thread_.join();
    }
    
    std::cout << "Closed data listener" << std::endl;
}

IrisDataInterface::~IrisDataInterface() {
    stop();
}

void IrisDataInterface::setPointCloudCallback(
        std::function<void(sensor_msgs::msg::PointCloud2&)>
            pointcloud_callback)
{
    pointcloud_callback_ = pointcloud_callback;
}

void IrisDataInterface::assignCombinedData(
    const lum::common::types::point_cloud::UnstructuredCommonLayer& common_data,
    const lum::common::types::point_cloud::UnstructuredPolarLayer& polar_data,
    const lum::drivers::lidar::iris::types::point_cloud::UnstructuredLidarLayer&
        lidar_data,
    lum::drivers::lidar::iris::types::point_cloud::UnstructuredCombinedLayer&
        combined_data) {
    for (std::size_t i = 0; i < combined_data.size(); i++) {
        auto& combined_point = combined_data.at(i);
        auto common_point = common_data.at(i);
        auto polar_point = polar_data.at(i);
        auto lidar_point = lidar_data.at(i);
        // assign common part
        combined_point.common_data.timestamp = common_point.timestamp;
        combined_point.common_data.x = common_point.x;
        combined_point.common_data.y = common_point.y;
        combined_point.common_data.z = common_point.z;
        combined_point.common_data.reflectance = common_point.reflectance;
        combined_point.common_data.return_index = common_point.return_index;
        combined_point.common_data.last_return_index =
            common_point.last_return_index;
        combined_point.common_data.sensor_id = common_point.sensor_id;
        // assign polar part
        combined_point.polar_data.azimuth = polar_point.azimuth;
        combined_point.polar_data.elevation = polar_point.elevation;
        combined_point.polar_data.depth = polar_point.depth;
        // assign lidar part
        combined_point.lidar_data.line_index = lidar_point.line_index;
        combined_point.lidar_data.detector_site_id =
            lidar_point.detector_site_id;
        combined_point.lidar_data.frame_index = lidar_point.frame_index;
        combined_point.lidar_data.scan_checkpoint = lidar_point.scan_checkpoint;
        combined_point.lidar_data.existence_probability_percent =
            lidar_point.existence_probability_percent;
        combined_point.lidar_data.data_qualifier = lidar_point.data_qualifier;
    }
}

void IrisDataInterface::listenForUdpPackets() {
    while (can_keep_listening_) {
        if (auto packet = udp_pool_.waitForCheckout()) {
            can_keep_listening_ =
                data_source_->listenOnce(std::move(packet));
        }
    }
}

bool IrisDataInterface::processIncomingData(
    const lum::drivers::lidar::iris::types::point_cloud::
        UnstructuredLayeredData& frame_data) {
    if (!pointcloud_callback_) {
        return false;
    }
    const auto end_time =
        std::chrono::high_resolution_clock::now().time_since_epoch();
    auto timestamp = frame_data.common_layer.at(frame_data.common_layer.size() - 1).timestamp;
    auto timestamp2 = frame_data.common_layer.at(0).timestamp;
    
    lum::drivers::lidar::iris::types::point_cloud::UnstructuredCombinedLayer
            combined_data(frame_data.common_layer.size());
    assignCombinedData(frame_data.common_layer, frame_data.polar_layer, frame_data.lidar_layer, combined_data);

    auto result = lum::common::swc::PointCloudCodec<
            lum::drivers::lidar::iris::types::point_cloud::
                UnstructuredCombinedLayer>::encode(combined_data);
    
    result.header.frame_id = lidar_name_;
    result.header.stamp.sec = timestamp.count() * 1e-9;
    result.header.stamp.nanosec = timestamp.count() % static_cast<uint64_t>(1e9);


    pointcloud_callback_(result);

    return true;
}

}  // namespace interface
}  // namespace ros2_iris_driver
