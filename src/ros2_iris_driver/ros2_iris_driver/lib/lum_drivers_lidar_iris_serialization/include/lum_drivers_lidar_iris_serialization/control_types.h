// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_SERIALIZATION_CONTROL_TYPES_H
#define LUM_DRIVERS_LIDAR_IRIS_SERIALIZATION_CONTROL_TYPES_H

#include <array>
#include <cstdint>

#include <lum_common_types/utils.h>
#include <lum_common_types_internal/macros.h>
#include <lum_platform_networking/utils.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace serialization {

using lum::common::types::utils::toUnderlyingValue;

// Iris control data types (wire-format)

/// @brief Events offered by all services of the Iris sensor
enum class SensorEvent : std::uint16_t
{
  SYSTEM_MODE = 0x0001,
  SYSTEM_HEARTBEAT = 0x0002,
  SCAN_SETTINGS = 0x0001,
  SCAN_DATUM = 0x0002,
  SCAN_COMMAND = 0x0003
};

/// @brief Event groups used by the Iris sensor
enum class SensorEventGroup : std::uint16_t
{
  SYSTEM_MODE = 0x0002,
  SCAN_PATTERN = 0x0003
};

/// @brief Iris sensor event group ids
constexpr std::uint16_t SYSTEM_MODE_EVENT_GROUP_ID{
  toUnderlyingValue(SensorEventGroup::SYSTEM_MODE)};
constexpr std::uint16_t SCAN_PATTERN_EVENT_GROUP_ID{
  toUnderlyingValue(SensorEventGroup::SCAN_PATTERN)};

/// @brief Methods offered by all services of the Iris sensor
enum class SensorMethod : std::uint16_t
{
  GET_SYSTEM_MODE = 0x8001,
  SET_SYSTEM_MODE = 0x8002,
  GET_SCAN_SETTINGS = 0x8001,
  SET_SCAN_SETTINGS = 0x8002,
  GET_SCAN_DATUM = 0x8003,
  SET_SCAN_DATUM = 0x8004,
  SET_SCAN_COMMAND = 0x8005,
  GET_SCAN_SETTINGS_STATUS = 0x8006,
  GET_SCAN_COMMAND_STATUS = 0x8007,
  GET_SOFTWARE_APP_INFO = 0x0008,
  GET_DATAPATH_DESTINATION_INFO = 0x000A,
  SET_DATAPATH_DESTINATION_INFO = 0x000B
};

/// @brief System mode command enumeration
enum class SystemModeCommand : std::uint8_t
{
  STANDBY,
  ACTIVE,
  SHUTDOWN,
};

/// @brief System mode status enumeration
enum class SystemModeStatus : std::uint8_t
{
  INIT,
  ACTIVE_UP_LASER_ON,
  ACTIVE_UP_AZ_ON,
  ACTIVE_UP_EL_ON,
  ACTIVE_UP_DATAPATH_ON,
  ACTIVE_UP_RECEIVER_ON,
  ACTIVE_UP_DIAGNOSTIC_CHECK,
  ACTIVE,
  ACTIVE_DOWN_LASER_OFF,
  ACTIVE_DOWN_AZ_OFF,
  ACTIVE_DOWN_EL_OFF,
  ACTIVE_DOWN_RECEIVER_OFF,
  ACTIVE_DOWN_DATAPATH_OFF,
  STANDBY,
  HW_SHUTDOWN
};

/// @brief Sensor health status
PACKED_DATA(struct SystemHeartbeatAppRecord {
  SystemModeStatus system_mode{SystemModeStatus::INIT};
  std::uint8_t system_ok{0U};
  std::uint8_t laser_ok{0U};
  std::uint8_t scanner_ok{0U};
  std::uint8_t receiver_ok{0U};
  std::uint8_t datapath_ok{0U};
  std::uint32_t v_battery{0U}; // 0.0F = binary 0 in any endianess
  std::uint32_t v_system{0U};
  std::uint32_t t_system{0U};
  std::uint32_t reserved_float{0U};
});

/// @brief Status of processing scan commands into scan pattern
enum class ScanPatternError : std::uint8_t
{
  SUCCESS,
  FAIL
};

/// @brief Scan pattern command type enumeration
enum class ScanPatternCmdType : std::uint8_t
{
  POINT = 1,        ///< Ensure given point lies on a scan line
  SNAP = 2,         ///< Use the fewest possible lines to get from the previous point to the next
  MIN_NUM_LINES = 3 ///< At least `min_lines` NEWLINEs are between the previous and next point
};

static constexpr std::uint16_t SCAN_PATTERN_CMD_PARAM_SIZE{12U};
static constexpr std::size_t SCAN_PATTERN_CMD_SIZE{
  SCAN_PATTERN_CMD_PARAM_SIZE + sizeof(ScanPatternCmdType) + sizeof(std::uint16_t)};
static constexpr std::size_t SCAN_PATTERN_MAX_NUM_CMDS{88U};

/// @brief the arguments used for the ScanPatternCmdType::POINT command type
PACKED_DATA(struct ScanPatternCmdPointArgs {
  ScanPatternCmdType type{ScanPatternCmdType::POINT};
  std::uint16_t length{lum::platform::networking::toNetworkByteOrder(SCAN_PATTERN_CMD_PARAM_SIZE)};
  std::uint32_t azimuth{0U}; // 0.0F = binary 0 in any endianess
  std::uint32_t elevation{0U};
  std::uint8_t checkpoint{0U};
  std::uint8_t reserved0{0U};
  std::uint8_t reserved1{0U};
  std::uint8_t reserved2{0U};
});

/// @brief the arguments used for the ScanPatternCmdType::MIN_NUM_LINES command type
PACKED_DATA(struct ScanPatternCmdMinLinesArgs {
  ScanPatternCmdType type{ScanPatternCmdType::MIN_NUM_LINES};
  std::uint16_t length{lum::platform::networking::toNetworkByteOrder(SCAN_PATTERN_CMD_PARAM_SIZE)};
  std::uint32_t num_lines{0U}; // 0.0F = binary 0 in any endianess
});

/// @brief the arguments used for the ScanPatternCmdType::SNAP command type
PACKED_DATA(struct ScanPatternCmdSnapArgs {
  ScanPatternCmdType type{ScanPatternCmdType::SNAP};
  std::uint16_t length{lum::platform::networking::toNetworkByteOrder(SCAN_PATTERN_CMD_PARAM_SIZE)};
});

PACKED_DATA(struct ScanPattern {
  std::uint16_t commands_size_in_bytes{0U}; // 0U is binary 0 in any endianess
  std::array<std::uint8_t, SCAN_PATTERN_CMD_SIZE * SCAN_PATTERN_MAX_NUM_CMDS> commands{};
});

/// @brief Scan frames datum
PACKED_DATA(struct ScanDatum {
  std::uint32_t datum_elevation{0U}; // 0.0F = binary 0 in any endianess
});

/// @brief Scan horizontal interlace mode enumeration
enum class ScanHorizontalInterlaceMode : std::uint8_t
{
  OFF = 0,
  ONE_HALF = 2,
  ONE_THIRD = 3,
  ONE_QUARTER = 4,
  ONE_EIGHTH = 8
};

/// @brief Scan control line index enumeration
enum class ScanControlLineIndex : std::uint8_t
{
  LINE_A,
  LINE_B
};

/// @brief Scan fill mode enumeration
enum class ScanFillMode : std::uint8_t
{
  UNIFORM,
  MIN_POWER
};

/// @brief Scan settings
PACKED_DATA(struct ScanSettings {
  std::uint32_t desired_frame_rate_hz{lum::platform::networking::floatToNetworkByteOrder(10.0F)};
  ScanHorizontalInterlaceMode horizontal_interlace_mode{ScanHorizontalInterlaceMode::OFF};
  ScanControlLineIndex control_line_index{ScanControlLineIndex::LINE_A};
  ScanFillMode fill_mode{ScanFillMode::UNIFORM};
});

/// @brief Scan settings status
PACKED_DATA(struct ScanSettingsStatus {
  std::uint32_t achieved_frame_rate_hz{0U}; // 0.0F = binary 0 in any endianess
  std::uint32_t lines_in_scan{0U};
});

static constexpr std::size_t VERSION_INFO_STRING_LENGTH{64U};

/// @brief Sensor software version info. ISO 8859-1 based encoding.
PACKED_DATA(struct SoftwareApplicationInfo {
  std::array<std::uint8_t, VERSION_INFO_STRING_LENGTH> hydra{};
  std::array<std::uint8_t, VERSION_INFO_STRING_LENGTH> system_v_70{};
  std::array<std::uint8_t, VERSION_INFO_STRING_LENGTH> laser_v_70{};
  std::array<std::uint8_t, VERSION_INFO_STRING_LENGTH> fpga{};
});

/// @brief Datapath destination info, including the ip address and port
PACKED_DATA(struct DatapathDestinationInfo {
  std::uint16_t destination_port{0U};
  std::uint32_t destination_ip{0U};
});

} // namespace serialization
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
