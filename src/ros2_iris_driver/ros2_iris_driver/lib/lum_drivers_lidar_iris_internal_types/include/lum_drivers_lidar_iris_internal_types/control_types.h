// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_CONTROL_TYPES_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_CONTROL_TYPES_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <lum_common_math/floating_point.h>
#include <lum_common_types/units.h>
#include <lum_common_types/utils.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace types {

using lum::common::types::utils::toUnderlyingValue;
using AngleInDegrees = lum::common::types::units::AngleInDegrees<float>;

// Iris control data types (C++-side)

/// @brief Services offered by the Iris sensor
enum class SensorService : std::uint16_t
{
  SYSTEM_MODE = 0x0002,
  SCAN_PATTERN = 0x0003,
  SYSTEM_INFO = 0x0004
};

/// @brief Sensor service ids
constexpr auto SYSTEM_MODE_SERVICE_ID{toUnderlyingValue(SensorService::SYSTEM_MODE)};
constexpr auto SCAN_PATTERN_SERVICE_ID{toUnderlyingValue(SensorService::SCAN_PATTERN)};
constexpr auto SYSTEM_INFO_SERVICE_ID{toUnderlyingValue(SensorService::SYSTEM_INFO)};

/// @brief System mode command enumeration
enum class SystemModeCommand : std::uint32_t
{
  /// Go to standby mode
  STANDBY,
  /// Go to active mode
  ACTIVE,
  /// Go to shutdown mode
  SHUTDOWN
};

/// @brief System mode status enumeration
enum class SystemModeStatus : std::uint32_t
{
  /// Nothing is known yet
  UNKNOWN,
  /// Is in standby or transitioning between modes
  STANDBY,
  /// Is active
  ACTIVE,
  /// Is shut down
  SHUTDOWN
};

/// @brief Sensor health status
struct SensorHealthStatus
{
  float battery_voltage{0.0F};
  float system_voltage{0.0F};
  lum::common::types::units::TemperatureInCelsius<float> system_temperature{0.0F};
  SystemModeStatus system_mode{SystemModeStatus::UNKNOWN};
  bool system_ok{false};
  bool laser_ok{false};
  bool scanner_ok{false};
  bool receiver_ok{false};
  bool datapath_ok{false};
};

/// @brief Status of processing scan commands into scan pattern
enum class ScanPatternCommandStatus : std::uint32_t
{
  /// Nothing is known yet
  UNKNOWN,
  /// Scan pattern processed successfully
  OK,
  /// Error occurred during creation of scan pattern
  CREATE_ERROR
};

/// @brief Scan pattern definition by creation commands
class ScanPattern
{
public:
  enum class CmdId : std::int32_t
  {
    POINT,
    SNAP,
    MIN_LINES
  };

  /// @brief Interface of commands
  class ICommand
  {
  public:
    /// A handle to the command which maintains the ownership
    using Handle = std::unique_ptr<ICommand>;

    ICommand() = default;
    virtual ~ICommand() = default;
    ICommand(const ICommand&) = delete;
    ICommand(ICommand&&) = delete;
    ICommand& operator=(const ICommand&) & = delete;
    ICommand& operator=(ICommand&&) & = delete;

    /// @brief makes a clone of an existing command
    /// @return a deep copy of the command
    virtual Handle clone() const = 0;

    /// @brief Returns the identifier of this specific command
    virtual CmdId getId() const = 0;
  };

  /// @brief Ensure given point lies on a scan line
  /// @details Ensure that the given azimuth and elevation lie on a scan line in the scan pattern.
  /// Several provided points may lie on the same line, unless a minimum number of lines greater
  /// than zero is specified.
  /// If a non-zero checkpoint is provided, then all rays in the output data will be marked with
  /// this checkpoint until another point with non-zero checkpoint is reached.
  class PointCommand : public ICommand
  {
  public:
    /// @brief Constructor
    /// @param[in] azimuth Azimuth
    /// @param[in] elevation Elevation
    /// @param[in] checkpoint Checkpoint
    PointCommand(const AngleInDegrees azimuth,
                 const AngleInDegrees elevation,
                 const std::uint8_t checkpoint)
        : ICommand{}, azimuth_{azimuth}, elevation_{elevation}, checkpoint_{checkpoint}
    {
    }

    /// @copydoc ICommand::clone
    Handle clone() const override
    {
      return std::make_unique<PointCommand>(azimuth_, elevation_, checkpoint_);
    }

    /// @copydoc ICommand::getId
    CmdId getId() const override { return CmdId::POINT; }

    /// @brief Get stored azimuth value
    /// @return Azimuth value
    AngleInDegrees getAzimuth() const noexcept { return azimuth_; }
    /// @brief Get stored elevation value
    /// @return Elevation value
    AngleInDegrees getElevation() const noexcept { return elevation_; }
    /// @brief Get stored checkpoint value
    /// @return Checkpoint value
    std::uint8_t getCheckpoint() const noexcept { return checkpoint_; }

  private:
    AngleInDegrees azimuth_;
    AngleInDegrees elevation_;
    std::uint8_t checkpoint_;
  };

  /// @brief Use the fewest possible lines to get from the previous point to the next
  /// @details The trajectory calculated to get from the previous line to the next line will use as
  /// few lines as possible.
  class SnapCommand : public ICommand
  {
  public:
    /// @copydoc ICommand
    SnapCommand() = default;

    /// @copydoc ICommand::clone
    Handle clone() const override { return std::make_unique<SnapCommand>(); }

    /// @copydoc ICommand::getId
    CmdId getId() const override { return CmdId::SNAP; }
  };

  /// @brief At least @a num_lines NEWLINEs are between the previous and next point
  /// @details This includes the line the previous point was on, but not the line the following
  /// point is on. If more than one @c MIN_NUM_LINES is specified between two points, the first is
  /// used. The value is stored in floating point to allow for fractional lines.
  class MinLinesCommand : public ICommand
  {
  public:
    /// @brief Constructor
    /// @param[in] num_lines Number of newlines required
    explicit MinLinesCommand(const float num_lines) : ICommand{}, num_lines_{num_lines} {}

    /// @copydoc ICommand::clone
    Handle clone() const override { return std::make_unique<MinLinesCommand>(num_lines_); }

    /// @copydoc ICommand::getId
    CmdId getId() const override { return CmdId::MIN_LINES; }

    /// @brief Get stored number of lines
    /// @return Number of lines
    float getNumLines() const noexcept { return num_lines_; }

  private:
    float num_lines_;
  };

  /// The type alias for a collection of owned commands
  using CommandList = std::vector<ICommand::Handle>;

  ScanPattern() = default;
  ~ScanPattern() = default;
  ScanPattern(ScanPattern&&) = default;
  ScanPattern& operator=(ScanPattern&&) & = default;

  /// @brief Custom copy constructor
  /// @param other object copying from
  ScanPattern(const ScanPattern& other) : scan_pattern_commands_{copy(other.scan_pattern_commands_)}
  {
  }

  /// @brief Custom copy assignment operator
  /// @param other object copying from
  /// @return copied object
  ScanPattern& operator=(const ScanPattern& other) &
  {
    if (&other != this)
    {
      scan_pattern_commands_ = copy(other.scan_pattern_commands_);
    }

    return *this;
  }

  /// @brief Add a snap command to the list
  void addSnapCommand() { scan_pattern_commands_.emplace_back(std::make_unique<SnapCommand>()); }

  /// @brief Add a point command to the list
  /// @param[in] azimuth Azimuth
  /// @param[in] elevation Elevation
  /// @param[in] checkpoint Checkpoint
  void addPointCommand(const AngleInDegrees azimuth,
                       const AngleInDegrees elevation,
                       const std::uint8_t checkpoint)
  {
    scan_pattern_commands_.emplace_back(
      std::make_unique<PointCommand>(azimuth, elevation, checkpoint));
  }

  /// @brief Add a min lines command to the list
  /// @param[in] num_lines Number of newlines required
  void addMinLinesCommand(float num_lines)
  {
    scan_pattern_commands_.emplace_back(std::make_unique<MinLinesCommand>(num_lines));
  }

  /// @brief Add a command to the list
  /// @param[in] command Pointer to the command
  /// @note This class will take ownership of @a command
  void addCommand(std::unique_ptr<ICommand>&& command)
  {
    scan_pattern_commands_.emplace_back(std::move(command));
  }

  /// @brief Get the current list of commands
  /// @return Command list
  const CommandList& getCommandList() const noexcept { return scan_pattern_commands_; }

  /// @brief Clear the list of commands
  void clear() { scan_pattern_commands_.clear(); }

private:
  /// @brief Copies a scan pattern command list
  /// @param in the command list to copy
  /// @return the copied command list
  static CommandList copy(const CommandList& in)
  {
    CommandList out;
    out.reserve(in.size());

    for (const auto& cmd : in)
    {
      out.emplace_back(cmd->clone());
    }

    return out;
  }

  /// list of scan pattern commands
  CommandList scan_pattern_commands_{};
};

/// @brief Scan frames datum
struct ScanDatum
{
  /// Scan frames begin and end at the datum elevation
  lum::common::types::units::AngleInDegrees<float> elevation{0.0F};
};

/// @brief Scan horizontal interlace mode enumeration
enum class ScanHorizontalInterlaceMode : std::uint32_t
{
  /// Stable points
  OFF,
  /// Points offset by half az spacing each frame, repeats every 2 frames
  ONE_HALF,
  /// Points offset by 1/3rd az spacing each frame, repeats every 3 frames
  ONE_THIRD,
  /// Points offset by 1/4th az spacing each frame, repeats every 4 frames
  ONE_QUARTER,
  /// Points offset by 1/8th az spacing each frame, repeats every 8 frames
  ONE_EIGHTH
};

/// @brief Scan control line index enumeration
enum class ScanControlLine : std::uint32_t
{
  LINE_A,
  LINE_B
};

/// @brief Scan fill mode enumeration
enum class ScanFillMode : std::uint32_t
{
  UNIFORM,
  MIN_POWER
};

/// @brief Scan settings that the sensor should try to achieve
struct DesiredScanSettings
{
  /// Frame rate of the sensor in Hz, i.e., how long it takes to complete one scan (default 10 Hz)
  lum::common::types::units::FrequencyInHertz<float> frame_rate{10.0F};
  /// Horizontal interlace mode, i.e., the fraction of the azimuth to offset between scan frames,
  /// where the inverse of of this value determines when the offset repeats (default: OFF, i.e., no
  /// offset between frames)
  ScanHorizontalInterlaceMode horizontal_interlace_mode{ScanHorizontalInterlaceMode::OFF};
  /// The control line for the sensor (default: LINE_A)
  ScanControlLine control_line{ScanControlLine::LINE_A};
  /// The scan fill mode for sensor (default: UNIFORM)
  ScanFillMode fill_mode{ScanFillMode::UNIFORM};
};

/// @brief Equality comparison for DesiredScanSettings
/// @param lhs value1 for equality comparison
/// @param rhs value2 for equality comparison
/// @return true if two settings objects are equal (within tolerance)
inline bool operator==(const DesiredScanSettings& lhs, const DesiredScanSettings& rhs) noexcept
{
  const bool is_frame_rate_equal =
    common::math::areApproximatelyEqual(lhs.frame_rate.inHertz(), rhs.frame_rate.inHertz());

  return (lhs.control_line == rhs.control_line) && (lhs.fill_mode == rhs.fill_mode) &&
         is_frame_rate_equal && (lhs.horizontal_interlace_mode == rhs.horizontal_interlace_mode);
}

/// @brief Scan settings that the sensor actually achieves
struct AchievedScanSettings
{
  /// Achieved scan frame rate of the sensor
  lum::common::types::units::FrequencyInHertz<float> frame_rate{0.0F};
  /// The number of lines generated by the applied scan pattern
  /// @note This is a float to allow for fractional lines
  float lines_in_scan{0.0F};
};

/// @brief Sensor software version info. UTF-8 based encoding. These strings include a
/// verbose description of the processor software version with both git hash and version ID.
struct SensorSoftwareVersion
{
  std::string hydra{};
  std::string system_v_70{};
  std::string laser_v_70{};
  std::string fpga{};
};

/// @brief Datapath destination info for a sensor, including the ip address and port
struct DatapathDestinationInfo
{
  std::string ip_address{"0.0.0.0"};
  std::uint16_t port{0U};
};

/// @brief Last set configuration of the scanner
struct LastSetScanConfig
{
  ScanPattern scan_pattern{};
  ScanDatum scan_datum{};
  DesiredScanSettings scan_settings{};
};

/// @brief Desired configuration of the scanner
struct DesiredScanConfig
{
  ScanPattern scan_pattern{};
  ScanDatum scan_datum{};
};

} // namespace types
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum
#endif
