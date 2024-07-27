// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_LEGACY_TYPES_POINT_H
#define LUM_DRIVERS_LIDAR_LEGACY_TYPES_POINT_H

#include <cstdint>

#include <lum_common_types_point/point.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace legacy {
namespace types {

/// @brief Structure to represent 3D points of a line with additional infomation
/// from the Model H packet format
struct alignas(16) LinePoint : public common::types::point_cloud::Point
{
  /// Coordinate x in cartesian space (in meters)
  float x{0.0F};
  /// Coordinate y in cartesian space (in meters)
  float y{0.0F};
  /// Coordinate z in cartesian space (in meters)
  float z{0.0F};

  /// @brief Return intensity. Intensity is a measure of optical power you get
  /// back from a target. Intensity values are between 0 and 4095:
  //////0 if no photons were returned
  //////2047 if photons are returned from perfect flat matte white body known
  /// as Lambertian target (94% reflective)
  //////2048 - 4095 shiny or specular object e.g. mirror or retro-reflector
  std::uint16_t intensity{0U};

  /// @brief Azimuth coordinate in spherical space (in radians)
  float azimuth{0.0F};
  /// @brief Elevation coordinate in spherical space; (in radians)
  float elevation{0.0F};
  /// @brief Depth coordinate in spherical space (in meters)
  float depth{0.0F};

  /// @brief return index.
  /// Each laser pulse that is sent out can encounter objects in its path and
  /// record up to 3 returns or objects that it encounters. Default value 0 means there current
  /// pulse got no returns.  Returns will be indexed 1, 2, 3 ...
  std::uint8_t return_index{0U};

  /// @brief nth index of last return, i.e. 0-3.
  /// Each laser pulse that is sent out can encounter objects in its path and
  /// record up to 3 returns or objects that it encounters
  std::uint8_t last_return_index{0U};

  /// Lidar eye (left or right)
  std::uint8_t eye{0U};
  /// Timestamp (in seconds.microseconds)
  double timestamp{0.0};
  /// reserved field
  std::uint8_t reserved{0U};
};

/// @brief PCL line meta struct, used for passing line-specific metadata out of the
/// PointCloudLineGenerator
struct PointCloudLineMeta
{
  /// @brief scan_profile from packet preamble
  std::uint8_t scan_profile{0U};
  /// @brief checkpoint from packet preamble
  std::uint8_t checkpoint{0U};
  /// @brief scan_count from containing scan
  std::uint8_t scan_count{0U};
  /// @brief interlace index from subscan
  std::uint8_t interlace_index{0U};
  /// @brief track number from playlist
  std::uint8_t track_number{0U};
  /// @brief The id of the sensor producing this line
  std::uint8_t sensor_id{0U};
  /// @brief line index from the line generator (this is running value of line index that increments
  /// and rolls over; it is not the same thing as line within the frame)
  std::uint8_t line_index{0U};
};

/// @brief Structure to represent 3D points with additional infomation
/// from the Model H packet format
struct alignas(16) Point : public common::types::point_cloud::Point
{
  /// Coordinate x in cartesian space (in meters)
  float x{0.0F};
  /// Coordinate y in cartesian space (in meters)
  float y{0.0F};
  /// Coordinate z in cartesian space (in meters)
  float z{0.0F};

  /// @brief Return intensity. Intensity is a measure of optical power you get
  /// back from a target. Intensity values are between 0 and 4095:
  //////0 if no photons were returned
  //////2047 if photons are returned from perfect flat matte white body known
  /// as Lambertian target (94% reflective)
  //////2048 - 4095 shiny or specular object e.g. mirror or retro-reflector
  std::uint16_t intensity{0U};

  /// @brief Azimuth coordinate in spherical space (in radians)
  float azimuth{0.0F};
  /// @brief Elevation coordinate in spherical space; (in radians)
  float elevation{0.0F};
  /// @brief Depth coordinate in spherical space (in meters)
  float depth{0.0F};

  /// @brief return index.
  /// Each laser pulse that is sent out can encounter objects in its path and
  /// record up to 3 returns or objects that it encounters. Default value 0 means there current
  /// pulse got no returns.  Returns will be indexed 1, 2, 3 ...
  std::uint8_t return_index{0U};

  /// @brief nth index of last return, i.e. 0-3.
  /// Each laser pulse that is sent out can encounter objects in its path and
  /// record up to 3 returns or objects that it encounters
  std::uint8_t last_return_index{0U};

  /// Index of the scan line
  std::int16_t scan_line{0};
  /// Lidar eye (left or right)
  std::uint8_t eye{0U};
  /// Interlace index
  std::uint8_t interlace_index{0U};
  /// Track index within scan playlist
  std::uint8_t track_number{0U};
  /// Checkpoint index within scan pattern
  std::uint8_t checkpoint{0U};
  /// Timestamp (in seconds.microseconds)
  double timestamp{0.0};
  /// Reserved field
  std::uint8_t reserved{0U};

  /// @brief Sensor ID this point was originated from (in multi-sensor setup)
  std::uint8_t sensor_id{0U};
};

/// @brief helper function to detect if this is an empty point or a real return from the lidar
/// @return true if this point is empty
inline bool isEmpty(const Point& p)
{
  return p.return_index == 0U;
}

/// @brief helper function to detect if this is an empty point or a real return from the lidar
/// @return true if this point is empty
inline bool isEmpty(const LinePoint& p)
{
  return p.return_index == 0U;
}

} // namespace types
} // namespace legacy
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
