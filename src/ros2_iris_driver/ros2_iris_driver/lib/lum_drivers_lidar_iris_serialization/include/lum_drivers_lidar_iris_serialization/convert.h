// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_SERIALIZATION_CONVERT_H
#define LUM_DRIVERS_LIDAR_IRIS_SERIALIZATION_CONVERT_H

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace serialization {

using Result = bool;

/// @brief Conversion function to/from serializable data types
/// @tparam Source Data type to convert from
/// @tparam Dest Data type to convert to
/// @param[in] in Data value to convert from
/// @param{out] out Data value to convert to
/// @return True on conversion success
template <class Source, class Dest>
Result convert(const Source& in, Dest& out);

} // namespace serialization
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum
#endif // LUM_DRIVERS_LIDAR_IRIS_SERIALIZATION_CONVERT_H
