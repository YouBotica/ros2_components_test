// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_POINTCLOUD_CONVERSION_TO_POLAR_H
#define LUM_COMMON_POINTCLOUD_CONVERSION_TO_POLAR_H

#include <lum_common_types_pointcloud/point_cloud_layer.h>

namespace lum {
namespace common {
namespace pointcloud_conversion {

lum::common::types::point_cloud::StructuredPolarLayer
toPolar(const lum::common::types::point_cloud::StructuredCommonLayer& common_layer);

} // namespace pointcloud_conversion
} // namespace common
} // namespace lum

#endif // LUM_COMMON_POINTCLOUD_CONVERSION_TO_POLAR_H
