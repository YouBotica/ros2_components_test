// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_DATA_PREPROCESSOR_IRIS_PREPROCESSOR_H
#define LUM_DRIVERS_LIDAR_IRIS_DATA_PREPROCESSOR_IRIS_PREPROCESSOR_H

#include <memory>

#include <lum_drivers_lidar_iris_internal_types/i_iris_preprocessor.h>
#include <lum_drivers_lidar_iris_internal_types/i_scan_line_processor.h>
#include <lum_drivers_lidar_iris_types/preprocessor_config.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief make an Iris Data Preprocessor
/// @param line_processor scan line processor
/// @param parameters data pipeline configuration
std::unique_ptr<IIrisPreprocessor>
makeIrisPreprocessor(std::unique_ptr<IScanLineProcessor> line_processor,
                     const types::PreprocessorConfig& parameters = {}) noexcept;

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
