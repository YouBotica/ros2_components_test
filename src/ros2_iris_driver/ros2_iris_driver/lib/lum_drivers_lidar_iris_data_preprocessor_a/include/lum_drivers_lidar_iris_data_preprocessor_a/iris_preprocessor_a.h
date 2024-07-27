// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_DATA_PREPROCESSOR_A_IRIS_PREPROCESSOR_A_H
#define LUM_DRIVERS_LIDAR_IRIS_DATA_PREPROCESSOR_A_IRIS_PREPROCESSOR_A_H

#include <memory>

#include <lum_drivers_lidar_iris_internal_types/i_iris_preprocessor.h>
#include <lum_drivers_lidar_iris_types/preprocessor_config.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief make an Iris Data Preprocessor A
/// @param parameters data pipeline configuration
std::unique_ptr<IIrisPreprocessor>
makeIrisPreprocessorA(const types::PreprocessorConfig& parameters = {});

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
