// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_DATA_PREPROCESSOR_RESIM_IRIS_PREPROCESSOR_RESIM_H
#define LUM_DRIVERS_LIDAR_IRIS_DATA_PREPROCESSOR_RESIM_IRIS_PREPROCESSOR_RESIM_H

#include <memory>

#include <lum_drivers_lidar_iris_internal_types/i_iris_preprocessor.h>
#include <lum_drivers_lidar_iris_types/preprocessor_config.h>
#include <lum_drivers_lidar_iris_types_resim/config.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief make a synchronous ReSim Iris Data Preprocessor
/// @param preprocessor_config preprocessor configuration
/// @param resim_config ReSim configuration
std::unique_ptr<IIrisPreprocessor>
makeIrisPreprocessorSyncResim(const types::PreprocessorConfig& preprocessor_config = {},
                              const resim::ResimConfig& resim_config = {});

/// @brief make an asynchronous ReSim Iris Data Preprocessor
/// @param preprocessor_config preprocessor configuration
/// @param resim_config ReSim configuration
/// @param async_config Asynchronous ReSim configuration
std::unique_ptr<IIrisPreprocessor>
makeIrisPreprocessorAsyncResim(const types::PreprocessorConfig& preprocessor_config = {},
                               const resim::ResimConfig& resim_config = {},
                               const resim::AsyncResimConfig& async_config = {});

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
