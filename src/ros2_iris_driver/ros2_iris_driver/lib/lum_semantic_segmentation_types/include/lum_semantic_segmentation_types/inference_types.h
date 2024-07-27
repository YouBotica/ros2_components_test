// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_SEMANTIC_SEGMENTATION_TYPES_INFERENCE_TYPES_H
#define LUM_SEMANTIC_SEGMENTATION_TYPES_INFERENCE_TYPES_H

#include <cstdint>

namespace lum {
namespace semantic_segmentation_types {
namespace inference_types {

/// @namespace lum::semantic_segmentation_types types ML Inference data types

/// @brief Type of model, can this model operate of just one lidar or multiple lidar
enum class ModelType : std::uint8_t
{
  FORWARD_VIEW_MODEL = 0,  ///< Model works on forward facing sensors, e.g. where point.x > 0
  SURROUND_VIEW_MODEL = 1, ///< Model works on surround facing sensors
  IRIS_FORWARD_VIEW_MODEL = 4,
  FORWARD_VIEW_MODEL_MV = 5
};

} // namespace inference_types
} // namespace semantic_segmentation_types
} // namespace lum

#endif // LUM_SEMANTIC_SEGMENTATION_TYPES_INFERENCE_TYPES_H
