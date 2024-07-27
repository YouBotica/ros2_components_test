// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_SEMANTIC_SEGMENTATION_TYPES_I_MODEL_H
#define LUM_SEMANTIC_SEGMENTATION_TYPES_I_MODEL_H

#include <lum_common_types_pointcloud/point_cloud_layer.h>
#include <lum_semantic_segmentation_types/types.h>

namespace lum {
namespace semantic_segmentation_types {

/// @brief Structure to pass inference configuration options
struct InferenceOptions
{
  /// Max height for which points will be classified. Points above this threshold will default to
  /// Ontology::UNCLASSIFIED_OBJECT_IN_VIEW
  float max_classification_height{std::numeric_limits<float>::max()};
  /// Flag to specify whether to get oriented bounding boxes for objects. If true, boxes will be
  /// oriented along the object. If false, boxes will be axis-aligned, but still enclose the entire
  /// object.
  bool get_oriented_bboxes{true};
  /// Flag to specify whether to perform clustering for bounding boxes in armadillo projection(true)
  /// or orthogonal top down projection(false).
  bool should_perform_armadillo{false};
};

/// @brief The class defines an abstract interface for ML models.
class IModel
{
public:
  /// @brief Classifies every point in @a input_cloud. Returns the top 3 classifications with
  /// confidence values.
  /// @param [in] input_cloud PointCloud of XYZI points
  /// @param [in] options Options to control the classification in the semantic seg model
  /// @return model output: classified cloud, objects, lane and road cloud
  /// @throw InferenceException
  /// @note The first call to this function loads a graph, consequently the first call is slower
  /// than subsequent calls.
  virtual std::shared_ptr<semantic_segmentation::types::ModelOutput>
  forward(const lum::common::types::point_cloud::StructuredCommonLayer& common_layer_data,
          const lum::common::types::point_cloud::StructuredPolarLayer& polar_layer_data,
          const semantic_segmentation_types::InferenceOptions& options) const = 0;

  // Polymorphic base class boilerplate
  virtual ~IModel() = default;
  IModel() = default;
  IModel(const IModel&) = delete;
  IModel(IModel&&) = delete;
  void operator=(const IModel&) & = delete;
  void operator=(IModel&&) & = delete;
};
} // namespace semantic_segmentation_types
} // namespace lum

#endif
