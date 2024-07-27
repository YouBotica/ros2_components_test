// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_CLASSIFICATION_UTILS_H
#define LUM_COMMON_TYPES_CLASSIFICATION_UTILS_H

#ifndef CUDA_HOST_DEVICE
#if defined __CUDACC__
#define CUDA_HOST_DEVICE __host__ __device__
#else
#define CUDA_HOST_DEVICE
#endif
#endif

#include <lum_common_types_classification/ontology.h>

namespace lum {
namespace common {
namespace types {
namespace classification {
namespace ontology_utils {

/// @brief Determines if an element from the ontology falls in the unclassified's section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsUnclassified(const OntologyType elem)
{
  return elem < WEATHER_ONTOLOGY_OFFSET;
}

/// @brief Determines if an element from the ontology falls in the weather section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsWeather(const OntologyType elem)
{
  return (elem >= WEATHER_ONTOLOGY_OFFSET) && (elem < DRIVABLE_SURFACE_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the drivable surface section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsDrivableSurface(const OntologyType elem)
{
  return (elem >= DRIVABLE_SURFACE_ONTOLOGY_OFFSET) && (elem < VEGETATION_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the vegetation section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsVegetation(const OntologyType elem)
{
  return (elem >= VEGETATION_ONTOLOGY_OFFSET) && (elem < TRAFFIC_BARRIER_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the traffic barrier section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsTrafficBarrier(const OntologyType elem)
{
  return (elem >= TRAFFIC_BARRIER_ONTOLOGY_OFFSET) && (elem < LANE_MARKING_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the lane markings section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsLaneMarking(const OntologyType elem)
{
  return (elem >= LANE_MARKING_ONTOLOGY_OFFSET) && (elem < VEHICLE_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the vehicle section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsVehicle(const OntologyType elem)
{
  return (elem >= VEHICLE_ONTOLOGY_OFFSET) && (elem < HUMAN_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the pedestrian section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsPedestrian(const OntologyType elem)
{
  return (elem >= HUMAN_ONTOLOGY_OFFSET) && (elem < ANIMAL_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the animal section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsAnimal(const OntologyType elem)
{
  return (elem >= ANIMAL_ONTOLOGY_OFFSET) && (elem < TRAFFIC_SIGN_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the traffic sign section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsTrafficSign(const OntologyType elem)
{
  return (elem >= TRAFFIC_SIGN_ONTOLOGY_OFFSET) && (elem < TRAFFIC_LIGHT_ONTOLOGY_OFFSET);
}

/// @brief Determines if an element from the ontology falls in the traffic light section
/// @param [in] elem index of ontology element
/// @return boolean, True if the element falls in this section, else False
CUDA_HOST_DEVICE inline bool classIsTrafficLight(const OntologyType elem)
{
  return elem >= TRAFFIC_LIGHT_ONTOLOGY_OFFSET;
}

} // namespace ontology_utils
} // namespace classification
} // namespace types
} // namespace common
} // namespace lum

#endif // LUM_COMMON_TYPES_CLASSIFICATION_UTILS_H
