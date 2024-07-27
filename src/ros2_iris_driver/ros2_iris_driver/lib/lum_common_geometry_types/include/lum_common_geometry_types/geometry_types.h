// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_GEOMETRY_TYPES_GEOMETRY_TYPES_H
#define LUM_COMMON_GEOMETRY_TYPES_GEOMETRY_TYPES_H

#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <type_traits>

#include <lum_common_types_classification/ontology.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>

/// @def Template parameter restricted to numeric types
#define LUM_NUMERIC_TEMPLATE_TYPE                                                                  \
  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>

/// @def Template parameter restricted to floating point types
#define LUM_FLOATING_TEMPLATE_TYPE                                                                 \
  template <typename T,                                                                            \
            typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>

namespace lum {
namespace common {
namespace types {
namespace geometry {

namespace detail {

/// @brief Calculate the dimensions of covariance matrix, given degrees of freedom
/// @tparam DegreesOfFreedom Degrees of freedom
template <std::uint8_t DegreesOfFreedom>
struct CalcNumCovarianceDimensions
{
  static const std::size_t value =
    static_cast<std::size_t>(DegreesOfFreedom) * static_cast<std::size_t>(DegreesOfFreedom);
};

} // namespace detail

/// @brief Generic data type for members with 2 components
LUM_NUMERIC_TEMPLATE_TYPE
struct Vector2
{
  T x{};
  T y{};

  Vector2() = default;
  Vector2(const T& x, const T& y) : x(x), y(y) {}
};

/// @brief Generic data type for members with 3 components
LUM_NUMERIC_TEMPLATE_TYPE
struct Vector3
{
  T x{};
  T y{};
  T z{};
};

/// @brief Generic data type for members with 4 components
LUM_NUMERIC_TEMPLATE_TYPE
struct Vector4
{
  T x{};
  T y{};
  T z{};
  T w{};
};

/// @brief Structure for representing 3-dimensional position of an entity in
/// free space
LUM_FLOATING_TEMPLATE_TYPE
using Position = Vector3<T>;

/// @brief Structure for representing 3-dimensional position of an entity in
/// free space with uncertainty
LUM_FLOATING_TEMPLATE_TYPE
struct PositionWithCovariance
{
  Position<T> position{}; ///< X, Y, Z position
  std::array<T, detail::CalcNumCovarianceDimensions<3>::value>
    covariance{}; ///< Row-major representation of 3x3 covariance matrix
};

/// @brief Structure for representing Euler Angles

LUM_FLOATING_TEMPLATE_TYPE
struct EulerAngle
{
  T roll{};  ///< rotation about X-axis
  T pitch{}; ///< rotation about Y-axis
  T yaw{};   ///< rotation about Z-axis
};

/// @brief Structure for representing quaternion
LUM_FLOATING_TEMPLATE_TYPE
struct Quaternion
{
  T x{0}; ///< x component of quaternion
  T y{0}; ///< y component of quaternion
  T z{0}; ///< z component of quaternion
  T w{1}; ///< w constant component of quaternion

  /// @brief Get the norm of the quaternion
  /// @return quaternion norm
  T norm() const { return std::sqrt((x * x) + (y * y) + (z * z) + (w * w)); }

  /// @brief Normalize the quaternion to be of unit length
  void normalize()
  {
    const auto this_norm = norm(); // get the norm of the quaternion

    x /= this_norm;
    y /= this_norm;
    z /= this_norm;
    w /= this_norm;
  }

  /// @brief Utility function to get a string representing the current quaternion values
  /// @return string representing the current quaternion values
  std::string toString()
  {
    std::stringstream quaternion_string;
    quaternion_string << "(x,y,z,w) : (" << x << "," << y << "," << z << "," << w << ")";
    return quaternion_string.str();
  }

  /// @brief Utility function to set the quaternion to represent 0 rad angle in roll, pitch, yaw
  void setZero()
  {
    x = 0;
    y = 0;
    z = 0;
    w = 1;
  }
};

/// @brief Structure for representing 3-dimensional orientation of an entity as
/// a quaternion in free space
LUM_FLOATING_TEMPLATE_TYPE
using Orientation = Quaternion<T>;

/// @brief Structure for representing 3-dimensional orientation of an entity as
/// a quaternion in free space with uncertainty Fixed axis representation for
/// orientation in covariance in order of rotation around x, y and z axis
LUM_FLOATING_TEMPLATE_TYPE
struct OrientationWithCovariance
{
  Orientation<T> orientation{}; ///< 3DOF orientation
  std::array<T, detail::CalcNumCovarianceDimensions<3>::value>
    covariance{}; ///< Row-major representation of 3x3 covariance matrix
};

/// @brief Structure for representing 6DOF pose of an entity in free space
LUM_FLOATING_TEMPLATE_TYPE
struct Pose
{
  Position<T> position{};       ///< 3DOF position
  Orientation<T> orientation{}; ///< 3DOF orientation represented as quaternion
};

/// @brief Returns the status of the query from the TF Tree
enum class QueryStatus : std::uint8_t
{
  PARENT_NOT_FOUND = 0, ///< Parent node was not found during query
  CHILD_NOT_FOUND = 1,  ///< Child node was not found during query
  SUCCESS = 2,          ///< Queried transform has been retrieved successfully
  TF_ERROR = 3          ///< Error in querying the TF Tree. A communication error in server-client
                        ///< set up results in this status
};

/// @brief Returns the cache status of the TF Tree query
enum class TransformCacheRetrieveStatus : std::uint8_t
{
  SUCCESS = 0,       ///< Transform successfully retrieved from the buffer given the time query
  INVALID_QUERY = 1, ///< Time query is invalid
  LATER_THAN_LATEST =
    2, ///< Queried time is later than the latest entry in the buffer, return latest transform
  EARLIER_THAN_EARLIEST =
    3, ///< Queried time is earlier than the earliest entry in the buffer, return earliest transform
  BUFFER_EMPTY = 4, ///< Empty buffer is queried
  CACHE_ERROR = 5   ///< Undefined error
};

/// @brief Returns the cache status, TF Tree query status and the retrieved pose
LUM_FLOATING_TEMPLATE_TYPE
struct TFStatusAndPose
{
  QueryStatus query_status{
    QueryStatus::TF_ERROR}; ///< Returns the status of the query from the TF Tree
  TransformCacheRetrieveStatus cache_retrieve_status{
    TransformCacheRetrieveStatus::CACHE_ERROR}; ///< Returns the cache status of the TF Tree query
  Pose<T> pose; ///< Pose between the parent and the child frame queried from the TF tree
};

/// @brief Structure for representing 6DOF pose of an entity in free space with
/// uncertainty. Fixed axis representation for orientation in covariance in
/// order of rotation around x, y and z axis
LUM_FLOATING_TEMPLATE_TYPE
struct PoseWithCovariance
{
  Pose<T> pose; ///< Position and orientation
  std::array<T, detail::CalcNumCovarianceDimensions<6>::value>
    covariance; ///< Row-major representation of 6x6 covariance matrix
};

/// @brief Structure for representing 3-dimensional scale of an entity in free
/// space
LUM_FLOATING_TEMPLATE_TYPE
using Scale = Vector3<T>;

/// @brief Structure for representing 3-dimensional scale of an entity in free
/// space with uncertainty
LUM_FLOATING_TEMPLATE_TYPE
struct ScaleWithCovariance
{
  Scale<T> scale{}; ///< 3D scale
  std::array<T, detail::CalcNumCovarianceDimensions<3>::value>
    covariance{}; ///< Row-major representation of 3x3 covariance matrix
};

/// @brief Enumeration of cardinal and intercardinal directions
enum class HeadingDirection : std::uint8_t
{
  NONE = 0,       ///< Sentinel: no direction supplied
  NORTH = 1,      ///< north direction
  NORTH_EAST = 2, ///< north east direction
  EAST = 3,       ///< east direction
  SOUTH_EAST = 4, ///< south east direction
  SOUTH = 5,      ///< south direction
  SOUTH_WEST = 6, ///< south west direction
  WEST = 7,       ///< west direction
  NORTH_WEST = 8  ///< north west direction
};

/// @brief Structure for representing shape of an entity in free space
template <typename T>
struct Shape
{
  common::types::point_cloud::PointCloudPtr<T>
    convex_hull; ///< Vector of points representing convex hull
  common::types::point_cloud::PointCloudPtr<T>
    segmented_points; ///< Vector of points in the segmented entity
};

LUM_FLOATING_TEMPLATE_TYPE
struct BoundingBox3D
{
  common::types::geometry::Pose<T> pose{}; ///< Pose of the frame attached to the center of the box
  common::types::geometry::Scale<T> scale{}; ///< Dimension of the box along x, y, z
};

/// @brief Bounding Box Structure for representing 3D bounding boxes with
/// classification
/// @tparam NumBoxClassifications number of classifications
template <std::size_t NumBoxClassifications>
struct ClassifiedBoundingBox3D : public common::types::geometry::BoundingBox3D<float>
{
  static const std::size_t num_classifications = NumBoxClassifications;
  std::array<lum::common::types::classification::ClassificationWithConfidence,
             NumBoxClassifications>
    classifications{};
  std::array<Vector2<float>, 4> box_corners;
  std::uint32_t instance_id{0};         ///< object instance id
  std::uint32_t sensor_id{0};           ///< object sensor id
  lum::common::types::Time timestamp{}; ///< Timestamp
};

/// @brief Structure for representing 3D convex hulls with
/// classification
/// @tparam NumBoxClassifications number of classifications
template <std::size_t NumBoxClassifications>
struct ConvexHull3D
{
  static const std::size_t num_classifications = NumBoxClassifications;
  std::array<lum::common::types::classification::ClassificationWithConfidence,
             NumBoxClassifications>
    classifications{};
  std::vector<Vector2<float>> hull_points{};
  float min_height{std::numeric_limits<float>::max()};
  float max_height{std::numeric_limits<float>::lowest()};
  std::uint32_t instance_id{0U}; ///< object instance id
  std::uint32_t sensor_id{0U};   ///< object sensor id
};

/// @brief Computes the component-wise difference of two 2D Vectors
/// @tparam T Type of member variables of vector
/// @param v1 2D Vector
/// @param v2 2D Vector
/// @return Vector with component-wise difference of v1 and v2
LUM_FLOATING_TEMPLATE_TYPE
inline Vector2<T> operator-(const Vector2<T>& v1, const Vector2<T>& v2)
{
  return {v1.x - v2.x, v1.y - v2.y};
}

/// @brief Computes the component-wise addition of two 2D Vectors
/// @tparam T Type of member variables of vector
/// @param v1 2D Vector
/// @param v2 2D Vector
/// @return Vector with component-wise sum of v1 and v2
LUM_FLOATING_TEMPLATE_TYPE
inline Vector2<T> operator+(const Vector2<T>& v1, const Vector2<T>& v2)
{
  return {v1.x + v2.x, v1.y + v2.y};
}

/// @brief Computes the component-wise negation of a 2D Vector
/// @tparam T Type of member variables of vector
/// @param v 2D Vector
/// @return Vector with components being the negative of the components of the argument
LUM_FLOATING_TEMPLATE_TYPE
inline Vector2<T> operator-(const Vector2<T>& v)
{
  return {-v.x, -v.y};
}

/// @brief Multiplies each component of the vector by a scalar
/// @tparam T Type of member variables of vector
/// @param scalar Number to scale the vector by
/// @param v 2D Vector
/// @return Vector with each component scaled by a scalar
LUM_FLOATING_TEMPLATE_TYPE
inline Vector2<T> operator*(const T& scalar, Vector2<T>& v)
{
  return {scalar * v.x, scalar * v.y};
}

/// @brief Checks if two vectors are equal
/// @tparam T Type of member variables of vector
/// @param v1 2D Vector
/// @param v2 2D Vector
/// @return True if each component of v1 and v2 are the same, else False
LUM_NUMERIC_TEMPLATE_TYPE
inline bool operator==(const Vector2<T>& v1, const Vector2<T>& v2)
{
  return (v1.x == v2.x && v1.y == v2.y);
}

/// @brief Checks if two vectors are not equal
/// @tparam T Type of member variables of vector
/// @param v1 2D Vector
/// @param v2 2D Vector
/// @return False if each component of v1 and v2 are the same, else True
LUM_NUMERIC_TEMPLATE_TYPE
inline bool operator!=(const Vector2<T>& v1, const Vector2<T>& v2)
{
  return !(v1 == v2);
}

/// @brief Computes the l2 norm of a vector
/// @tparam T Type of member variables of vector
/// @param v 2D Vector
/// @return l2-norm of the vector
LUM_FLOATING_TEMPLATE_TYPE
inline T l2Norm(const Vector2<T>& v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

/// @brief Computes the dot product between two vectors
/// @tparam T Type of member variables of vector
/// @param v1 2D Vector
/// @param v2 2D Vector
/// @return Dot product of the input vectors
LUM_FLOATING_TEMPLATE_TYPE
inline T dot(const Vector2<T>& v1, const Vector2<T>& v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

/// @brief Computes the cross product between two vectors
/// @tparam T Type of member variables of vector
/// @param v1 2D Vector
/// @param v2 2D Vector
/// @return Computes the cross product of the input vectors
LUM_FLOATING_TEMPLATE_TYPE
inline T cross(const Vector2<T>& v1, const Vector2<T>& v2)
{
  return v1.x * v2.y - v2.x * v1.y;
}

/// @brief Computes the component-wise difference of two 3D Vectors
/// @tparam T Type of member variables of vector
/// @param v1 3D Vector
/// @param v2 3D Vector
/// @return Vector with component-wise difference of v1 and v2
LUM_FLOATING_TEMPLATE_TYPE
inline Vector3<T> operator-(const Vector3<T>& v1, const Vector3<T>& v2)
{
  Vector3<T> result;
  result.x = v1.x - v2.x;
  result.y = v1.y - v2.y;
  result.z = v1.z - v2.z;
  return result;
}

/// @brief Computes the l2 norm of a vector
/// @tparam T Type of member variables of vector
/// @param v 3D Vector
/// @return l2-norm of the vector
LUM_FLOATING_TEMPLATE_TYPE
inline T l2Norm(const Vector3<T>& v1)
{
  return std::sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
}

/// @brief Computes the dot product between two vectors
/// @tparam T Type of member variables of vector
/// @param v1 3D Vector
/// @param v2 3D Vector
/// @return Dot product of the input vectors
LUM_FLOATING_TEMPLATE_TYPE
inline T dot(const Vector3<T>& v1, const Vector3<T>& v2)
{
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

} // namespace geometry
} // namespace types
} // namespace common
} // namespace lum

#endif // LUM_COMMON_GEOMETRY_GEOMETRY_H
