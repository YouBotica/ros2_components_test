// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_IOU_H
#define LUM_COMMON_UTILS_IOU_H

#include <algorithm>
#include <array>
#include <unordered_set>

#include <lum_common_geometry_types/geometry_types.h>
#include <lum_common_log/log.h>

namespace lum {
namespace common {
namespace utils {

struct PointXY
{
  float x{0}; ///< x coordinate of the point
  float y{0}; ///< y coordinate of the point

  /// @brief Constructors
  explicit PointXY(float x, float y) : x(x), y(y) {}
  explicit PointXY() = default;

  /// @brief helper to print a line segment
  void print() const { LUM_LOG_INFO << "(" << x << "," << y << ")"; }

  /// @brief helper to find if two points are the same
  bool operator==(const PointXY& other) const { return (this->x == other.x && this->y == other.y); }

  /// @brief helper to find the vector perpendicular to the position vector of this point
  PointXY perpendicular() const { return PointXY{-y, x}; }

  /// @brief helper to find difference between 2 points
  PointXY operator-(const PointXY& other) const { return PointXY{x - other.x, y - other.y}; }

  /// @brief helper to add 2 points
  PointXY operator+(const PointXY& other) const { return PointXY{x + other.x, y + other.y}; }

  /// @brief helper to divide by a scalar
  PointXY operator/(float scalar) const { return PointXY{x / scalar, y / scalar}; }
};

/// @brief helper function to compute dot products of 2 points
inline float dot(const PointXY& p1, const PointXY& p2)
{
  return p1.x * p2.x + p1.y * p2.y;
}

// class for hash function
class PointXYHashFunction
{
public:
  // id is returned as hash function
  size_t operator()(const PointXY& p) const
  {
    return std::hash<float>{}(p.x) ^ std::hash<float>{}(p.y);
  }
};

struct LineSegment
{
  PointXY start; ///< starting point of the line segment
  PointXY end;   ///< ending point of the line segment

  /// @brief Constructors
  explicit LineSegment(PointXY start, PointXY end) : start(start), end(end) {}
  explicit LineSegment() = default;

  /// @brief helper to print a line segment
  void print()
  {
    LUM_LOG_INFO << "(" << start.x << "," << start.y << ") -> (" << end.x << "," << end.y << ")";
  }

  /// @brief helper function to determine if a given point is in between the line segment
  /// @param query given point on line joining start and end to be checked if it is on line segment
  /// @return true if query is on this line segment
  bool isInBetween(const PointXY& query) const
  {
    return (((start.x - query.x) * (end.x - query.x) + (start.y - query.y) * (end.y - query.y)) <=
            0);
  }
};

/// @brief function to find if a point's position vector is on the right of another point's position
/// vector. That is, checks if unit vector along p2 can be obtained by rotating the unit vector
/// along p1 in clockwise direction by an acute angle
/// @param p1 first point
/// @param p2 second point
/// @return 1 if p2 is on the right of p1, 0 if neither on right or left, -1 if on left
inline int isOnRight(const PointXY& p1, const PointXY& p2)
{
  auto p2_perpendicular = p2.perpendicular();
  float dot_product = dot(p1, p2_perpendicular);
  // On right
  if (dot_product > 0)
  {
    return 1;
  }
  // neither right not left
  else if (dot_product == 0)
  {
    return 0;
  }
  // On left
  else
  {
    return -1;
  }
}

/// @brief Struct to represent a Rectangle. Assumes the corners are cyclic or acyclic
using Rectangle = std::array<PointXY, 4>;

/// @brief helper function that returns true if a point is strictly interior to the
/// given rectangle
/// @param r given rectangle
/// @param p query point to check if it is interior to the rectangle
inline bool isInterior(const Rectangle& r, const PointXY& p)
{
  // Finds if a point is to the same side of the edges for all 4 edges
  std::size_t number_of_sides{4};
  bool is_on_right{false};
  for (std::size_t index{0}; index < number_of_sides; ++index)
  {

    auto side_vector = r[(index + 1) % number_of_sides] - r[index];
    auto query_position_vector = (r[(index + 1) % number_of_sides] - p);
    int is_on_right_result = isOnRight(side_vector, query_position_vector);
    // On the rectangle
    if (is_on_right_result == 0)
    {
      return false;
    }

    bool current_is_on_right = is_on_right_result == 1;
    if (index != 0 && current_is_on_right != is_on_right)
    {
      return false;
    }
    is_on_right = current_is_on_right;
  }
  return true;
}

/// @brief helper function that returns area of a given rectangle
/// @param r given rectangle
inline float areaOfRectangle(const Rectangle& r)
{
  auto length = r[0] - r[1];
  auto breadth = r[1] - r[2];
  return sqrt(pow(length.x, 2) + pow(length.y, 2)) * sqrt(pow(breadth.x, 2) + pow(breadth.y, 2));
}

/// @brief helper function to determine if a given point is in between the line segment connecting 2
/// points
/// @param p1 start point of line segment p as a pair of x-y coordinate
/// @param p2 end point of line segment p as a pair of x-y coordinate
/// @param q1 given point on line joining p1 and p2 to be checked if it is on line segment
/// p
/// @return true if q1 is on p

inline bool isInBetween(const std::pair<float, float>& p1,
                        const std::pair<float, float>& p2,
                        const std::pair<float, float>& q1)
{
  return (((p1.first - q1.first) * (p2.first - q1.first) +
           (p1.second - q1.second) * (p2.second - q1.second)) <= 0);
}

/// @brief function to get a point from substituting a paremeter in the parametric form of line
/// equation p = p1 + t(p2-p1)
/// @param p1 start point of line segment p as a pair of x-y coordinate
/// @param p2 end point of line segment p as a pair of x-y coordinate
/// @param t parameter of the parametric form of line p
/// @return p0 = p1 + t(p2-p1)
inline std::pair<float, float>
getPointfromParameter(const std::pair<float, float>& p1, const std::pair<float, float>& p2, float t)
{
  return {p1.first + t * (p2.first - p1.first), p1.second + t * (p2.second - p1.second)};
}

/// @brief helper function to find the intersection of 2 line segments p and q
/// @param p first line segment
/// @param q secopnd line segment
/// @return points of intersections of the line segments as pairs of x-y coordinate. Empty if they
/// do not intersect, 1 if the intersect exactly at 1 point, and 2 if they intersect and have the
/// same slope (the 2 end points of the intersection)
inline std::unordered_set<PointXY, PointXYHashFunction> getIntersections(const LineSegment& p,
                                                                         const LineSegment& q)
{
  /***
  Taking the parametric form of line p => p1 + t_p * (p2 - p1) , s.t (0 <= t_p <= 1)
  Taking the parametric form of line q => q1 + t_q * (q2 - q1) , s.t (0 <= t_q <= 1)
  Intersection is at:
                        (q2x-q1x)(p1y-q1y)-(q2y-q1y)(p1x-q1x)
                 t_p = ----------------------------------------
                        (p2x-p1x)(q2y-q1y)-(p2y-p1y)(q2x-q1x)

                        (p1y-q1y)(p2x-p1x)-(p1x-q1x)(p2y-p1y)
                 t_q = ----------------------------------------
                        (p2x-p1x)(q2y-q1y)-(p2y-p1y)(q2x-q1x)

  Cases:
     Case 1) Parallel line segments with no intersection
             =>  t_p, t_q -> inf (only denominator is 0)
     Case 2) Parallel line segments with intersections (collinear points)
             =>  t_p, t_q are undefined (both numerator and denominator are 0) declare the end
                 points of the intersection as the intersecting points
     Case 3) Intersecting line segments at an angle (single intersection)
             =>  (0 <= t_p <= 1) and (0 <= t_q <= 1)
  ***/

  std::unordered_set<PointXY, PointXYHashFunction> intersecting_points{};

  const float denominator =
    (p.end.x - p.start.x) * (q.end.y - q.start.y) - (p.end.y - p.start.y) * (q.end.x - q.start.x);

  const float t_p_numerator = (q.end.x - q.start.x) * (p.start.y - q.start.y) -
                              (q.end.y - q.start.y) * (p.start.x - q.start.x);

  const float t_q_numerator = (p.start.y - q.start.y) * (p.end.x - p.start.x) -
                              (p.start.x - q.start.x) * (p.end.y - p.start.y);

  // Case 1, check if denominator is zero
  if (denominator == 0)
  {
    // Numerator is non zero
    if (t_p_numerator != 0) // It is assured that if t_p is non zero here tq will be non zero
    {
      // This is case1 with no intersections
      return {};
    }

    // Numerator is zero => collinear points
    else
    {
      if (p.isInBetween(q.start))
      {
        intersecting_points.insert(q.start);
      }
      if (p.isInBetween(q.end))
      {
        intersecting_points.insert(q.end);
      }
      // There can only be 2 ends max of the intersecting region
      if (q.isInBetween(p.start) && intersecting_points.size() < 2)
      {
        intersecting_points.insert(p.start);
      }
      // There can only be 2 ends max of the intersecting region
      if (q.isInBetween(p.end) && intersecting_points.size() < 2)
      {
        intersecting_points.insert(p.end);
      }
    }
  }
  else
  {
    float t_p = t_p_numerator / denominator;
    float t_q = t_q_numerator / denominator;
    if (0 <= t_p && t_p <= 1 && 0 <= t_q && t_q <= 1)
    { // Substitute parameter t_p to get back the intersection
      intersecting_points.insert(
        PointXY{p.start.x + t_p * (p.end.x - p.start.x), p.start.y + t_p * (p.end.y - p.start.y)});
    }
  }
  return intersecting_points;
}

/// @brief helper function to find the intersection of 2 rectangles r1 and r2
/// @param r1 first rectangle
/// @param r2 second rectangle
/// @return points of intersections of the rectangles r1 and r2
inline std::unordered_set<PointXY, PointXYHashFunction> getIntersections(const Rectangle& r1,
                                                                         const Rectangle& r2)
{
  std::unordered_set<PointXY, PointXYHashFunction> intersections;
  const std::size_t number_of_sides{4};
  for (std::size_t r1_index{0}; r1_index < number_of_sides; ++r1_index)
  {
    for (std::size_t r2_index{0}; r2_index < number_of_sides; ++r2_index)
    {
      const auto& segment_intersections =
        getIntersections(LineSegment{r1[r1_index], r1[(r1_index + 1) % number_of_sides]},
                         LineSegment{r2[r2_index], r2[(r2_index + 1) % number_of_sides]});
      intersections.insert(segment_intersections.begin(), segment_intersections.end());
    }
  }

  // Check if any of the corner points of r1 is interior to the other rectangle
  for (const auto& corner : r1)
  {
    if (isInterior(r2, corner))
    {
      intersections.insert(corner);
    }
  }

  // Check if any of the corner points of r2 is interior to the other rectangle
  for (const auto& corner : r2)
  {
    if (isInterior(r1, corner))
    {
      intersections.insert(corner);
    }
  }

  return intersections;
}

/// @brief helper function to arrange a set of points in anticlockwise order
/// @param[out] points points contained in a stl like container to be sorted based on azimuth in a
/// clockwise fashion
template <typename ContainerType>
void sortedPoints(ContainerType& points)
{

  PointXY center{0, 0};
  // average the points to find the center
  for (const auto& point : points)
  {
    center = center + point;
  }

  center = center / points.size();

  std::sort(points.begin(), points.end(), [&center](const PointXY& p1, const PointXY& p2) {
    // Compute the difference with an interior point (center)
    auto d1 = p1 - center;
    auto d2 = p2 - center;
    int is_on_right = isOnRight(d1, d2);
    // Check if d1 and d2 are anti parallel
    if (is_on_right == 0)
    {
      if (dot(d1, d2) > 0) // overlapping vectors
      {
        return false;
      }
      if (d2.y > 0) // In this case, d2 is in first 2 quadrants while d1 is in 3 or 4. So
                    // azimuth(d1) > azimuth(d2)
      {
        return true;
      }
      else if (d2.y == 0) // In this case, they are x axis aligned
      {
        return d2.x > 0;
      }
      else // d1 is in the first 2 quadrants
      {
        return false;
      }
    }

    // Check if positive span of d1 and d2 contain positive x. If yes, then we need to account for
    // the 2-pi rotation (flip sign)
    return (d2.y != 0) && (d1.y * d2.y < 0) && (d1.x - (d1.y / d2.y) * d2.x > 0) ? is_on_right != 1
                                                                                 : is_on_right == 1;
  });
}
/// @brief helper function to arage a set of points in anticlockwise order
/// @param[in] points points contained in a stl like container
/// @return points ordered clockwise
template <typename ContainerType>
std::vector<PointXY> getSortedPoints(const ContainerType& points)
{
  std::vector<PointXY> sorted_points{points.begin(), points.end()};
  sortedPoints(sorted_points);
  return sorted_points;
}

/// @brief helper function to find the area of a triangle
/// @param v1 first vertex of a triangle
/// @param v2 second vertex of a triangle
/// @param v3 third vertex of a triangle
/// @return Returns area of triangle given the vertices
/// https://www.mathopenref.com/coordtrianglearea.html
inline float getAreaOfTriangle(const PointXY& v1, const PointXY& v2, const PointXY& v3)
{
  return std::abs(v1.x * (v2.y - v3.y) + v2.x * (v3.y - v1.y) + v3.x * (v1.y - v2.y)) /
         2.0F; // NOLINT
}

/// @brief helper function to find the area of a polygon given its vertices
/// @param corners points contained in a stl like container that represent the corners of the
/// polygon. https://www.mathopenref.com/coordpolygonarea.html
/// @return area of the polygon
template <typename ContainerType>
float getAreaOfPolygon(const ContainerType& corners)
{
  if (corners.size() < 3)
  {
    return 0.0F;
  }
  // Sort the corners and find the area of the polygon using one triangle at a time
  auto sorted_corners = getSortedPoints(corners);
  float total_area{0.0F};
  for (std::size_t index = 0; index < sorted_corners.size(); ++index)
  {
    const auto& p1 = sorted_corners[index];
    const auto& p2 = sorted_corners[(index + 1) % sorted_corners.size()];
    total_area += (p1.x * p2.y - p1.y * p2.x) / 2;
  }
  return std::abs(total_area);
}

/// @brief get intersection over union of 2 rectangles
/// @param r1 first rectangle
/// @param r2 second rectangle
/// @return intersection over union
inline float iou(const Rectangle& r1, const Rectangle& r2)
{
  const auto intersection_corners = getIntersections(r1, r2);

  float intersection_area = getAreaOfPolygon(intersection_corners);
  float union_area = areaOfRectangle(r1) + areaOfRectangle(r2) - intersection_area;
  if (union_area == 0)
  {
    return 0;
  }
  return intersection_area / union_area;
}

/// @brief convert a Axis aligned BoundingBox3D to type Rectangle
/// @param bounding_box box to be converted to Rectangle
/// @return A rectangle
inline Rectangle getRectangleFromAxisAlignedBoundingBox(
  const lum::common::types::geometry::BoundingBox3D<float>& bounding_box)
{
  Rectangle rectangle;
  rectangle[0] = PointXY{bounding_box.pose.position.x + bounding_box.scale.x / 2.F,
                         bounding_box.pose.position.y - bounding_box.scale.y / 2.F};
  rectangle[1] = PointXY{bounding_box.pose.position.x - bounding_box.scale.x / 2.F,
                         bounding_box.pose.position.y - bounding_box.scale.y / 2.F};
  rectangle[2] = PointXY{bounding_box.pose.position.x - bounding_box.scale.x / 2.F,
                         bounding_box.pose.position.y + bounding_box.scale.y / 2.F};
  rectangle[3] = PointXY{bounding_box.pose.position.x + bounding_box.scale.x / 2.F,
                         bounding_box.pose.position.y + bounding_box.scale.y / 2.F};

  return rectangle;
}

/// @brief convert a yaw-ed BoundingBox3D to type Rectangle by taking the top down projection of
/// the box
/// @param bounding_box box to be converted to Rectangle
/// @return A rectangle that is formed by the top most 4 points of the box (highest z) projected in
/// x-y plane
inline Rectangle getRectangleFromBoundingBoxYawOnly(
  const lum::common::types::geometry::BoundingBox3D<float>& bounding_box)
{
  using PointXYZ = lum::common::types::geometry::Position<float>;
  const auto& orientation = bounding_box.pose.orientation;

  // Compute the rotated x axis and rotated y_axis directions in the x-y plane
  PointXYZ rotated_x;
  PointXYZ rotated_y;

  // Compute the components of rotated x and y axes according to
  // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion.

  // Contribution of rotated x-scale in x, y dimensions
  rotated_x.x = (pow(orientation.x, 2) + pow(orientation.w, 2) - pow(orientation.y, 2) -
                 pow(orientation.z, 2)) *
                bounding_box.scale.x / 2.F;
  rotated_x.y = 2 * (orientation.x * orientation.y + orientation.w * orientation.z) *
                bounding_box.scale.x / 2.F;

  // Contribution of rotated y-scale in x, y dimensions
  rotated_y.x = 2 * (orientation.x * orientation.y - orientation.w * orientation.z) *
                bounding_box.scale.y / 2.F;
  rotated_y.y = (pow(orientation.y, 2) + pow(orientation.w, 2) - pow(orientation.x, 2) -
                 pow(orientation.z, 2)) *
                bounding_box.scale.y / 2.F;

  // Compute all the 4 corners of the rectangle
  Rectangle rectangle{PointXY{bounding_box.pose.position.x + rotated_x.x - rotated_y.x,
                              bounding_box.pose.position.y + rotated_x.y - rotated_y.y},
                      PointXY{bounding_box.pose.position.x - rotated_x.x - rotated_y.x,
                              bounding_box.pose.position.y - rotated_x.y - rotated_y.y},
                      PointXY{bounding_box.pose.position.x - rotated_x.x + rotated_y.x,
                              bounding_box.pose.position.y - rotated_x.y + rotated_y.y},
                      PointXY{bounding_box.pose.position.x + rotated_x.x + rotated_y.x,
                              bounding_box.pose.position.y + rotated_x.y + rotated_y.y}};

  return rectangle;
}

/// @brief convert a rotated BoundingBox3D to type Rectangle by taking the top down projection of
/// the box
/// @param bounding_box box to be converted to Rectangle
/// @return A rectangle that is formed by the top most 4 points of the box (highest z) projected in
/// x-y plane
inline Rectangle
getRectangleFromBoundingBox(const lum::common::types::geometry::BoundingBox3D<float>& bounding_box)
{
  using PointXYZ = lum::common::types::geometry::Position<float>;
  const auto& orientation = bounding_box.pose.orientation;

  // Sides in a rectangle
  const std::size_t num_corners = 4;

  // Compute the rotated x axis and rotated y_axis directions in the x-y plane
  PointXYZ rotated_x;
  PointXYZ rotated_y;
  PointXYZ rotated_z;

  // Compute the components of rotated x, y, and z axes according to
  // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion.

  // Contribution of rotated x-scale in x, y, and z dimensions
  rotated_x.x = (pow(orientation.x, 2) + pow(orientation.w, 2) - pow(orientation.y, 2) -
                 pow(orientation.z, 2)) *
                bounding_box.scale.x / 2.F;
  rotated_x.y = 2 * (orientation.x * orientation.y + orientation.w * orientation.z) *
                bounding_box.scale.x / 2.F;
  rotated_x.z = 2 * (orientation.x * orientation.z - orientation.w * orientation.y) *
                bounding_box.scale.x / 2.F;

  // Contribution of rotated y-scale in x, y, and z dimensions
  rotated_y.x = 2 * (orientation.x * orientation.y - orientation.w * orientation.z) *
                bounding_box.scale.y / 2.F;
  rotated_y.y = (pow(orientation.y, 2) + pow(orientation.w, 2) - pow(orientation.x, 2) -
                 pow(orientation.z, 2)) *
                bounding_box.scale.y / 2.F;
  rotated_y.z = 2 * (orientation.y * orientation.z + orientation.w * orientation.x) *
                bounding_box.scale.y / 2.F;

  // Contribution of rotated z-scale in x, y, and z dimensions
  rotated_z.x = 2 * (orientation.x * orientation.z + orientation.w * orientation.y) *
                bounding_box.scale.z / 2.F;
  rotated_z.y = 2 * (orientation.y * orientation.z - orientation.w * orientation.x) *
                bounding_box.scale.z / 2.F;
  rotated_z.z = (pow(orientation.z, 2) + pow(orientation.w, 2) - pow(orientation.x, 2) -
                 pow(orientation.y, 2)) *
                bounding_box.scale.z / 2.F;

  // Compute all the 8 corners of the cuboid
  std::array<PointXYZ, 2 * num_corners> corners;
  for (std::size_t index{0}; index < 2 * num_corners; ++index)
  {

    // Corner point to be computed
    float x = bounding_box.pose.position.x;
    float y = bounding_box.pose.position.y;
    float z = bounding_box.pose.position.z;

    // Compute the signs of the x, y, z components from the rotation in a binary fashion from
    // (0,0,0), (1,0,0) ... to (1,1,1). That is, (-,-,-), (+,-,-) ... to (+,+,+)

    x += (index & 1U) != 0U ? rotated_x.x : -rotated_x.x;
    y += (index & 1U) != 0U ? rotated_x.y : -rotated_x.y;
    z += (index & 1U) != 0U ? rotated_x.z : -rotated_x.z;

    x += (index >> 1U & 1U) != 0U ? rotated_y.x : -rotated_y.x;
    y += (index >> 1U & 1U) != 0U ? rotated_y.y : -rotated_y.y;
    z += (index >> 1U & 1U) != 0U ? rotated_y.z : -rotated_y.z;

    x += (index >> 2U & 1U) != 0U ? rotated_z.x : -rotated_z.x;
    y += (index >> 2U & 1U) != 0U ? rotated_z.y : -rotated_z.y;
    z += (index >> 2U & 1U) != 0U ? rotated_z.z : -rotated_z.z;

    corners[index].x = x;
    corners[index].y = y;
    corners[index].z = z;
  }

  //  Find the top 4 points with highest z and take their x, y projection
  std::partial_sort(corners.begin(),
                    corners.begin() + num_corners,
                    corners.end(),
                    [](const PointXYZ& p1, const PointXYZ& p2) { return p1.z > p2.z; });

  Rectangle rectangle;

  // Compute 4 corners as (center-x-y+z, center+x-y+z, center+x+y+z, center-x+y+z) to get the top
  // down projection of the 3D box
  for (std::size_t index{0}; index < num_corners; ++index)
  {
    rectangle[index] = PointXY{corners[index].x, corners[index].y};
  }

  sortedPoints(rectangle);

  return rectangle;
}

/// @brief get intersection over union of 2 BoundingBoxes3D without rotation
/// @param b1 first bounding_box
/// @param b2 second bounding_box
/// @return intersection over union
inline float iouAxisAligned(const lum::common::types::geometry::BoundingBox3D<float>& b1,
                            const lum::common::types::geometry::BoundingBox3D<float>& b2)
{
  return iou(getRectangleFromAxisAlignedBoundingBox(b1),
             getRectangleFromAxisAlignedBoundingBox(b2));
}

/// @brief get intersection over union of 2 BoundingBoxes3D with yaw only
/// @param b1 first bounding_box
/// @param b2 second bounding_box
/// @return intersection over union
inline float iouYawOnly(const lum::common::types::geometry::BoundingBox3D<float>& b1,
                        const lum::common::types::geometry::BoundingBox3D<float>& b2)
{
  return iou(getRectangleFromBoundingBoxYawOnly(b1), getRectangleFromBoundingBoxYawOnly(b2));
}

/// @brief get intersection over union of 2 BoundingBoxes3D with arbitrary rotaion
/// @param b1 first bounding_box
/// @param b2 second bounding_box
/// @return intersection over union
inline float iou(const lum::common::types::geometry::BoundingBox3D<float>& b1,
                 const lum::common::types::geometry::BoundingBox3D<float>& b2)
{
  return iou(getRectangleFromBoundingBox(b1), getRectangleFromBoundingBox(b2));
}

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_IOU_H
