/**
 * @file torres_etal_2016.hpp
 * @brief Utility for computational geometry
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

#ifndef INCLUDED_cgutil_hpp_
#define INCLUDED_cgutil_hpp_

// c++ libraries
#include <algorithm>
#include <cmath>
#include <list>
#include <stack>
#include <vector>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/random_polygon_2.h>

// roscpp
#include <ros/ros.h>

// geometry_msgs
#include <geometry_msgs/Point.h>

using PointVector = std::vector<geometry_msgs::Point>;
using LineSegment = std::array<geometry_msgs::Point, 2>;
using LineSegmentVector = std::vector<LineSegment>;

// type aliases for cgal
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Traits = CGAL::Partition_traits_2<K>;
using Polygon_2 = Traits::Polygon_2;
using Point_2 = Traits::Point_2;
using Vertex_iterator = Polygon_2::Vertex_const_iterator;
using Polygon_list = std::list<Polygon_2>;

/**
 * @brief Calculates signed area of given triangle
 * @param p1 The origin of vector \f$ \vec{p_1p_2} \f$ and \f$ \vec{p_1p_3} \f$
 * @param p2 The end point of vector \f$ \vec{p_1p_2} \f$
 * @param p3 The end point of vector \f$ \vec{p_1p_3} \f$
 * @return Signed area of given triangle
 *
 * @details
 * Signed area of triangle \f$ S(p1, p2, p3) \f$ is
 * half of the outer product of vector \f$ \vec{p_1p_2} \f$ and \f$ \vec{p_1p_3}
 * \f$.\n
 * \f[ S(p_1, p_2, p_3) = \frac{1}{2} \vec{p_1p_2} \times \vec{p_1p_3}\f] \n
 * And that can be written as follows,\n
 *   \f{eqnarray*}{
 *       S(p_1, p_2, p_3) & = & \frac{1}{2} \left|
 *           \begin{array}{ccc}
 *               p_1.x & p_1.y & 1 \\
 *               p_2.x & p_2.y & 1 \\
 *               p_3.x & p_3.y & 1
 *           \end{array}
 *       \right| \\
 *           & = & p_1.x(p_2.y - p_3.y) - p_1.y(p_2.x - p_3.x) - (p_2.x\times
 * p_3.y - p_2.y\times p_3.x)
 *   \f}
 */
inline double calculateSignedArea(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
                                  const geometry_msgs::Point& p3)
{
  return p1.x * (p2.y - p3.y) - p1.y * (p2.x - p3.x) + (p2.x * p3.y - p2.y * p3.x);
}

/**
 * @brief Check equality of two points
 * @param p1
 * @param p2
 * @return bool
 * @details See https://stackoverflow.com/questions/4010240/comparing-doubles
 */
bool operator==(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  bool x =
      p1.x == p2.x || std::abs(p1.x - p2.x) < std::abs(std::min(p1.x, p2.x)) * std::numeric_limits<double>::epsilon();
  bool y =
      p1.y == p2.y || std::abs(p1.y - p2.y) < std::abs(std::min(p1.y, p2.y)) * std::numeric_limits<double>::epsilon();
  bool z =
      p1.z == p2.z || std::abs(p1.z - p2.z) < std::abs(std::min(p1.z, p2.z)) * std::numeric_limits<double>::epsilon();

  return x and y and z;
}

/**
 * @brief Check equality of two points
 * @param p1
 * @param p2
 * @return bool
 */
bool operator!=(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  return !(p1 == p2);
}

/**
 * @brief Generate Vector of line segment from given PointVector
 * @param vec
 * @param isClosed Set true if the first point and the last are connected
 * @return LineSegmentVector Vector of line segment a.k.a. std::vector<std::array<geometry_msgs::Point, 2>>
 */
LineSegmentVector generateEdgeVector(const PointVector& vec, bool isClosed)
{
  LineSegmentVector edgeVector;

  // break if vector is empty
  if (vec.empty() == true)
  {
    return edgeVector;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    LineSegment edge;

    edge.at(0) = vec.at(i);

    if (i < vec.size() - 1)
    {
      // except for the last vertex
      edge.at(1) = vec.at(i + 1);
    }
    else
    {
      // for the last vertex
      edge.at(1) = vec.at(0);
      if (not isClosed)
      {
        break;
      }
    }

    edgeVector.push_back(edge);
  }
  return edgeVector;
}

/**
 * @brief Calculates distance between given two points
 * @param p1
 * @param p2
 * @return double Distance between two points
 */
inline double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

/**
 * @brief Calculates angle between segment p1p2 and p1p3
 * @param p1 A vertex which is the origin of segment p1p2 and p1p3
 * @param p2 The other point of segment p1p2
 * @param p3 The other point of segment p1p3
 * @return Angle between segment p1p2 and p1p3 in radian [0, pi)
 */
double calculateVertexAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
                            const geometry_msgs::Point& p3)
{
  // Length of edges composed of vertices
  // e1: (p1, p2)
  // e2: (p2, p3)
  // e3: (p3, p1)
  double lenE1, lenE2, lenE3;
  lenE1 = calculateDistance(p2, p1);
  lenE2 = calculateDistance(p2, p3);
  lenE3 = calculateDistance(p3, p1);

  // Cosine of angle  between segment p1p2 and p1p3 (Law of cosines)
  double cosineP1;
  cosineP1 = (std::pow(lenE1, 2) + std::pow(lenE3, 2) - std::pow(lenE2, 2)) / (2 * lenE1 * lenE3);

  // vertex angle is 0.0 if lenE1 or lenE3 is zero
  // that means p1 and p2 or p1 and p3 is the same point
  if (std::isnan(cosineP1) != 0)
  {
    return 0.0;
  }

  return std::acos(cosineP1);
}

/**
 * @brief Calculates angle between segment p1p2 and horizontal line
 * @param p1 A vertex which is the origin of segment p1p2
 * @param p2 The other vertex of segment p1p2
 * @return Vertex angle of p1 in radian [-pi, pi]
 */
double calculateHorizontalAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  geometry_msgs::Point p3;
  p3.x = p1.x + 1.0;
  p3.y = p1.y;
  return p1.y >= p2.y ? -calculateVertexAngle(p1, p2, p3) : calculateVertexAngle(p1, p2, p3);
}

/**
 * @brief Calculates distance between given edge and vertex
 * @param edge An edge of given polygon
 * @param vertex A vertex of given polygon
 * @return double Distance between edge and vertex
 */
double calculateDistance(const LineSegment& edge, const geometry_msgs::Point& vertex)
{
  // Vertices of triangle
  geometry_msgs::Point pointA, pointB;
  pointA = edge.front();
  pointB = edge.back();

  // Calculate length of each edge
  // Edge A: An edge facing vertex A
  // Edge B: An edge facing vertex B
  double lenEdgeA, lenEdgeB;
  lenEdgeA = calculateDistance(pointB, vertex);
  lenEdgeB = calculateDistance(vertex, pointA);

  // Vertex angles
  // alpha: vertex angle of pointA
  // beta: vertex angle of pointB
  double alpha, beta;
  alpha = calculateVertexAngle(pointA, pointB, vertex);
  beta = calculateVertexAngle(pointB, pointA, vertex);

  double distance = alpha < M_PI_2 ? std::sin(alpha) * lenEdgeB : std::sin(beta) * lenEdgeA;

  return distance;
}

/**
 * @brief Returns convex hull of given points
 * @param points A set of points in the plane
 * @return Convex hull of given points
 *
 * This function is based on graham scan algorithm
 */
PointVector computeConvexHull(PointVector points)
{
  PointVector convexHull;

  if (points.empty() or points.size() < 3)
  {
    return convexHull;
  }

  // remove points that have same coodinate with other points
  for (size_t i = 0; i < points.size() - 1; ++i)
  {
    if (points.at(i).x == points.at(i + 1).x and points.at(i).y == points.at(i + 1).y)
    {
      points.erase(points.begin() + i);
    }
  }

  if (points.size() < 3)
  {
    return convexHull;
  }

  // sort by vertex's y coordinate in an ascending order
  std::stable_sort(points.begin(), points.end(),
                   [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return p1.y < p2.y; });

  // point with minimum y coordinate
  geometry_msgs::Point pointMinY = points.front();
  points.erase(points.begin());

  // sort by an angle between a segment composed of pointMinY and pj (in a set
  // of points) and horizontal line
  // in an ascending order
  std::stable_sort(points.begin(), points.end(), [&](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return calculateHorizontalAngle(pointMinY, p1) < calculateHorizontalAngle(pointMinY, p2);
  });

  // add pointMinY in convex hull
  convexHull.push_back(pointMinY);

  // add the point with minimum angle
  convexHull.push_back(points.front());
  points.erase(points.begin());

  for (const auto& point : points)
  {
    for (std::size_t i = convexHull.size() - 1; i > 1; --i)
    {
      if (calculateSignedArea(convexHull.at(i - 1), convexHull.at(i), point) >= 0)
      {
        break;
      }

      convexHull.pop_back();
    }
    convexHull.push_back(point);
  }

  geometry_msgs::Point origin;
  origin.x = 0;
  origin.y = 0;
  std::stable_sort(convexHull.begin(), convexHull.end(),
                   [&](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
                     return calculateHorizontalAngle(origin, p1) < calculateHorizontalAngle(origin, p2);
                   });

  return convexHull;
}

/**
 * @brief Checks if given polygon is convex or not
 * @param points Points consisting of polygon is to be checked
 * @return True if given polygon is convex, false if it's not convex
 */
inline bool isConvex(PointVector points)
{
  return computeConvexHull(points).size() == points.size();
}

/**
 * @brief Checks if given edges intersect each other
 * @param edge1 An edge
 * @param edge2 An edge
 * @return True if two edges intersect
 */
bool hasIntersection(const LineSegment& edge1, const LineSegment& edge2)
{
  geometry_msgs::Point p1, p2, p3, p4;

  try
  {
    p1 = edge1.at(0);
    p2 = edge1.at(1);
    p3 = edge2.at(0);
    p4 = edge2.at(1);
  }
  catch (std::out_of_range& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  bool condA = ((p1.x - p2.x) * (p3.y - p1.y) + (p1.y - p2.y) * (p1.x - p3.x)) *
                   ((p1.x - p2.x) * (p4.y - p1.y) + (p1.y - p2.y) * (p1.x - p4.x)) <
               0;
  bool condB = ((p3.x - p4.x) * (p1.y - p3.y) + (p3.y - p4.y) * (p3.x - p1.x)) *
                   ((p3.x - p4.x) * (p2.y - p3.y) + (p3.y - p4.y) * (p3.x - p2.x)) <
               0;

  if (condA and condB)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Checks if given vectors of edges have at least one intersection
 * @param vec1 Vector of line segments
 * @param vec2 Vector of line segments
 * @return True if given two vectors of edges have at least one intersection
 */
bool hasIntersection(const LineSegmentVector& vec1, const LineSegmentVector& vec2)
{
  for (const auto& segment1 : vec1)
  {
    for (const auto& segment2 : vec2)
    {
      if (hasIntersection(segment1, segment2) == true)
      {
        return true;
      }
    }
  }

  return false;
}

/**
 * @brief Find the location where given edges intersect each other
 * @param edge1
 * @param edge2
 * @return geometry_msgs::Point Point of intersection
 * @details See http://mf-atelier.sakura.ne.jp/mf-atelier/modules/tips/program/algorithm/a1.html
 */
geometry_msgs::Point localizeIntersection(const LineSegment& edge1, const LineSegment& edge2)
{
  geometry_msgs::Point p1, p2, p3, p4;

  try
  {
    p1 = edge1.at(0);
    p2 = edge1.at(1);
    p3 = edge2.at(0);
    p4 = edge2.at(1);
  }
  catch (const std::out_of_range& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  double xi, eta, delta;
  xi = (p4.y - p3.y) * (p4.x - p1.x) - (p4.x - p3.x) * (p4.y - p1.y);
  eta = -(p2.y - p1.y) * (p4.x - p1.x) + (p2.x - p1.x) * (p4.y - p1.y);
  delta = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);

  double lambda, mu;
  lambda = xi / delta;
  mu = eta / delta;

  geometry_msgs::Point intersection;

  intersection.x = p1.x + lambda * (p2.x - p1.x);
  intersection.y = p1.y + lambda * (p2.y - p1.y);

  return intersection;
}

/**
 * @brief Rotate points by given angle around the origin
 * @param points Points to be rotated
 * @param angle_rad Rotation angle in radian
 * @return PointVector Rotated points
 */
PointVector rotatePoints(const PointVector& points, double angle_rad)
{
  std::array<double, 4> rotationMatrix;
  rotationMatrix.at(0) = std::cos(angle_rad);
  rotationMatrix.at(1) = -std::sin(angle_rad);
  rotationMatrix.at(2) = std::sin(angle_rad);
  rotationMatrix.at(3) = std::cos(angle_rad);

  PointVector rotatedPoints;

  for (const auto& point : points)
  {
    geometry_msgs::Point pt;
    pt.x = rotationMatrix.at(0) * point.x + rotationMatrix.at(1) * point.y;
    pt.y = rotationMatrix.at(2) * point.x + rotationMatrix.at(3) * point.y;
    rotatedPoints.push_back(pt);
  }
  return rotatedPoints;
}

/**
 * @brief Decompose given polygon
 * @param polygon Polygon to be decomposed
 * @return std::vector<PointVector> Decomposed polygons
 * @details
 * This function uses CGAL::optimal_convex_partition_2 in order to perform optimal polygon decomposition.
 * Note that this function has O(n^4) time complexity and O(n^3) space complexity.
 * Use approx_convex_partition_2 instead if the number of vertices are big because its time complexity is O(n).
 * But apptox_convex_partition_2 generates more polygons.
 * For detail, see https://doc.cgal.org/latest/Partition_2/index.html
 */
std::vector<PointVector> decomposePolygon(const PointVector& polygon)
{
  std::vector<PointVector> decomposedPolygons;

  // generate Polygon of CGAL from PointVector
  Polygon_2 cgalPolygon;
  for (const auto& vertex : polygon)
  {
    cgalPolygon.push_back(Point_2(vertex.x, vertex.y));
  }

  Polygon_list partialCGALPolygons;
  Traits partitionTraits;

  // note that this function has O(n^4) time complexity and O(n^3) space complexity
  // use approx_convex_partition_2 instead if the number of vertices are big because its time complexity is O(n)
  // but apptox_convex_partition_2 generates more polygons
  CGAL::optimal_convex_partition_2(cgalPolygon.vertices_begin(), cgalPolygon.vertices_end(),
                                   std::back_inserter(partialCGALPolygons), partitionTraits);

  // generate std::vector<PointVector> from polygon of CGAL
  for (const auto& partialCGALPolygon : partialCGALPolygons)
  {
    PointVector partialPolygon;
    for (auto itr = partialCGALPolygon.vertices_begin(); itr != partialCGALPolygon.vertices_end(); ++itr)
    {
      geometry_msgs::Point pt;
      pt.x = itr->x();
      pt.y = itr->y();
      partialPolygon.push_back(pt);
    }
    decomposedPolygons.push_back(partialPolygon);
  }

  return decomposedPolygons;
}

#endif