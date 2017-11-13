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
#include <stack>
#include <vector>

// roscpp
#include <ros/ros.h>
// geometry_msgs
#include <geometry_msgs/Point.h>

/**
* @brief Calculates signed area of given triangle
* @param p1 The origin of vector \f$ \vec{p_1p_2} \f$ and \f$ \vec{p_1p_3} \f$
* @param p2 The end point of vector \f$ \vec{p_1p_2} \f$
* @param p3 The end point of vector \f$ \vec{p_1p_3} \f$
* @return Signed area of given triangle
*
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
inline double signedArea(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3)
{
  return p1.x * (p2.y - p3.y) - p1.y * (p2.x - p3.x) + (p2.x * p3.y - p2.y * p3.x);
}

/**
* @brief Calculates angle between segment p1p2 and p1p3
* @param p1 A vertex which is the origin of segment p1p2 and p1p3
* @param p2 The other point of segment p1p2
* @param p3 The other point of segment p1p3
* @return Angle between segment p1p2 and p1p3 in radian [0, pi)
*/
double vertexAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3)
{
  // Length of edges composed of vertices
  // e1: (p1, p2)
  // e2: (p2, p3)
  // e3: (p3, p1)
  double len_e1, len_e2, len_e3;
  len_e1 = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
  len_e2 = std::sqrt(std::pow(p2.x - p3.x, 2) + std::pow(p2.y - p3.y, 2));
  len_e3 = std::sqrt(std::pow(p3.x - p1.x, 2) + std::pow(p3.y - p1.y, 2));

  // Cosine of angle  between segment p1p2 and p1p3 (Law of cosines)
  double cosine_p1;
  cosine_p1 = (std::pow(len_e1, 2) + std::pow(len_e3, 2) - std::pow(len_e2, 2)) / (2 * len_e1 * len_e3);

  // vertex angle is 0.0 if len_e1 or len_e3 is zero
  // that means p1 and p2 or p1 and p3 is the same point
  if (std::isnan(cosine_p1) != 0)
  {
    return 0.0;
  }

  return std::acos(cosine_p1);
}

/**
* @brief Calculates angle between segment p1p2 and horizontal line
* @param p1 A vertex which is the origin of segment p1p2
* @param p2 The other vertex of segment p1p2
* @return Vertex angle of p1 in radian [0, pi)
*/
double horizontalAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  geometry_msgs::Point p3;
  p3.x = p1.x + 1.0;
  p3.y = p1.y;
  return vertexAngle(p1, p2, p3);
}

/**
* @brief Calculates distance between given edge and vertex
* @param edge An edge of given polygon
* @param vertex A vertex of given polygon
* @return double Distance between edge and vertex
*/
double distance(const std::array<geometry_msgs::Point, 2>& edge, const geometry_msgs::Point& vertex)
{
  // Vertices of triangle
  geometry_msgs::Point point_a, point_b;
  point_a = edge.front();
  point_b = edge.back();

  // Calculate length of each edge
  // Edge A: An edge facing vertex A
  // Edge B: An edge facing vertex B
  double len_edge_a, len_edge_b, len_edge_c;
  len_edge_a = std::sqrt(std::pow(point_b.x - vertex.x, 2) + std::pow(point_b.y - vertex.y, 2));
  len_edge_b = std::sqrt(std::pow(vertex.x - point_a.x, 2) + std::pow(vertex.y - point_a.y, 2));

  // Vertex angles
  // alpha: vertex angle of point_a
  // beta: vertex angle of point_b
  double alpha, beta;
  alpha = vertexAngle(point_a, point_b, vertex);
  beta = vertexAngle(point_b, point_a, vertex);

  double distance = alpha < M_PI_2 ? std::sin(alpha) * len_edge_b : std::sin(beta) * len_edge_a;

  return distance;
}

/**
* @brief Returns convex hull of given points
* @param points A set of points in the plane
* @return Convex hull of given points
*
* This function is based on graham scan algorithm
*/
std::vector<geometry_msgs::Point> grahamScan(std::vector<geometry_msgs::Point> points)
{
  // convex hull
  std::vector<geometry_msgs::Point> convex_hull;

  if (points.empty())
  {
    return convex_hull;
  }

  for (size_t i = 0; i < points.size() - 1; ++i)
  {
    if (points.at(i).x == points.at(i + 1).x and points.at(i).y == points.at(i + 1).y)
    {
      points.erase(points.begin() + i);
    }
  }

  if (points.size() < 3)
  {
    return convex_hull;
  }

  // sort by vertex's y coordinate in an ascending order
  std::stable_sort(points.begin(), points.end(),
                   [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return p1.y < p2.y; });

  // point with minimum y coordinate
  geometry_msgs::Point min_y_point = points.front();
  points.erase(points.begin());

  // sort by an angle between a segment composed of min_y_point and pj (in a set
  // of points) and horizontal line
  // in an ascending order
  std::stable_sort(points.begin(), points.end(), [&](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return horizontalAngle(min_y_point, p1) < horizontalAngle(min_y_point, p2);
  });

  // add min_y_point in convex hull
  convex_hull.push_back(min_y_point);

  // add the point with minimum angle
  convex_hull.push_back(points.front());
  points.erase(points.begin());

  for (const auto& point : points)
  {
    for (size_t i = convex_hull.size() - 1; i > 1; --i)
    {
      if (signedArea(convex_hull.at(i - 1), convex_hull.at(i), point) >= 0)
      {
        break;
      }

      convex_hull.pop_back();
    }
    convex_hull.push_back(point);
  }

  return convex_hull;
}

bool inBetween(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3,
               double epsilon = 1e-5)
{
  if (p1.x < p2.x)
  {
    if (p3.x < p1.x or p2.x < p3.x)
    {
      return false;
    }
  }
  else if (p2.x < p1.x)
  {
    if (p3.x < p2.x or p1.x < p3.x)
    {
      return false;
    }
  }
  else
  {
    if (p3.x < p1.x)
    {
      return false;
    }
  }
  return std::abs(vertexAngle(p1, p2, p3)) < epsilon;
}

bool intersect(const std::array<geometry_msgs::Point, 2>& edge1, const std::array<geometry_msgs::Point, 2>& edge2)
{
  if (inBetween(edge1.at(0), edge1.at(1), edge2.at(0)) or inBetween(edge1.at(0), edge1.at(1), edge2.at(1)) or
      inBetween(edge2.at(0), edge2.at(1), edge1.at(0)) or inBetween(edge2.at(0), edge2.at(1), edge1.at(0)))
  {
    return true;
  }
  return (signedArea(edge1.at(0), edge1.at(1), edge2.at(0)) * signedArea(edge1.at(0), edge1.at(1), edge2.at(1)) < 0 and
          signedArea(edge2.at(0), edge2.at(1), edge1.at(0)) * signedArea(edge2.at(0), edge2.at(1), edge1.at(1)) < 0);
}

std::vector<std::array<std::array<geometry_msgs::Point, 2>, 2> >
intersect(const std::vector<std::array<geometry_msgs::Point, 2> >& segments)
{
  std::vector<std::array<std::array<geometry_msgs::Point, 2>, 2> > intersecting_segments;
  for (size_t i = 0; i < segments.size() - 1; ++i)
  {
    for (size_t j = i + 1; j < segments.size(); ++j)
    {
      if (intersect(segments.at(i), segments.at(j)))
      {
        std::array<std::array<geometry_msgs::Point, 2>, 2> segment;
        segment.at(0) = segments.at(i);
        segment.at(1) = segments.at(j);
        intersecting_segments.push_back(segment);
      }
    }
  }
  return intersecting_segments;
}

#endif
