/**
 * @file torres_etal_2016.hpp
 * @brief Header file for torres_etal_2016.cpp
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

#ifndef INCLUDED_torres_etal_2016_hpp_
#define INCLUDED_torres_etal_2016_hpp_

// cgutil
#include <cgutil.hpp>

// cpp standard libraries
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

// Boost
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/reversed.hpp>

// roscpp
#include <ros/ros.h>

// std_msgs
#include <std_msgs/Float64.h>

// geometry_msgs
#include <geometry_msgs/Point.h>

using PointVector = std::vector<geometry_msgs::Point>;
using LineSegment = std::array<geometry_msgs::Point, 2>;
using LineSegmentVector = std::vector<LineSegment>;

/**
 * @struct Direction
 * @brief Storage for line sweep direction
 *
 * Sweep direction is from edge to vertex
 */
struct Direction
{
  LineSegment baseEdge;
  geometry_msgs::Point opposedVertex;
};

/**
 * @brief Checks if given path is clockwise (the first turn made to the left) or not
 * @param path
 * @return bool True if path is clockwise
 * @detail the definition of "clockwise" is based on Fig.8 in Torres et al. 2016
 *
 */
inline bool isClockWise(const PointVector& path)
{
  return path.at(0).x < path.at(1).x ? true : false;
}

/**
 * @brief Calculates line sweep direction for given polygon
 * @param polygon Line sweep direction is calculated on this region
 * @return direction Struct containing edge and vertex
 */
Direction identifyOptimalSweepDir(const PointVector& polygon)
{
  Direction sweepDirection;

  PointVector convexHull = computeConvexHull(polygon);

  // Edges of polygon
  LineSegmentVector edges;

  // Make a list of edges of polygon
  for (std::size_t i = 0; i < convexHull.size(); ++i)
  {
    LineSegment ar;

    ar.at(0) = convexHull.at(i);

    // if vertex is the last one,
    // that vertex makes an edge whose end is the first vertex
    if (i == convexHull.size() - 1)
    {
      ar.at(1) = convexHull.at(0);
    }
    else
    {
      ar.at(1) = convexHull.at(i + 1);
    }
    edges.push_back(ar);
  }

  double optimalDistance = 0;

  // Calculate line sweep direction
  // Algorithm 1 in Torres et al, 2016
  for (const auto& edge : edges | boost::adaptors::indexed())
  {
    double edgeMaxDistance = 0;
    geometry_msgs::Point opposedVertex;

    for (const geometry_msgs::Point& vertex : convexHull)
    {
      // calculateDistance() function returns distance
      // between given edge and vertex
      double distance = calculateDistance(edge.value(), vertex);

      if (distance > edgeMaxDistance)
      {
        edgeMaxDistance = distance;
        opposedVertex = vertex;
      }
    }

    if ((edgeMaxDistance < optimalDistance) or edge.index() == 0)
    {
      optimalDistance = edgeMaxDistance;
      sweepDirection.baseEdge = edge.value();
      sweepDirection.opposedVertex = opposedVertex;
    }
  }

  return sweepDirection;
}

PointVector generateZigzagPath(const PointVector& path, double padding)
{
  PointVector zigzagPath;
  for (int i = 0; i < std::round(path.size() / 2); ++i)
  {
    if (i % 2 == 0)
    {
      try
      {
        if (path.at(2 * i).x < path.at(2 * i + 1).x)
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          p1.x += padding;
          p2.x -= padding;

          zigzagPath.push_back(p1);
          zigzagPath.push_back(p2);
        }
        else
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          p1.x -= padding;
          p2.x += padding;

          zigzagPath.push_back(p2);
          zigzagPath.push_back(p1);
        }
      }
      catch (const std::out_of_range& ex)
      {
        geometry_msgs::Point p = path.at(2 * i);
        if (isClockWise(path))
        {
          p.x += padding;
          zigzagPath.push_back(p);
        }
        else
        {
          p.x -= padding;
          zigzagPath.push_back(p);
        }
      }
    }
    else
    {
      try
      {
        if (path.at(2 * i).x > path.at(2 * i + 1).x)
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          p1.x -= padding;
          p2.x += padding;

          zigzagPath.push_back(p1);
          zigzagPath.push_back(p2);
        }
        else
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          p1.x += padding;
          p2.x -= padding;

          zigzagPath.push_back(p2);
          zigzagPath.push_back(p1);
        }
      }
      catch (const std::out_of_range& ex)
      {
        geometry_msgs::Point p = path.at(2 * i);
        if (isClockWise(path))
        {
          p.x -= padding;
          zigzagPath.push_back(p);
        }
        else
        {
          p.x += padding;
          zigzagPath.push_back(p);
        }
      }
    }
  }
  return zigzagPath;
}

/**
 * @brief Calculates coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           const Direction& sweepDirection, PointVector& path)
{
  if (polygon.size() < 3)
  {
    return false;
  }

  const double padding = 5.0;

  // rotate input polygon so that baseEdge become horizontal
  double rotationAngle = calculateHorizontalAngle(sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back());
  PointVector rotatedPolygon = rotatePoints(polygon, -rotationAngle);

  // find x coordinate of most left and most right point
  double minX(0), maxX(0);
  for (const auto& vertex : rotatedPolygon)
  {
    if (vertex.x < minX)
    {
      minX = vertex.x;
    }
    else if (vertex.x > maxX)
    {
      maxX = vertex.x;
    }
  }

  double stepWidth = footprintWidth * (1 - horizontalOverwrap);

  // calculate sweep direction of rotated polygon
  PointVector dir{ sweepDirection.opposedVertex, sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back() };
  dir = rotatePoints(dir, -rotationAngle);
  Direction rotatedDir;
  rotatedDir.opposedVertex = dir.at(0);
  rotatedDir.baseEdge.front() = dir.at(1);
  rotatedDir.baseEdge.back() = dir.at(2);

  int stepNum = std::ceil(calculateDistance(rotatedDir.baseEdge, rotatedDir.opposedVertex) / stepWidth);

  LineSegmentVector sweepLines;

  // generate list of sweep lines which is horizontal against the base edge
  for (int i = 0; i < stepNum; ++i)
  {
    LineSegment ar;
    geometry_msgs::Point p1, p2;
    p1.x = minX;
    p1.y = rotatedDir.baseEdge.at(0).y + (i * stepWidth) + padding;
    p2.x = maxX;
    p2.y = rotatedDir.baseEdge.at(1).y + (i * stepWidth) + padding;

    ar.at(0) = p1;
    ar.at(1) = p2;

    sweepLines.push_back(ar);
  }

  LineSegmentVector rotatedEdges = generateEdgeVector(rotatedPolygon);

  PointVector intersections;

  for (const auto& sweepLine : sweepLines)
  {
    int intersectionCount = 0;
    for (const auto& edge : rotatedEdges)
    {
      if (hasIntersection(sweepLine, edge))
      {
        intersections.push_back(localizeIntersection(edge, sweepLine));
        ++intersectionCount;
      }
      if (intersectionCount > 3)
      {
        return false;
      }
    }
  }

  std::stable_sort(intersections.begin(), intersections.end(),
                   [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return p1.y < p2.y; });

  PointVector rotatedPath = generateZigzagPath(intersections, padding);

  path = rotatePoints(rotatedPath, rotationAngle);

  if (hasIntersection(polygon, path) == true)
  {
    return false;
  }

  return true;
}

bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           PointVector& path)
{
  Direction sweepDirection = identifyOptimalSweepDir(polygon);
  return computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, sweepDirection, path);
}

/**
 * @brief Calculates length of path
 * @param path
 * @return double Length of path
 */
double calculatePathLength(const PointVector& path)
{
  if (path.size() < 2)
  {
    return 0;
  }

  double pathLength = 0;
  for (int i = 0; i < path.size() - 1; ++i)
  {
    pathLength += calculateDistance(path.at(i), path.at(i + 1));
  }
  return pathLength;
}

/**
 * @brief Return counter clock wise-ed path of given path
 * @param path
 * @return PointVector Counter clock wise version of given path
 */
PointVector computeCCWPath(PointVector path)
{
  for (int i = 0; i < std::round(path.size() / 2); ++i)
  {
    geometry_msgs::Point tmp = path.at(2 * i);

    path.at(2 * i) = path.at(2 * i + 1);
    try
    {
      path.at(2 * i + 1) = tmp;
    }
    catch (const std::out_of_range& ex)
    {
    }
  }
  return path;
}

/**
 * @brief Return opposite path of given path
 * @param path
 * @return PointVector Path with points of reversed order of given path
 */
PointVector computeOppositePath(const PointVector& path)
{
  PointVector oppositePath;

  for (int i = path.size() - 1; i >= 0; --i)
  {
    oppositePath.push_back(path.at(i));
  }

  return oppositePath;
}

/**
 * @brief Identify optimal path from 4 coverage alternatives
 * @param path
 * @param start Start point
 * @return PointVector Optimal path that minimizes the length of path
 * @detail The naming of the following variable follows torres et al. 2016
 */
PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const geometry_msgs::Point& start, const geometry_msgs::Point& end)
{
  // The naming of the following variable follows torres et al. 2016
  std::unordered_map<int, std::unordered_map<std::string, geometry_msgs::Point>> coverageAlternatives;
  std::unordered_map<std::string, geometry_msgs::Point> a1, a2, a3, a4;

  PointVector pathCW = isClockWise(path) ? path : computeCCWPath(path);
  PointVector pathCCW = isClockWise(path) ? computeCCWPath(path) : path;

  a1["SP"] = pathCW.front();
  a1["EP"] = pathCW.back();

  a2["SP"] = pathCCW.front();
  a2["EP"] = pathCCW.back();

  a3["SP"] = pathCW.back();
  a3["EP"] = pathCW.front();

  a4["SP"] = pathCCW.back();
  a4["EP"] = pathCCW.front();

  coverageAlternatives[1] = a1;
  coverageAlternatives[2] = a2;
  coverageAlternatives[3] = a3;
  coverageAlternatives[4] = a4;

  bool hasIntersectionCW = hasIntersection(polygon, pathCW);
  bool hasIntersectionCCW = hasIntersection(polygon, pathCCW);

  double minDistance;
  int optimalPath;
  for (const auto& coverage : coverageAlternatives | boost::adaptors::indexed())
  {
    if ((hasIntersectionCW and coverage.index() % 2 == 0) or (hasIntersectionCCW and coverage.index() % 2 != 0))
    {
      continue;
    }

    double distance = calculateDistance(coverage.value().second.at("SP"), start) +
                      calculateDistance(end, coverage.value().second.at("EP"));

    if (distance < minDistance or coverage.index() == 0)
    {
      minDistance = distance;
      optimalPath = coverage.value().first;
    }
  }

  switch (optimalPath)
  {
    case 1:
    {
      return pathCW;
    }
    case 2:
    {
      return pathCCW;
    }
    case 3:
    {
      return computeOppositePath(pathCW);
    }
    default:
    {
      return computeOppositePath(pathCCW);
    }
  }
}

PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const geometry_msgs::Point& start)
{
  return identifyOptimalAlternative(polygon, path, start, start);
}

bool findSecondOptimalPath(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           PointVector& path)
{
  std::vector<Direction> sweepDirections;
  PointVector convexHull = computeConvexHull(polygon);
  LineSegmentVector edges = generateEdgeVector(convexHull);

  for (const auto& edge : edges)
  {
    double maxDistance = 0;
    Direction direction;
    direction.baseEdge = edge;
    for (const auto& vertex : convexHull)
    {
      double distance = calculateDistance(edge, vertex);
      if (distance > maxDistance)
      {
        maxDistance = distance;
        direction.opposedVertex = vertex;
      }
    }
    sweepDirections.push_back(direction);
  }

  double pathLength = 0;
  for (const auto& sweepDirection : sweepDirections)
  {
    PointVector tempPath;
    if (computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, sweepDirection, tempPath) and
        (calculatePathLength(tempPath) < pathLength or pathLength == 0))
    {
      path = tempPath;
      pathLength = calculatePathLength(path);
    }
  }

  if (path.size() <= 1)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool isAdjacent(const PointVector& polygon1, const PointVector& polygon2)
{
  for (const auto& vertex1 : polygon1)
  {
    for (const auto& vertex2 : polygon2)
    {
      if (vertex1 == vertex2)
      {
        return true;
      }
    }
  }
  return false;
}

PointVector computeMultiplePolygonCoverage(std::vector<PointVector> subPolygons, double footprintWidth,
                                           double horizontalOverwrap, int adjacencyCriteria = 1)
{
  PointVector path;

  std::vector<int> permutation(subPolygons.size());
  std::iota(permutation.begin(), permutation.end(), 0);

  double minPathLength = -1;

  do
  {
    if (permutation.front() != 0)
    {
      continue;
    }

    int adjacencyCount = 0;
    for (auto itr = permutation.begin(); itr != permutation.end() - 1; ++itr)
    {
      if (isAdjacent(subPolygons.at(*itr), subPolygons.at(*(itr + 1))) == true)
      {
        ++adjacencyCount;
      }
    }
    if (adjacencyCount < adjacencyCriteria)
    {
      continue;
    }

    double pathLength = 0;
    std::vector<PointVector> candidatePath;

    for (auto itr = permutation.begin(); itr != permutation.end(); ++itr)
    {
      PointVector partPath, optimalAlternative;
      if (itr == permutation.begin())
      {
        geometry_msgs::Point start = subPolygons.at(*(permutation.end() - 1)).back();
        geometry_msgs::Point end = subPolygons.at(*(itr + 1)).front();
        PointVector polygon = subPolygons.at(*itr);
        computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, partPath);
        optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
        candidatePath.push_back(optimalAlternative);
        pathLength += calculatePathLength(optimalAlternative);
      }
      else if (itr == permutation.end() - 1)
      {
        geometry_msgs::Point start = subPolygons.at(*(itr - 1)).back();
        geometry_msgs::Point end = subPolygons.at(*permutation.begin()).front();
        PointVector polygon = subPolygons.at(*itr);
        computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, partPath);
        optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
        candidatePath.push_back(optimalAlternative);
        pathLength += calculatePathLength(optimalAlternative);
      }
      else
      {
        geometry_msgs::Point start = subPolygons.at(*(itr - 1)).back();
        geometry_msgs::Point end = subPolygons.at(*(itr + 1)).front();
        PointVector polygon = subPolygons.at(*itr);
        computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, partPath);
        optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
        candidatePath.push_back(optimalAlternative);
        pathLength += calculatePathLength(optimalAlternative);
      }
    }

    if (minPathLength < 0 or pathLength < minPathLength)
    {
      minPathLength = pathLength;
      for (const auto& part : candidatePath)
      {
        path.insert(path.begin(), part.begin(), part.end());
      }
    }

  } while (next_permutation(permutation.begin(), permutation.end()));

  return path;
}

#endif
