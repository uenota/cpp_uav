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
 * @brief Calculates line sweep direction for given polygon
 * @param polygon Line sweep direction is calculated on this region
 * @return direction Struct containing edge and vertex
 */
Direction identifySweepDir(const PointVector& polygon)
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

/**
 * @brief Calculates coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           PointVector& path)
{
  const double padding = 5.0;

  Direction dir = identifySweepDir(polygon);

  // rotate input polygon so that baseEdge become horizontal
  double rotationAngle = calculateHorizontalAngle(dir.baseEdge.front(), dir.baseEdge.back());
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
  Direction rotatedDir = identifySweepDir(rotatedPolygon);

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

  // generate list of edge of rotated polygon
  LineSegmentVector rotatedEdges;
  for (int i = 0; i < rotatedPolygon.size(); ++i)
  {
    LineSegment edge;

    edge.at(0) = rotatedPolygon.at(i);

    if (i < rotatedPolygon.size() - 1)
    {
      edge.at(1) = rotatedPolygon.at(i + 1);
    }
    else
    {
      edge.at(1) = rotatedPolygon.at(0);
    }

    rotatedEdges.push_back(edge);
  }

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

  PointVector waypts;
  for (int i = 0; i < std::round(intersections.size() / 2); ++i)
  {
    if (i % 2 == 0)
    {
      try
      {
        if (intersections.at(2 * i).x < intersections.at(2 * i + 1).x)
        {
          waypts.push_back(intersections.at(2 * i));
          waypts.push_back(intersections.at(2 * i + 1));
        }
        else
        {
          waypts.push_back(intersections.at(2 * i + 1));
          waypts.push_back(intersections.at(2 * i));
        }
      }
      catch (const std::out_of_range& ex)
      {
        waypts.push_back(intersections.at(2 * i));
      }
    }
    else
    {
      try
      {
        if (intersections.at(2 * i).x > intersections.at(2 * i + 1).x)
        {
          waypts.push_back(intersections.at(2 * i));
          waypts.push_back(intersections.at(2 * i + 1));
        }
        else
        {
          waypts.push_back(intersections.at(2 * i + 1));
          waypts.push_back(intersections.at(2 * i));
        }
      }
      catch (const std::out_of_range& ex)
      {
        waypts.push_back(intersections.at(2 * i));
      }
    }
  }

  path = rotatePoints(waypts, rotationAngle);

  return true;
}

/**
 * @brief Calculates length of path
 * @param path
 * @return double Length of path
 */
inline double calculatePathLength(const PointVector& path)
{
  double pathLength = 0;
  for (int i = 0; i < path.size() - 1; ++i)
  {
    pathLength += calculateDistance(path.at(i), path.at(i + 1));
  }
  return pathLength;
}

/**
 * @brief Checks if given path is clockwise (the first turn made to the left) or not
 * @param path
 * @return bool True if path is clockwise
 * @detail the definition of "clockwise" is based on Fig.8 in Torres et al. 2016
 *
 */
inline bool isClockWise(PointVector path)
{
  return path.at(0).x < path.at(1).x ? true : false;
}

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

PointVector computeOppositePath(const PointVector& path)
{
  PointVector oppositePath;

  for (int i = path.size() - 1; i >= 0; --i)
  {
    oppositePath.push_back(path.at(i));
  }

  return oppositePath;
}

PointVector identifyOptimalPath(const PointVector& path, const geometry_msgs::Point& start)
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

  double minDistance;
  int optimalPath;
  for (const auto& coverage : coverageAlternatives | boost::adaptors::indexed())
  {
    double distance = calculateDistance(coverage.value().second.at("SP"), start) +
                      calculateDistance(start, coverage.value().second.at("EP"));

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

#endif
