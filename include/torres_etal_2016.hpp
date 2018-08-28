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
#include <string>
#include <unordered_map>

// Boost
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/reversed.hpp>

// std_msgs
#include <std_msgs/Float64.h>

// geometry_msgs
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

// Service
#include "cpp_uav/Torres16.h"

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

/**
 * @brief Reshape given path
 * @param path The sweep lines of path should be horizontal about x axis
 * @param padding
 * @return PointVector
 * @detail Reshape given path so that generated path becomes the sequence of "C" shapes and add padding
 */
PointVector reshapePath(const PointVector& path, double padding)
{
  PointVector zigzagPath;

  // reshape every traverse
  for (int i = 0; i < std::round(path.size() / 2); ++i)
  {
    // even-numbered traverse
    if (i % 2 == 0)
    {
      try
      {
        // in case that the first point of the traverse is located on LEFT side
        if (path.at(2 * i).x < path.at(2 * i + 1).x)
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);

          // add padding
          p1.x += padding;
          p2.x -= padding;

          // be careful with the order of points
          zigzagPath.push_back(p1);
          zigzagPath.push_back(p2);
        }
        // in case that the first point of the traverse is located on RIGHT side
        else
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);

          // add padding
          p1.x -= padding;
          p2.x += padding;

          // be careful with the order of points
          zigzagPath.push_back(p2);
          zigzagPath.push_back(p1);
        }
      }
      // in case that the traverse has only one vertex
      catch (std::out_of_range& ex)
      {
        geometry_msgs::Point p = path.at(2 * i);
        if (isClockWise(path))
        {
          // the first vertex of even-numbered traverse of clockwise path is located on RIGHT side of polygon
          p.x += padding;
          zigzagPath.push_back(p);
        }
        else
        {
          // the first vertex of even-numbered traverse of counterclockwise path is located on LEFT side of polygon
          p.x -= padding;
          zigzagPath.push_back(p);
        }
        ROS_ERROR("%s", ex.what());
      }
    }
    // odd-numbered traverse
    else
    {
      try
      {
        // in case that the first point of the traverse is located on RIGHT side
        if (path.at(2 * i).x > path.at(2 * i + 1).x)
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);

          // add padding
          p1.x -= padding;
          p2.x += padding;

          // be careful with the order of points
          zigzagPath.push_back(p1);
          zigzagPath.push_back(p2);
        }
        // in case that the first point of the traverse is located on LEFT side
        else
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);

          // add padding
          p1.x += padding;
          p2.x -= padding;

          // be careful with the order of points
          zigzagPath.push_back(p2);
          zigzagPath.push_back(p1);
        }
      }
      // in case that the traverse has only one vertex
      catch (std::out_of_range& ex)
      {
        geometry_msgs::Point p = path.at(2 * i);
        if (isClockWise(path))
        {
          // the first vertex of odd-numbered traverse of clockwise path is located on LEFT side of polygon
          p.x -= padding;
          zigzagPath.push_back(p);
        }
        else
        {
          // the first vertex of odd-numbered traverse of clockwise path is located on RIGHT side of polygon
          p.x += padding;
          zigzagPath.push_back(p);
        }
        ROS_ERROR("%s", ex.what());
      }
    }
  }
  return zigzagPath;
}

/**
 * @brief Compute coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param sweepDirection
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           const Direction& sweepDirection, PointVector& path)
{
  // Unable to make polygon with less than 3 points
  if (polygon.size() < 3)
  {
    return false;
  }

  // TODO: Change to configurable
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

  // Localize intersections of sweeplines and edges of rotated polygon
  LineSegmentVector rotatedEdges = generateEdgeVector(rotatedPolygon, true);

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

      // sweep line in optimal path does not have more than 2 intersections
      if (intersectionCount >= 3)
      {
        return false;
      }
    }
  }

  // sort points by y coordinate in ascending order
  std::stable_sort(intersections.begin(), intersections.end(),
                   [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return p1.y < p2.y; });

  PointVector rotatedPath = reshapePath(intersections, padding);

  path = rotatePoints(rotatedPath, rotationAngle);

  if (hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(path, false)) == true)
  {
    return false;
  }

  return true;
}

/**
 * @brief Compute coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
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
 * @param path Clockwise path
 * @return PointVector Counter clock wise version of given path
 */
PointVector computeCCWPath(PointVector path)
{
  for (int i = 0; i < std::round(path.size() / 2); ++i)
  {
    // swap the first point and the last point in each sweep line
    geometry_msgs::Point tmp = path.at(2 * i);

    path.at(2 * i) = path.at(2 * i + 1);
    try
    {
      path.at(2 * i + 1) = tmp;
    }
    catch (std::out_of_range& ex)
    {
      ROS_ERROR("%s", ex.what());
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

  // inversely iterate given points
  for (int i = path.size() - 1; i >= 0; --i)
  {
    oppositePath.push_back(path.at(i));
  }

  return oppositePath;
}

/**
 * @brief Identify optimal path from 4 coverage alternatives
 * @param polygon
 * @param path
 * @param start Start point
 * @param end End point
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

  // a1: clockwise current path
  a1["SP"] = pathCW.front();
  a1["EP"] = pathCW.back();

  // a2: counterclockwise current path
  a2["SP"] = pathCCW.front();
  a2["EP"] = pathCCW.back();

  // a3: clockwise opposite path
  a3["SP"] = pathCW.back();
  a3["EP"] = pathCW.front();

  // a4: counterclockwise opposite path
  a4["SP"] = pathCCW.back();
  a4["EP"] = pathCCW.front();

  coverageAlternatives[1] = a1;
  coverageAlternatives[2] = a2;
  coverageAlternatives[3] = a3;
  coverageAlternatives[4] = a4;

  bool hasIntersectionCW = hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(pathCW, false));
  bool hasIntersectionCCW = hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(pathCCW, false));

  double minDistance;
  int optimalPath;

  // check which coverage alternative has the shortest path
  for (const auto& coverage : coverageAlternatives | boost::adaptors::indexed())
  {
    // skip calculating length if the path has intersections
    if ((hasIntersectionCW and coverage.index() % 2 == 0) or (hasIntersectionCCW and coverage.index() % 2 != 0))
    {
      continue;
    }

    // only length of transition need to be considered because the length of coverage is almost same
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

/**
 * @brief Identify optimal path from 4 coverage alternatives
 * @param polygon
 * @param path
 * @param start Start point
 * @return PointVector Optimal path that minimizes the length of path
 * @detail The naming of the following variable follows torres et al. 2016
 */
PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const geometry_msgs::Point& start)
{
  return identifyOptimalAlternative(polygon, path, start, start);
}

/**
 * @brief Find second optimal path
 * @param polygon
 * @param footprintWidth
 * @param horizontalOverwrap
 * @param path
 * @return bool True if second optimal path exists
 */
bool findSecondOptimalPath(const PointVector& polygon, double footprintWidth, double horizontalOverwrap,
                           PointVector& path)
{
  std::vector<Direction> sweepDirections;
  PointVector convexHull = computeConvexHull(polygon);
  LineSegmentVector edges = generateEdgeVector(convexHull, true);

  // compute optimal sweep directions for each edge
  for (const auto& edge : edges)
  {
    double maxDistance = 0;
    Direction direction;
    direction.baseEdge = edge;
    for (const auto& vertex : convexHull)
    {
      double distance = calculateDistance(edge, vertex);

      // optimal sweep direction for a edge is the direction with the largest distance
      if (distance > maxDistance)
      {
        maxDistance = distance;
        direction.opposedVertex = vertex;
      }
    }
    sweepDirections.push_back(direction);
  }

  // compute second optimal path which has the shortest coverage path
  double pathLength = 0;
  PointVector tempPath;
  for (const auto& sweepDirection : sweepDirections)
  {
    PointVector p;

    // isValidPath is true if computed coverage does not have intersection
    bool isValidPath = computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, sweepDirection, p);

    // second optimal path is the shortest path without intersection
    if (isValidPath and ((calculatePathLength(tempPath) < pathLength) or (pathLength == 0)))
    {
      tempPath = p;
      pathLength = calculatePathLength(tempPath);
    }
  }

  if (tempPath.size() <= 1)
  {
    return false;
  }
  else
  {
    path = tempPath;
    return true;
  }
}

/**
 * @brief Check if given two polygons are adjacent
 * @param polygon1
 * @param polygon2
 * @return True if given two polygons are adjacent
 */
bool isAdjacent(const PointVector& polygon1, const PointVector& polygon2)
{
  for (const auto& vertex1 : polygon1)
  {
    for (const auto& vertex2 : polygon2)
    {
      // consider that two polygons are adjacent if they have at least one point in common
      if (vertex1 == vertex2)
      {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Compute coverage path for multiple convex polygons
 * @param subPolygons
 * @param footprintWidth
 * @param horizontalOverwrap
 * @param adjacencyCriteria Ignore paths which have less adjacent polygons than this number
 * @return PointVector Computed Path
 * @detail See section 6.2 of torres et al. 2016 for the detail
 */
PointVector computeMultiplePolygonCoverage(std::vector<PointVector> subPolygons, double footprintWidth,
                                           double horizontalOverwrap, int adjacencyCriteria = 1)
{
  PointVector path;

  std::vector<int> permutation(subPolygons.size());
  std::iota(permutation.begin(), permutation.end(), 0);

  double minPathLength = -1;

  do
  {
    // ignore permutations which do not start from the same polygon as the first one of given subpolygons
    if (permutation.front() != 0)
    {
      continue;
    }

    // count adjacent polygons
    int adjacencyCount = 0;
    for (auto itr = permutation.begin(); itr != permutation.end() - 1; ++itr)
    {
      if (isAdjacent(subPolygons.at(*itr), subPolygons.at(*(itr + 1))) == true)
      {
        ++adjacencyCount;
      }
    }

    // ignore if enough number of polygons do not exist
    if (adjacencyCount < adjacencyCriteria)
    {
      continue;
    }

    double pathLength = 0;
    std::vector<PointVector> candidatePath;

    for (auto itr = permutation.begin(); itr != permutation.end(); ++itr)
    {
      PointVector partPath, optimalAlternative;
      // first polygon of given subpolygon
      if (itr == permutation.begin())
      {
        try
        {
          // start point and end point of first subpolygon are ...
          // start point: the last point of coverage of the last subpolygon
          // end point  : the first point of coverage of the second subpolygon
          geometry_msgs::Point start = subPolygons.at(*(permutation.end() - 1)).back();
          geometry_msgs::Point end;
          PointVector polygon = subPolygons.at(*itr);
          if (permutation.size() == 1)
          {
            // end point is the same as start point if subPolygons.at(*(itr+1)) is out of range
            end = start;
          }
          else
          {
            end = subPolygons.at(*(itr + 1)).front();
          }

          // break if computed path has intersections
          if (computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, partPath) == false)
          {
            break;
          }

          // set optimal alternative as candidate
          optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
          candidatePath.push_back(optimalAlternative);

          // calculate the length of candidate path
          pathLength += calculatePathLength(optimalAlternative);
        }
        catch (std::out_of_range& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      // the last polygon of the given subpolygons
      else if (itr == permutation.end() - 1)
      {
        try
        {
          // start point and end point of the last subpolygon are ...
          // start point: the last point of coverage of the previous subpolygon
          // end point  : the first point of coverage of the first subpolygon
          geometry_msgs::Point start = subPolygons.at(*(itr - 1)).back();
          geometry_msgs::Point end = subPolygons.at(*permutation.begin()).front();
          PointVector polygon = subPolygons.at(*itr);

          // break if computed path has intersections
          if (computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, partPath) == false)
          {
            break;
          }

          // set optimal alternative as candidate
          optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
          candidatePath.push_back(optimalAlternative);

          // calculate the length of candidate path
          pathLength += calculatePathLength(optimalAlternative);
        }
        catch (std::out_of_range& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      // middle polygons
      else
      {
        try
        {
          // start point and end point of middle subpolygons are ...
          // start point: the last point of coverage of the previous subpolygon
          // end point  : the first point of coverage of the next subpolygon
          geometry_msgs::Point start = subPolygons.at(*(itr - 1)).back();
          geometry_msgs::Point end = subPolygons.at(*(itr + 1)).front();
          PointVector polygon = subPolygons.at(*itr);

          // break if computed path has intersections
          if (computeConvexCoverage(polygon, footprintWidth, horizontalOverwrap, partPath) == false)
          {
            break;
          }

          // set optimal alternative as candidate
          optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
          candidatePath.push_back(optimalAlternative);

          // calculate the length of candidate path
          pathLength += calculatePathLength(optimalAlternative);
        }
        catch (std::out_of_range& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
    }

    if (minPathLength < 0 or pathLength < minPathLength)
    {
      minPathLength = pathLength;

      if (not path.empty())
      {
        path.clear();
      }

      // insert coverages of subpolygons
      for (const auto& part : candidatePath)
      {
        path.insert(path.begin(), part.begin(), part.end());
      }
    }

  } while (next_permutation(permutation.begin(), permutation.end()));

  return path;
}

#endif
