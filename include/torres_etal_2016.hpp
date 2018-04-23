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

/**
 * @struct Direction
 * @brief Storage for line sweep direction
 *
 * Sweep direction is from edge to vertex
 */
struct Direction
{
  std::array<geometry_msgs::Point, 2> base_edge;
  geometry_msgs::Point opposed_vertex;
};

/**
 * @brief Calculates line sweep direction for given polygon
 * @param polygon Line sweep direction is calculated on this region
 * @return direction Struct containing edge and vertex
 */
Direction sweepDirection(const std::vector<geometry_msgs::Point>& polygon)
{
  Direction line_sweep;

  std::vector<geometry_msgs::Point> convex_hull = grahamScan(polygon);

  // Edges of polygon
  std::vector<std::array<geometry_msgs::Point, 2>> edges;

  // Make a list of edges of polygon
  for (std::size_t i = 0; i < convex_hull.size(); ++i)
  {
    std::array<geometry_msgs::Point, 2> ar;

    ar.at(0) = convex_hull.at(i);

    // if vertex is the last one,
    // that vertex makes an edge whose end is the first vertex
    if (i == convex_hull.size() - 1)
    {
      ar.at(1) = convex_hull.at(0);
    }
    else
    {
      ar.at(1) = convex_hull.at(i + 1);
    }
    edges.push_back(ar);
  }

  double optimal_dist = 0;

  // Calculate line sweep direction
  // Algorithm 1 in Torres et al, 2016
  for (const auto& edge : edges | boost::adaptors::indexed())
  {
    double max_dist_edge = 0;
    geometry_msgs::Point opposed_vertex;

    for (const geometry_msgs::Point& vertex : convex_hull)
    {
      // distance() function returns distance
      // between given edge and vertex
      double dist = distance(edge.value(), vertex);

      if (dist > max_dist_edge)
      {
        max_dist_edge = dist;
        opposed_vertex = vertex;
      }
    }

    if ((max_dist_edge < optimal_dist) or edge.index() == 0)
    {
      optimal_dist = max_dist_edge;
      line_sweep.base_edge = edge.value();
      line_sweep.opposed_vertex = opposed_vertex;
    }
  }

  return line_sweep;
}

/**
 * @brief Calculates coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprint_width Width of the area taken by one sweep
 * @param horizontal_overwrap Horizontal overwrap of each sweep
 * @param waypoints Waypoints of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool convexCoverage(const std::vector<geometry_msgs::Point>& polygon, double footprint_width,
                    double horizontal_overwrap, std::vector<geometry_msgs::Point>& waypoints)
{
  const double padding = 5.0;

  Direction dir = sweepDirection(polygon);

  // rotate input polygon so that base_edge become horizontal
  double rotationAngle = horizontalAngle(dir.base_edge.front(), dir.base_edge.back());
  std::vector<geometry_msgs::Point> rotatedPolygon = rotatePoints(polygon, -rotationAngle);

  // find x coordinate of most left and most right point
  double smallest_x(0), largest_x(0);
  for (const auto& vertex : rotatedPolygon)
  {
    if (vertex.x < smallest_x)
    {
      smallest_x = vertex.x;
    }
    else if (vertex.x > largest_x)
    {
      largest_x = vertex.x;
    }
  }

  double stepWidth = footprint_width * (1 - horizontal_overwrap);

  // calculate sweep direction of rotated polygon
  Direction rotatedDir = sweepDirection(rotatedPolygon);

  int stepNum = std::ceil(distance(rotatedDir.base_edge, rotatedDir.opposed_vertex) / stepWidth);

  std::vector<std::array<geometry_msgs::Point, 2>> sweepLines;

  // generate list of sweep lines which is horizontal against the base edge
  for (int i = 0; i < stepNum; ++i)
  {
    std::array<geometry_msgs::Point, 2> ar;
    geometry_msgs::Point p1, p2;
    p1.x = smallest_x;
    p1.y = rotatedDir.base_edge.at(0).y + (i * stepWidth) + padding;
    p2.x = largest_x;
    p2.y = rotatedDir.base_edge.at(1).y + (i * stepWidth) + padding;

    ar.at(0) = p1;
    ar.at(1) = p2;

    sweepLines.push_back(ar);
  }

  // generate list of edge of rotated polygon
  std::vector<std::array<geometry_msgs::Point, 2>> rotatedEdges;
  for (int i = 0; i < rotatedPolygon.size(); ++i)
  {
    std::array<geometry_msgs::Point, 2> edge;

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

  std::vector<geometry_msgs::Point> intersections;

  for (const auto& sweepLine : sweepLines)
  {
    int intersection_num = 0;
    for (const auto& edge : rotatedEdges)
    {
      if (intersect(sweepLine, edge))
      {
        intersections.push_back(localizeIntersection(edge, sweepLine));
        ++intersection_num;
      }
      if (intersection_num > 3)
      {
        return false;
      }
    }
  }

  std::stable_sort(intersections.begin(), intersections.end(),
                   [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return p1.y < p2.y; });

  std::vector<geometry_msgs::Point> waypts;
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

  waypoints = rotatePoints(waypts, rotationAngle);

  return true;
}

inline double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

inline bool isClockWise(std::vector<geometry_msgs::Point> waypoints)
{
  return waypoints.at(0).x < waypoints.at(1).x ? true : false;
}

std::vector<geometry_msgs::Point> counterClockWise(std::vector<geometry_msgs::Point> waypoints)
{
  for (int i = 0; i < std::round(waypoints.size() / 2); ++i)
  {
    geometry_msgs::Point tmp = waypoints.at(2 * i);

    waypoints.at(2 * i) = waypoints.at(2 * i + 1);
    try
    {
      waypoints.at(2 * i + 1) = tmp;
    }
    catch (const std::out_of_range& ex)
    {
    }
  }
  return waypoints;
}

std::vector<geometry_msgs::Point> oppositePath(const std::vector<geometry_msgs::Point>& waypoints)
{
  std::vector<geometry_msgs::Point> oppositeWaypoints;

  for (int i = waypoints.size() - 1; i >= 0; --i)
  {
    oppositeWaypoints.push_back(waypoints.at(i));
  }

  return oppositeWaypoints;
}

std::vector<geometry_msgs::Point> findOptimalPath(const std::vector<geometry_msgs::Point>& waypoints,
                                                  const geometry_msgs::Point& start)
{
  // The naming of the following variable follows torres et al. 2016
  std::unordered_map<int, std::unordered_map<std::string, geometry_msgs::Point>> coverageAlternatives;
  std::unordered_map<std::string, geometry_msgs::Point> a1, a2, a3, a4;

  std::vector<geometry_msgs::Point> waypointsCW = isClockWise(waypoints) ? waypoints : counterClockWise(waypoints);
  std::vector<geometry_msgs::Point> waypointsCCW = isClockWise(waypoints) ? counterClockWise(waypoints) : waypoints;

  a1["SP"] = waypointsCW.front();
  a1["EP"] = waypointsCW.back();

  a2["SP"] = waypointsCCW.front();
  a2["EP"] = waypointsCCW.back();

  a3["SP"] = waypointsCW.back();
  a3["EP"] = waypointsCW.front();

  a4["SP"] = waypointsCCW.back();
  a4["EP"] = waypointsCCW.front();

  coverageAlternatives[1] = a1;
  coverageAlternatives[2] = a2;
  coverageAlternatives[3] = a3;
  coverageAlternatives[4] = a4;

  double coverageDistance;
  int optimalPath;
  for (const auto& coverage : coverageAlternatives | boost::adaptors::indexed())
  {
    double dist = distance(coverage.value().second.at("SP"), start) + distance(start, coverage.value().second.at("EP"));

    if (dist < coverageDistance or coverage.index() == 0)
    {
      coverageDistance = dist;
      optimalPath = coverage.value().first;
    }
  }

  switch (optimalPath)
  {
    case 1:
    {
      return waypointsCW;
    }
    case 2:
    {
      return waypointsCCW;
    }
    case 3:
    {
      std::vector<geometry_msgs::Point> oppositeWaypointsCW = oppositePath(waypointsCW);
      return oppositeWaypointsCW;
    }
    default:
    {
      std::vector<geometry_msgs::Point> oppositeWaypointsCCW = oppositePath(waypointsCCW);
      return oppositeWaypointsCCW;
    }
  }
}

#endif
