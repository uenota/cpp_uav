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
#include <vector>

// Boost
#include <boost/range/adaptor/indexed.hpp>

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

// compute coverage path for convex polygon
std::vector<geometry_msgs::Point> convexCoverage(const std::vector<geometry_msgs::Point>& polygon,
                                                 double footprint_width, double horizontal_overwrap)
{
  std::vector<geometry_msgs::Point> waypoints;

  Direction dir = sweepDirection(polygon);

  // Base edge vector and its norm
  std::array<double, 2> base_edge_vector;
  double base_edge_vector_x, base_edge_vector_y, base_edge_vector_norm;
  base_edge_vector_x = dir.base_edge.at(1).x - dir.base_edge.at(0).x;
  base_edge_vector_y = dir.base_edge.at(1).y - dir.base_edge.at(0).y;
  base_edge_vector_norm = std::sqrt(std::pow(base_edge_vector_x, 2) + std::pow(base_edge_vector_y, 2));

  // The vector whose origin is the same as base_edges' one
  // and the end is opposed_vertex in Direction
  std::array<double, 2> another_edge_vector;
  double another_edge_vector_x, another_edge_vector_y, another_edge_vector_norm;
  another_edge_vector_x = dir.opposed_vertex.x - dir.base_edge.at(0).x;
  another_edge_vector_y = dir.opposed_vertex.y - dir.base_edge.at(0).y;
  another_edge_vector_norm = std::sqrt(std::pow(another_edge_vector_x, 2) - std::pow(another_edge_vector_y, 2));

  // inner product
  double inner_product = base_edge_vector_x * another_edge_vector_x + base_edge_vector_y * another_edge_vector_y;

  // the distance from base_edge's origin to the point where the perpendicular line and base_edge intersect
  double distance_to_foot = inner_product / base_edge_vector_norm;

  // the vector from the origin of base_edge_vector to foot of orthogonal
  double orthogonal_vector_x, orthogonal_vector_y;
  orthogonal_vector_x = (distance_to_foot / base_edge_vector_norm) * base_edge_vector_x;
  orthogonal_vector_y = (distance_to_foot / base_edge_vector_norm) * base_edge_vector_y;

  // convert orthogonal vector's origion into the global origin
  orthogonal_vector_x += dir.base_edge.at(0).x;
  orthogonal_vector_y += dir.base_edge.at(0).y;

  geometry_msgs::Point orth_pt;
  orth_pt.x = orthogonal_vector_x;
  orth_pt.y = orthogonal_vector_y;

  o_pt.x = dir.opposed_vertex.x;
  o_pt.y = dir.opposed_vertex.y;

  std::array<geometry_msgs::Point, 2> sweep_direction_vec;
  sweep_direction_vec.at(0) = orth_pt;
  sweep_direction_vec.at(1) = dir.opposed_vertex;

  double sweep_direction_vec_norm = std::sqrt(std::pow(sweep_direction_vec.at(1).x - sweep_direction_vec.at(0).x, 2) +
                                              std::pow(sweep_direction_vec.at(1).y - sweep_direction_vec.at(0).y, 2));

  std::array<double, 2> sweep_dir_vec_normalized;
  sweep_dir_vec_normalized.at(0) =
      (sweep_direction_vec.at(1).x - sweep_direction_vec.at(0).x) / sweep_direction_vec_norm;
  sweep_dir_vec_normalized.at(1) =
      (sweep_direction_vec.at(1).y - sweep_direction_vec.at(0).y) / sweep_direction_vec_norm;

  double smallest_y(0), largest_y(0);
  for (const auto& vertex : polygon)
  {
    if (vertex.y < smallest_y)
      smallest_y = vertex.y;
    else if (vertex.y > largest.y)
      largest_y = vertex.y;
  }

  double smallest_x(0), largest_x(0);
  for (const auto& vertex : polygon)
  {
    if (vertex.x < smallest_x)
      smallest_x = vertex.x;
    else if (vertex.x > largest.x)
      largest_x = vertex.x;
  }

  double base_edge_a, base_edge_b;
  base_edge_a = (dir.base_edge.at(1).y - dir.base_edge.at(0).y) / (dir.base_edge.at(1).x - dir.base_edge.at(0).x);
  base_edge_b = dir.base_edge.at(1).y - base_edge_a * dir.base_edge.at(1).x;

  std::array<geometry_msgs::Point, 2> base_edge_extended;
  geometry_msgs::Point base_vertex_1, base_vertex_2;

  // ROS_INFO("%f", 180 * horizontalAngle(orth_pt, o_pt) / M_PI);
  // ROS_INFO("%f", 180 * horizontalAngle(dir.base_edge.at(1), dir.base_edge.at(0)) / M_PI);

  // ROS_INFO("%f",
  //         180 * std::abs(horizontalAngle(orth_pt, o_pt) - horizontalAngle(dir.base_edge.at(1),
  //         dir.base_edge.at(0)))
  //         /
  //             M_PI);

  return waypoints;
}

#endif
