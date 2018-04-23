/**
 * @file torres_etal_2016.cpp
 * @brief Coverage path planner based on M. Torres et al, 2016
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

// header
#include <torres_etal_2016.hpp>

// cpp standard libraries
#include <array>
#include <vector>

// roscpp
#include <ros/ros.h>

// geometry_msgs
#include <geometry_msgs/Point.h>

// Service
#include "cpp_uav/Torres16.h"

/**
 * @brief Plans coverage path
 * @param req[in] Contains values neccesary to plan a path
 * @param res[out] Contains resulting waypoints
 * @return bool
 * @details For details of this service, cf. srv/Torres16.srv
 */
bool plan(cpp_uav::Torres16::Request& req, cpp_uav::Torres16::Response& res)
{
  // Initialization
  std::vector<geometry_msgs::Point> polygon_vertices, waypoints;
  geometry_msgs::Point start;
  std_msgs::Float64 footprint_length, footprint_width, horizontal_overwrap, vertical_overwrap;

  polygon_vertices = req.polygon_vertices;

  start = req.start;

  footprint_length = req.footprint_length;
  footprint_width = req.footprint_width;
  horizontal_overwrap = req.horizontal_overwrap;
  vertical_overwrap = req.vertical_overwrap;

  bool isOptimal = convexCoverage(polygon_vertices, footprint_width.data, horizontal_overwrap.data, waypoints);

  if (isOptimal == true)
  {
    res.waypoints = findOptimalPath(waypoints, start);
  }
  else
  {
    std::vector<std::vector<geometry_msgs::Point>> dec_poly = decomposePolygon(polygon_vertices);
    std::vector<geometry_msgs::Point> waypoints;

    for (const auto& polygon : dec_poly)
    {
      std::vector<geometry_msgs::Point> waypoints_part;
      convexCoverage(polygon, footprint_width.data, horizontal_overwrap.data, waypoints_part);

      for (const auto& waypoint : waypoints_part)
      {
        waypoints.push_back(waypoint);
      }
    }

    res.waypoints = waypoints;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "torres_etal_2016");
  ros::NodeHandle nh;

  ros::ServiceServer planner = nh.advertiseService("cpp_torres16", plan);
  ROS_INFO("Ready to plan.");

  ros::spin();

  return 0;
}
