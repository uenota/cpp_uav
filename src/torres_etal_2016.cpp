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
 * @param res[out] Contains resulting path
 * @return bool
 * @details For details of this service, cf. srv/Torres16.srv
 */
bool plan(cpp_uav::Torres16::Request& req, cpp_uav::Torres16::Response& res)
{
  // Initialization
  PointVector polygon, path;
  geometry_msgs::Point start;
  std_msgs::Float64 footprintLength, footprintWidth, horizontalOverwrap, verticalOverwrap;

  polygon = req.polygon;

  start = req.start;

  footprintLength = req.footprint_length;
  footprintWidth = req.footprint_width;
  horizontalOverwrap = req.horizontal_overwrap;
  verticalOverwrap = req.vertical_overwrap;

  bool isOptimal = computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, path);

  if (isOptimal == true)
  {
    res.path = identifyOptimalPath(path, start);
  }
  else
  {
    std::vector<PointVector> subPolygons = decomposePolygon(polygon);
    double pathLengthSum = 0;

    for (const auto& polygon : subPolygons)
    {
      PointVector partialPath;
      computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, partialPath);
      pathLengthSum += calculatePathLength(partialPath);
    }

    res.path = path;
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
