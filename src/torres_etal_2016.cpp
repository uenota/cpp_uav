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
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

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
    geometry_msgs::Polygon poly;

    for (const auto& vertex : polygon)
    {
      geometry_msgs::Point32 p;
      p.x = vertex.x;
      p.y = vertex.y;
      poly.points.push_back(p);
    }

    std::vector<geometry_msgs::Polygon> polygons = { poly };
    res.subpolygons = polygons;
    res.path = identifyOptimalAlternative(polygon, path, start);
  }
  else
  {
    ROS_INFO("decomposing polygon");
    std::vector<PointVector> subPolygons = decomposePolygon(polygon);
    ROS_INFO("polygon decomposed");
    ROS_INFO("Number of Sub Polygon: %d", subPolygons.size());
    double pathLengthSum = 0;

    ROS_INFO("computing coverage of subpolygons");
    for (const auto& polygon : subPolygons)
    {
      PointVector partialPath;
      ROS_INFO("1");
      ROS_INFO("Num. of polygon vertices: %d", polygon.size());
      computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, partialPath);
      ROS_INFO("2");
      pathLengthSum += calculatePathLength(partialPath);
      ROS_INFO("3");
    }
    ROS_INFO("finish computing coverage of subpolygons");

    ROS_INFO("searching second optimal path");
    PointVector secondOptimalPath;
    bool existsSecondOptimalPath = findSecondOptimalPath(polygon, footprintWidth.data, horizontalOverwrap.data, path);
    ROS_INFO("finish searching second optimal path");
    if (existsSecondOptimalPath == true)
    {
      secondOptimalPath = identifyOptimalAlternative(polygon, path, start);
      if (pathLengthSum > calculatePathLength(secondOptimalPath))
      {
        geometry_msgs::Polygon poly;

        for (const auto& vertex : polygon)
        {
          geometry_msgs::Point32 p;
          p.x = vertex.x;
          p.y = vertex.y;
          poly.points.push_back(p);
        }

        std::vector<geometry_msgs::Polygon> polygons = { poly };
        res.subpolygons = polygons;

        res.path = secondOptimalPath;
        ROS_INFO("second optimal path is considered as optimal");
        return true;
      }
    }
    else if (subPolygons.size() < 2)
    {
      ROS_WARN("Unable to generate path.");
      return true;
    }

    /*
    PointVector srvRetSubPolygons;
    for (const auto& subPolygon : subPolygons)
    {
      srvRetSubPolygons.insert(srvRetSubPolygons.begin(), subPolygon.begin(), subPolygon.end());
    }
    res.subpolygons = srvRetSubPolygons;
    */
    std::vector<geometry_msgs::Polygon> subPolygonsRet;

    for (const auto& subPolygon : subPolygons)
    {
      geometry_msgs::Polygon poly;
      for (const auto& vertex : subPolygon)
      {
        geometry_msgs::Point32 pt;
        pt.x = vertex.x;
        pt.y = vertex.y;
        poly.points.push_back(pt);
      }
      subPolygonsRet.push_back(poly);
    }

    res.subpolygons = subPolygonsRet;

    ROS_INFO("computing multiple polygon coverage");
    PointVector gePath = computeMultiplePolygonCoverage(subPolygons, footprintWidth.data, horizontalOverwrap.data);
    ROS_INFO("finish computing multiple polygon coverage");

    res.path = gePath;
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
