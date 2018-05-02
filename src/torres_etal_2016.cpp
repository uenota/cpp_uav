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
  // see torres et al. 2016 for the flow of this algorithm

  // polygon from request and path for response
  PointVector polygon, candidatePath;
  // start point of coverage path
  geometry_msgs::Point start;
  // parameters of coverage path
  std_msgs::Float64 footprintLength, footprintWidth, horizontalOverwrap, verticalOverwrap;

  polygon = req.polygon;
  start = req.start;

  footprintLength = req.footprint_length;
  footprintWidth = req.footprint_width;
  horizontalOverwrap = req.horizontal_overwrap;
  verticalOverwrap = req.vertical_overwrap;

  // isOptimal is true if computed path is optimal
  bool isOptimal = computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, candidatePath);

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

    // sum of length of all coverage path
    double pathLengthSum = 0;

    ROS_INFO("computing coverage of subpolygons");
    // compute length of coverage path of each subpolygon
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
    // existsSecondOptimalPath is true if there is at least one coverage that has no intersection with polygon
    // second optimal path is the path that has second shortest sweep direction without any intersection with polygon
    PointVector secondOptimalPath;
    ROS_INFO("finish searching second optimal path");
    bool existsSecondOptimalPath =
        findSecondOptimalPath(polygon, footprintWidth.data, horizontalOverwrap.data, candidatePath);

    if (existsSecondOptimalPath == true)
    {
      // compute optimal alternative for second optimal path
      secondOptimalPath = identifyOptimalAlternative(polygon, candidatePath, start);

      // if the length of second optimal path is shorter than the sum of coverage path of subpolygons,
      // set second optimal path as the path
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
      // if number of subpolygon is smaller than 2,
      // it means no valid path can be computed
      ROS_ERROR("Unable to generate path.");
      return true;
    }

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
    ROS_INFO("finish computing multiple polygon coverage");
    // compute coverage path of subpolygons
    PointVector multipleCoveragePath =
        computeMultiplePolygonCoverage(subPolygons, footprintWidth.data, horizontalOverwrap.data);

    res.path = multipleCoveragePath;
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
