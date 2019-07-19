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

/**
 * @brief Generate vector of polygon from PointVector
 * @param polygon
 * @return std::vector<geometry_msgs::Polygon> Vector of subpolygons assumed to be passed to ROS msg
 */
std::vector<geometry_msgs::Polygon> generatePolygonVector(const PointVector& polygon)
{
  // convert PointVector (a.k.a. std::vector<geometry_msgs::Point>) to geometry_msgs::Polygon
  // so that polygon is visualized on the window
  geometry_msgs::Polygon poly;
  for (const auto& vertex : polygon)
  {
    geometry_msgs::Point32 p;
    p.x = vertex.x;
    p.y = vertex.y;
    poly.points.push_back(p);
  }

  std::vector<geometry_msgs::Polygon> polygons = { poly };

  return polygons;
}

/**
 * @brief Generate vector of polygon from std::vector<PointVector>
 * @param subPolygons
 * @return std::vector<geometry_msgs::Polygon> Vector of subpolygons assumed to be passed to ROS msg
 */
std::vector<geometry_msgs::Polygon> generatePolygonVector(const std::vector<PointVector>& subPolygons)
{
  std::vector<geometry_msgs::Polygon> subPolygonsRet;

  // convert PointVector (a.k.a. std::vector<geometry_msgs::Point>) to geometry_msgs::Polygon
  // so that polygon is visualized on the window
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

  return subPolygonsRet;
}

/**
 * @brief Plans coverage path
 * @param req Contains values neccesary to plan a path
 * @param res Contains resulting path
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

  // isOptimal is true if computed path does not have intersection with polygon
  bool isOptimal = computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, candidatePath);

  if (isOptimal == true)
  {
    // fill "subpolygon" field of response so that polygon is visualized
    res.subpolygons = generatePolygonVector(polygon);

    // set optimal alternative as optimal path
    // see torres et al. 2016 for Optimal Alternative
    res.path = identifyOptimalAlternative(polygon, candidatePath, start);
  }
  else
  {
    std::vector<PointVector> subPolygons = decomposePolygon(polygon);

    // sum of length of all coverage path
    double pathLengthSum = 0;

    int i = 1;
    // compute length of coverage path of each subpolygon
    for (const auto& polygon : subPolygons)
    {
      PointVector partialPath;
      computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, partialPath);
      pathLengthSum += calculatePathLength(partialPath);
    }

    // existsSecondOptimalPath is true if there is at least one coverage that has no intersection with polygon
    // second optimal path is the path that has second shortest sweep direction and no intersection with polygon
    PointVector secondOptimalPath;
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
        // fill "subpolygon" field of response so that polygon is visualized
        res.subpolygons = generatePolygonVector(polygon);

        res.path = secondOptimalPath;

        return true;

      }
      // returns second optimal path when shortest path is not optimal and polygon cannot be decomposed
      else if(subPolygons.size() == 1)
      {
        res.subpolygons = generatePolygonVector(polygon);

        res.path = secondOptimalPath;

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

    // fill "subpolygon" field of response so that polygon is visualized
    res.subpolygons = generatePolygonVector(subPolygons);

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
