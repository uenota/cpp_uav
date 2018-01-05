/**
 * @file test_cgutil.cpp
 * @brief Test program for cgutil.hpp
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

#include <cgutil.hpp>

// gtest
#include <gtest/gtest.h>

// c++ libraries
#include <array>
#include <cmath>
#include <vector>

// geometry_msgs
#include <geometry_msgs/Point.h>

TEST(CGUtilTest, IntersectTest1)
{
  geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
  std::array<geometry_msgs::Point, 2> e1, e2, e3, e4;
  std::vector<std::array<geometry_msgs::Point, 2> > segments;

  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 5.0;
  p2.y = 1.0;
  p3.x = 2.0;
  p3.y = 5.0;
  p4.x = 6.0;
  p4.y = 3.0;
  p5.x = 7.0;
  p5.y = 1.0;
  p6.x = 8.0;
  p6.y = 5.0;
  p7.x = 3.0;
  p7.y = 3.0;
  p8.x = 5.0;
  p8.y = 5.0;

  e1.at(0) = p1;
  e1.at(1) = p2;
  e2.at(0) = p3;
  e2.at(1) = p4;
  e3.at(0) = p5;
  e3.at(1) = p6;
  e4.at(0) = p7;
  e4.at(1) = p8;

  segments.push_back(e1);
  segments.push_back(e2);
  segments.push_back(e3);
  segments.push_back(e4);

  std::vector<std::array<std::array<geometry_msgs::Point, 2>, 2> > intersecting_segments = intersect(segments);
  EXPECT_EQ(1, intersecting_segments.size());
  EXPECT_DOUBLE_EQ(p3.x, intersecting_segments.at(0).at(0).at(0).x);
  EXPECT_DOUBLE_EQ(p3.y, intersecting_segments.at(0).at(0).at(0).y);
  EXPECT_DOUBLE_EQ(p4.x, intersecting_segments.at(0).at(0).at(1).x);
  EXPECT_DOUBLE_EQ(p4.y, intersecting_segments.at(0).at(0).at(1).y);
  EXPECT_DOUBLE_EQ(p7.x, intersecting_segments.at(0).at(1).at(0).x);
  EXPECT_DOUBLE_EQ(p7.y, intersecting_segments.at(0).at(1).at(0).y);
  EXPECT_DOUBLE_EQ(p8.x, intersecting_segments.at(0).at(1).at(1).x);
  EXPECT_DOUBLE_EQ(p8.y, intersecting_segments.at(0).at(1).at(1).y);
}

TEST(CGUtilTest, IntersectTest2)
{
  geometry_msgs::Point p1, p2, p3, p4;
  std::array<geometry_msgs::Point, 2> e1, e2;

  // TestCase1
  // p1: (0, 0)
  // p2: (3, 2)
  // p3: (2, 0)
  // p4: (0, 2)
  // Expected value: true
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 3.0;
  p2.y = 2.0;
  p3.x = 2.0;
  p3.y = 0.0;
  p4.x = 0.0;
  p4.y = 2.0;

  e1.at(0) = p1;
  e1.at(1) = p2;
  e2.at(0) = p3;
  e2.at(1) = p4;

  EXPECT_TRUE(intersect(e1, e2));

  // TestCase2
  // p1: (0, 0)
  // p2: (4, 2)
  // p3: (2, 1)
  // p4: (0, 2)
  // Expected value: true
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 4.0;
  p2.y = 2.0;
  p3.x = 2.0;
  p3.y = 1.0;
  p4.x = 0.0;
  p4.y = 2.0;

  e1.at(0) = p1;
  e1.at(1) = p2;
  e2.at(0) = p3;
  e2.at(1) = p4;

  EXPECT_TRUE(intersect(e1, e2));

  // TestCase3
  // p1: (0, 0)
  // p2: (4, 2)
  // p3: (1, 1)
  // p4: (0, 2)
  // Expected value: false
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 4.0;
  p2.y = 2.0;
  p3.x = 1.0;
  p3.y = 1.0;
  p4.x = 0.0;
  p4.y = 2.0;

  e1.at(0) = p1;
  e1.at(1) = p2;
  e2.at(0) = p3;
  e2.at(1) = p4;

  EXPECT_FALSE(intersect(e1, e2));
}

TEST(CGUtilTest, InBetweenTest)
{
  geometry_msgs::Point p1, p2, p3;

  // TestCase1
  // p1: (0, 0)
  // p2: (2, 2)
  // p3: (1, 1)
  // Expected value: true
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 2.0;
  p2.y = 2.0;
  p3.x = 1.0;
  p3.y = 1.0;
  EXPECT_TRUE(inBetween(p1, p2, p3));

  // TestCase2
  // p1: (0, 0)
  // p2: (2, 2)
  // p3: (1, 0)
  // Expected value: false
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 2.0;
  p2.y = 2.0;
  p3.x = 1.0;
  p3.y = 0.0;
  EXPECT_FALSE(inBetween(p1, p2, p3));

  // TestCase3
  // p1: (0, 0)
  // p2: (0, 0)
  // p3: (0, 0)
  // Expected value: true
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 0.0;
  p2.y = 0.0;
  p3.x = 0.0;
  p3.y = 0.0;
  EXPECT_TRUE(inBetween(p1, p2, p3));
}

/**
 * @brief Test for distance()
 */
TEST(CGUtilTest, CalcDistanceTest)
{
  // TestCase1
  // coordinate of edge's start point: (0.0, 0.0)
  // coordinate of edge's end point: (1.0, 0.0)
  // coordinate of opposite vertex: (0.5, 0.5)
  // expected distance: 0.5
  std::array<geometry_msgs::Point, 2> edge;
  geometry_msgs::Point e1, e2, vertex;

  e1.x = 0.0;
  e1.y = 0.0;
  e2.x = 1.0;
  e2.y = 0.0;
  edge.front() = e1;
  edge.back() = e2;

  vertex.x = 0.5;
  vertex.y = 0.5;

  EXPECT_NEAR(0.5, distance(edge, vertex), 1.0e-5);

  // TestCase2
  // coordinate of edge's start point: (0.0, 0.0)
  // coordinate of edge's end point: (0.0, sqrt(3.0)/2)
  // coordinate of opposite vertex: (0.0, 0.5)
  // expected distance: 0.5
  e1.x = 0.0;
  e1.y = 0.0;
  e2.x = 0.866;
  e2.y = 0.0;
  edge.front() = e1;
  edge.back() = e2;

  vertex.x = 0.0;
  vertex.y = 0.5;

  EXPECT_NEAR(0.5, distance(edge, vertex), 1.0e-5);

  // TestCase3
  // coordinate of edge's start point: (0.0, 0.0)
  // coordinate of edge's end point: (1.0, 0.0)
  // coordinate of opposite vertex: (-1.0, 1.0)
  // expected distance: 1.0
  e1.x = 0.0;
  e1.y = 0.0;
  e2.x = 1.0;
  e2.y = 0.0;
  edge.front() = e1;
  edge.back() = e2;

  vertex.x = -1.0;
  vertex.y = 1.0;

  EXPECT_NEAR(1.0, distance(edge, vertex), 1.0e-5);

  // TestCase4
  // coordinate of edge's start point: (0.0, 0.0)
  // coordinate of edge's end point: (1.0, 0.0)
  // coordinate of opposite vertex: (1.5, 0.5)
  // expected distance: 0.5
  e1.x = 0.0;
  e1.y = 0.0;
  e2.x = 1.0;
  e2.y = 0.0;
  edge.front() = e1;
  edge.back() = e2;

  vertex.x = 1.5;
  vertex.y = 0.5;

  EXPECT_NEAR(0.5, distance(edge, vertex), 1.0e-5);

  // TestCase5
  // coordinate of edge's start point: (0.0, 0.0)
  // coordinate of edge's end point: (1.0, -1/2)
  // coordinate of opposite vertex: (2/3, 1/3)
  // expected distance: 4*sqrt(5)/15
  e1.x = 0.0;
  e1.y = 0.0;
  e2.x = 1.0;
  e2.y = -0.5;
  edge.front() = e1;
  edge.back() = e2;

  vertex.x = 2.0 / 3.0;
  vertex.y = 1.0 / 3.0;

  EXPECT_NEAR(0.596284794, distance(edge, vertex), 1.0e-5);

  // TestCase6
  // coordinate of edge's start point: (1.5, 1.0)
  // coordinate of edge's end point: (0.0, 2.5)
  // coordinate of opposite vertex: (0.5, 1.5)
  // expected distance: sqrt(2)/4
  e1.x = 1.5;
  e1.y = 1.0;
  e2.x = 0;
  e2.y = 2.5;
  edge.front() = e1;
  edge.back() = e2;

  vertex.x = 0.5;
  vertex.y = 1.5;

  EXPECT_NEAR(0.3535533906, distance(edge, vertex), 1.0e-5);

  // TestCase7
  // coordinate of edge's start point: (6.0, 1.0)
  // coordinate of edge's end point: (0.5, 1.0)
  // coordinate of opposite vertex: (1.5, 2.5)
  // expected distance: 3/2
  e1.x = 6.0;
  e1.y = 1.0;
  e2.x = 0.5;
  e2.y = 1.0;
  edge.front() = e1;
  edge.back() = e2;

  vertex.x = 1.5;
  vertex.y = 2.5;

  EXPECT_NEAR(1.5, distance(edge, vertex), 1.0e-5);
}

/**
 * @brief Test for vertexAngle()
 */
TEST(CGUtilTest, CalcVertexAngleTest)
{
  geometry_msgs::Point p1, p2, p3;

  // TestCase1
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (1, 1)
  // Expected angle[rad]: pi/4
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = 1.0;
  p3.y = 1.0;
  EXPECT_DOUBLE_EQ(M_PI_4, vertexAngle(p1, p2, p3));

  // TestCase2
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (0, 0)
  // Expected angle[rad]: 0
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = 0.0;
  p3.y = 0.0;
  EXPECT_DOUBLE_EQ(0.0, vertexAngle(p1, p2, p3));

  // TestCase3
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (0, 1)
  // Expected angle[rad]: pi/2
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = 0.0;
  p3.y = 1.0;
  EXPECT_DOUBLE_EQ(M_PI_2, vertexAngle(p1, p2, p3));

  // TestCase4
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (-1, 1)
  // Expected angle[rad]: 3*pi/4
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = -1.0;
  p3.y = 1.0;
  EXPECT_DOUBLE_EQ(3 * M_PI_4, vertexAngle(p1, p2, p3));

  // TestCase5
  // p1: (0, 0)
  // p2: (-1, 0)
  // p3: (0.5, sqrt(3)/2)
  // Expected angle[rad]: 2*pi/3
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = -1.0;
  p2.y = 0.0;
  p3.x = 0.5;
  p3.y = std::sqrt(3.0) / 2.0;
  EXPECT_DOUBLE_EQ(2 * M_PI / 3.0, vertexAngle(p1, p2, p3));

  // TestCase6
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (-sqrt(3)/2, -0.5)
  // Expected angle[rad]: 2*pi/3
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = -std::sqrt(3.0) / 2.0;
  p3.y = -0.5;
  EXPECT_DOUBLE_EQ(5 * M_PI / 6.0, vertexAngle(p1, p2, p3));

  // TestCase7
  // p1: (0, 0)
  // p2: (0, 0)
  // p3: (0, 0)
  // Expected angle[rad]: 0.0
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 0.0;
  p2.y = 0.0;
  p3.x = 0.0;
  p3.y = 0.0;
  EXPECT_DOUBLE_EQ(0.0, vertexAngle(p1, p2, p3));
}

/**
 * @brief Test for horizontalAngle()
 */
TEST(CGUtilTest, CalcHorizontalAngleTest)
{
  geometry_msgs::Point p1, p2;

  // TestCase1
  // p1: (0, 0)
  // p2: (1, 0)
  // Expected angle[rad]: 0
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  EXPECT_DOUBLE_EQ(0, horizontalAngle(p1, p2));

  // TestCase2
  // p1: (0, 0)
  // p2: (1, 1)
  // Expected angle[rad]: pi/4
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 1.0;
  EXPECT_DOUBLE_EQ(M_PI_4, horizontalAngle(p1, p2));

  // TestCase3
  // p1: (0, 0)
  // p2: (-0.5, sqrt(3)/2)
  // Expected angle[rad]: 3*pi/4
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = -0.5;
  p2.y = std::sqrt(3) / 2.0;
  EXPECT_DOUBLE_EQ(2 * M_PI / 3, horizontalAngle(p1, p2));

  // TestCase4
  // p1: (0, 0)
  // p2: (0, -1)
  // Expected angle[rad]: pi/2
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 0.0;
  p2.y = -1.0;
  EXPECT_DOUBLE_EQ(M_PI_2, horizontalAngle(p1, p2));

  // TestCase5
  // p1: (0, 0)
  // p2: (0, 0)
  // Expected angle[rad]: 0.0
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 0.0;
  p2.y = 0.0;
  EXPECT_DOUBLE_EQ(0.0, horizontalAngle(p1, p2));
}

/**
 * @brief Test for signedArea()
 */
TEST(CGUtilTest, CalcsignedAreaTest)
{
  geometry_msgs::Point p1, p2, p3;

  // TestCase1
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (0, 1)
  // Expected value: 1
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = 0.0;
  p3.y = 1.0;
  EXPECT_DOUBLE_EQ(1.0, signedArea(p1, p2, p3));

  // TestCase2
  // p1: (0, 0)
  // p2: (-1, 0)
  // p3: (0, 1)
  // Expected value: -1
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = -1.0;
  p2.y = 0.0;
  p3.x = 0.0;
  p3.y = 1.0;
  EXPECT_DOUBLE_EQ(-1.0, signedArea(p1, p2, p3));

  // TestCase3
  // p1: (-1, 0)
  // p2: (1, 1)
  // p3: (2, -1)
  // Expected value: -5
  p1.x = -1.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 1.0;
  p3.x = 2.0;
  p3.y = -1.0;
  EXPECT_DOUBLE_EQ(-5.0, signedArea(p1, p2, p3));

  // TestCase3
  // p1: (-1, 0)
  // p2: (1, 1)
  // p3: (1, 1)
  // Expected value: 0
  p1.x = -1.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 1.0;
  p3.x = 1.0;
  p3.y = 1.0;
  EXPECT_DOUBLE_EQ(0.0, signedArea(p1, p2, p3));
}

/**
 * @brief Test for grahamScan()
 */
TEST(CGUtilTest, GrahamScanTest)
{
  geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7;
  std::vector<geometry_msgs::Point> points, convex_hull;

  // TestCase1
  // p1: (0, 0)
  // p2: (4, 1)
  // p3: (6, 4)
  // p4: (3, 5)
  // p5: (0, 3)
  // Expected points in convex hull: (p1, p2, p3, p4, p5)
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 4.0;
  p2.y = 1.0;
  p3.x = 6.0;
  p3.y = 4.0;
  p4.x = 3.0;
  p4.y = 5.0;
  p5.x = 0.0;
  p5.y = 3.0;

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);

  convex_hull = grahamScan(points);

  EXPECT_DOUBLE_EQ(p1.x, convex_hull.at(0).x);
  EXPECT_DOUBLE_EQ(p1.y, convex_hull.at(0).y);
  EXPECT_DOUBLE_EQ(p2.x, convex_hull.at(1).x);
  EXPECT_DOUBLE_EQ(p2.y, convex_hull.at(1).y);
  EXPECT_DOUBLE_EQ(p3.x, convex_hull.at(2).x);
  EXPECT_DOUBLE_EQ(p3.y, convex_hull.at(2).y);
  EXPECT_DOUBLE_EQ(p4.x, convex_hull.at(3).x);
  EXPECT_DOUBLE_EQ(p4.y, convex_hull.at(3).y);
  EXPECT_DOUBLE_EQ(p5.x, convex_hull.at(4).x);
  EXPECT_DOUBLE_EQ(p5.y, convex_hull.at(4).y);

  points.clear();

  // TestCase2
  // p1: (0, 0)
  // p2: (0, 3)
  // p3: (3, 3)
  // p4: (5, 4)
  // p5: (3, 0)
  // p6: (2, 5)
  // Expected points in convex hull: (p1, p5, p4, p6, p2)
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 0.0;
  p2.y = 3.0;
  p3.x = 3.0;
  p3.y = 3.0;
  p4.x = 5.0;
  p4.y = 4.0;
  p5.x = 3.0;
  p5.y = 0.0;
  p6.x = 2.0;
  p6.y = 5.0;

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);

  convex_hull = grahamScan(points);

  EXPECT_DOUBLE_EQ(p1.x, convex_hull.at(0).x);
  EXPECT_DOUBLE_EQ(p1.y, convex_hull.at(0).y);
  EXPECT_DOUBLE_EQ(p5.x, convex_hull.at(1).x);
  EXPECT_DOUBLE_EQ(p5.y, convex_hull.at(1).y);
  EXPECT_DOUBLE_EQ(p4.x, convex_hull.at(2).x);
  EXPECT_DOUBLE_EQ(p4.y, convex_hull.at(2).y);
  EXPECT_DOUBLE_EQ(p6.x, convex_hull.at(3).x);
  EXPECT_DOUBLE_EQ(p6.y, convex_hull.at(3).y);
  EXPECT_DOUBLE_EQ(p2.x, convex_hull.at(4).x);
  EXPECT_DOUBLE_EQ(p2.y, convex_hull.at(4).y);

  points.clear();

  // TestCase3
  // p1: (0, 0)
  // p2: (5, 1)
  // p3: (5, 4)
  // p4: (3, 2)
  // p5: (1, 3)
  // p6: (2, 5)
  // p7: (0, 4)
  // Expected points in convex hull: (p1, p2, p3, p6, p7)
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 5.0;
  p2.y = 1.0;
  p3.x = 5.0;
  p3.y = 4.0;
  p4.x = 3.0;
  p4.y = 2.0;
  p5.x = 1.0;
  p5.y = 3.0;
  p6.x = 2.0;
  p6.y = 5.0;
  p7.x = 0.0;
  p7.y = 4.0;

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);
  points.push_back(p7);

  convex_hull = grahamScan(points);

  EXPECT_DOUBLE_EQ(p1.x, convex_hull.at(0).x);
  EXPECT_DOUBLE_EQ(p1.y, convex_hull.at(0).y);
  EXPECT_DOUBLE_EQ(p2.x, convex_hull.at(1).x);
  EXPECT_DOUBLE_EQ(p2.y, convex_hull.at(1).y);
  EXPECT_DOUBLE_EQ(p3.x, convex_hull.at(2).x);
  EXPECT_DOUBLE_EQ(p3.y, convex_hull.at(2).y);
  EXPECT_DOUBLE_EQ(p6.x, convex_hull.at(3).x);
  EXPECT_DOUBLE_EQ(p6.y, convex_hull.at(3).y);
  EXPECT_DOUBLE_EQ(p7.x, convex_hull.at(4).x);
  EXPECT_DOUBLE_EQ(p7.y, convex_hull.at(4).y);

  points.clear();

  // TestCase4
  // no points
  // Expected points in convex hull: ()
  convex_hull = grahamScan(points);

  EXPECT_TRUE(convex_hull.empty());

  points.clear();

  // TestCase5
  // p1: (0, 0)
  // Expected points in convex hull: ()
  p1.x = 0.0;
  p1.y = 0.0;

  points.push_back(p1);

  convex_hull = grahamScan(points);

  EXPECT_TRUE(convex_hull.empty());

  points.clear();

  // TestCase6
  // p1: (0, 0)
  // p2: (1, 0)
  // Expected points in convex hull: ()
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;

  points.push_back(p1);
  points.push_back(p2);

  convex_hull = grahamScan(points);

  EXPECT_TRUE(convex_hull.empty());

  points.clear();

  // TestCase7
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (1, 0)
  // Expected points in convex hull: ()
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = 1.0;
  p3.y = 0.0;

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);

  convex_hull = grahamScan(points);

  EXPECT_TRUE(convex_hull.empty());

  points.clear();

  // TestCase8
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (1, 0)
  // p4: (1, 1)
  // Expected points in convex hull: (p1, p3, p4)
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = 1.0;
  p3.y = 0.0;
  p4.x = 1.0;
  p4.y = 1.0;

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);

  convex_hull = grahamScan(points);

  EXPECT_DOUBLE_EQ(p1.x, convex_hull.at(0).x);
  EXPECT_DOUBLE_EQ(p1.y, convex_hull.at(0).y);
  EXPECT_DOUBLE_EQ(p3.x, convex_hull.at(1).x);
  EXPECT_DOUBLE_EQ(p3.y, convex_hull.at(1).y);
  EXPECT_DOUBLE_EQ(p4.x, convex_hull.at(2).x);
  EXPECT_DOUBLE_EQ(p4.y, convex_hull.at(2).y);

  points.clear();
}

TEST(CGUtilTest, IsConvexTest1)
{
  geometry_msgs::Point p1, p2, p3, p4, p5;
  std::vector<geometry_msgs::Point> points;

  // TestCase1
  // p1: (0, 0)
  // p2: (4, 1)
  // p3: (6, 4)
  // p4: (3, 5)
  // p5: (0, 3)
  // Expected value: True
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 4.0;
  p2.y = 1.0;
  p3.x = 6.0;
  p3.y = 4.0;
  p4.x = 3.0;
  p4.y = 5.0;
  p5.x = 0.0;
  p5.y = 3.0;

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);

  EXPECT_TRUE(isConvex(points));
}

TEST(CGUtilTest, IsConvexTest2)
{
  geometry_msgs::Point p1, p2, p3, p4, p5;
  std::vector<geometry_msgs::Point> points;

  // TestCase2
  // p1: (0, 0)
  // p2: (1, 0)
  // p3: (1, 0)
  // p4: (1, 1)
  // Expected points in convex hull: (p1, p3, p4)
  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 1.0;
  p2.y = 0.0;
  p3.x = 1.0;
  p3.y = 0.0;
  p4.x = 1.0;
  p4.y = 1.0;

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);

  EXPECT_FALSE(isConvex(points));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
