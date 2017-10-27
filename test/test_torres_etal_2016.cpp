/**
* @file test_torres_etal_2016.cpp
* @brief Test program for torres_etal_2016.hpp
* @author Takaki Ueno
*/

#include <torres_etal_2016.hpp>

// gtest
#include <gtest/gtest.h>

// c++ libraries
#include <array>

// geometry_msgs
#include <geometry_msgs/Point.h>

/**
* @brief Test for calc_sweep_direction()
*/
TEST(Torres16Test, CalcSweepDirectionTest)
{
    std::vector<geometry_msgs::Point> polygon;

    // Vertices of polygon
    geometry_msgs::Point p1, p2, p3, p4, p5;

    // Edges of polygon
    std::array<geometry_msgs::Point, 2> e1, e2, e3, e4, e5;

    // TestCase1
    // coordinate of vertex1: (1.5, 2.5)
    // coordinate of vertex2: (0.5, 1.0)
    // coordinate of vertex3: (6.0, 1.0)
    // expected direction: edge2 -> vertex1
    p1.x = 1.5;
    p1.y = 2.5;
    p2.x = 0.5;
    p2.y = 1.0;
    p3.x = 6.0;
    p3.y = 1.0;
    polygon.push_back(p1);
    polygon.push_back(p2);
    polygon.push_back(p3);

    e1.at(0) = p1;
    e1.at(1) = p2;
    e2.at(0) = p2;
    e2.at(1) = p3;
    e3.at(0) = p3;
    e3.at(1) = p1;

    Direction dir = calc_sweep_direction(polygon);

    EXPECT_DOUBLE_EQ(e2.front().x, dir.base_edge.front().x);
    EXPECT_DOUBLE_EQ(e2.front().y, dir.base_edge.front().y);
    EXPECT_DOUBLE_EQ(e2.back().x, dir.base_edge.back().x);
    EXPECT_DOUBLE_EQ(e2.back().y, dir.base_edge.back().y);
    EXPECT_DOUBLE_EQ(p1.x, dir.opposed_vertex.x);
    EXPECT_DOUBLE_EQ(p1.y, dir.opposed_vertex.y);

    polygon.clear();

    // TestCase2
    // coordinate of vertex1: (2.0, 4.0)
    // coordinate of vertex2: (1.0, 1.0)
    // coordinate of vertex3: (5.0, 1.0)
    // coordinate of vertex4: (6.0, 3.0)
    // expected direction: edge2 -> vertex1
    p1.x = 2.0;
    p1.y = 4.0;
    p2.x = 1.0;
    p2.y = 1.0;
    p3.x = 5.0;
    p3.y = 1.0;
    p4.x = 6.0;
    p4.y = 3.0;
    polygon.push_back(p1);
    polygon.push_back(p2);
    polygon.push_back(p3);
    polygon.push_back(p4);

    e1.at(0) = p1;
    e1.at(1) = p2;
    e2.at(0) = p2;
    e2.at(1) = p3;
    e3.at(0) = p3;
    e3.at(1) = p4;
    e4.at(0) = p4;
    e4.at(1) = p1;

    dir = calc_sweep_direction(polygon);

    EXPECT_DOUBLE_EQ(e2.front().x, dir.base_edge.front().x);
    EXPECT_DOUBLE_EQ(e2.front().y, dir.base_edge.front().y);
    EXPECT_DOUBLE_EQ(e2.back().x, dir.base_edge.back().x);
    EXPECT_DOUBLE_EQ(e2.back().y, dir.base_edge.back().y);
    EXPECT_DOUBLE_EQ(p1.x, dir.opposed_vertex.x);
    EXPECT_DOUBLE_EQ(p1.y, dir.opposed_vertex.y);

    polygon.clear();

    // TestCase3
    // coordinate of vertex1: (1.0, 3.0)
    // coordinate of vertex2: (2.0, 1.0)
    // coordinate of vertex3: (5.0, 0.5)
    // coordinate of vertex4: (5.5, 2.5)
    // coordinate of vertex5: (3.0, 4.0)
    // expected direction: edge2 -> vertex5
    p1.x = 1.0;
    p1.y = 3.0;
    p2.x = 2.0;
    p2.y = 1.0;
    p3.x = 5.0;
    p3.y = 0.5;
    p4.x = 5.5;
    p4.y = 2.5;
    p5.x = 3.0;
    p5.y = 4.0;
    polygon.push_back(p1);
    polygon.push_back(p2);
    polygon.push_back(p3);
    polygon.push_back(p4);
    polygon.push_back(p5);

    e1.at(0) = p1;
    e1.at(1) = p2;
    e2.at(0) = p2;
    e2.at(1) = p3;
    e3.at(0) = p3;
    e3.at(1) = p4;
    e4.at(0) = p4;
    e4.at(1) = p5;
    e5.at(0) = p5;
    e5.at(1) = p1;

    dir = calc_sweep_direction(polygon);

    EXPECT_DOUBLE_EQ(e4.front().x, dir.base_edge.front().x);
    EXPECT_DOUBLE_EQ(e4.front().y, dir.base_edge.front().y);
    EXPECT_DOUBLE_EQ(e4.back().x, dir.base_edge.back().x);
    EXPECT_DOUBLE_EQ(e4.back().y, dir.base_edge.back().y);
    EXPECT_DOUBLE_EQ(p2.x, dir.opposed_vertex.x);
    EXPECT_DOUBLE_EQ(p2.y, dir.opposed_vertex.y);

    polygon.clear();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}