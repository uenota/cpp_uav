#include <torres_etal_2016.hpp>

// gtest
#include <gtest/gtest.h>

#include <array>
#include <geometry_msgs/Point.h>


TEST(Torres16Test, DistanceTest)
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

    vertex.x = 2.0/3.0;
    vertex.y = 1.0/3.0;

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

TEST(Torres16Test, SweepDirectionTest)
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

    direction dir = sweep_direction(polygon);

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

    dir = sweep_direction(polygon);

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

    dir = sweep_direction(polygon);

    EXPECT_DOUBLE_EQ(e4.front().x, dir.base_edge.front().x);
    EXPECT_DOUBLE_EQ(e4.front().y, dir.base_edge.front().y);
    EXPECT_DOUBLE_EQ(e4.back().x, dir.base_edge.back().x);
    EXPECT_DOUBLE_EQ(e4.back().y, dir.base_edge.back().y);
    EXPECT_DOUBLE_EQ(p2.x, dir.opposed_vertex.x);    
    EXPECT_DOUBLE_EQ(p2.y, dir.opposed_vertex.y);

    polygon.clear();
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}