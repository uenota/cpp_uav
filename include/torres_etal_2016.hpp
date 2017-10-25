/**
* @file torres_etal_2016.hpp
* @brief Header file for torres_etal_2016.cpp
* @author Takaki Ueno
*/

#ifndef INCLUDED_torres_etal_2016_hpp_
#define INCLUDED_torres_etal_2016_hpp_

// roscpp
#include <ros/ros.h>

// cpp standard libraries
#include <array>
#include <vector>

// std_msgs
#include <std_msgs/Float64.h>

// geometry_msgs
#include <geometry_msgs/Point.h>

// Boost
#include <boost/range/adaptor/indexed.hpp>


/**
* @struct direction
* @brief Storage for line sweep direction
* 
* Sweep direction is from edge to vertex
*/
struct direction{
    std::array<geometry_msgs::Point, 2> base_edge;
    geometry_msgs::Point opposed_vertex;
};


/**
* @brief Calculates distance between given edge and vertex
* @param edge An edge of given polygon
* @param vertex A vertex of given polygon
* @return double Distance between edge and vertex
*/
double distance(const std::array<geometry_msgs::Point, 2>& edge, const geometry_msgs::Point& vertex)
{
    // Vertices of triangle
    geometry_msgs::Point point_a, point_b;
    point_a = edge.front();
    point_b = edge.back();

    ROS_DEBUG("PointA x: %f", point_a.x);
    ROS_DEBUG("PointA y: %f", point_a.y);
    ROS_DEBUG("PointB x: %f", point_b.x);
    ROS_DEBUG("PointB y: %f", point_b.y);

    // Calculate length of each edge
    // Edge A: An edge facing vertex A
    // Edge B: An edge facing vertex B
    // Edge C: An edge facing vertex C
    double len_edge_a, len_edge_b, len_edge_c;
    len_edge_a = std::sqrt(std::pow(point_b.x - vertex.x, 2)+std::pow(point_b.y - vertex.y, 2));
    len_edge_b = std::sqrt(std::pow(vertex.x - point_a.x, 2)+std::pow(vertex.y - point_a.y, 2));
    len_edge_c = std::sqrt(std::pow(point_a.x - point_b.x, 2)+std::pow(point_a.y - point_b.y, 2));

    ROS_DEBUG("len a: %f", len_edge_a);
    ROS_DEBUG("len b: %f", len_edge_b);
    ROS_DEBUG("len c: %f", len_edge_c);

    if((std::abs(len_edge_a)<1.0e-6) or (std::abs(len_edge_b)<1.0e-6))
    {
        return 0.0;
    } 

    // cosine_alpha: Cosine of the angle between edge B and edge C
    // cosine_beta: Cosine of the angle between edge A and edge C
    double cosine_alpha, cosine_beta;
    // Law of cosines
    cosine_alpha = 
        (std::pow(len_edge_b, 2)+std::pow(len_edge_c, 2)-std::pow(len_edge_a, 2))/(2*len_edge_b*len_edge_c);
    cosine_beta = 
        (std::pow(len_edge_a, 2)+std::pow(len_edge_c, 2)-std::pow(len_edge_b, 2))/(2*len_edge_a*len_edge_c);

    ROS_DEBUG("cosine alpha: %f", cosine_alpha);
    ROS_DEBUG("cosine beta: %f", cosine_beta);

    // Relation between sine and cosine
    double distance = cosine_alpha >= 0 ?
                          std::sqrt(1-std::pow(cosine_alpha,2)) * len_edge_b :
                          std::sqrt(1-std::pow(cosine_beta,2)) * len_edge_a;
               
    ROS_DEBUG("Distance: %f", distance);

    return distance;
}

/**
* @brief Calculates line sweep direction for given polygon
* @param polygon Line sweep direction is calculated on this region
* @return direction Struct containing edge and vertex
*/
direction sweep_direction(std::vector<geometry_msgs::Point>& polygon)
{
    direction line_sweep;

    // Edges of polygon
    std::vector<std::array<geometry_msgs::Point, 2>> edges;

    // Make a list of edges of polygon
    for(std::size_t i=0; i<polygon.size(); ++i)
    {
        std::array<geometry_msgs::Point, 2> ar;

        ar.at(0) = polygon.at(i);

        // if vertex is the last one,
        // that vertex makes an edge whose end is the first vertex
        if(i==polygon.size()-1){
            ar.at(1) = polygon.at(0);
        }else{
            ar.at(1) = polygon.at(i+1);
        }
        edges.push_back(ar);
    }

    double optimal_dist = 0;

    // Calculate line sweep direction
    // Algorithm 1 in Torres et al, 2016
    for (const auto& edge: edges | boost::adaptors::indexed())
    {
        double max_dist_edge = 0;
        geometry_msgs::Point opposed_vertex;

        ROS_DEBUG("e1.x: %f", edge.value().front().x);
        ROS_DEBUG("e1.y: %f", edge.value().front().y);
        ROS_DEBUG("e2.x: %f", edge.value().back().x);
        ROS_DEBUG("e2.y: %f", edge.value().back().y);

        for (const geometry_msgs::Point& vertex: polygon)
        {
            // distance() function returns distance 
            // between given edge and vertex
            double dist = distance(edge.value(), vertex);

            ROS_DEBUG("vertex.x: %f", vertex.x);
            ROS_DEBUG("vertex.y: %f", vertex.y);
            ROS_DEBUG("distance: %f", dist);

            if (dist > max_dist_edge)
            {
                max_dist_edge = dist;
                opposed_vertex = vertex;
                ROS_DEBUG("max_dist_edge: %f", max_dist_edge);                
            }
        }

        if ((max_dist_edge < optimal_dist) or edge.index()==0)
        {
            optimal_dist = max_dist_edge;
            line_sweep.base_edge = edge.value();
            line_sweep.opposed_vertex = opposed_vertex;
            ROS_DEBUG("index: %ld", edge.index());
            ROS_DEBUG("optimal dist: %f", optimal_dist);            
        }
    }

    ROS_DEBUG("End");

    return line_sweep;
}

#endif