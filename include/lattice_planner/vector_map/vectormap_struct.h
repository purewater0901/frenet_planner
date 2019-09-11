
#ifndef VECTORMAP_STRUCT_H
#define VECTORMAP_STRUCT_H

#include <vector>

enum GoalLaneType
{   Left = 0, 
    Straight = 1, 
    Right = 2,
    NotGoal = 3
};


// struct LanePoint
// {
//     double tx;  // translation of X axis
//     double ty;  // translation of Y axis
//     double rz;  // rotation of Z axis (yaw)
//     double sr;  // squared radius
//     double cumulated_s; // cumulated s in frenet coordinate from the origin lane point
//     double curvature;
//     std::vector<LanePoint> goal_points;
//     std::vector<std::vector<LanePoint>> goal_paths;
//     GoalLaneType type;
//     bool is_in_intersection;
//     // std::vector<LanePoint> goal_lane_point;
// };

struct Point
{
    double tx;  // translation of X axis
    double ty;  // translation of Y axis
    double rz;  // rotation of Z axis (yaw)
    // double sr;  // squared radius
    double curvature;
    double curvature_dot;
    double cumulated_s; // cumulated s in frenet coordinate from the origin lane point 
    std::vector<Point> points;
};

// struct LanePath
// {
//   std::vector<LanePoint> path;
// };

#endif