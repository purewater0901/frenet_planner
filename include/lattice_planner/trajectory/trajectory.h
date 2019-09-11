#ifndef STATELATTICEPLANNER_TRAJECTORY_H
#define STATELATTICEPLANNER_TRAJECTORY_H

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "lattice_planner/tool/math.h"
class TrajectoryPoint
{
public:
    TrajectoryPoint() : x_(0.0), y_(0.0), yaw_(0.0), s_(0.0), l_(0.0), curvature_(0.0), id_(0) {};
    TrajectoryPoint(double x, double y, double yaw, double curvature=0.0, double s=0.0, double l=0.0, int id=0)
                  : x_(x), y_(y), yaw_(yaw), s_(s), l_(l), curvature_(curvature), id_(id) {};

    double x_;
    double y_;
    double yaw_;
    double s_;
    double l_;
    double curvature_;
    int id_;

protected:

};

class Trajectory
{
public:
    Trajectory() : pathCost_(0.0), length_(-1), isCollide_(-1) {}
    explicit Trajectory(double pathCost) : pathCost_(pathCost), length_(-1), isCollide_(-1) {}

    void setPathCost(const double& cost){pathCost_ = cost;}

    std::vector<TrajectoryPoint> trajectoryPoints_;
    double pathCost_;
    double length_;
    int isCollide_;

private:
};

#endif //STATELATTICEPLANNER_TRAJECTORY_H