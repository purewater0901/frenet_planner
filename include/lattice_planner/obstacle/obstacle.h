#ifndef STATELATTICEPLANNER_OBSTACLE_H
#define STATELATTICEPLANNER_OBSTACLE_H

#include <vector>
#include <array>

class Obstacle
{
public:
    Obstacle() : x_(0.0), y_(0.0), radius_(0.0), translational_velocity_(0.0) {}
    Obstacle(double x, double y) : x_(x), y_(y) {}
    double x_;
    double y_;
    double radius_;
    double translational_velocity_;
};

#endif //STATELATTICEPLANNER_OBSTACLE_H
