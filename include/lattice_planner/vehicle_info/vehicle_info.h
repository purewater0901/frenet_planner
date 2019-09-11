#ifndef SHIMIZU_FRENET_PLANNER_VEHICLE_INFO_H
#define SHIMIZU_FRENET_PLANNER_VEHICLE_INFO_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>

class VehicleInfo
{
public:
    VehicleInfo(const double length, const double width, const double wheelBase, const double safetyDistance);
    ~VehicleInfo() = default;

    double length_;
    double width_;
    double wheelBase_;
    double safetyDistance_;
    double circumcircleRadius_;
    double middlecircleRadius_;
    double footprintcircleRadius_;

    std::vector<Eigen::Vector2d> middlecircleCenters_;
    std::vector<Eigen::Vector2d> footprintcircleCenters_;
};

#endif //SHIMIZU_FRENET_PLANNER_VEHICLE_INFO_H
