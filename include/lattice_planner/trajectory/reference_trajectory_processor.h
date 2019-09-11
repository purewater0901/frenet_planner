#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include "lattice_planner/tool/cubic_spline.h"
#include "trajectory.h"

class ReferenceTrajectoryProcessor
{
public:
    /*
        @brief
        *x_, y_ is waypoint of the reference trajectory
    */
    ReferenceTrajectoryProcessor(const std::vector<double>& x, const std::vector<double>& y) : spline_(x, y)
    {
        trajectory_.reserve(spline_.s.size());

        int id = 0;
        for(float s=0; s<spline_.s.back(); s+=0.1)
        {
            std::array<double, 2> point = spline_.calc_position(s);
            TrajectoryPoint waypoint;

            //pose
            waypoint.x_ = point[0];
            waypoint.y_ = point[1];
            waypoint.yaw_ = spline_.calc_yaw(s);

            //others
            waypoint.s_  = s;
            waypoint.l_ = 0.0;
            waypoint.id_ = id;
            id++;
            waypoint.curvature_ = spline_.calc_curvature(s);

            trajectory_.push_back(waypoint);
        }
    }

    std::vector<TrajectoryPoint> trajectory_;
    Spline2D spline_;
};