#pragma once 

#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include "lattice_planner/trajectory/trajectory.h"
#include "lattice_planner/tool/math.h"
#include "lattice_planner/trajectory/reference_trajectory_processor.h"

class PolynomialSplineModelParameter
{
public:
    PolynomialSplineModelParameter(double distance, std::vector<double>& curvature) : distance_(distance), curvature_(curvature){}
    PolynomialSplineModelParameter() : distance_(0.0), curvature_({0,0,0,0}) {}

    double distance_;
    std::vector<double> curvature_;
};

class PolynomialSplineModel
{
public:
    PolynomialSplineModel(const double ds) : ds_(ds){}
    void updateState(TrajectoryPoint &state,
                     const std::vector<double> &coefficients,
                     ReferenceTrajectoryProcessor& rtprocessor,
                     const double initialPosition,
                     const double terminalPosition,
                     const double initialYaw,
                     const double ds_dash,
                     bool calcFrenet=false);

    Trajectory generateTrajectory(const TrajectoryPoint& initialState,
                                  const TrajectoryPoint& targetState,
                                  const PolynomialSplineModelParameter& p,
                                  ReferenceTrajectoryProcessor& rtprocessor);

    TrajectoryPoint generateLastState(const TrajectoryPoint& initialState,
                                      const TrajectoryPoint& targetState,
                                      const PolynomialSplineModelParameter& p,
                                      ReferenceTrajectoryProcessor& rtProcessor);

    double ds_;
};