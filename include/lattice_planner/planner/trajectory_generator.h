#ifndef STATELATTICEPLANNER_TRAJECTORY_GENERATOR_H
#define STATELATTICEPLANNER_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/tokenizer.hpp>
#include "lattice_planner/model/polynomial_spline_model.h"
#include "lattice_planner/trajectory/trajectory.h"
#include "lattice_planner/tool/math.h"

class TrajectoryGenerator 
{
public:
    PolynomialSplineModel model_;

    int maxIteration_;
    double costThreshold;

    TrajectoryGenerator(double ds, int maxIteration, double costThreshold)
                                        : model_(ds), maxIteration_(maxIteration), costThreshold(costThreshold){}

    //Generate Optimize trajectory(For lattice)
    Trajectory getOptimizeTrajectory(PolynomialSplineModelParameter& parameter, const TrajectoryPoint& current,
                                           const TrajectoryPoint& target, const std::vector<double>& h, ReferenceTrajectoryProcessor& rtprocessor);

    PolynomialSplineModelParameter getOptimizeParameter(PolynomialSplineModelParameter& parameter, const TrajectoryPoint& current,
                                         const TrajectoryPoint& target, const std::vector<double>& h, ReferenceTrajectoryProcessor& rtprocessor);

    //for generate Table
    //Generate primitives
    void visualize(const Trajectory& primitive, const TrajectoryPoint& current, const TrajectoryPoint& target);

private:
    //for frenel model
    Eigen::Matrix3d calcJacobian(const std::vector<double>& h, const TrajectoryPoint& current,
                                       const TrajectoryPoint& target, const PolynomialSplineModelParameter& parameter, ReferenceTrajectoryProcessor& rtprocessor);
    Eigen::Vector3d calcErrorVector(const PolynomialSplineModelParameter& currentParameter, const TrajectoryPoint& current, const TrajectoryPoint& target, ReferenceTrajectoryProcessor& rtprocessor);
    Eigen::Vector3d calcDiffBetweenTwoPoints(const TrajectoryPoint& current, const TrajectoryPoint& target);
    bool calcOptimizeParameter(PolynomialSplineModelParameter& parameter, const TrajectoryPoint& current,
                                                const TrajectoryPoint& target, const std::vector<double>& h, ReferenceTrajectoryProcessor& rtprocessor);
};


#endif //STATELATTICEPLANNER_MODEL_PREDICTIVE_TRAJECTORY_H
