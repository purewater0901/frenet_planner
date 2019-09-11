#include "lattice_planner/model/polynomial_spline_model.h"

void PolynomialSplineModel::updateState(TrajectoryPoint &state,
                                        const std::vector<double> &coefficients,
                                        ReferenceTrajectoryProcessor& rtprocessor,
                                        const double initialPosition,
                                        const double terminalPosition,
                                        const double ds_dash,
                                        const double initialYaw,
                                        bool calcFrenet)
{
    //calculate position s
    double initialS = initialPosition;
    double terminalS = terminalPosition;
    double intermediateS = (initialS+terminalS)/2.0;

    //calculate curvature
    double terminalCurvature = interpolation(coefficients, terminalS);

    //calculate yaw
    double initialTheta  = integratedInterpolation(coefficients, initialS)+initialYaw;
    double terminalTheta = integratedInterpolation(coefficients, terminalS)+initialYaw;
    double intermediateTheta = integratedInterpolation(coefficients, intermediateS)+initialYaw;

    double f_a_x = cos(initialTheta);
    double f_b_x = cos(terminalTheta);
    double f_intermediate_x = cos(intermediateTheta);
    double f_a_y = sin(initialTheta);
    double f_b_y = sin(terminalTheta);
    double f_intermediate_y = sin(intermediateTheta);

    //Simpson's formulation
    state.x_ = state.x_ + (ds_dash/6) * (f_a_x + 4 * f_intermediate_x + f_b_x);
    state.y_ = state.y_ + (ds_dash/6) * (f_a_y + 4 * f_intermediate_y + f_b_y);
    state.yaw_ = terminalTheta;
    state.curvature_ = terminalCurvature;
    state.s_ += ds_;
    if(calcFrenet)
    {
        //calc l_
        std::array<double,2> rpoint = rtprocessor.spline_.calc_position(state.s_);
        double rx = rpoint[0];
        double ry = rpoint[1];
        double ryaw = rtprocessor.spline_.calc_yaw(state.s_);
        double snew, lnew;
        cartesian_to_frenet(state.s_, rx, ry, ryaw, state.x_, state.y_, &snew, &lnew);
        state.l_ = lnew;
    }
}


Trajectory PolynomialSplineModel::generateTrajectory(const TrajectoryPoint& initialState,
                                                     const TrajectoryPoint& targetState,
                                                     const PolynomialSplineModelParameter& p,
                                                     ReferenceTrajectoryProcessor& rtprocessor)
{
    /*
     * @breif generate Trajectory from initial Point using parameter p
     * @parameter
     * p parameter(distance from initial point to goal, steering angle)
     */
    Trajectory output;

    if(std::fabs(initialState.s_-targetState.s_)<1e-8)
        return output;

    std::vector<double> sPoints;
    sPoints.resize(4);
    sPoints[0] = 0;
    sPoints[1] = p.distance_ / 3;
    sPoints[2] = p.distance_ * 2 / 3;
    sPoints[3] = p.distance_;

    //スプラインの係数を計算
    std::vector<double> coefficients = calcSplineCoeffcient(sPoints, p.curvature_);
    TrajectoryPoint state = initialState;
    double initialYaw= initialState.yaw_;
    double ds_dash = ds_ * p.distance_/std::fabs(targetState.s_ - initialState.s_);

    for(double traversalLength=0.0; traversalLength + ds_dash < p.distance_; traversalLength+=ds_dash)
    {
        updateState(state, coefficients, rtprocessor, traversalLength, traversalLength+ds_dash, ds_dash,
                   initialYaw, true);

        TrajectoryPoint tmp(state);
        tmp.yaw_ = normalizeAngle(tmp.yaw_);
        output.trajectoryPoints_.push_back(tmp);       //add new state to the output

        if(state.s_+ds_>rtprocessor.spline_.max_s_value_)
            break;
    }

    return output;
}

TrajectoryPoint PolynomialSplineModel::generateLastState(const TrajectoryPoint& initialState,
                                                         const TrajectoryPoint& targetState,
                                                         const PolynomialSplineModelParameter& p,
                                                         ReferenceTrajectoryProcessor& rtprocessor)
{
    /*
     * @breif generate Trajectory from initial Point using parameter p
     * @parameter
     * p parameter(distance from initial point to goal, steering angle)
     */
    if(std::fabs(initialState.s_-targetState.s_)<1e-8)
        return initialState;

    std::vector<double> sPoints;
    sPoints.resize(4);
    sPoints[0] = 0;
    sPoints[1] = p.distance_ / 3;
    sPoints[2] = p.distance_ * 2 / 3;
    sPoints[3] = p.distance_;

    //スプラインの係数を計算
    std::vector<double> coefficients = calcSplineCoeffcient(sPoints, p.curvature_);
    TrajectoryPoint state = initialState;
    double initialYaw= initialState.yaw_;
    double traversalLength = 0.0;  //traversal length at the self frame
    double ds_dash=ds_ * p.distance_/std::fabs(targetState.s_ - initialState.s_);

    while(traversalLength + ds_dash < p.distance_)
    {
        updateState(state, coefficients, rtprocessor, traversalLength, traversalLength+ds_dash, ds_dash, initialYaw);
        traversalLength+=ds_dash;
    }

    return state;
}