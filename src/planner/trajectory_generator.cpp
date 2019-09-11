#include "lattice_planner/planner/trajectory_generator.h"

Trajectory TrajectoryGenerator::getOptimizeTrajectory(PolynomialSplineModelParameter& parameter,
                                                      const TrajectoryPoint& current,
                                                      const TrajectoryPoint& target,
                                                      const std::vector<double>& h,
                                                      ReferenceTrajectoryProcessor& rtprocessor)
{
    bool check = calcOptimizeParameter(parameter, current, target, h, rtprocessor);
   Trajectory trajectory;
   if(check)
   {
       trajectory = model_.generateTrajectory(current, target, parameter, rtprocessor);
       trajectory.length_ = parameter.distance_;
   }

   return trajectory;
}                                           

PolynomialSplineModelParameter TrajectoryGenerator::getOptimizeParameter(PolynomialSplineModelParameter &parameter,
                                                                      const TrajectoryPoint &current,
                                                                      const TrajectoryPoint &target,
                                                                      const std::vector<double> &h, ReferenceTrajectoryProcessor& rtprocessor)

{
    bool check = calcOptimizeParameter(parameter, current, target, h, rtprocessor);

    return parameter;
}

bool TrajectoryGenerator::calcOptimizeParameter(PolynomialSplineModelParameter& parameter, const TrajectoryPoint& current,
                                            const TrajectoryPoint& target, const std::vector<double>& h, ReferenceTrajectoryProcessor& rtprocessor)
{
    int iteration=0;
    double previousCost=std::numeric_limits<double>::max();
    while(iteration<maxIteration_)
    {
        Eigen::Vector3d error = calcErrorVector(parameter, current, target, rtprocessor);
        //calc cost
        double cost = std::sqrt(std::pow(error(0), 2)+std::pow(error(1), 2)+std::pow(error(2), 2));
        if(cost < costThreshold)
        {
            if(parameter.distance_ > 2.0*std::fabs(target.s_-current.s_)) //delete long path
            {
                std::cerr << "Generated Path Length is too long" << std::endl;
                return false;
            }
            return true;
        }

        Eigen::Matrix3d Jacobian = calcJacobian(h, current, target, parameter, rtprocessor);
        Eigen::PartialPivLU<Eigen::Matrix3d> lu;
        lu.compute(Jacobian);
        Eigen::Vector3d dParameter = -lu.solve(error);

        for(int i=0; i<3; ++i)
        {
            if(std::isinf(dParameter(i)) || std::isnan(dParameter(i)))
            {
                std::cout << Jacobian << std::endl;
                std::cout << dParameter(i) << std::endl;
                std::cerr << "Invalid value" << std::endl;
                return false;
            }
        }

        parameter.distance_ += dParameter(0);
        parameter.curvature_[1] += dParameter(1);
        parameter.curvature_[2] += dParameter(2);
        previousCost = cost;

        ++iteration;
    }

    std::cerr << "Cannot Converge" << std::endl;
    return false;
}

Eigen::Matrix3d TrajectoryGenerator::calcJacobian(const std::vector<double>& h, const TrajectoryPoint& current,
                                                              const TrajectoryPoint& target, const PolynomialSplineModelParameter& parameter, ReferenceTrajectoryProcessor& rtprocessor)
{
    //まずはdistanceに摂動を加えた場合
    PolynomialSplineModelParameter p00 = parameter;
    p00.distance_ = parameter.distance_ + h[0];
    Eigen::Vector3d d00_vct = calcErrorVector(p00, current, target, rtprocessor); //パラメータp00で計算した場合の誤差
    PolynomialSplineModelParameter p01 = parameter;
    p01.distance_ = parameter.distance_ - h[0];
    Eigen::Vector3d d01_vct = calcErrorVector(p01, current, target, rtprocessor); //パラメータp01で計算した場合の誤差
    Eigen::Vector3d d0 = (d00_vct - d01_vct)/(2*h[0]);

    //curvature_[1]に摂動を加えた場合
    PolynomialSplineModelParameter p10 = parameter;
    p10.curvature_[1] = parameter.curvature_[1] + h[1];
    Eigen::Vector3d d10_vct = calcErrorVector(p10, current, target, rtprocessor); //パラメータp10で計算した場合の誤差
    PolynomialSplineModelParameter p11 = parameter;
    p11.curvature_[1] = parameter.curvature_[1] - h[1];
    Eigen::Vector3d d11_vct = calcErrorVector(p11, current, target, rtprocessor); //パラメータp11で計算した場合の誤差
    Eigen::Vector3d d1 = (d10_vct - d11_vct)/(2*h[1]);

    //curvature_[2]に摂動を加えた場合
    PolynomialSplineModelParameter p20 = parameter;
    p20.curvature_[2] = parameter.curvature_[2] + h[2];
    Eigen::Vector3d d20_vct = calcErrorVector(p20, current, target, rtprocessor); //パラメータp20で計算した場合の誤差
    PolynomialSplineModelParameter p21 = parameter;
    p21.curvature_[2] = parameter.curvature_[2] - h[2];
    Eigen::Vector3d d21_vct = calcErrorVector(p21, current, target, rtprocessor); //パラメータp21で計算した場合の誤差
    Eigen::Vector3d d2 = (d20_vct - d21_vct)/(2*h[2]);

    Eigen::Matrix3d J;
    J << d0, d1, d2;

    return J;
}

Eigen::Vector3d TrajectoryGenerator::calcDiffBetweenTwoPoints(const TrajectoryPoint& current, const TrajectoryPoint& target)
{
    double yaw = target.yaw_- current.yaw_;
    return {target.x_ - current.x_, target.y_ - current.y_, normalizeAngle(yaw)};
}

Eigen::Vector3d TrajectoryGenerator::calcErrorVector(const PolynomialSplineModelParameter& currentParameter,
                                                   const TrajectoryPoint& current, const TrajectoryPoint& target, ReferenceTrajectoryProcessor& rtprocessor)
{
    TrajectoryPoint lastTrajectoryPoint = model_.generateLastState(current, target, currentParameter, rtprocessor);
    Eigen::Vector3d error = calcDiffBetweenTwoPoints(lastTrajectoryPoint, target);
    return error;
}

void TrajectoryGenerator::visualize(const Trajectory& primitive, const TrajectoryPoint& current, const TrajectoryPoint& target)
{
    cv::namedWindow("trajectoryANDpoint");
    cv::Mat bg(800, 1800, CV_8UC3, cv::Scalar(255, 255, 255));

    for(int i=1; i<primitive.trajectoryPoints_.size(); ++i)
    {
        TrajectoryPoint previousPoint = primitive.trajectoryPoints_[i-1];
        TrajectoryPoint currentPoint = primitive.trajectoryPoints_[i];
        cv::line(bg,
                 cv_offset(previousPoint.x_, previousPoint.y_, bg.cols, bg.rows),
                 cv_offset(currentPoint.x_, currentPoint.y_, bg.cols, bg.rows),
                 cv::Scalar(0,0,int((i/255.0)*255)), 2);
    }

    cv::circle(bg, cv_offset(target.x_, target.y_, bg.cols, bg.rows),
               30, cv::Scalar(255, 0, 0), 5);
    cv::circle(bg, cv_offset(current.x_, current.y_, bg.cols, bg.rows),
               30, cv::Scalar(100, 255, 123), 5);

    cv::imshow("trajectoryANDpoint", bg);
    cv::waitKey(0);

}
