#include "lattice_planner/planner/lattice_planner.h"
#include <chrono>

LatticePlanner::LatticePlanner(double d,  
                               double ds, 
                               double sampleLateralLength, 
                               double dSampleLength, 
                               double maxSpeed, 
                               double maxAccel,
                               double vehicleLength,
                               double vehicleWidth,
                               double vehicleWheelBase,
                               double vehicleSafetyDistance,
                               double dt,
                               double ws, 
                               double wd, 
                               double wc, 
                               double min_curvature, 
                               double max_curvature,
                               int maxIteration,
                               double costThreshold)
        : d_(d), optimizer_(ds, maxIteration, costThreshold), sampleLateralLength_(sampleLateralLength), dSampleLength_(dSampleLength),
          maxSpeed_(maxSpeed), maxAccel_(maxAccel),
          dt_(dt), previousFrontReferencePoint_(nullptr), previousBestTrajectory_(nullptr),
          ws_(ws), wd_(wd), wc_(wc), min_curvature_(min_curvature), max_curvature_(max_curvature), sdv_(2*dSampleLength)
{
    egoVehicleInfoPtr_ = std::make_shared<VehicleInfo>(vehicleLength,
                                                       vehicleWidth,
                                                       vehicleWheelBase,
                                                       vehicleSafetyDistance);
}


void LatticePlanner::doPlan(TrajectoryPoint& currentState,  
                            const std::vector<Obstacle>& obstacles, 
                            ReferenceTrajectoryProcessor& rtprocessor, 
                            Trajectory& bestTrajectory, 
                            std::vector<Trajectory>& primitives,
                            const int maxIteration,
                            const double costThreshold)
{
    std::chrono::system_clock::time_point  start, end; 
    start = std::chrono::system_clock::now(); 

    //Globalパスに一番近い点を見つける(距離d_だけサンプリングするのでそれに一番近い点)
    TrajectoryPoint nearestToReferencePoint = calcNearestReferencePoint(currentState, rtprocessor);

    //一番近い点の周りでサンプリング
    std::vector<TrajectoryPoint> samplePoint = samplePointsFromNearestPoint(nearestToReferencePoint);
    assert(!samplePoint.empty());

    std::vector<double> h{0.1, 0.0001, 0.0001};
    std::vector<Trajectory> trajectories;
    if(!generateTrajectory(currentState, samplePoint, h, maxIteration, costThreshold, rtprocessor, trajectories))
        std::cerr << "No Trajectory to the sampled points" << std::endl;

    calcPathCost(trajectories, nearestToReferencePoint, obstacles, rtprocessor);
    primitives = trajectories;

    //get safest trajectory
    bestTrajectory= getSafestPath(trajectories);

    std::cout << "Best Trajectories Cost: " << bestTrajectory.pathCost_<< std::endl;

    //update previous path
    previousFrontReferencePoint_.reset();
    previousFrontReferencePoint_ =
            std::make_shared<std::array<double,2>>(rtprocessor.spline_.calc_position(bestTrajectory.trajectoryPoints_.front().s_));
    previousBestTrajectory_.reset();
    previousBestTrajectory_ = std::make_shared<Trajectory>(bestTrajectory);

    end = std::chrono::system_clock::now(); 
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Do Plan Calculation time is " << elapsed  << "[ms]" << std::endl;
}


bool LatticePlanner::checkCollision(Trajectory &trajectory, const std::vector<Obstacle> &obstacles)
{

    for(Obstacle obstacle : obstacles)
    {
        for(size_t i=0; i<trajectory.trajectoryPoints_.size(); i+=100)
        {
            //step1 check the largest radius around the vehicle
            double x = trajectory.trajectoryPoints_[i].x_;
            double y = trajectory.trajectoryPoints_[i].y_;
            double dist = std::pow((x - obstacle.x_), 2) + std::pow((y - obstacle.y_), 2);
            double clearanceRadius = egoVehicleInfoPtr_->circumcircleRadius_+egoVehicleInfoPtr_->safetyDistance_;

            if(dist <= std::pow(obstacle.radius_, 2)+ std::pow(clearanceRadius, 2))
            {
                //step2 check the middle circles
                double middleClearanceRadius = egoVehicleInfoPtr_->middlecircleRadius_
                                            + egoVehicleInfoPtr_->safetyDistance_;
                for(size_t mcircleId=0; mcircleId<2; ++mcircleId)
                {
                    double xmc = x + egoVehicleInfoPtr_->middlecircleCenters_[mcircleId](0);
                    double ymc = y + egoVehicleInfoPtr_->middlecircleCenters_[mcircleId](1);
                    double middleDist = std::pow(xmc-obstacle.x_, 2)+std::pow(ymc-obstacle.y_, 2);

                    if(middleDist <= std::pow(obstacle.radius_, 2)+ std::pow(middleClearanceRadius, 2))
                    {
                        //step3 check the 4 footprint circles
                        double footprintClearanceRadius = egoVehicleInfoPtr_->footprintcircleRadius_
                                                       + egoVehicleInfoPtr_->safetyDistance_;
                        for(size_t fcircleId=0; fcircleId<4; ++fcircleId)
                        {
                            double xfc = x + egoVehicleInfoPtr_->footprintcircleCenters_[fcircleId](0);
                            double yfc = y + egoVehicleInfoPtr_->footprintcircleCenters_[fcircleId](1);
                            double footprintDist = std::pow(xfc-obstacle.x_, 2)+std::pow(yfc-obstacle.y_, 2);

                            if(footprintDist<= std::pow(obstacle.radius_, 2)+ std::pow(footprintClearanceRadius, 2))
                            {
                                std::cerr << "This path will Collide" << std::endl;
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}


Trajectory LatticePlanner::getSafestPath(std::vector<Trajectory>& trajectories)
{
    auto compare = [](const Trajectory& lhs, const Trajectory& rhs){return lhs.pathCost_ > rhs.pathCost_;};
    std::priority_queue<Trajectory, std::vector<Trajectory>, decltype(compare)> safeTrajectory(compare);
    for(Trajectory trajectory : trajectories)
        //if(trajectory.isCollide_==0)
        safeTrajectory.push(trajectory);  //add no collision path

    if(safeTrajectory.empty())
    {
        std::cerr << "No safest path" << std::endl;
        exit(1);
    }

    // minimum cost in the trajectories
    return safeTrajectory.top();
}

double LatticePlanner::calcStaticSafeCost(const int currentPathId, const std::vector<Trajectory>& trajectories)
{
    int N = (trajectories.size()-1)/2;

    double cost=0;
    for(int k=-N; k<=N; ++k)
    {
        double R = 0;
        if(k+currentPathId<0 || k+currentPathId>=trajectories.size())
            R=1.0;
        else
            R = static_cast<double>(trajectories[k+currentPathId].isCollide_);

        
        double G = 1/(std::sqrt(2*M_PI*sdv_))*exp(-std::pow((k), 2)/(2*sdv_*sdv_)); 

        cost+=G*R;
    }

    return cost;
}

double LatticePlanner::calcConsistencyCost(const Trajectory& trajectory, ReferenceTrajectoryProcessor& rtprocessor)
{
    double cost=0.0;
    //TODO: set accurate diffs
    std::array<double, 2> currentFrontReferencePoint =
                                rtprocessor.spline_.calc_position(trajectory.trajectoryPoints_.front().s_);

    double diffx = currentFrontReferencePoint[0] - previousFrontReferencePoint_->at(0);
    double diffy = currentFrontReferencePoint[1] - previousFrontReferencePoint_->at(1);
    int diff = (std::sqrt(diffx*diffx+diffy*diffy))/optimizer_.model_.ds_;

    for(int id=0; id+diff+1<previousBestTrajectory_->trajectoryPoints_.size(); ++id)
    {
        if(id>=trajectory.trajectoryPoints_.size())
            break;

        cost += std::fabs(trajectory.trajectoryPoints_[id].l_ - previousBestTrajectory_->trajectoryPoints_[id+diff+1].l_);
    }

    return cost;
}

void LatticePlanner::calcPathCost(std::vector<Trajectory>& trajectories,
                                    const TrajectoryPoint& nearestReferencePoint,
                                    const std::vector<Obstacle>& obstacles, 
                                    ReferenceTrajectoryProcessor& rtprocessor)
{
    std::vector<std::vector<double>> costVector(trajectories.size(), std::vector<double>(4, 0.0));

    for(int trajectoryNum=0; trajectoryNum<trajectories.size(); ++trajectoryNum)
    {
        //check collision
        if(checkCollision(trajectories[trajectoryNum], obstacles)) //will collide
            trajectories[trajectoryNum].isCollide_ = 1;
        else
            trajectories[trajectoryNum].isCollide_ = 0;
        
    }

    double maxStaticSafesCost  = 0.0;
    double maxDeviationCost    = 0.0;
    double maxCurvatureCost    = 0.0;
    double maxConsistencyCost  = 0.0;
    for(int trajectoryNum=0; trajectoryNum < trajectories.size(); ++trajectoryNum)
    {
        assert(nearestReferencePoint.id_>0);
        assert(!trajectories[trajectoryNum].trajectoryPoints_.empty());
        assert(trajectories[trajectoryNum].length_>0.0);

        //step1 calc static safe cost
        double staticSafeCost=calcStaticSafeCost(trajectoryNum, trajectories);

        //step2 calc comfortable cost
        double deviationCost = 0.0;
        double curvatureCost = 0.0;
        for(size_t i=0; i<trajectories[trajectoryNum].trajectoryPoints_.size(); ++i)
        {
            double deviation_at_point = std::fabs(trajectories[trajectoryNum].trajectoryPoints_[i].l_);
            double curvature_at_point = std::fabs(trajectories[trajectoryNum].trajectoryPoints_[i].curvature_);
            deviationCost += deviation_at_point;
            curvatureCost += curvature_at_point;
        }
    
        //step3 calc consistency cost
        double consistencyCost = 0.0;
        if(previousFrontReferencePoint_!=nullptr && previousBestTrajectory_!=nullptr)
           consistencyCost = calcConsistencyCost(trajectories[trajectoryNum], rtprocessor);

        //update max value
        if(maxStaticSafesCost<staticSafeCost)
            maxStaticSafesCost = staticSafeCost;

        if(maxDeviationCost<deviationCost)
            maxDeviationCost = deviationCost;

        if(maxCurvatureCost<curvatureCost)
            maxCurvatureCost = curvatureCost;

        if(maxConsistencyCost<consistencyCost)
            maxConsistencyCost = consistencyCost;

        costVector[trajectoryNum][0] = staticSafeCost;
        costVector[trajectoryNum][1] = deviationCost;
        costVector[trajectoryNum][2] = curvatureCost;
        costVector[trajectoryNum][3] = consistencyCost;

    }

    for(int trajectoryNum=0; trajectoryNum<costVector.size(); ++trajectoryNum)
    {
        double staticSafeCost = 0.0;
        double deviationCost  = 0.0;
        double curvatureCost  = 0.0;
        double consistencyCost = 0.0;

        if(maxStaticSafesCost>1e-6)
            staticSafeCost   = costVector[trajectoryNum][0]/maxStaticSafesCost;

        if(maxDeviationCost>1e-6)
            deviationCost    = costVector[trajectoryNum][1]/maxDeviationCost;
        
        if(maxCurvatureCost>1e-6)
            curvatureCost    = costVector[trajectoryNum][2]/maxCurvatureCost;
        
        if(maxConsistencyCost>1e-6)
            consistencyCost  = costVector[trajectoryNum][3]/maxConsistencyCost;

        std::cout << "staticSafe  Cost: " << staticSafeCost << std::endl;
        std::cout << "Deviation   Cost: " << deviationCost << std::endl;
        std::cout << "Curvature   Cost: " << curvatureCost << std::endl;
        std::cout << "consistency Cost: " << consistencyCost << std::endl;
        std::cout << "Is Collide: " << trajectories[trajectoryNum].isCollide_ << std::endl;

        double cost = (wd_*deviationCost+wc_*(0.7*consistencyCost+0.3*curvatureCost)+ws_*staticSafeCost)/trajectories[trajectoryNum].length_;

        std::cout << "Total Cost: " << cost << std::endl;
        std::cout << "------------------------------------" << std::endl;
        trajectories[trajectoryNum].setPathCost(cost);
    }
    
}

TrajectoryPoint LatticePlanner::calcNearestReferencePoint(TrajectoryPoint &current,
                                                          ReferenceTrajectoryProcessor& rtprocessor)
{
    double minDistance = std::numeric_limits<double>::max();
    int minId=0;

    for(int referencePointNum = 0; referencePointNum<rtprocessor.trajectory_.size(); ++referencePointNum)
    {
        double dx = current.x_ - rtprocessor.trajectory_[referencePointNum].x_;
        double dy = current.y_ - rtprocessor.trajectory_[referencePointNum].y_;
        double distance = dx*dx + dy*dy;
        if(distance < minDistance)
        {
            minDistance = distance;
            minId= referencePointNum;
        }
    }

    //現在位置に最も近い点がsとなる
    current.s_ = rtprocessor.trajectory_[minId].s_;
    //calc l_
    double rx = rtprocessor.trajectory_[minId].x_;
    double ry = rtprocessor.trajectory_[minId].y_;
    double ryaw = rtprocessor.trajectory_[minId].yaw_;
    double snew, lnew;
    cartesian_to_frenet(current.s_, rx, ry, ryaw, current.x_, current.y_, &snew, &lnew);
    current.l_ = lnew;
    //id
    current.id_ = minId;

    //get nearest center line ahead of d
    double distanceS=0.0;
    while(true)
    {
        distanceS += (rtprocessor.trajectory_[minId+1].s_ - rtprocessor.trajectory_[minId].s_);
        if(distanceS > d_ || minId>rtprocessor.trajectory_.size()-2)
            break;
        else
            minId++;
    }

    return rtprocessor.trajectory_[minId];
}

std::vector<TrajectoryPoint> LatticePlanner::samplePointsFromNearestPoint(const TrajectoryPoint& nearestPoint)
{
    std::vector<TrajectoryPoint> samplePoints;
    for (double len = -sampleLateralLength_; len <= sampleLateralLength_; len += dSampleLength_)
   {
        double xnew = -len * sin(nearestPoint.yaw_) + nearestPoint.x_;
        double ynew = len * cos(nearestPoint.yaw_) + nearestPoint.y_;
        double yawnew = nearestPoint.yaw_;
        double snew = nearestPoint.s_;
        double lnew = len;

        double curvaturenew=0.0;
        if(nearestPoint.curvature_>1e-6)
            curvaturenew = 1/((1/nearestPoint.curvature_) - lnew);

        curvaturenew = std::max(min_curvature_, std::min(curvaturenew, max_curvature_));

        int idnew = nearestPoint.id_;
        samplePoints.emplace_back(TrajectoryPoint{xnew, ynew, yawnew, curvaturenew, snew, lnew, idnew});
    }

    return samplePoints;
}

bool LatticePlanner::generateTrajectory(const TrajectoryPoint& current, 
                                        const std::vector<TrajectoryPoint>& sampledPoints, 
                                        const std::vector<double>& h, 
                                        const int maxIteration, 
                                        const double costThreshold,
                                        ReferenceTrajectoryProcessor& rtprocessor, 
                                        std::vector<Trajectory>& trajectories)
{
    for(TrajectoryPoint target : sampledPoints)
    {
        //ここに必要な値をテーブルから持ってくるようにする
        PolynomialSplineModelParameter parameter;
        parameter.curvature_[0] = current.curvature_;
        parameter.curvature_[3] = target.curvature_;
        parameter.distance_ = std::sqrt(std::pow(current.x_-target.x_, 2)+std::pow(current.y_-target.y_, 2));

        //最適な経路を獲得する
        Trajectory traj = optimizer_.getOptimizeTrajectory(parameter, current, target, h, rtprocessor);
        if(traj.trajectoryPoints_.empty())
            std::cout << "Cannnot create the path" << std::endl;
        else
        {
            trajectories.push_back(traj);
        }
    }

    return !trajectories.empty();
}