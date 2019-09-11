#ifndef STATELATTICEPLANNER_LATTICE_PLANNER_H
#define STATELATTICEPLANNER_LATTICE_PLANNER_H

#include <cassert>
#include <vector>
#include <Eigen/Eigen>
#include <algorithm>
#include <queue>
#include <memory>
#include <array>
#include "trajectory_generator.h"
#include "lattice_planner/obstacle/obstacle.h"
#include "lattice_planner/model/polynomial_spline_model.h"
#include "lattice_planner/vehicle_info/vehicle_info.h"

class LatticePlanner
{
public:
    LatticePlanner(double d, 
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
                   double costThreshold);

    ~LatticePlanner() = default;

    void doPlan(TrajectoryPoint& currentState, 
                const std::vector<Obstacle>& obstacles, 
                ReferenceTrajectoryProcessor& rtprocessor, 
                Trajectory& bestTrajectory, 
                std::vector<Trajectory>& primitives, 
                const int maxIteration=100, 
                const double costThreshold=0.1);

    TrajectoryGenerator optimizer_;
    std::shared_ptr<VehicleInfo> egoVehicleInfoPtr_;
    double d_;        //ターゲットまでの距離
    const double sampleLateralLength_;
    const double dSampleLength_;
    double maxSpeed_;
    double maxAccel_;
    const double dt_;
    const double ws_;
    const double wd_;
    const double wc_;
    const double min_curvature_;
    const double max_curvature_;
    const double sdv_;

private:
    bool checkCollision(Trajectory& trajectory, const std::vector<Obstacle>& obstacles); //check collision

    //get No Collision Path
    Trajectory getSafestPath(std::vector<Trajectory>& trajectories);

    //calculation of path cost
    void calcPathCost(std::vector<Trajectory>& trajectories,
                        const TrajectoryPoint& nearestReferencePoint,
                        const std::vector<Obstacle>& obstacles,
                        ReferenceTrajectoryProcessor& rtprocessor);

    //get nearest reference point
    TrajectoryPoint calcNearestReferencePoint(TrajectoryPoint& current,
                                              ReferenceTrajectoryProcessor& rtprocessor);

    //sample points from nearest point
    std::vector<TrajectoryPoint> samplePointsFromNearestPoint(const TrajectoryPoint& nearestPoint);

    //generate trajectory
    bool generateTrajectory(const TrajectoryPoint& current, 
                            const std::vector<TrajectoryPoint>& sampledPoints, 
                            const std::vector<double>& h, 
                            const int maxIteration ,
                            const double costThreshold,
                            ReferenceTrajectoryProcessor& rtprocessor, 
                            std::vector<Trajectory>& trajectories);

    //previous Best Path
    std::shared_ptr<Trajectory> previousBestTrajectory_;
    std::shared_ptr<std::array<double, 2>> previousFrontReferencePoint_;

private:
    double calcStaticSafeCost(const int currentPathId, const std::vector<Trajectory>& trajectories);
    double calcConsistencyCost(const Trajectory& trajectory, ReferenceTrajectoryProcessor& rtprocessor);

};


#endif //STATELATTICEPLANNER_LATTICE_PLANNER_H
