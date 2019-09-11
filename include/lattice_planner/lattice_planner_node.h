/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FRENET_PLANNER_ROS_H
#define FRENET_PLANNER_ROS_H

#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <random>
#include <chrono>
#include <cstring>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "lattice_planner/planner/lattice_planner.h"
#include "lattice_planner/trajectory/reference_trajectory_processor.h"
#include "lattice_planner/vector_map/vectormap_ros.h"
#include "lattice_planner/trajectory/trajectory.h"
#include "lattice_planner/vehicle_info/vehicle_info.h"

namespace tf2_ros{
  class Buffer;
  class TransformListener;
}

class LatticePlannerNode 
{
public:

  LatticePlannerNode();
  ~LatticePlannerNode()=default;

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher optimized_waypoints_pub_;
  ros::Publisher optimized_waypoints_debug_;
  ros::Publisher markers_pub_;
  ros::Subscriber current_status_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber final_waypoints_sub_;
  ros::Subscriber nav_goal_sub_;
  ros::Subscriber objects_sub_;
  
  ros::Timer timer_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_ptr_;
  
  std::unique_ptr<autoware_msgs::Lane> in_lane_ptr_;
  std::unique_ptr<autoware_msgs::VehicleStatus> in_status_ptr_;
  std::unique_ptr<geometry_msgs::PoseStamped> in_pose_ptr_;
  std::unique_ptr<geometry_msgs::TwistStamped> in_twist_ptr_;
  std::unique_ptr<geometry_msgs::PoseStamped> in_nav_goal_ptr_;
  std::unique_ptr<autoware_msgs::DetectedObjectArray> in_objects_ptr_;
  
  std::unique_ptr<LatticePlanner> lattice_planner_ptr_;
  std::unique_ptr<VectorMap> vectormap_load_ptr_;

  void waypointsCallback(const autoware_msgs::Lane& msg);
  void objectsCallback(const autoware_msgs::DetectedObjectArray& msg);
  void currentStatusCallback(const autoware_msgs::VehicleStatus& msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  void navGoalCallback(const geometry_msgs::PoseStamped& msg);
  void timerCallback(const ros::TimerEvent &e);
  void loadVectormap();
};

#endif
