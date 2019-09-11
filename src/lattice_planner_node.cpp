/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apacpoints_marker_arrayion 2.0 (the "License");
 * you may not use this file except in compliance with thxpxoe License.
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
#include "lattice_planner/lattice_planner_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frenet_planner");
    LatticePlannerNode node;
    ros::spin();

    return 0;
}

LatticePlannerNode::LatticePlannerNode()
  : nh_()
  , private_nh_("~")
{
  double front_sampling_distance;
  double ds;
  double lateral_offset;
  double lateral_sampling_interval;
  double max_speed;
  double max_accelereation;
  double vehicle_length;
  double vehicle_width;
  double vehicle_wheel_base;
  double vehicle_safety_distance;
  double dt;
  double timer_callback_dt;
  double ws;
  double wd;
  double wc;
  double min_curvature;
  double max_curvature;
  int max_iteration;
  double cost_threshold;
  std::string input_waypoints_topic_name;
  std::string output_waypoints_topic_name;
  std::string lidar_detection_topic_name;

  private_nh_.param<double>("front_sampling_distance", front_sampling_distance, 10.0);
  private_nh_.param<double>("sampling_interval_length", ds, 0.01);
  private_nh_.param<double>("lateral_offset", lateral_offset, 3.0);
  private_nh_.param<double>("lateral_sampling_interval", lateral_sampling_interval, 0.5);
  private_nh_.param<double>("max_speed", max_speed, 13.8);
  private_nh_.param<double>("max_acceleration", max_accelereation, 2.0);
  private_nh_.param<double>("vehicle_length", vehicle_length, 5.0);
  private_nh_.param<double>("vehicle_width", vehicle_width, 1.895);
  private_nh_.param<double>("vehicle_wheel_base", vehicle_wheel_base, 2.790);
  private_nh_.param<double>("vehicle_safety_distance", vehicle_safety_distance, 0.2);
  private_nh_.param<double>("time_interval", dt, 0.2);
  private_nh_.param<double>("planner_callback_time", timer_callback_dt, 0.05);
  private_nh_.param<double>("ws", ws, 0.6);
  private_nh_.param<double>("wd", wd, 0.3);
  private_nh_.param<double>("wc", wc, 0.1);
  private_nh_.param<double>("min_curvature", min_curvature, -0.2);
  private_nh_.param<double>("max_curvature", max_curvature, 0.2);
  private_nh_.param<int>("max_iteration", max_iteration, 100);
  private_nh_.param<double>("cost_threshold", cost_threshold, 0.1);
  private_nh_.param<std::string>("input_waypoints_topic_name", input_waypoints_topic_name, "base_waypoints");
  private_nh_.param<std::string>("output_waypoints_topic_name", output_waypoints_topic_name, "safety_waypoints");
  private_nh_.param<std::string>("lidar_detection_topic_name", lidar_detection_topic_name, "/detection/lidar_detector/objects");


  lattice_planner_ptr_.reset(new LatticePlanner(front_sampling_distance, 
                                                ds, 
                                                lateral_offset,
                                                lateral_sampling_interval,
                                                max_speed,
                                                max_accelereation,
                                                vehicle_length,
                                                vehicle_width,
                                                vehicle_wheel_base,
                                                vehicle_safety_distance,
                                                dt,
                                                ws,
                                                wd,
                                                wc,
                                                min_curvature,
                                                max_curvature,
                                                max_iteration,
                                                cost_threshold));

  vectormap_load_ptr_.reset(new VectorMap());
  loadVectormap();
  
  tf2_buffer_ptr_.reset(new tf2_ros::Buffer());
  tf2_listener_ptr_.reset(new tf2_ros::TransformListener(*tf2_buffer_ptr_));
  
  optimized_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>(output_waypoints_topic_name, 1, true);
  optimized_waypoints_debug_ = nh_.advertise<visualization_msgs::Marker>("optimized_waypoints_debug", 1, true);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frenet_planner_debug_markes", 1, true);
  final_waypoints_sub_ = nh_.subscribe(input_waypoints_topic_name, 1, &LatticePlannerNode::waypointsCallback, this);
  current_pose_sub_ = nh_.subscribe("/current_pose", 1, &LatticePlannerNode::currentPoseCallback, this);
  current_status_sub_ = nh_.subscribe("/vehicle_status", 1, &LatticePlannerNode::currentStatusCallback, this);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &LatticePlannerNode::currentVelocityCallback, this);
  nav_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &LatticePlannerNode::navGoalCallback, this);
  objects_sub_ = nh_.subscribe(lidar_detection_topic_name, 1, &LatticePlannerNode::objectsCallback, this);
  timer_ = nh_.createTimer(ros::Duration(timer_callback_dt), &LatticePlannerNode::timerCallback, this);
}

void LatticePlannerNode::waypointsCallback(const autoware_msgs::Lane& msg)
{
  in_lane_ptr_.reset(new autoware_msgs::Lane(msg));
}

void LatticePlannerNode::currentPoseCallback(const geometry_msgs::PoseStamped & msg)
{
  in_pose_ptr_.reset(new geometry_msgs::PoseStamped(msg));
}

void LatticePlannerNode::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  
  in_twist_ptr_.reset(new geometry_msgs::TwistStamped(msg));
}

void LatticePlannerNode::currentStatusCallback(const autoware_msgs::VehicleStatus& msg)
{
  in_status_ptr_.reset(new autoware_msgs::VehicleStatus(msg));
}

void LatticePlannerNode::navGoalCallback(const geometry_msgs::PoseStamped& msg)
{
  std::string target_frame = "map";
  std::string source_frame = "world";
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf2_buffer_ptr_->lookupTransform(target_frame, source_frame, ros::Time(0));
    geometry_msgs::PoseStamped msg_in_map;
    tf2::doTransform(msg, msg_in_map, transform);
    in_nav_goal_ptr_.reset(new geometry_msgs::PoseStamped(msg_in_map));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void LatticePlannerNode::timerCallback(const ros::TimerEvent &e)
{
  
  //if(in_pose_ptr_ && in_twist_ptr_ && in_nav_goal_ptr_ && in_lane_ptr_) 
  if(in_pose_ptr_ && in_twist_ptr_ && in_lane_ptr_) 
  {
    std::vector<Trajectory> primitives;
    Trajectory bestTrajectory;

    double current_x = in_pose_ptr_->pose.position.x;
    double current_y = in_pose_ptr_->pose.position.y;
    double yaw = tf::getYaw(in_pose_ptr_->pose.orientation);
    double rotational_velocity = in_twist_ptr_ ->twist.angular.z;
    double linear_velocity = std::sqrt(std::pow(in_twist_ptr_ ->twist.linear.x, 2)+std::pow(in_twist_ptr_->twist.linear.y, 2));
    double curvature = (2*tan(in_status_ptr_->angle))/lattice_planner_ptr_->egoVehicleInfoPtr_->wheelBase_;
    curvature =std::max(-0.4, std::min(curvature, 0.4));

    TrajectoryPoint currentPosition(current_x, current_y, yaw, curvature);

    //get reference path from nearest point
    std::vector<double> rx;
    std::vector<double> ry;
    size_t closestId= -1;
    if(!in_lane_ptr_->waypoints.empty())
    {
      double min_dist = std::numeric_limits<double>::max();
      for(size_t i=0; i<in_lane_ptr_->waypoints.size(); ++i)
      {
        double dx = in_lane_ptr_->waypoints[i].pose.pose.position.x - in_pose_ptr_->pose.position.x;
        double dy = in_lane_ptr_->waypoints[i].pose.pose.position.y - in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        if(distance < min_dist)
        {
          min_dist = distance;
          closestId = i;
        }
      }

      rx.push_back(in_lane_ptr_->waypoints[closestId].pose.pose.position.x);
      ry.push_back(in_lane_ptr_->waypoints[closestId].pose.pose.position.y);
      double len=0.0;
      for(size_t i=closestId+1; i<in_lane_ptr_->waypoints.size(); ++i)
      {
        rx.push_back(in_lane_ptr_->waypoints[i].pose.pose.position.x);
        ry.push_back(in_lane_ptr_->waypoints[i].pose.pose.position.y);

        double dx = rx[i-closestId]-rx[i-1-closestId];
        double dy = ry[i-closestId]-ry[i-1-closestId];
        len+=std::sqrt(dx*dx+dy*dy);

        if(len>lattice_planner_ptr_->d_+10)
          break;
      }
    }
    else
      return;

    ReferenceTrajectoryProcessor rtprocessor(rx, ry);

    std::vector<Obstacle> obstacles;
    if(in_objects_ptr_)
    {
      for(int i=0; i<in_objects_ptr_->objects.size(); ++i)
      {
        Obstacle tmp;
        tmp.x_ = in_objects_ptr_->objects[i].pose.position.x;
        tmp.y_ = in_objects_ptr_->objects[i].pose.position.y;
        tmp.radius_ = std::sqrt(std::pow(in_objects_ptr_->objects[i].dimensions.x, 2) + std::pow(in_objects_ptr_->objects[i].dimensions.y, 2));
        tmp.translational_velocity_ = in_objects_ptr_->objects[i].velocity.linear.x;
        obstacles.push_back(tmp);
      }
    }

    ROS_INFO("#########################################");
    ROS_INFO("#########################################");
    ROS_INFO("Currnet x:         %f", currentPosition.x_);
    ROS_INFO("Currnet y:         %f", currentPosition.y_);
    ROS_INFO("Currnet yaw:       %f", currentPosition.yaw_);
    ROS_INFO("Currnet curvature: %f", currentPosition.curvature_);
    ROS_INFO("#########################################");
    ROS_INFO("#########################################");

    lattice_planner_ptr_->doPlan(currentPosition, obstacles, rtprocessor, bestTrajectory, primitives);

    autoware_msgs::Lane bestLane;
    bestLane.lane_id = in_lane_ptr_->lane_id;
    bestLane.lane_index = in_lane_ptr_->lane_index;
    bestLane.is_blocked = in_lane_ptr_->is_blocked;
    bestLane.increment = in_lane_ptr_->increment;
    bestLane.header = in_lane_ptr_->header;
    bestLane.cost = in_lane_ptr_->cost;
    bestLane.closest_object_distance = in_lane_ptr_->closest_object_distance;
    bestLane.closest_object_velocity = in_lane_ptr_->closest_object_velocity;
    bestLane.waypoints.reserve(bestTrajectory.trajectoryPoints_.size());

    for(int i=0; i<bestTrajectory.trajectoryPoints_.size(); i+=100)
    {
        autoware_msgs::Waypoint waypoint;
        waypoint.pose.pose.position.x = bestTrajectory.trajectoryPoints_[i].x_;
        waypoint.pose.pose.position.y = bestTrajectory.trajectoryPoints_[i].y_;
        waypoint.pose.pose.position.z = in_lane_ptr_->waypoints[0].pose.pose.position.z;
        waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(bestTrajectory.trajectoryPoints_[i].yaw_);
        waypoint.pose.header = in_lane_ptr_->header;

        //暫定
        //waypoint.twist = in_lane_ptr_->waypoints[0].twist;
        waypoint.twist.twist.linear.x = 1;

        bestLane.waypoints.push_back(waypoint);
    }

    //currently relay waypoints
    //optimized_waypoints_pub_.publish(*in_lane_ptr_);
    optimized_waypoints_pub_.publish(bestLane);

    //debug
    visualization_msgs::MarkerArray points_marker_array;
    int unique_id = 0;
    visualization_msgs::Marker reference_lane_points_marker;
    reference_lane_points_marker.lifetime = ros::Duration(0.2);
    reference_lane_points_marker.header = in_pose_ptr_->header;
    reference_lane_points_marker.ns = std::string("reference_lane_points_marker");
    reference_lane_points_marker.action = visualization_msgs::Marker::MODIFY;
    reference_lane_points_marker.pose.orientation.w = 1.0;
    reference_lane_points_marker.id = unique_id;
    reference_lane_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    reference_lane_points_marker.scale.x = 0.5;

    // Points are yellow
    reference_lane_points_marker.color.r = 1.0f;
    reference_lane_points_marker.color.g = 1.0f;
    reference_lane_points_marker.color.a = 1;
    for (size_t i=closestId; i<in_lane_ptr_->waypoints.size(); ++i)
    {
      geometry_msgs::Point geometry_point;
      geometry_point.x = in_lane_ptr_->waypoints[i].pose.pose.position.x;
      geometry_point.y = in_lane_ptr_->waypoints[i].pose.pose.position.y;
      geometry_point.z = in_lane_ptr_->waypoints[i].pose.pose.position.z;
      reference_lane_points_marker.points.push_back(geometry_point);

      if(i-closestId>30)
        break;
    }
    points_marker_array.markers.push_back(reference_lane_points_marker);
    unique_id++;

    for(int i=0; i<primitives.size(); ++i)
    {
        visualization_msgs::Marker primitive_maker;
        primitive_maker.lifetime = ros::Duration(0.2);
        primitive_maker.header = in_pose_ptr_->header;
        primitive_maker.ns = std::string("trajectory_marker");
        primitive_maker.action = visualization_msgs::Marker::MODIFY;
        primitive_maker.pose.orientation.w = 1.0;
        primitive_maker.id = unique_id;
        primitive_maker.type = visualization_msgs::Marker::SPHERE_LIST;
        primitive_maker.scale.x = 0.2;

        // Points are green
        primitive_maker.color.g = 1.0f;
        primitive_maker.color.a = 0.5;
        for(int j=0; j<primitives[i].trajectoryPoints_.size(); j+=100)
        {
            geometry_msgs::Point geometry_point;
            geometry_point.x = primitives[i].trajectoryPoints_[j].x_;
            geometry_point.y = primitives[i].trajectoryPoints_[j].y_;
            geometry_point.z = in_pose_ptr_->pose.position.z;
            primitive_maker.points.push_back(geometry_point);
        }
        points_marker_array.markers.push_back(primitive_maker);
        unique_id++;
    }

    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.lifetime = ros::Duration(0.2);
    trajectory_marker.header = in_pose_ptr_->header;
    trajectory_marker.ns = std::string("best_trajectory_marker");
    trajectory_marker.action = visualization_msgs::Marker::MODIFY;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.id = 0;
    trajectory_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    trajectory_marker.scale.x = 0.5;

    // Points are red
    trajectory_marker.color.r = 1.0f;
    trajectory_marker.color.a = 1.0;
    for(int i=0; i<bestTrajectory.trajectoryPoints_.size(); i+=100)
    {
      geometry_msgs::Point geometry_point;
      geometry_point.x = bestTrajectory.trajectoryPoints_[i].x_;
      geometry_point.y = bestTrajectory.trajectoryPoints_[i].y_;
      geometry_point.z = in_pose_ptr_->pose.position.z;
      trajectory_marker.points.push_back(geometry_point);
    }
    optimized_waypoints_debug_.publish(trajectory_marker);
    
    markers_pub_.publish(points_marker_array);
  }
}

void LatticePlannerNode::loadVectormap()
{
  std::cout << 111 << std::endl;
  //vectormap_load_ptr_->load();
  std::cout << 222 << std::endl;
}

void FrenetPlannerROpoints_marker_array()
{
  
}

void LatticePlannerNode::objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  if(in_lane_ptr_)
  {
    if(msg.objects.size() == 0)
    {
      std::cerr << "ssize of objects is 0" << std::endl;
      return;
    }
    geometry_msgs::TransformStamped objects2map_tf;
    try
    {
        objects2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_lane_ptr_->header.frame_id, 
          /*src*/ msg.header.frame_id,
          ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    in_objects_ptr_.reset(new autoware_msgs::DetectedObjectArray(msg));
    in_objects_ptr_->header.frame_id = in_lane_ptr_->header.frame_id;
    for(auto& object: in_objects_ptr_->objects)
    {
      object.header.frame_id = in_lane_ptr_->header.frame_id;
      geometry_msgs::PoseStamped current_object_pose;
      current_object_pose.header = object.header;
      current_object_pose.pose = object.pose;
      geometry_msgs::PoseStamped transformed_pose;
      tf2::doTransform(current_object_pose, transformed_pose, objects2map_tf);
      object.pose = transformed_pose.pose;
    }
  }
}
