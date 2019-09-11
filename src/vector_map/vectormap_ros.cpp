#include <cmath>
#include <algorithm>
#include <ros/ros.h>


#include "lattice_planner/vector_map/vectormap_ros.h"

bool VectorMap::load()
{
  auto points = ros::topic::waitForMessage<vector_map_msgs::PointArray>("/vector_map_info/point", ros::Duration());
  if(!points)
  {
      return false;
  }

  auto dtlanes = ros::topic::waitForMessage<vector_map_msgs::DTLaneArray>("/vector_map_info/dtlane", ros::Duration());
  if(!dtlanes)
  {
      return false;
  }

  auto nodes = ros::topic::waitForMessage<vector_map_msgs::NodeArray>("/vector_map_info/node", ros::Duration());
  if(!nodes)
  {
      return false;
  }

  auto lanes = ros::topic::waitForMessage<vector_map_msgs::LaneArray>("/vector_map_info/lane", ros::Duration());
  if(!lanes)
  {
      return false;
  }

  std::unordered_map<int, vector_map_msgs::Point> points_map;
  for(const auto& point : points->data)
  {
      points_map[point.pid] = point;
  }

  std::unordered_map<int, vector_map_msgs::DTLane> dtlanes_map;
  for(const auto& dtlane : dtlanes->data)
  {
    dtlanes_map[dtlane.did] = dtlane;
  }

  std::unordered_map<int, vector_map_msgs::Node> nodes_map;
  for(const auto& node : nodes->data)
  {
    nodes_map[node.nid] = node;
  }

  std::unordered_map<int, vector_map_msgs::Lane> lanes_map;
  for(const auto& lane : lanes->data)
  {
    lanes_map[lane.lnid] = lane;
  }
  
  
  for (const auto& lane: lanes->data)
  {
    Point lane_point;
    const auto& node = nodes_map[lane.bnid];
    const auto& point = points_map[node.pid];
    const auto& dtlane = dtlanes_map[lane.did];
    lane_point.tx = point.ly;
    lane_point.ty = point.bx;
    lane_point.rz = dtlane.dir;
    lane_point.cumulated_s = 0;
    lane_point.curvature = 1/dtlane.r;
    
    if(lane.blid != 0)
    {
      const auto& previous_lane = lanes_map[lane.blid];
      const auto& previous_dtlane = dtlanes_map[previous_lane.did];
      const auto& previous_node = nodes_map[previous_lane.bnid];
      const auto& previous_point = points_map[previous_node.pid];
      double dx = point.ly - previous_point.ly;
      double dy = point.bx - previous_point.bx;
      double delta_distance = std::sqrt(std::pow(dx, 2)+ std::pow(dy, 2));
      double delta_curvature = 1/dtlane.r - 1/previous_dtlane.r;
      lane_point.curvature_dot = delta_curvature/delta_distance;
      // std::cerr << "distance1 " << delta_distance << std::endl;
      // std::cerr << "delta_curvarute1 " << delta_curvature << std::endl;
      // std::cerr << "delta_curvarute_dot1 " << lane_point.curvature_dot << std::endl;
    }
    else
    {
      lane_point.curvature_dot = 0;
    }
    
    
    size_t current_lane_id = lane.blid;
    while(current_lane_id != 0)
    {
      const auto& current_lane = lanes_map[current_lane_id];
      const auto& forward_node = nodes_map[current_lane.fnid];
      const auto& forward_point = points_map[forward_node.pid];
      const auto& backward_node = nodes_map[current_lane.bnid];
      const auto& backward_point = points_map[backward_node.pid];
      
      double dx = forward_point.ly - backward_point.ly;
      double dy = forward_point.bx - backward_point.bx;
      double distance = std::sqrt(std::pow(dx, 2)+ std::pow(dy, 2));
      lane_point.cumulated_s += distance;
      
      current_lane_id = current_lane.blid;
      
    }
    
    // push back the first lane point
    lane_point.points.push_back(lane_point);
    const int NUM_SEARCH_POINT = 300;
    // TODO lane.flid sanity check 
    auto next_lane = lanes_map[lane.flid];
    for (size_t i = 0; i < NUM_SEARCH_POINT; i++)
    {
      const auto& node = nodes_map[next_lane.bnid];
      const auto& point = points_map[node.pid];
      const auto& dtlane = dtlanes_map[next_lane.did];
      Point next_lane_point;
      next_lane_point.tx = point.ly;
      next_lane_point.ty = point.bx;
      next_lane_point.rz = dtlane.dir;
      next_lane_point.curvature = 1/dtlane.r;
      
      Point previous_lane_point = lane_point.points.back();
      double dx = next_lane_point.tx - previous_lane_point.tx;
      double dy = next_lane_point.ty - previous_lane_point.ty;
      double distance = std::sqrt(std::pow(dx, 2)+ std::pow(dy, 2));
      next_lane_point.cumulated_s = previous_lane_point.cumulated_s + distance;
      
      const auto& previous_lane = lanes_map[next_lane.blid];
      const auto& previous_dtlane = dtlanes_map[previous_lane.did];
      double delta_curvature = 1/dtlane.r - 1/previous_dtlane.r;
      next_lane_point.curvature_dot = delta_curvature/distance;
      // std::cerr << "distance2 " << distance << std::endl;
      // std::cerr << "curvature2 " << delta_curvature << std::endl;
      // std::cerr << "curvature_dot2 " << next_lane_point.curvature_dot << std::endl;
      
      
      lane_point.points.push_back(next_lane_point);
      if(next_lane.flid != 0)
      {
        next_lane = lanes_map[next_lane.flid];
      }
      else
      {
        break;
      }
    }
    points_.push_back(lane_point);
    
  }
  return true;
}

