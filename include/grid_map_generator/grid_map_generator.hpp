#ifndef MAP_GENERATOR_HPP_
#define MAP_GENERATOR_HPP_

// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

// ROS package
#include <grid_map_generator/base_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>


namespace grid_map_generator{
class GridMapGenerator : public rclcpp::Node
{
public:
  explicit GridMapGenerator(const rclcpp::NodeOptions & options);
private:
  std::string grid_map_topic ;
  std::string point_cloud_topic;
  void setup_grid_map();
  void pc_callback(const sensor_msgs::msg::PointCloud2 & msg);
  void timer_callback();
  void update_occupancy_map();
  
  //ros    
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;  
  rclcpp::TimerBase::SharedPtr timer_;
  GridMap grid_map;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Occupancy tracking
  std::vector<std::vector<rclcpp::Time>> last_observed_times_;
  nav_msgs::msg::OccupancyGrid persistent_grid_;
  double observation_timeout_;  // 5 seconds
};
}
#endif