#include <grid_map_generator/grid_map_generator.hpp>

namespace grid_map_generator
{
GridMapGenerator::GridMapGenerator(const rclcpp::NodeOptions & node_options)
: Node("grid_map_generator", node_options), observation_timeout_(5.0)
{ 
  declare_parameter<std::string>("map_topic_name","/local_map");
  declare_parameter<std::string>("cloud_topic_name","/lidar_points");
  declare_parameter<std::string>("map_frame","map");
  declare_parameter<double>("observation_timeout", 5.0);
  declare_parameter<double>("update_frequency", 10.0);
  
  get_parameter("map_topic_name",grid_map_topic);
  get_parameter("cloud_topic_name",point_cloud_topic);
  get_parameter("map_frame",grid_map.map_frame);
  get_parameter("observation_timeout", observation_timeout_);
  
  double update_frequency;
  get_parameter("update_frequency", update_frequency);

  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map_topic, 10);
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic, 10, std::bind(&GridMapGenerator::pc_callback, this, std::placeholders::_1)
  );
  
  // Create timer for periodic updates
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency)),
    std::bind(&GridMapGenerator::timer_callback, this)
  );
  
  setup_grid_map();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize persistent grid and observation tracking
  grid_map.initGrid(persistent_grid_);
  persistent_grid_.header.frame_id = grid_map.map_frame;
  persistent_grid_.data.resize(grid_map.getSize(), -1);
  
  // Initialize observation time tracking
  last_observed_times_.resize(grid_map.grid_height);
  for (auto& row : last_observed_times_) {
    row.resize(grid_map.grid_width, rclcpp::Time(0, 0, RCL_ROS_TIME));
  }
}

void GridMapGenerator::setup_grid_map()
{
  declare_parameter<float>("resolution", 0.01);
  declare_parameter<float>("length_x", 10.0);
  declare_parameter<float>("length_y", 10.0);
  declare_parameter<float>("origin_position_x", -5.0);
  declare_parameter<float>("origin_position_y", -5.0);
  declare_parameter<float>("origin_position_z", 0.0);
  declare_parameter<float>("origin_orientation_w", 1.0);
  declare_parameter<float>("origin_orientation_x", 0.0);
  declare_parameter<float>("origin_orientation_y", 0.0);
  declare_parameter<float>("origin_orientation_z", 0.0);

  get_parameter("resolution", grid_map.grid_resolution);
  get_parameter("length_x", grid_map.length_x);
  get_parameter("length_y", grid_map.length_y);
  get_parameter("origin_position_x", grid_map.origin_position_x);
  get_parameter("origin_position_y", grid_map.origin_position_y);
  get_parameter("origin_position_z", grid_map.origin_position_z); 
  get_parameter("origin_orientation_w", grid_map.origin_orientation_w); 
  get_parameter("origin_orientation_x", grid_map.origin_orientation_x); 
  get_parameter("origin_orientation_y", grid_map.origin_orientation_y); 
  get_parameter("origin_orientation_z", grid_map.origin_orientation_z);  
  grid_map.set_param();
}

void GridMapGenerator::pc_callback(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  try
  {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform("map", msg.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(msg, transformed_cloud, transform_stamped);
  }
  catch (tf2::TransformException &ex)
  {
      RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
      return;
  }
  
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(transformed_cloud, cloud);
  
  rclcpp::Time current_time = this->get_clock()->now();

  // Update observation times for each point
  for (const auto& point : cloud) {
    if (point.x >= grid_map.bottomright_x && point.x < grid_map.topleft_x &&
        point.y >= grid_map.bottomright_y && point.y < grid_map.topleft_y
    ){
      int grid_x = static_cast<int>((point.x - grid_map.bottomright_x) / grid_map.grid_resolution);
      int grid_y = static_cast<int>((point.y - grid_map.bottomright_y) / grid_map.grid_resolution);
      
      if (grid_x >= 0 && grid_x < grid_map.grid_width && 
          grid_y >= 0 && grid_y < grid_map.grid_height) {
        last_observed_times_[grid_y][grid_x] = current_time;
      }
    }
  }
}

void GridMapGenerator::timer_callback()
{
  update_occupancy_map();
  persistent_grid_.header.stamp = this->get_clock()->now();
  grid_pub_->publish(persistent_grid_);
}

void GridMapGenerator::update_occupancy_map()
{
  rclcpp::Time current_time = this->get_clock()->now();
  
  for (int y = 0; y < grid_map.grid_height; ++y) {
    for (int x = 0; x < grid_map.grid_width; ++x) {
      int index = y * grid_map.grid_width + x;
      
      // Check if this cell has been observed recently
      rclcpp::Duration time_since_observation = current_time - last_observed_times_[y][x];
      
      if (time_since_observation.seconds() <= observation_timeout_ && 
          last_observed_times_[y][x].seconds() > 0) {
        // Cell has been observed within the timeout period
        persistent_grid_.data[index] = 100;  // Set to occupied
      } else if (last_observed_times_[y][x].seconds() > 0) {
        // Cell was observed before but not recently
        persistent_grid_.data[index] = 0;    // Set to free
      } else {
        // Cell has never been observed
        persistent_grid_.data[index] = -1;   // Unknown
      }
    }
  }
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(grid_map_generator::GridMapGenerator)