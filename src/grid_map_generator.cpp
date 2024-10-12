#include <grid_map_generator/grid_map_generator.hpp>

namespace grid_map_generator
{
GridMapGenerator::GridMapGenerator(const rclcpp::NodeOptions & node_options)
: Node("grid_map_generator", node_options)
{ 
  declare_parameter<std::string>("map_topic_name","/local_map");
  declare_parameter<std::string>("cloud_topic_name","/lidar_points");
  declare_parameter<std::string>("map_frame","map");
  get_parameter("map_topic_name",grid_map_topic);
  get_parameter("cloud_topic_name",point_cloud_topic);
  get_parameter("map_frame",grid_map.map_frame);

  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map_topic, 10);
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic, 10, std::bind(&GridMapGenerator::pc_callback, this, std::placeholders::_1)
  );
  setup_grid_map();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
  
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  grid_map.initGrid(occupancy_grid);
  occupancy_grid.header.stamp = msg.header.stamp;
  occupancy_grid.header.frame_id = grid_map.map_frame;
  occupancy_grid.data.resize(grid_map.getSize(),-1); 
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(transformed_cloud, cloud);

  for (const auto& point : cloud) {
    if (point.x >= grid_map.bottomright_x && point.x < grid_map.topleft_x &&
        point.y >= grid_map.bottomright_y && point.y < grid_map.topleft_y
    ){
      int grid_x = static_cast<int>((point.x - grid_map.bottomright_x) / grid_map.grid_resolution);
      int grid_y = static_cast<int>((point.y - grid_map.bottomright_y) / grid_map.grid_resolution);
      int index = grid_y * grid_map.grid_width + grid_x;
      occupancy_grid.data[index] = std::min(100,occupancy_grid.data[index]+5); 
    }
  }
  grid_pub_->publish(occupancy_grid);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(grid_map_generator::GridMapGenerator)