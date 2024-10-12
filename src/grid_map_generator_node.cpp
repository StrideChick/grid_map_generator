#include <grid_map_generator/grid_map_generator.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<grid_map_generator::GridMapGenerator>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}