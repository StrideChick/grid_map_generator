#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <memory>

class GridMap
{
public:
  float grid_resolution;
  float length_x;
  float length_y; 
  int grid_width;
  int grid_height;

  float origin_position_x;
  float origin_position_y;
  float origin_position_z;
  float origin_orientation_w;
  float origin_orientation_x;
  float origin_orientation_y;
  float origin_orientation_z;

  float topleft_x;
  float topleft_y;
  float bottomright_x;
  float bottomright_y;

  int cell_num_x;
  int cell_num_y;

  std::string map_frame;

  void initGrid(nav_msgs::msg::OccupancyGrid & grid)
  {
    // Header
    grid.header.frame_id = map_frame;

    //MapMetaData
    grid.info.resolution = grid_resolution; //[m/cell]
    grid.info.width = length_x / grid_resolution;
    grid.info.height = length_y / grid_resolution;
    grid.info.origin.position.x = origin_position_x;
    grid.info.origin.position.y = origin_position_y;
    grid.info.origin.position.z = origin_position_z;
    grid.info.origin.orientation.w = origin_orientation_w;
    grid.info.origin.orientation.x = origin_orientation_x;
    grid.info.origin.orientation.y = origin_orientation_y;
    grid.info.origin.orientation.z = origin_orientation_z;
  }

  void set_param()
  {
    topleft_x     = int(length_x / 2);
    bottomright_x = int(-(length_x / 2));
    topleft_y     = int(length_y / 2);
    bottomright_y = int(-(length_y / 2));
    grid_width    = int(length_x / grid_resolution);
    grid_height   = int(length_y / grid_resolution);
  }

  int getSize(){return grid_width * grid_height;}  // grid_size   [cell]
  int getSizeX(){return grid_width;}               // grid_width  [cell]
  int getSizeY(){return grid_height;}              // grid_height [cell]
  double getLengthX(){return length_x;}            // grid_width  [m]
  double getLengthY(){return length_y;}            // grid_height [m] 
  double getResolution(){return grid_resolution;}  // grid__resolution [m/cell]
};