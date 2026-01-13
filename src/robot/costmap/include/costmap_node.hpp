#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


#include <vector>

#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    void publishMessage();
    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void initializeGrid();
    void convertWorldToGrid(double x_world, double y_world, int& x, int& y);
    void markObstacle(int x, int y);
    void inflateObstacles();
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_msg_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr string_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::vector<float>> OccupancyGrid; // dynamic size
    double resolution = 0.1; // meters per cell
    int grid_size = 300; // number of cells per side assuming square (300m default)

    // Inflation parameters
    double inflation_radius = 1.5; // meters
    float max_inflation_cost = 100.0f; // max cost indicate confidently occupied space

    double origin_x = 0.0;
    double origin_y = 0.0;
};
#endif 