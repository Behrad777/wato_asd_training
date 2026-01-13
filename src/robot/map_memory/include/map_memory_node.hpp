#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"


class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    
  private:
    robot::MapMemoryCore map_memory_;

    //subcriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    //publish
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y, last_yaw;
    const double distance_threshold;
    bool costmap_updated_ = false;
 
    // Callback for costmap updates
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
 
    // Callback for odometry updates
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
 
    // Timer-based map update
    void updateMap();
 
    // Integrate the latest costmap into the global map
    void integrateCostmap();
 
    // Flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;
    std::mutex latest_mutex_; //protect subscription callbacks and flags from timer callback
    const double map_resolution_scale_ = 2.0; //map is 2x coarser than costmap, keeping it precise!

};

#endif 
