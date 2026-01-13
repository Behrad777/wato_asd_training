#include <chrono>
#include <rclcpp/time.hpp>
#include "map_memory_node.hpp"

using std::placeholders::_1;


MapMemoryNode::MapMemoryNode() : Node("map_memory"), last_x(0.0), last_y(0.0), last_yaw(0.0), distance_threshold(1.5), map_memory_(robot::MapMemoryCore(this->get_logger())) { //initialize position to origin

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, _1)); 
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, _1)); 

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this)); //every 1 second
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    //update the latest costmap from subscription
    std::lock_guard<std::mutex> lk(latest_mutex_);
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

//global map is updated only when the robot moves at least 5 meters and at a controlled frequency
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  //quaternion to euler yaw
  const auto &q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  std::lock_guard<std::mutex> lk(latest_mutex_); //lock here 
  if (distance >= distance_threshold) {
      last_x = x;
      last_y = y;
      last_yaw = yaw;
      should_update_map_ = true;
  } else{
    last_yaw = yaw; //update yaw since its not affected by the distance
  }
}

void MapMemoryNode::updateMap() {
  {
    std::lock_guard<std::mutex> lk(latest_mutex_);
    if (!should_update_map_ || !costmap_updated_) return;
    should_update_map_ = false;
    costmap_updated_ = false;
  }
  integrateCostmap();
  map_pub_->publish(global_map_);
}

void MapMemoryNode::integrateCostmap() {
    // Transform and merge the latest costmap into the global map
    // (Implementation would handle grid alignment and merging logic)
  std::lock_guard<std::mutex> lk(latest_mutex_);
  if(latest_costmap_.data.empty()) return; //didnt get any data

  //mape map coarser than costmap to avoid holes
  double costmap_res = latest_costmap_.info.resolution;
  double map_res = costmap_res * map_resolution_scale_;


  int cm_w = latest_costmap_.info.width;
  int cm_h = latest_costmap_.info.height;

  //initialize the global map if empty cenetered around robot 
  if(global_map_.data.empty()){

    //copy over the properties of the costmap into global map
    const double global_size = 30.0;
    int gm_width = std::max(1, (int)std::round(global_size / map_res));
    int gm_height = gm_width; //since square 
    global_map_.info.resolution = map_res;
    global_map_.info.width = gm_width;
    global_map_.info.height = gm_height;
    //robot is origin 
    global_map_.info.origin.position.x = - (global_size / 2.0); //-15.0 for 30m world
    global_map_.info.origin.position.y = - (global_size / 2.0);
    global_map_.data.assign((size_t)gm_width * gm_height, -1); //all to -1 init
    global_map_.header.frame_id = "sim_world";
  }

  double cost_origin_x = latest_costmap_.info.origin.position.x;
  double cost_origin_y = latest_costmap_.info.origin.position.y;

  double cos_y = std::cos(last_yaw);
  double sin_y = std::sin(last_yaw);

  int gm_w = global_map_.info.width;
  int gm_h = global_map_.info.height;
  double gm_origin_x = global_map_.info.origin.position.x;
  double gm_origin_y = global_map_.info.origin.position.y;

  //transformong all points to global frame
  for (int cy = 0; cy < cm_h; ++cy) {
        for (int cx = 0; cx < cm_w; ++cx) {
            int cidx = cy * cm_w + cx; //current cell, since we flattened
            int8_t cell = latest_costmap_.data[cidx];
            if (cell < 0) continue; //skip unknowns

            //transform into global frame using robot pose (last_x,last_y,last_yaw) rotation matrix 
            double world_x = cost_origin_x + (cx + 0.5) * costmap_res;
            double world_y = cost_origin_y + (cy + 0.5) * costmap_res;

            //world -> global map indices
            int gx = (int)std::floor((world_x - gm_origin_x) / map_res);
            int gy = (int)std::floor((world_y - gm_origin_y) / map_res);

            if (gx < 0 || gy < 0 || gx >= gm_w || gy >= gm_h) continue; //out of bounds

            int gidx = gy * gm_w + gx;
            global_map_.data[gidx] = cell; //override
        }
    }

    global_map_.header.stamp = this->now();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
