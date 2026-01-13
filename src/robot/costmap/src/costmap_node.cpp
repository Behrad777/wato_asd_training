#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include <queue>
#include <limits>
#include <tuple>
#include <rclcpp/time.hpp>
 
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  RCLCPP_INFO(this->get_logger(), "CostmapNode starting...");
  
  pub_msg_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  RCLCPP_INFO(this->get_logger(), "Publisher created for /costmap");
  
  string_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::lidar_callback, this, _1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to /lidar");

  // fixed 30m from discord
  const double world_size_m = 30.0;
  grid_size = std::max(1, (int)std::round(world_size_m / resolution)); // 300 for res=0.1
  origin_x = -world_size_m / 2.0; //center grid at robot: -15.0
  origin_y = -world_size_m / 2.0;

  // initialize OccupancyGrid size
  OccupancyGrid.assign(grid_size, std::vector<float>(grid_size, -1.0f));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

//define the timer to publish a message every 500ms
void CostmapNode::lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  initializeGrid();

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      "sim_world",
      msg->header.frame_id,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not transform %s to sim_world: %s",
                         msg->header.frame_id.c_str(), ex.what());
    return;
  }

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;
    double range = msg->ranges[i];
    if (range < msg->range_max && range > msg->range_min) {
      geometry_msgs::msg::PointStamped point_in;
      point_in.header.frame_id = msg->header.frame_id;
      point_in.header.stamp = msg->header.stamp;
      point_in.point.x = range * std::cos(angle);
      point_in.point.y = range * std::sin(angle);
      point_in.point.z = 0.0;

      geometry_msgs::msg::PointStamped point_out;
      tf2::doTransform(point_in, point_out, transform_stamped);

      int x_grid, y_grid;
      convertWorldToGrid(point_out.point.x, point_out.point.y, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
  publishMessage();
}

void CostmapNode::initializeGrid(){
  OccupancyGrid.assign(grid_size, std::vector<float>(grid_size, 0.0f));
}
void CostmapNode::convertWorldToGrid(double x_world, double y_world, int& x, int& y) {
  x = static_cast<int>(std::floor((x_world - origin_x) / resolution));
  y = static_cast<int>(std::floor((y_world - origin_y) / resolution));
}


void CostmapNode::markObstacle(int x, int y){
  if (x < 0 || y < 0) return;
  if (x >= grid_size || y >= grid_size) return;
  OccupancyGrid[y][x] = max_inflation_cost; // mark occupied with 
}

//using djikstra Multi-source single destination algorithm
void CostmapNode::inflateObstacles(){
  if (OccupancyGrid.empty()) return;

  const int rows = static_cast<int>(OccupancyGrid.size());
  const int cols = static_cast<int>(OccupancyGrid[0].size());
  const double INF = std::numeric_limits<double>::infinity();

  std::vector<std::vector<double>> dist(rows, std::vector<double>(cols, INF)); //all distances infinity then we ovveride 

  using Node = std::tuple<double,int,int>;
  struct Cmp { bool operator()(const Node &a, const Node &b) const { return std::get<0>(a) > std::get<0>(b); } }; //specialize the comparison
  std::priority_queue<Node, std::vector<Node>, Cmp> pq;

  //push all max cost (confident obstacle cells) as sources for algorithm
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      if (OccupancyGrid[y][x] >= max_inflation_cost) {
        dist[y][x] = 0.0; //max cost node has a distance of 0 from itself 
        pq.emplace(0.0, y, x);
      }
    }
  }

  if (pq.empty()) return;

  const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}}; //the 8 cells sorrounding a cell
  const double weights[8] = {
    resolution, resolution, resolution, resolution, //first 4, right left up down are all just 1 resolution away from the source
    std::sqrt(2.0) * resolution, std::sqrt(2.0) * resolution, //the other 4 diagonals 45 degree calculation
    std::sqrt(2.0) * resolution, std::sqrt(2.0) * resolution
  }; 

  //Dijkstra propagation until queue empty of all sources or distances exceed inflation_radius 
  //multi source djikstra, its faster than doing every cell in the grid
  while (!pq.empty()) {
    auto [d, y, x] = pq.top(); pq.pop();
    if (d > dist[y][x]) continue;
    if (d > inflation_radius) continue; // no need to expand beyond neighbors

    for (int i = 0; i < 8; ++i) {
      int ny = y + dirs[i][1];
      int nx = x + dirs[i][0];
      if (ny < 0 || nx < 0 || ny >= rows || nx >= cols) continue;
      double nd = d + weights[i];
      if (nd < dist[ny][nx] && nd <= inflation_radius) {
        dist[ny][nx] = nd;
        pq.emplace(nd, ny, nx);
      }
    }
  }

  //convert distances to costs and update occupancy grid if more than current score
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      double d = dist[y][x];
      if (d <= inflation_radius) {
        float cost = static_cast<float>(max_inflation_cost * (1.0 - (d / inflation_radius)));
        if (cost > OccupancyGrid[y][x]) OccupancyGrid[y][x] = cost;
      }
    }
  }
}

//publish the updated occpuancy grid
void CostmapNode::publishMessage() {

  nav_msgs::msg::OccupancyGrid message;

  //header
  message.header.set__frame_id("sim_world");
  message.header.set__stamp(this->now());

  //info
  message.info.set__resolution((float)CostmapNode::resolution);
  message.info.set__width(CostmapNode::grid_size);
  message.info.set__height(CostmapNode::grid_size);


  //make the position geometry msg object for origin
  geometry_msgs::msg::Pose origin;
  origin.position.x = CostmapNode::origin_x;
  origin.position.y = CostmapNode::origin_y;
  origin.position.z = 0.0;
  origin.orientation.x = 0.0;
  origin.orientation.y = 0.0;
  origin.orientation.z = 0.0;
  origin.orientation.w = 1.0;
  message.info.set__origin(origin);

  //flatten the grid and explictly convert the floats to int_8, shouldve had int_8 vaues in the grid from the start?
  std::vector<int8_t> flat;
  flat.reserve(static_cast<size_t>(grid_size) * grid_size);

  for (const auto &row : OccupancyGrid) {
    for (float v : row) {
      int8_t cell;
      if (v < 0.0f) {
        cell = -1; // unknown
      } else {
        float scaled = (v >= max_inflation_cost) ? 100.0f : (v / max_inflation_cost) * 100.0f;
        int intval = static_cast<int>(std::lround(scaled));
        intval = std::clamp(intval, 0, 100);
        cell = static_cast<int8_t>(intval);
      }
      flat.push_back(cell);
    }
  }
  message.set__data(flat);


  pub_msg_->publish(message);
}


 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}