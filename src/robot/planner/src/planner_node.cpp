#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode(): Node("planner_node"), state_(State::WAITING_FOR_GOAL) {
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
        // Timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

}
    //we need to update the map on this callback
    void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            planPath();
        }
    }
 
    //we reached the goal, reset state to wait for user to give it a new goal
    void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        planPath();
    }
 
    //position of bot 
    void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
    }

    //evaulate which state we need to be at
    void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            
            //publish empty path to signal control node to stop
            nav_msgs::msg::Path empty_path;
            empty_path.header.stamp = this->get_clock()->now();
            empty_path.header.frame_id = "sim_world";
            path_pub_->publish(empty_path);
            
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

    bool PlannerNode::goalReached() {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 2; // Threshold for reaching the goal
    }
 
    //use A star to plan minimized path
    void PlannerNode::planPath() {
        if (!goal_received_ || current_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
            return;
        }

        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "sim_world";
 
        //helper lambda functions to check states and convert from grids to continous metric world 

        const int width = static_cast<int>(current_map_.info.width);
        const int height = static_cast<int>(current_map_.info.height);
        const double res = current_map_.info.resolution;
        const double ox = current_map_.info.origin.position.x; //origin x
        const double oy = current_map_.info.origin.position.y; //origin y 

        auto worldToGrid = [&](double wx, double wy, CellIndex &out) -> bool {
            int gx = static_cast<int>(std::floor((wx - ox) / res)); 
            int gy = static_cast<int>(std::floor((wy - oy) / res));
            if (gx < 0 || gy < 0 || gx >= width || gy >= height) {
                return false; //out of bounds 
            }
            out = CellIndex(gx, gy);
            return true;
        };

        //indexing into the grid 
        auto gridIndex = [&](const CellIndex &c) {
            return c.y * width + c.x;
        };
        
        // decide if a cell is an obstacle
        auto isFree = [&](const CellIndex &c) {
            int8_t v = current_map_.data[gridIndex(c)];
            return v >= 0 && v < 10; //decrease this range if robot stuck in walls and edges, raise if too in the middle
        };

        //using euclidean distance of the grid cells to get distance to the goal
        auto heuristic = [&](const CellIndex &a, const CellIndex &b) {
            double dx = static_cast<double>(a.x - b.x);
            double dy = static_cast<double>(a.y - b.y);
            return std::sqrt(dx * dx + dy * dy);
        };

        //convert from discrete grid to metric world 
        auto gridToWorldPose = [&](const CellIndex &c) {
            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;
            p.pose.position.x = ox + (c.x + 0.5) * res;
            p.pose.position.y = oy + (c.y + 0.5) * res;
            p.pose.position.z = 0.0;
            p.pose.orientation.w = 1.0;
            return p;
        };

        //start algorithm 
        CellIndex start;
        CellIndex goal;

        //if the goal is outside the grid or inside of an obstacle 
        if (!worldToGrid(robot_pose_.position.x, robot_pose_.position.y, start) ||
            !worldToGrid(goal_.point.x, goal_.point.y, goal)) {
            RCLCPP_WARN(this->get_logger(), "Start or goal outside map bounds.");
            return;
        }
        if (!isFree(goal)) {
            RCLCPP_WARN(this->get_logger(), "Goal is occupied or unknown.");
            return;
        }

        if (!isFree(start)) {
            //we start a bit close to that big circle so make the first path doable
            current_map_.data[gridIndex(start)] = 0;
        }

        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open; //always expanding the most promising node next, discovered nodes
        std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from; //storing the previous one, so we can backtrack when we finish and get the full path
        std::unordered_map<CellIndex, double, CellIndexHash> g_score; //cost to move 
        std::unordered_set<CellIndex, CellIndexHash> closed; //fully processed nodes 

        g_score[start] = 0.0;
        open.push(AStarNode(start, heuristic(start, goal))); //initial euclidean distance from the start to the goal

        const int dirs[8][2] = {
            {1,0},{-1,0},{0,1},{0,-1},
            {1,1},{1,-1},{-1,1},{-1,-1}
        }; //sorrounding nodes to explore

        bool found = false;
        while (!open.empty()) { //as long as min heap still has an item 
            CellIndex current = open.top().index;
            open.pop();

            if (closed.count(current)) {
                continue;
            }
            if (current == goal) {
                found = true;
                break; //when the goal node is at the top of our min heap
            }
            closed.insert(current); //fully processed

            double current_g = g_score[current];
            for (const auto &d : dirs) { //for every sorrounding cell
                CellIndex nb(current.x + d[0], current.y + d[1]); //a neighbor cell
                if (nb.x < 0 || nb.y < 0 || nb.x >= width || nb.y >= height) { //if a node is on an edge of the grid, this is gonna happen, skip this neighbor
                    continue;
                } 
                if (!isFree(nb)) { //skip obstacle neighbors 
                    continue;
                }

                double step = (d[0] == 0 || d[1] == 0) ? 1.0 : std::sqrt(2.0); //diagonal or straight
                double tentative = current_g + step;
                auto it = g_score.find(nb);
                if (it == g_score.end() || tentative < it->second) { //havnt seen neighbor or cheapter path found 
                    came_from[nb] = current; //set the prev 
                    g_score[nb] = tentative;
                    double f = tentative + heuristic(nb, goal); //cost function 
                    open.push(AStarNode(nb, f)); //explore this node next 
                }
            }
        }

        std::vector<CellIndex> path_cells; //start backtracking from where a node comes from to get from the end to the start 
        if (start == goal) { //just in case lol
            path_cells.push_back(start); 
        } else if (found && came_from.count(goal)) { //if we found a path and the goal has a prev node in the path
            CellIndex cur = goal;
            path_cells.push_back(cur);
            while (cur != start) { //insert the rest of the path into cells vector
                cur = came_from[cur];
                path_cells.push_back(cur); 
            }
            std::reverse(path_cells.begin(), path_cells.end()); //push back and reverse is faster than insert in front because insert reorders the elemnts
        }

        if (path_cells.empty()) {
            RCLCPP_WARN(this->get_logger(), "A* failed to find a path.");
            return;
        }

        for (const auto &c : path_cells) {
            path.poses.push_back(gridToWorldPose(c));
        } //convert each cell in the optimal path to metric world pose 
       
        path_pub_->publish(path);
    }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
