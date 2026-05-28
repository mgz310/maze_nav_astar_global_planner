#include "astar_planner.h"
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace astar_planner
{
    struct AstarNode
    {
        Index pos;
        double g, h;
        Index parent;
    };

    struct CompareNode
    {
        bool operator()(const AstarNode& a, const AstarNode& b) const
        {
            return (a.g + a.h) > (b.g + b.h);
        }
    };

    AStarPlanner::AStarPlanner() : initialized_(false), costmap_(nullptr) {}

    AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (initialized_)
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
            return;
        }
        costmap_ = costmap_ros->getCostmap();
        frame_id_ = costmap_ros->getGlobalFrameID();
        obstacle_cost_threshold_ = 253; // 255是不可达，253以上视为障碍
        initialized_ = true;
    }

    double AStarPlanner::heuristic(int x1, int y1, int x2, int y2)
    {
        // 使用欧几里得距离作为启发式函数
        return std::hypot(x2 - x1, y2 - y1);
    }

    bool AStarPlanner::getMapCoords(const geometry_msgs::PoseStamped& pose, unsigned int& mx, unsigned int& my)
    {
        double wx = pose.pose.position.x;
        double wy = pose.pose.position.y;
        return worldToMap(wx, wy, mx, my);
    }

    void AStarPlanner::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
    {
        costmap_->mapToWorld(mx, my, wx, wy);
    }

    bool AStarPlanner::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
    {
        return costmap_->worldToMap(wx, wy, mx, my);
    }

    std::vector<Index> AStarPlanner::getNeighbors(const Index& idx)
    {
        std::vector<Index> neighbors;
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                if (dx == 0 && dy == 0) continue;
                int nx = idx.x + dx;
                int ny = idx.y + dy;

                // 基本边界和自由检查
                if (!isFree(nx, ny))
                    continue;

                // 对角线移动时，检查相邻的两个栅格，防止角落穿越
                if (dx != 0 && dy != 0)
                {
                    if (!isFree(idx.x + dx, idx.y) || !isFree(idx.x, idx.y + dy))
                        continue;
                }

                neighbors.push_back({nx, ny});
            }
        }
        return neighbors;
    }

    bool AStarPlanner::isFree(int x, int y)
    {
        unsigned char cost = costmap_->getCost(x, y);
        // 允许通过未知区域（否则大量灰色区域会阻断路径）
        if (cost == costmap_2d::NO_INFORMATION)
            return true;
        return cost < obstacle_cost_threshold_;
    }

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
    { 
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() before using the planner");
            return false;
        }

        // 清空输出计划
        plan.clear();

        unsigned int start_x, start_y, goal_x, goal_y;

        // 转换并检查坐标有效性
         if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y))
    {
        ROS_ERROR("A* FAIL: start point out of map");
        return false;
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
    {
        ROS_ERROR("A* FAIL: goal point out of map");
        return false;
    }
    if (!isFree(start_x, start_y))
    {
        ROS_ERROR("A* FAIL: start cell occupied [%d,%d] cost=%d",
                  start_x, start_y, costmap_->getCost(start_x, start_y));
        return false;
    }
    if (!isFree(goal_x, goal_y))
    {
        ROS_ERROR("A* FAIL: goal cell occupied [%d,%d] cost=%d",
                  goal_x, goal_y, costmap_->getCost(goal_x, goal_y));
        return false;
    }

        // 编码函数：将二维坐标映射为唯一的64位键
        auto encode = [](unsigned int x, unsigned int y) -> long long {
            return (static_cast<long long>(x) << 32) | static_cast<long long>(y);
        };

        std::priority_queue<AstarNode, std::vector<AstarNode>, CompareNode> open_list;
        std::unordered_map<long long, AstarNode> visited;
        std::unordered_set<long long> closed_set;   

        AstarNode start_node;
        start_node.pos = {static_cast<int>(start_x), static_cast<int>(start_y)};
        start_node.g = 0;
        start_node.h = heuristic(start_x, start_y, goal_x, goal_y);
        start_node.parent = start_node.pos;
        open_list.push(start_node);
        visited[encode(start_x, start_y)] = start_node;

        bool found = false;
        AstarNode goal_node;

        while (!open_list.empty())
        {
            AstarNode cur = open_list.top();
            open_list.pop();

            long long key = encode(cur.pos.x, cur.pos.y);
            if (closed_set.count(key))
                continue;
            closed_set.insert(key);

            // 到达目标
            if (cur.pos.x == static_cast<int>(goal_x) && cur.pos.y == static_cast<int>(goal_y))
            {
                found = true;
                goal_node = cur;
                break;
            }
            auto neighbors = getNeighbors(cur.pos);
            for (const auto& n : neighbors)
            {
                long long n_key = encode(n.x, n.y);
                if (closed_set.count(n_key))
                    continue;

                double tentative_g = cur.g + heuristic(cur.pos.x, cur.pos.y, n.x, n.y);

                if (!visited.count(n_key) || tentative_g < visited[n_key].g)
                {
                    AstarNode neighbor_node;
                    neighbor_node.pos = n;
                    neighbor_node.g = tentative_g;
                    neighbor_node.h = heuristic(n.x, n.y, goal_x, goal_y);
                    neighbor_node.parent = cur.pos;
                    open_list.push(neighbor_node);
                    visited[n_key] = neighbor_node;
                }
            }
        }

        if (!found)
        {
            ROS_WARN("No path found from start to goal.");
            return false;
        }

        // 回溯路径
        std::vector<Index> path_idx;
        AstarNode node = goal_node;
        while (!(node.pos.x == static_cast<int>(start_x) && node.pos.y == static_cast<int>(start_y)))
        {
            path_idx.push_back(node.pos);
            long long pkey = encode(node.parent.x, node.parent.y);
            if (visited.find(pkey) == visited.end())
            {
                ROS_ERROR("Parent node not found in visited set. Path reconstruction failed.");
                return false;
            }
            node = visited[pkey];
        }
        path_idx.push_back({static_cast<int>(start_x), static_cast<int>(start_y)});
        std::reverse(path_idx.begin(), path_idx.end());

        // 生成最终规划消息
        plan.resize(path_idx.size());
        for (size_t i = 0; i < path_idx.size(); ++i)
        {
            double wx, wy;
            mapToWorld(path_idx[i].x, path_idx[i].y, wx, wy);
            plan[i].header.frame_id = frame_id_;
            plan[i].header.stamp = ros::Time::now();
            plan[i].pose.position.x = wx;
            plan[i].pose.position.y = wy;
            plan[i].pose.orientation = goal.pose.orientation;
        }

        return true;
    }

} // namespace astar_planner

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)