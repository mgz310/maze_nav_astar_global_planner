#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <vector>
#include <queue>

namespace astar_planner {

struct Index {
    int x, y;
    bool operator==(const Index& o) const { return x==o.x && y==o.y; }
    bool operator<(const Index& o) const { return false; } // 仅用于 priority_queue
};

class AStarPlanner : public nav_core::BaseGlobalPlanner {
public:
    AStarPlanner();
    AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    costmap_2d::Costmap2D* costmap_;
    std::string frame_id_;
    bool initialized_;
    double obstacle_cost_threshold_;
    
    double heuristic(int x1, int y1, int x2, int y2);
    bool getMapCoords(const geometry_msgs::PoseStamped& pose, unsigned int& mx, unsigned int& my);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    std::vector<Index> getNeighbors(const Index& idx);
    bool isFree(int x, int y);
};

};
#endif