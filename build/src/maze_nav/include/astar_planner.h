#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <vector>
#include <queue>

namespace astar_planner {

struct Index //地图坐标索引结构体，包含x和y坐标，并重载了相等和小于运算符
{
    int x, y;
    bool operator==(const Index& o) const { return x==o.x && y==o.y; }
    bool operator<(const Index& o) const { return false; } // 仅用于 priority_queue
};

class AStarPlanner : public nav_core::BaseGlobalPlanner 
{
public:
    AStarPlanner(); //默认构造函数
    AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros); //构造函数，直接调用初始化函数

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override; //初始化函数，设置成本地图指针、坐标系和一些参数，同时检查是否已经初始化过
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override; //实现A*算法的核心函数，输入起点和终点，输出路径规划结果

private:
    costmap_2d::Costmap2D* costmap_; //成本地图指针，用于查询地图信息
    std::string frame_id_;  //坐标系ID，通常与成本地图的全局坐标系一致
    bool initialized_; //初始化标志，确保在使用规划器之前已经正确初始化
    double obstacle_cost_threshold_; //障碍物成本阈值，255是不可达，253以上视为障碍
    double turn_penalty_; //对转弯增加的惩罚，鼓励更直的路径
    
    double heuristic(int x1, int y1, int x2, int y2); //启发式函数，计算当前节点与目标节点之间的距离，这里使用欧几里得距离
    bool getMapCoords(const geometry_msgs::PoseStamped& pose, unsigned int& mx, unsigned int& my); //将世界坐标转换为地图坐标，并检查是否在地图范围内
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my); //将世界坐标转换为地图坐标，并检查是否在地图范围内
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy); //将地图坐标转换为世界坐标，供路径点生成使用
    std::vector<Index> getNeighbors(const Index& idx); //获取当前节点的8个邻居（包括对角线），并进行基本的边界和障碍检查
    bool isFree(int x, int y); //检查给定坐标是否可通行，允许通过未知区域（NO_INFORMATION），否则大量灰色区域会阻断路径
};

};
#endif