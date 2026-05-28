
```markdown
# 迷宫机器人导航仿真项目（Maze Navigation）

基于 ROS Noetic 的迷宫环境自主导航系统，包含机器人模型、Gazebo 仿真、SLAM 建图与导航（A*全局规划+ TEB 局部规划）。

## 功能简介

-   **Gazebo 仿真**：加载自定义迷宫世界，使用轮式机器人模型。
-   **定位**：通过 EKF 融合里程计数据（`robot_localization`）。
-   **导航**：
    -   `move_base` 框架
    -   全局规划（A*）和局部规划器（TEB）
    -   静态地图（已建好的 `maze_map`）
-   **可视化**：RViz 实时显示机器人、代价地图及导航目标。

## 目录结构

```
# A* Global Planner for ROS Move Base

该项目提供了一个基于 A* 搜索算法的自定义全局路径规划器，作为 `move_base` 的插件。规划器利用 2D 代价地图进行全局路径搜索，支持 8 连通邻居扩展和欧几里得启发式。

## 功能特点

- 实现 A* 路径搜索，适用于静态环境
- 可配置障碍物代价阈值
- 与 ROS `move_base` 完全兼容，动态加载
- 支持任意分辨率的代价地图
- 代码清晰，易于二次开发

## 依赖

- ROS (Noetic)
- `nav_core` – 规划器接口
- `costmap_2d` – 代价地图数据
- `pluginlib` – 插件注册
- `tf` – 坐标变换（由 `costmap_2d` 依赖）
- C++11 编译支持

## 编译

1. 将本功能包放入 ROS 工作空间的 `src` 目录下，例如：
   ```bash
   cd ~/catkin_ws/src
   git clone <your_repo_url> astar_global_planner

## 依赖安装

所有指令基于 **ROS Noetic (Ubuntu 20.04)**。

### 1. ROS Noetic 完整安装
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### 2. 导航必备包
```bash
sudo apt install ros-noetic-navigation
```

### 3. TEB 局部规划器
```bash
sudo apt install ros-noetic-teb-local-planner
```

### 4. robot_localization（EKF 融合）
```bash
sudo apt install ros-noetic-robot-localization
```

### 5. Gazebo 相关
```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

### 6. 其他基础包（通常已包含）
```bash
sudo apt install ros-noetic-joint-state-publisher \
                 ros-noetic-robot-state-publisher \
                 ros-noetic-tf2-ros
```

### 7. 编译工作空间
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 如果你的工作空间路径不同，请相应修改。

## 使用方法

1. **启动完整导航仿真**
```bash
roslaunch maze_nav maze_nav.launch
```

2. **RViz 中设置导航目标**
启动后 RViz 会自动打开。点击上方工具栏 `2D Nav Goal`，在地图上（可通行区域）选一个目标点，机器人将自主规划路径并行驶过去。

## 注意事项

-   确保机器人在 `my_robot` 包中有正确的 URDF/XACRO 模型，且所有传感器话题与配置一致。
-   迷宫世界文件路径已在 launch 中硬编码为 `$(find maze_mapping)/worlds/maze.world`，请勿移动该文件。
-   如果首次运行出现 TF 错误，尝试先发布一次静态变换：
    ```bash
    rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```
-   `robot_localization` 中的 EKF 节点将里程计 `odom` 融合输出 `odom_combined`，如果你的机器人里程计话题不是 `/odom`，请自行修改。
