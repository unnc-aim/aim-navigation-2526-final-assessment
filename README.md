# aim-navigation-2526-final-assessment
AIM导航组2025-26赛季入队考核题

本目录是一个**简化版 ROS2 工作空间**，用于演示如何在二维栅格地图上复用 `a_star.py`
实现 A* 导航，并按“**地图环境** / **核心实现**”的思路做包划分。

## 目录结构

```
homework/
  src/
    a_star_core/   # 纯算法（不依赖 ROS2）
    a_star_ros2/   # ROS2 节点：地图发布 + 路径规划
```

## 作业要求（学生需完成）

- 在 `a_star_core/a_star_planner.py` 中完成标记为 **TODO** 的核心函数：
  - `planning()`：A* 主循环（open/closed、邻居扩展、代价更新）
  - `calc_final_path()`：路径回溯
  - `calc_heuristic()`：启发式函数
- 未实现时会抛出 `NotImplementedError`，属于正常提示。

## 构建与运行（参考）

> 注意：以下命令以类 Unix 环境为例；Windows 需要使用 `.bat` 或 PowerShell 版本。

```bash
cd homework
colcon build
source install/setup.bash
```

### 启动地图与规划器（含 RViz2）

```bash
ros2 launch a_star_ros2 a_star_demo.launch.py
```

若不需要 RViz2，可关闭：

```bash
ros2 launch a_star_ros2 a_star_demo.launch.py use_rviz:=false
```

或分别运行：

```bash
ros2 run a_star_ros2 map_provider_node --ros-args -p map_id:=room_simple
ros2 run a_star_ros2 planner_node
```

### 发送起点 / 终点

你可以在 RViz2 中使用 “2D Pose Estimate” 作为起点、
“2D Goal Pose” 作为终点：

- 起点话题：`/initialpose`（PoseWithCovarianceStamped）
- 终点话题：`/goal`（PoseStamped）

同时也保留对 `/start`（PoseStamped）的支持，可用于脚本或命令行发布。
