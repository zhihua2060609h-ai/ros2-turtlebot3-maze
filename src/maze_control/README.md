# 地老鼠走迷宫系统

最终演示使用 `worlds/maze.world`。方形外框为 6 × 6 米，内部有四组错开的直角隔墙，出口位于右下方。默认启用自适应左墙跟随。机器人满足 `x > 4.4 m`、`y < -0.65 m` 且累计行程超过 3 米时停止，并在日志中记录 `EXIT_REACHED`。仿真超时上限为 600 秒。

## 环境及依赖

实测环境为 VMware 中的 Ubuntu 22.04、Python 3.10、ROS 2 Humble、Gazebo Classic 11.10.2 和 Humble 对应的 RViz2。机器人使用 TurtleBot3 Burger。以下命令适用于已配置 ROS 2 Humble 软件源的 Ubuntu 22.04，缺少依赖时执行。

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions ros-humble-rviz2 \
  ros-humble-turtlebot3-simulations ros-humble-turtlebot3-description \
  ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher
```

## 编译和启动

解压代码包，将其中的 `src` 放到 `~/ros2maze_ws` 下，使 `~/ros2maze_ws/src/maze_control/package.xml` 存在。在 Ubuntu 桌面终端执行：

```bash
cd ~/ros2maze_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix maze_control
ros2 launch maze_control gazebo_maze.launch.py
```

默认启动 Gazebo、TurtleBot3 Burger、RViz2 和控制节点。

## 代码结构和组员分工

| 成员 | 负责模块 | 对应文件或代码区域 |
| --- | --- | --- |
| 成员1 | Gazebo 迷宫、机器人模型配置、启动集成、开环和航点运动实验 | `worlds/`、`launch/gazebo_maze.launch.py`、`controller.py` 中的开环、点到点和航点分支、`control_math.py` 的 `point_command` |
| 成员2 | 雷达分区、反应式避障、左墙跟随、自适应调速与轨迹数据整理 | `control_math.py` 的 `sector`、`controller.py` 中的避障和沿墙分支及记录逻辑、`rviz/maze.rviz` 的雷达与轨迹显示 |

共同负责整机联调、仿真结果核对和报告整理。`package.xml`、`setup.py`、`setup.cfg` 和 `resource/maze_control` 用于声明依赖、安装资源和注册功能包。

## 参考和 AI 辅助说明

- [课程教程 Ch6 机器人运动控制](https://www.yuque.com/shellcore/rpide7/eztkspzf49s55ypi)和[Ch7.4 左墙跟随](https://www.yuque.com/shellcore/rpide7/yyll3ya9h21p0mgo)提供运动实验和沿墙算法参考。
- Burger 的 URDF、SDF 和 Gazebo 插件由 [ROBOTIS TurtleBot3 仿真包](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)提供。项目启动文件在运行时从安装的 ROS 包加载这些外部资源。
- AI 辅助了代码草拟与调试、迷宫布局调整、实测数据制图和技术说明整理。视频及实验数据来自 VMware 中的 Gazebo 仿真运行。
