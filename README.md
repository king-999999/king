# autonomous-racecara （基于 ROS2 的智能驾驶竞速小车）

## 1. 项目简介
本仓库为基于 **ROS 2 Humble** 的小型无人驾驶/竞速车软件栈，整合激光雷达（LiDAR）、IMU、编码器等传感器，实现：
- SLAM 建图（GMapping）
- AMCL 定位 + Nav2 自主导航
- 多点巡航（路点循环）
- 车体底层控制与遥控脚本
- 红绿灯识别与巡线（OpenCV HSV）

仓库仅保留源码与运行脚本，构建产物（build / install / log 等）已忽略，需用户自行在 Linux/ROS2 环境编译。

## 2. 主要功能
| 功能 | 说明 |
| ---- | ---- |
| 传感器接入 | LiDAR（lslidar_driver）、IMU（hipnuc_imu）、编码器（encoder） |
| 消息与数据流 | 发布 /lslidar_points 或 /scan、/IMU_data、/encoder、里程计 /odom |
| 建图 | openslam_gmapping + slam_gmapping |
| 定位 | AMCL（或在 Nav2 框架中使用） |
| 导航 | Nav2 路径规划（global planner / DWA / costmap） |
| 多点巡航 | 自定义 Python 脚本记录并循环发布目标点 |
| 控制 | racecar / racecar_driver 下控制/遥控脚本 |
| 可视化 | RViz 预设（navigation.rviz / slam.rviz / odom.rviz） |
| 任务脚本 | car.sh / nav.sh / gmapping.sh / save.sh 等一键流程 |

## 3. 目录结构（节选）
```
davinci-mini/
  .gitignore
  navigation/                # ROS1 导航栈源码（参考 / 备用）
  catkin_ws/src/navigation/  # 同上（历史保留）
  racecar/
    src/
      lslidar_driver/        # LiDAR 驱动（ROS2）
      lslidar_msgs/          # LiDAR 自定义消息
      hipnuc_imu/            # IMU 驱动 / 解析
      encoder/               # 编码器数据节点
      serial-foxy/           # 串口封装
      openslam_gmapping/     # GMapping 底层库
      slam_gmapping/         # GMapping ROS2 封装
      nav2_waypoint_cycle/   # 多点巡航（路点循环）
      racecar/               # 应用/控制层节点与脚本
      racecar_driver/        # 底盘/车辆接口
    car.sh nav.sh gmapping.sh save.sh ...
```
## 4. ROS2 包角色速览
| 包 | 作用（简述） |
| --- | --- |
| lslidar_msgs | 定义 LiDAR 消息类型 |
| lslidar_driver | 采集并解析 LiDAR UDP 数据，发布点云/扫描 |
| hipnuc_imu | 串口 IMU 数据读取与输出 |
| encoder | 里程计/轮速编码器计数及发布 |
| serial-foxy | 串口辅助库 |
| openslam_gmapping / slam_gmapping | GMapping 建图算法与 ROS2 接口 |
| nav2_waypoint_cycle | 多点巡航（循环发送导航目标） |
| racecar_driver | 车辆运动底层控制接口（速度/转向） |
| racecar | 应用整合：控制节点、脚本、配置、RViz 预设 |
## 5. 环境准备
建议：Ubuntu 22.04 + ROS 2 Humble + colcon + Python3 + OpenCV。
```
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-tf2-tools ros-humble-rviz2 ros-humble-diagnostic-updater \
  ros-humble-pointcloud-to-laserscan
```
