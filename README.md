# SLAM 工作空間

這是一個基於 ROS 的 SLAM 專案，使用 Cartographer 進行建圖。

## 依賴項
- ROS Noetic
- Cartographer
- ...

## 使用方法
1. 克隆倉庫
2. 編譯工作空間
3. 運行節點
---
測量RPi溫度
vcgencmd measure_temp 

啟用SLAM
ros2 launch slam_toolbox online_async_launch.py

更新軟體
sudo apt update
sudo apt upgrade

啟用光達(無視窗)
ros2 launch rplidar_ros rplidar_c1_launch.py

啟用光達 有視窗
ros2 launch rplidar_ros view_rplidar_c1_launch.py

USB設備
ls -l /dev/rplidar

訂閱 topic
ros2 topic list


## 使用SLAM toolbox
### 回到工作空間根目錄
cd ~/slam_ws

### 刪除之前的build和install目錄
rm -rf build/ install/

### 重新編譯
colcon build

### 設置環境變量
source install/setup.bash

### 終端1：啟動SLAM
ros2 launch my_slam_pkg slam_launch.py

### 終端2：啟動RViz2
ros2 run rviz2 rviz2


## 使用 Cartographer SLAM
### 回到工作空間
cd ~/slam_ws

### 重新編譯
colcon build

### 設置環境變量
source install/setup.bash

### 終端1：運行SLAM
ros2 launch rplidar_cartographer cartographer_launch.py

### 終端2：啟動RViz2
ros2 run rviz2 rviz2
