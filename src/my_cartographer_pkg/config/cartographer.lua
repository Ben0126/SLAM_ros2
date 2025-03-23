include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",              -- 地圖坐標系
  tracking_frame = "base_link",   -- 跟踪坐標系（通常是機器人基座）
  published_frame = "base_link",  -- 發布的坐標系
  odom_frame = "odom",           -- 里程計坐標系
  provide_odom_frame = true,            -- 是否提供里程計框架
  publish_frame_projected_to_2d = true, -- 是否將框架投影到2D平面
  use_odometry = false,                 -- 是否使用里程計數據
  use_nav_sat = false,                  -- 是否使用GPS數據
  use_landmarks = false,                -- 是否使用地標
  num_laser_scans = 1,                      -- 激光掃描儀數量
  num_multi_echo_laser_scans = 0,           -- 多回波激光掃描儀數量
  num_subdivisions_per_laser_scan = 1,      -- 每次掃描的細分數
  num_point_clouds = 0,                     -- 點雲數量
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,          -- 子地圖發布週期
  pose_publish_period_sec = 5e-3,           -- 位姿發布週期
  trajectory_publish_period_sec = 30e-3,    -- 軌跡發布週期
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.

}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.min_range = 0.15   -- 最小測量範圍
TRAJECTORY_BUILDER_2D.max_range = 12.    -- 最大測量範圍
TRAJECTORY_BUILDER_2D.use_imu_data = false -- 是否使用IMU數據
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 使用在線相關性掃描匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15    -- 線性搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.) -- 角度搜索窗口

POSE_GRAPH.constraint_builder.min_score = 0.65                    -- 最小匹配分數
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 -- 全局定位最小分數
POSE_GRAPH.optimization_problem.huber_scale = 1e2                 -- Huber損失函數比例因子

return options
