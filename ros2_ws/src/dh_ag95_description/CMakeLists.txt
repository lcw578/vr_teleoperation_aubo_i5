cmake_minimum_required(VERSION 3.8)
project(dh_ag95_description)

# ---------------------------------------------------------------------------
# 这是 ian-chuang/dh_gripper_ros 里 dh_ag95_description 的 ament_cmake 移植版。
#
# 原包是 catkin（ROS 1，2018 年，format="2"），colcon 无法编译。
# 本包是纯描述包——只有 xacro 和 mesh，没有任何可执行代码——
# 所以移植只做两件事：换构建系统、把 urdf/ 和 meshes/ 装出去。
# 运动学、惯性、几何数据一个字未改。
#
# 原版保留在 VR_teleoperation/_refs/dh_gripper_ros/ 作为参照。
# 上游 License: MIT (Copyright (c) 2018 Toshinori Kitamura)
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
