cmake_minimum_required(VERSION 3.8)
project(aubo_i5_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# moveit_servo 导出了 imported target `moveit_servo::pose_tracking`，
# 它自带全部传递依赖（moveit_core / moveit_ros_planning / control_toolbox /
# tf2_eigen / 各消息包），所以下面只显式列我们自己直接 include 的部分。
find_package(moveit_servo REQUIRED)

# ── 上游打包不一致的变通（不是业务逻辑）────────────────────────────────
# moveit_servo 导出的 `moveit_servo::pose_tracking` 接口里引用了
#   filters::filter_base / filters::filter_chain / filters::params /
#   filters::increment / filters::mean / filters::median /
#   filters::transfer_function / filters::realtime_circular_buffer
# 这些 modern target 名，但本机安装的 filters 包**只提供 filtersConfig.cmake、
# 不导出任何 filters:: 目标**（磁盘上只有 lib/libfilters.so）。
# 后果：链接时 CMake 把它们当库名，报 `找不到 -lfilters::params`。
#
# 已核实这些条目对本节点是多余的：libpose_tracking.so 的 ldd 里没有 filters，
# nm 里也没有任何 filters 未定义符号。所以这里给这几个名字建 INTERFACE 占位目标、
# 统一指向真正的 libfilters，让 CMake 能解析即可（不改动实际链接内容）。
# 将来 filters 包修好了，TARGET 判断会自动跳过这一段。
find_package(filters REQUIRED)
find_library(FILTERS_LIBRARY_FILE NAMES filters REQUIRED)
foreach(_filters_component filter_base filter_chain params increment mean median
                           transfer_function realtime_circular_buffer)
  if(NOT TARGET filters::${_filters_component})
    add_library(filters::${_filters_component} INTERFACE IMPORTED)
    set_target_properties(filters::${_filters_component} PROPERTIES
      INTERFACE_LINK_LIBRARIES "${FILTERS_LIBRARY_FILE}")
  endif()
endforeach()

add_executable(pose_tracking_node src/pose_tracking_node.cpp)
target_link_libraries(pose_tracking_node
  moveit_servo::pose_tracking
  rclcpp::rclcpp
)

install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS pose_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/plan_execute_test.py
    scripts/servo_twist_test.py
    scripts/add_scene_floor.py
    scripts/pose_target_test.py
    scripts/go_ready.py
    scripts/servo_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
