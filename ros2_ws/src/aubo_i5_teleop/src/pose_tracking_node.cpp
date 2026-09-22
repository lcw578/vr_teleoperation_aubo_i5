// 阶段 3：MoveIt Servo 的 **pose tracking** 入口（替代原先的 delta_twist_cmds 流）。
//
// 本文件的来源与边界（重要）：
//   骨架**照抄官方** moveit_servo 的 cpp_interface_demo/pose_tracking_demo.cpp
//   （Humble，同版本，见 /opt/ros/humble/lib/moveit_servo/servo_pose_tracking_demo）。
//   **算法完全不在本文件里**——PID、雅可比伪逆、奇异/碰撞降速、平滑、停止
//   全在 MoveIt 的库中：moveit_servo::PoseTracking（libpose_tracking.so）
//   + moveit_servo::Servo（libmoveit_servo_lib.so）。
//   本文件只做三件事：装配（参数 / planning scene monitor / PoseTracking）、
//   主循环反复调用 moveToPose、打印状态。
//
// 与官方 demo 的差别，共四处，都有明确理由：
//   1. 删掉 demo 自带的 target_pose 发布者与演示动作——它会和我们自己的目标位姿源
//      （未来是 VR 手柄，现在是 scripts/pose_target_test.py）抢同一个话题。
//   2. 不调用 providePlanningSceneService() / startPublishingPlanningScene()——
//      我们的 move_group 才是主 planning scene 持有者，两个节点同时提供
//      /get_planning_scene 会冲突。只保留"接收"侧的三个 start*Monitor。
//   3. 等仿真时钟真正走起来之后再调 waitForCurrentRobotState（见下方注释），
//      因为官方 demo 不设 use_sim_time，而我们设。
//   4. 用 run() 包住主体、main 里统一 join 执行器线程：官方 demo 直接 exit()，
//      早退路径不会析构 joinable 的 std::thread（会 std::terminate / SIGABRT）。
//
// 目标位姿话题名：库内部用的是**相对名** "target_pose"（见 pose_tracking.cpp:100），
//   相对名解析到**节点命名空间**（不含节点名），所以默认就是 /target_pose。
//   ROS 1 的 LARA 用 /servo_server/target_pose，是因为那边节点名就叫 servo_server。

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("aubo_i5_teleop.pose_tracking");

// 这三个值的取法：
//   pos/ang tolerance 与 target_pose_timeout 取 LARA（Aubo i5 + Servo 的同类先例）的
//   {0.01,0.01,0.01} / 0.1 / 1.0，而不是官方 panda demo 的 0.001 / 0.01 / 0.1——
//   demo 是"走一次就退出"的演示，容差取得极紧；遥操要的是持续跟随。
constexpr double POSITIONAL_TOLERANCE = 0.01;  // [m]
constexpr double ANGULAR_TOLERANCE = 0.1;      // [rad]
constexpr double TARGET_POSE_TIMEOUT = 1.0;    // [s]
}  // namespace

// 打印 Servo 状态变化（照抄官方 demo 的 StatusMonitor，只改了日志名）
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(
        topic, rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) {
          return statusCB(msg);
        });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    auto latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

int run(const rclcpp::Node::SharedPtr& node)
{
  // Servo 参数在 moveit_servo 命名空间下（与官方 launch 的 {"moveit_servo": ...} 一致）
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    return EXIT_FAILURE;
  }

  // planning scene monitor：只接收，不对外提供（见文件头的说明）
  auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    return EXIT_FAILURE;
  }
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);

  // ⚠️ 仿真时钟就绪之前不能调 waitForCurrentRobotState。
  //   本节点设了 use_sim_time，刚启动时还没收到 /clock，node->now() 返回 0；
  //   而 PlanningSceneMonitor::waitForCurrentRobotState 对 t==0 会**立刻返回 false**
  //   （planning_scene_monitor.cpp:1030 `if (t.nanoseconds() == 0) return false;`），
  //   5 秒超时根本不会生效。官方 demo 不设 use_sim_time（墙钟永不为 0），所以没这个坑。
  //   这里用墙钟计时等仿真时钟就绪，避免依赖尚未初始化的那个时钟。
  const rclcpp::Time t_zero(0, 0, RCL_ROS_TIME);
  const auto clock_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  rclcpp::WallRate clock_wait_rate(50.0);
  while (rclcpp::ok() && node->now() == t_zero && std::chrono::steady_clock::now() < clock_deadline)
  {
    clock_wait_rate.sleep();
  }
  if (node->now() == t_zero)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "等了 10 秒仍未收到 /clock——仿真时间源没起来？");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(LOGGER, "仿真时钟就绪：%.3f s", node->now().seconds());

  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    return EXIT_FAILURE;
  }

  // PoseTracking 的构造函数内部会自己调 servo_->start()，
  // 所以**不需要**像 twist 流那条路一样调 /servo_node/start_servo 服务。
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  RCLCPP_INFO(LOGGER, "planning_frame=%s  ee_frame=%s  move_group=%s  command_out=%s",
              servo_parameters->planning_frame.c_str(), servo_parameters->ee_frame_name.c_str(),
              servo_parameters->move_group_name.c_str(), servo_parameters->command_out_topic.c_str());
  RCLCPP_INFO(LOGGER, "等待 target_pose（PoseStamped，frame 必须是 %s）...",
              servo_parameters->planning_frame.c_str());

  tracker.resetTargetPose();

  const Eigen::Vector3d lin_tol = Eigen::Vector3d::Constant(POSITIONAL_TOLERANCE);

  while (rclcpp::ok())
  {
    const auto status = tracker.moveToPose(lin_tol, ANGULAR_TOLERANCE, TARGET_POSE_TIMEOUT);
    if (status == moveit_servo::PoseTrackingStatusCode::SUCCESS)
    {
      RCLCPP_INFO_STREAM(LOGGER, "已到达目标位姿");
    }
    else
    {
      // 没有新目标 / 末端位姿不新鲜 / 收到停止请求时都会走到这里。
      // 最常见的是操作者停手、目标位姿流停了超过 TARGET_POSE_TIMEOUT——不是错误，
      // 所以这里用 WARN 而不是 ERROR。
      RCLCPP_WARN_STREAM(LOGGER, "PoseTracking 返回: "
                                     << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(status));
    }
  }

  tracker.stopMotion();
  return EXIT_SUCCESS;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("servo_pose_tracking");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  const int rc = run(node);

  // 统一在这里收尾：任何早退路径都不会漏掉 join（否则 joinable 的 std::thread
  // 析构会 std::terminate，表现为 "terminate called without an active exception"）
  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return rc;
}
