"""阶段 3：MoveIt Servo。

在已运行的 stage2a（MuJoCo + 控制器）与 stage2b（move_group + RViz）之上，
只加一个 servo_node。Servo 做三件事：
  1. 把末端的速度意图（Twist）解算成关节指令
  2. 用**基于当前构型的局部雅可比迭代**——所以不会像解析 IK 那样突然"翻臂"
     （这正是方案文档里标为"高概率高影响"的那条风险，Servo 从架构上绕过）
  3. 同时做自碰撞检查、奇异位形减速、关节限位保护

依赖：**move_group 必须在跑**（Servo 的碰撞检查依赖它发布的 planning scene），
而且 servo.yaml 里 `is_primary_planning_scene_monitor` 必须是 false。

启动顺序：
    ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py     # 终端 1
    ros2 launch aubo_i5_teleop stage2b_moveit.launch.py     # 终端 2
    ros2 launch aubo_i5_teleop stage3_servo.launch.py       # 终端 3
    # 然后：
    ros2 service call /servo_node/start_servo std_srvs/srv/Trigger   # 必须先启动 Servo
    python3 lib/<pkg>/servo_twist_test.py                      # 持续发 Twist 并观察
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml_abs(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    teleop_share = get_package_share_directory("aubo_i5_teleop")
    moveit_share = get_package_share_directory("aubo_moveit_config")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_i5_teleop"), "urdf", "aubo_i5_teleop.urdf.xacro"]
            ),
        ]
    )

    srdf_file = LaunchConfiguration("srdf_file").perform(context)
    with open(os.path.join(teleop_share, "config", srdf_file)) as f:
        semantic = {"robot_description_semantic": f.read()}

    servo_yaml = load_yaml_abs(os.path.join(teleop_share, "config", "servo.yaml"))
    servo_params = {"moveit_servo": servo_yaml}

    # 滤波器系数：**节点根命名空间**（插件 ParamListener 的 prefix 默认为空），
    # 放 moveit_servo 块里会被静默忽略。排查"滤波器对命令的衰减 α"时用它做扫描。
    bfc = LaunchConfiguration("butterworth_filter_coeff").perform(context)
    extra = [{"butterworth_filter_coeff": float(bfc)}] if bfc else []

    kinematics = load_yaml_abs(os.path.join(moveit_share, "config", "kinematics.yaml"))

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="both",
        parameters=[
            servo_params,
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            semantic,
            {"robot_description_kinematics": kinematics},
            {"use_sim_time": True},
            *extra,
        ],
    )

    return [servo_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("butterworth_filter_coeff", default_value="",
                                  description="覆盖节点根命名空间的滤波系数（默认插件 1.5，>=1）。空=不设。"),
            DeclareLaunchArgument("srdf_file", default_value="teleop.srdf"),
            OpaqueFunction(function=launch_setup),
        ]
    )
