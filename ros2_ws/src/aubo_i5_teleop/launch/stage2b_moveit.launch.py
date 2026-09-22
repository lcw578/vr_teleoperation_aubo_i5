"""阶段 2b：在已运行的阶段 2a（MuJoCo + 控制器）之上，只起 move_group + RViz2。

与阶段 1 的 launch 有两处关键区别：
  - **不起 joint_state_publisher**：阶段 1 没有控制器，需要一个假的关节状态源；
    阶段 2 里 MuJoCo 通过 joint_state_broadcaster 提供真实 /joint_states，
    再起一个假的会让 MoveIt 在两套状态之间跳。
  - joint_limits 用本包 config/joint_limits.yaml（含 ±3.04 位置限位），
    不用官方那份（官方那份没有位置限位，会让 MoveIt 按 URDF 的 ±360° 规划）。

用法（先跑阶段 2a）：
    ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py      # 终端 1
    ros2 launch aubo_i5_teleop stage2b_moveit.launch.py      # 终端 2
    # 终端 3：用 MoveGroup action 做一次规划+执行（见 scripts/plan_execute_test.py）
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


def load_yaml(package_name, file_path):
    absolute = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(absolute) as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def load_yaml_abs(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    teleop_share = get_package_share_directory("aubo_i5_teleop")
    srdf_file = LaunchConfiguration("srdf_file").perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("aubo_i5_teleop"),
                    "urdf",
                    "aubo_i5_teleop.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # SRDF 用本包自己的 teleop.srdf（臂 + 夹爪合并），不是官方那份——
    # 官方那份不含夹爪，缺了夹爪的碰撞排除会让 MoveIt 认为永远自碰撞。
    srdf_path = os.path.join(teleop_share, "config", srdf_file)
    with open(srdf_path) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics_yaml = load_yaml("aubo_moveit_config", "config/kinematics.yaml")

    # 用本包的限位（含 ±3.04 位置限位），不是官方那份
    joint_limits_yaml = {
        "robot_description_planning": load_yaml_abs(
            os.path.join(teleop_share, "config", "joint_limits.yaml")
        )
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
            "sample_duration": 0.005,
        }
    }
    ompl_planning_yaml = load_yaml("aubo_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "aubo_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("aubo_i5_teleop"), "config", "teleop.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    # 把地面加进 planning scene —— 地面只在 MJCF 里，URDF 里没有，
    # 所以 MoveIt 和 Servo 本来完全看不见它（实测能把末端命令到 z<0）。
    floor_node = Node(
        package="aubo_i5_teleop",
        executable="add_scene_floor.py",
        name="add_scene_floor",
        output="log",
    )

    return [move_group_node, rviz_node, floor_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "srdf_file",
                default_value="teleop.srdf",
                description="SRDF 文件名",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
