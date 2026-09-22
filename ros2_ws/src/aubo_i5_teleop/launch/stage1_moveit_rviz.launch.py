"""阶段 1：只起 MoveIt + RViz2，确认能看到 Aubo i5。

不接控制器、不接 MuJoCo 物理——这一步只验证运动学模型与 MoveIt 配置是否正确加载。
刻意不发 world->base_link 的静态 TF：URDF 里已经有 world_joint 定义了这个变换，
再发一遍会让 TF 树出现两个来源（官方 aubo_moveit.launch.py 就有这个问题）。

用法：
    ros2 launch aubo_i5_teleop stage1_moveit_rviz.launch.py
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
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    srdf_file = LaunchConfiguration("srdf_file")

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

    # SRDF 用本包自己的 teleop.srdf（臂 + AG95 夹爪合并），不是官方那份——
    # 官方那份不含夹爪，缺了夹爪的碰撞排除会让 MoveIt 认为机器人永远自碰撞。
    srdf_path = os.path.join(
        get_package_share_directory("aubo_i5_teleop"),
        "config",
        srdf_file.perform(context),
    )
    with open(srdf_path) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # 关节限位也用本包那份（含 ±3.04 位置限位）；官方那份只有速度、没有位置限位
    kinematics_yaml = load_yaml("aubo_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = {
        "robot_description_planning": load_yaml_abs(
            os.path.join(
                get_package_share_directory("aubo_i5_teleop"),
                "config",
                "joint_limits.yaml",
            )
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
        ],
    )

    # 用本包的 rviz 配置：官方那份焦点在基座附近 (z=0.138)，看不到顶端的夹爪
    rviz_config = os.path.join(
        get_package_share_directory("aubo_i5_teleop"), "config", "teleop.rviz"
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
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 阶段 1 没有控制器，所以没有 /joint_states 来源。
    # 用 joint_state_publisher 发一个静态零位，让 MoveIt 有初始状态可用
    # （官方 launch 里定义了 joint_state_publisher_gui 却没加进 nodes_to_start，
    #  结果 move_group 要空等 10 秒）。
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log",
        parameters=[robot_description],
    )

    return [
        move_group_node,
        rviz_node,
        robot_state_pub_node,
        joint_state_publisher_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="提供 SRDF / kinematics / joint_limits 的包",
        ),
        DeclareLaunchArgument(
            "srdf_file",
            default_value="teleop.srdf",
            description="SRDF 文件名（aubo_robot.srdf 或 aubo_i5.srdf，内容一致）",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
