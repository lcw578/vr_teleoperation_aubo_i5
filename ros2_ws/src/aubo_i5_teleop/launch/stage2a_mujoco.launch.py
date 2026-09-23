"""阶段 2a：ros2_control + MuJoCo + JTC。不接 MoveIt，不接 Servo。

目的：验证「ros2_control 控制器 -> MuJoCo 物理」这条链路。
验收：直接给 JTC 发一条 FollowJointTrajectory，机械臂在 MuJoCo 窗口里动起来。

关键点（都与标准 ros2_control 不同）：
  - 必须用 mujoco_ros2_control 自带的 ros2_control_node，不能用 controller_manager 的标准节点。
  - 必须传 use_sim_time: True。
  - MuJoCo 会另开一个窗口（headless 默认 false），可以看着机械臂动。

用法：
    ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py
    # 另开终端：
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint],
        points: [{positions: [0.5, -0.8, 1.2, 0.3, 0.6, -0.4], time_from_start: {sec: 3}}]}}"
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    teleop_share = get_package_share_directory("aubo_i5_teleop")
    controllers_file = os.path.join(teleop_share, "config", "controllers.yaml")

    # 2026-09-22 修正：原来这里没有把下面声明的 mujoco_model 参数传给 xacro，
    # 于是 `ros2 launch ... mujoco_model:=<别的文件>` 被**静默忽略**，永远加载 xacro 里的默认值。
    # 排查"无指令下沉"时用这个参数做无重力对照实验，才发现参数根本没生效。
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
            " ",
            "mujoco_model:=",
            LaunchConfiguration("mujoco_model"),
            " ",
            # 命令接口类型必须与 MJCF 的执行器类型一致，否则 mujoco_ros2_control 会在
            # register_urdf_joints 里抛异常并 abort（实测过）。
            "arm_control_mode:=",
            LaunchConfiguration("arm_control_mode"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # mujoco_ros2_control 自带的节点（不是 controller_manager 的标准 ros2_control_node）
    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
            controllers_file,
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # 先起 joint_state_broadcaster，成功后再起轨迹控制器
    # （直接并行起的话 JTC 会因为拿不到关节状态而激活失败）
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
        output="both",
    )
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
        output="both",
    )

    # 臂的控制器：位置或速度，二选一。
    # 为什么必须二选一：mujoco_ros2_control 对命令接口类型是**互斥**处理的
    # （mujoco_system_interface.cpp：激活 position 就打印
    #  "position control enabled (velocity, effort disabled)"，激活 velocity 反之），
    # 所以两个都 spawn 只会让后激活的覆盖前者，行为不可预期。
    # 选哪个由 arm_control_mode 参数决定；速度模式还需配套
    #   mujoco_model:=.../scene_ros2_velocity.xml
    # 且 stage3 用 output_mode:=velocity。
    arm_control_mode = LaunchConfiguration("arm_control_mode").perform(context).lower()
    arm_controller = ("forward_command_controller_velocity" if arm_control_mode == "velocity"
                      else "forward_command_controller_position")
    fwd_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            arm_controller,
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file,
        ],
        output="both",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
        output="both",
    )

    return [
        control_node,
        robot_state_pub_node,
        jsb_spawner,
        # 注意：这里刻意**不启动 JTC**。JTC 与 forward_command_controller_position
        # 争夺同一个命令接口（shoulder_joint/position），只能二选一。
        # Servo 是高频流式位置命令，需要 forward_command_controller；
        # 代价是 MoveIt 的 action 式轨迹执行（FollowJointTrajectory）暂时不可用。
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[gripper_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gripper_spawner,
                on_exit=[fwd_spawner],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mujoco_model",
                default_value="/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml",
                description="传给 URDF 的 MuJoCo 模型路径",
            ),
            DeclareLaunchArgument(
                "arm_control_mode",
                default_value="position",
                choices=["position", "velocity"],
                description="臂的控制器类型：position（默认）或 velocity。"
                            "速度模式需同时给 mujoco_model:=scene_ros2_velocity.xml，"
                            "并在 stage3 用 output_mode:=velocity。",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
