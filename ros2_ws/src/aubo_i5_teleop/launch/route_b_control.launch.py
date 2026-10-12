"""Route B input, mapper, tracker, gripper and optional recording/cameras.
Run after the selected simulator is ready and go_ready has completed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clock = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    python = LaunchConfiguration("python_executable")
    common = dict(package="aubo_i5_teleop", output="screen", prefix=[python, " "])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("python_executable", default_value="/home/lcw/tomato_robot/.venv/bin/python"),
        DeclareLaunchArgument("input", default_value="quest", choices=["quest", "mock_vr"]),
        DeclareLaunchArgument("quest", default_value="true"),
        DeclareLaunchArgument("relay_uri", default_value="wss://127.0.0.1:8443/ws"),
        DeclareLaunchArgument("cameras", default_value="true"),
        DeclareLaunchArgument("recording", default_value="false",
                              description="是否启动 rosbag logger；默认关闭，避免普通遥操作占用磁盘"),
        DeclareLaunchArgument("scene"),
        Node(executable="quest_adapter_node.py", arguments=["--uri", LaunchConfiguration("relay_uri")], parameters=[clock],
             condition=IfCondition(LaunchConfiguration("quest")), **common),
        Node(executable="clutch_mapper_node.py", arguments=["--input", LaunchConfiguration("input"), "--no-scale-toggle"],
             parameters=[clock], **common),
        Node(executable="lara_style_tracker.py", parameters=[clock], **common),
        Node(executable="gripper_fsm_node.py", parameters=[clock, {"input_prefix": LaunchConfiguration("input")}], **common),
        Node(executable="controller_axes_viz.py", arguments=["--input", LaunchConfiguration("input")], parameters=[clock], **common),
        Node(executable="multi_camera_renderer.py", arguments=["--scene", LaunchConfiguration("scene")],
             parameters=[clock], condition=IfCondition(LaunchConfiguration("cameras")), **common),
        Node(executable="session_manager.py", parameters=[clock],
             condition=IfCondition(LaunchConfiguration("recording")), **common),
    ])
