"""阶段 3（新路线）：MoveIt Servo 的 pose tracking 节点。

与官方示例的对应关系：
  参数装配方式照抄 /opt/ros/humble/share/moveit_servo/launch/pose_tracking_example.launch.py
  ——它把 **两个 yaml 合并进同一个 moveit_servo 命名空间**：
      ParameterBuilder("moveit_servo").yaml("config/pose_tracking_settings.yaml")
                                        .yaml("config/panda_simulated_config_pose_tracking.yaml")
  这里用等价的字典合并实现（我们的其它 launch 也是这个写法，不引入 launch_param_builder 依赖）。
  因为参数名是 `moveit_servo.<键>`（launch_ros 会把嵌套字典按 "." 扁平化），
  PID 参数（x_proportional_gain 等）和 Servo 参数共用这个命名空间。

本 launch 只起 **pose tracking 节点本身**：
  - 不起 ros2_control / MuJoCo（那是 stage2a）
  - 不起 move_group（那是 stage2b）——move_group 持主 planning scene，
    本节点的 planning scene monitor 只接收不对外提供（见 pose_tracking_node.cpp 的说明）

目标位姿入口：话题 target_pose（PoseStamped），默认在 /target_pose。
  仿真里由 scripts/pose_target_test.py 代替 VR 手柄来发；将来换成 oculus_reader 的映射层。

用法：
    ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py     # 终端 1
    ros2 launch aubo_i5_teleop stage2b_moveit.launch.py     # 终端 2
    ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py   # 终端 3
    python3 scripts/pose_target_test.py --axis z --speed 0.02 --duration 5   # 终端 4
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
    srdf_file = LaunchConfiguration("srdf_file").perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_i5_teleop"), "urdf", "aubo_i5_teleop.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # SRDF 必须与 URDF 同名（aubo_i5_robot），否则 Servo 找不到 move_group_name 指定的组
    srdf_path = os.path.join(teleop_share, "config", srdf_file)
    with open(srdf_path) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics_yaml = load_yaml_abs(
        os.path.join(
            get_package_share_directory("aubo_moveit_config"), "config", "kinematics.yaml"
        )
    )
    joint_limits_yaml = {
        "robot_description_planning": load_yaml_abs(
            os.path.join(teleop_share, "config", "joint_limits.yaml")
        )
    }

    # 两个 yaml 合并进同一个 moveit_servo 命名空间（照官方 launch 的做法）
    servo_cfg = load_yaml_abs(
        os.path.join(teleop_share, "config", "servo_pose_tracking.yaml")
    )
    pid_cfg = load_yaml_abs(
        os.path.join(teleop_share, "config", "pose_tracking_settings.yaml")
    )
    merged = {}
    merged.update(pid_cfg or {})
    merged.update(servo_cfg or {})

    # 碰撞检查的 A/B 开关。为什么需要它而不是 ros2 param set：
    # 实测 check_collisions 在节点运行中用 ros2 param set 改**不生效**
    # （status 仍恒为 3 = DECELERATE_FOR_COLLISION，跟随度不变），
    # 这个参数看来只在构造碰撞检查器时读一次，所以必须重启才有效。
    cc = LaunchConfiguration("check_collisions").perform(context).lower()
    if cc in ("1", "true"):
        merged["check_collisions"] = True
    elif cc in ("0", "false"):
        merged["check_collisions"] = False

    # 接近度阈值的二分用开关（同样只能重启生效）。
    # 传 0.0 等于关掉那一类检查（`distance < 0` 永不成立）。
    # 用途：把"Servo 持续 DECELERATE_FOR_COLLISION"分成自碰撞还是场景碰撞。
    for arg, key in (("self_collision_proximity_threshold", "self_collision_proximity_threshold"),
                     ("scene_collision_proximity_threshold", "scene_collision_proximity_threshold")):
        v = LaunchConfiguration(arg).perform(context)
        if v != "":
            merged[key] = float(v)

    # PID 增益覆盖（调参用）。同样只在构造时读取，所以必须重启才生效。
    # 只覆盖平移三项；角向暂不动（当前任务以平移跟随验证为主）。
    for arg, keys in (("pid_p", ["x_proportional_gain", "y_proportional_gain", "z_proportional_gain"]),
                      ("pid_i", ["x_integral_gain", "y_integral_gain", "z_integral_gain"]),
                      ("pid_angular", ["angular_proportional_gain"])):
        v = LaunchConfiguration(arg).perform(context)
        if v != "":
            for k in keys:
                merged[k] = float(v)

    # 输出模式：位置（默认）或速度。
    # 速度模式只需改这三项——**Servo 侧不用改代码**：
    # servo_calcs.cpp 的 Float64MultiArray 分支是 `if (positions) ... else if (velocities)`，
    # 关掉位置、打开速度，它自然就把速度数组发出去。
    # 注意：速度值来自 applyJointUpdate 里对位置命令做差分再除以 publish_period，
    # 所以平滑滤波器仍然在通路上（这一点与位置模式一致）。
    output_mode = LaunchConfiguration("output_mode").perform(context).lower()
    if output_mode == "velocity":
        merged["publish_joint_positions"] = False
        merged["publish_joint_velocities"] = True
        merged["command_out_topic"] = "/forward_command_controller_velocity/commands"
    else:
        merged["publish_joint_positions"] = True
        merged["publish_joint_velocities"] = False
        # ⚠️ 位置模式下 Servo 的输出**不能**直接发给控制器，要先过"伺服接口层"
        #    （scripts/servo_interface.py）。原因：Servo 的流是 `实测 + Δθ(单周期)`，
        #    锚在实测位置上，位置执行器的弹簧永远只被压一个周期、误差建不起来，
        #    实测速度增益只有 0.140–0.753；把同一串增量搬到绝对命令上则变成 1.000。
        #    这一步等价于厂家 servoj 的"速度前馈"那一环，不是我们发明的控制律。
        #    （2026-09-24 曾改用 mujoco_ros2_control 的内置 PID 来承担，已否决：
        #      它的 error ≡ Δ、与实际位置无关 → 静止无保持力矩、运动时积分冲到限幅。）
        merged["command_out_topic"] = "/servo_position_stream"

    # publish_period 同时决定 Servo 的输出周期与 PoseTracking 的 PID 循环频率，
    # 也是"位置模式跟随比值 = period/(period+δ)"里的那个 period —— 排查状态延迟 δ 时用它做扫描。
    pp = LaunchConfiguration("publish_period").perform(context)
    if pp != "":
        merged["publish_period"] = float(pp)

    # 奇异阈值覆盖（排查"保护性缩放/急停"对跟随的贡献时用；设成很大值即等于关掉保护）
    for arg, key in (("lower_singularity_threshold", "lower_singularity_threshold"),
                     ("hard_stop_singularity_threshold", "hard_stop_singularity_threshold")):
        v = LaunchConfiguration(arg).perform(context)
        if v != "":
            merged[key] = float(v)

    ll = LaunchConfiguration("low_latency_mode").perform(context).lower()
    if ll in ("1", "true"):
        merged["low_latency_mode"] = True
    elif ll in ("0", "false"):
        merged["low_latency_mode"] = False

    # 滤波器系数（节点根命名空间的那一个）
    bfc = LaunchConfiguration("butterworth_filter_coeff").perform(context)

    servo_params = {"moveit_servo": merged}

    # ⚠️ 是否把运动学求解器交给 Servo，是一个**会改变控制律**的选择，不是可有可无的配置：
    #   servo_calcs.cpp:186-197 —— 若组有 IK 求解器且插件支持该组，Servo 走 **IK 路径**；
    #   否则打印 "No kinematics solver instantiated ... Will use inverse Jacobian" 并走
    #   **雅可比伪逆路径**。
    #   实测（2026-09-22）：我们的 aubo_moveit_config/config/kinematics.yaml 给 manipulator
    #   配了 kdl_kinematics_plugin/KDLKinematicsPlugin，于是 Servo 走 IK 路径；而 Servo 给 IK 的
    #   预算是 publish_period/2 = 2.5 ms（比 KDL 自己配的 kinematics_solver_timeout 5 ms 还短），
    #   并开了 opts.return_approximate_solution = true。结果：发纯 +z 1cm/s 走 3 s，
    #   Servo 算出的关节命令经 MJCF 正运动学是 **-0.211 m（向下）**，方向反、幅度大 7 倍。
    #   用 use_ik_solver:=true 可以复现那条 IK 路径，便于对比。
    use_ik_solver = LaunchConfiguration("use_ik_solver").perform(context).lower() in ("1", "true")

    params = [
        servo_params,
        robot_description,
        robot_description_semantic,
        joint_limits_yaml,
    ]
    if use_ik_solver:
        params.append(kinematics_yaml)

    pose_tracking_node = Node(
        package="aubo_i5_teleop",
        executable="pose_tracking_node",
        name="servo_pose_tracking",
        output="screen",
        parameters=params + [
            # ⚠️ 必须显式设 true：PoseTracking 内部用 node 时钟建 TF buffer
            #   （pose_tracking.cpp 构造函数 transform_buffer_(node_->get_clock())），
            #   而 haveRecentTargetPose/EndEffectorPose 是拿节点时钟和 TF 时间戳比的。
            #   官方 launch 没写这一项，我们是仿真时间，不写会直接报
            #   "The end effector pose was not updated in time. Aborting."
            {"use_sim_time": True},
            # ⚠️ 滤波器的系数由**插件**在节点根命名空间读取（ParamListener 的 prefix 默认为空），
            #    所以它**不能**放进 moveit_servo 块里——放进去会被静默忽略、实际用默认 1.5。
            #    默认值 1.5；库限制系数必须 >= 1（<1 会让滤波器不稳定）；越大越平滑、滞后越大。
            *([{"butterworth_filter_coeff": float(bfc)}] if bfc else []),
        ],
    )

    nodes = [pose_tracking_node]
    if output_mode != "velocity":
        # 位置模式下叠加"伺服接口层"：把 Servo 的"实测+增量"流积分成绝对位置命令，
        # 再发给 forward_command_controller_position（它订阅自己的 ~/commands 话题）。
        nodes.append(Node(
            package="aubo_i5_teleop",
            executable="servo_interface.py",
            name="servo_interface",
            output="screen",
            parameters=[{
                "input_topic": "/servo_position_stream",
                "output_topic": "/forward_command_controller_position/commands",
                "lag_max": 0.15,   # 2026-09-24 与 servo_interface.py 默认一起从 0.3 收紧（见该文件头的说明）
                "use_sim_time": True,
            }],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "srdf_file",
                default_value="teleop.srdf",
                description="SRDF 文件名（须与 URDF 的 robot name 一致）",
            ),
            DeclareLaunchArgument(
                "use_ik_solver",
                default_value="false",
                description="是否把运动学求解器传给 Servo（true = 走 IK 路径，false = 走雅可比伪逆路径）。"
                            "默认 false；详见 launch_setup 里的说明。",
            ),
            DeclareLaunchArgument(
                "check_collisions",
                default_value="true",
                description="覆盖 servo_pose_tracking.yaml 里的 check_collisions。"
                            "默认 true；设 false 用于排查碰撞降速（该参数只能靠重启生效）。",
            ),
            DeclareLaunchArgument("self_collision_proximity_threshold", default_value="",
                                  description="覆盖自碰撞接近度阈值；0.0 = 关掉该类检查。空 = 用 yaml 值。"),
            DeclareLaunchArgument("scene_collision_proximity_threshold", default_value="",
                                  description="覆盖场景碰撞接近度阈值；0.0 = 关掉该类检查。空 = 用 yaml 值。"),
            DeclareLaunchArgument("pid_p", default_value="",
                                  description="覆盖 x/y/z 比例增益（调参用，需重启生效）。空 = 用 yaml 值。"),
            DeclareLaunchArgument("pid_i", default_value="",
                                  description="覆盖 x/y/z 积分增益（调参用，需重启生效）。空 = 用 yaml 值。"),
            DeclareLaunchArgument("publish_period", default_value="",
                                  description="覆盖 publish_period（秒）。它同时是 Servo 输出周期与 "
                                              "PoseTracking 的 PID 循环频率；默认空=用 yaml 的 0.005。"
                                              "排查状态延迟时用（跟随比值 = period/(period+δ)）。"),
            DeclareLaunchArgument("lower_singularity_threshold", default_value="",
                                  description="覆盖 lower_singularity_threshold（默认 50）。设很大=关掉奇异降速。"),
            DeclareLaunchArgument("hard_stop_singularity_threshold", default_value="",
                                  description="覆盖 hard_stop_singularity_threshold（默认 200）。设很大=关掉奇异急停。"),
            DeclareLaunchArgument("low_latency_mode", default_value="",
                                  description="覆盖 low_latency_mode（true/false）。空=用 yaml 值（我们为 false，LARA 为 true）。"),
            DeclareLaunchArgument("butterworth_filter_coeff", default_value="1.5",
                                  description="滤波系数（必须在**节点根命名空间**读，故不能放 yaml 的 moveit_servo 块）。"
                                              "默认 1.5 = 插件默认值。2026-09-23 实测：速度模式下系数 30 会让环路进入"
                                              "极限环（命令长期饱和在 ±3.15 rad/s、末端 ~25 Hz 抖动），系数 1.5 安静；"
                                              "代价是平滑变弱。扫描见 config/pose_tracking_settings.yaml 的注释。"),
            DeclareLaunchArgument("pid_angular", default_value="",
                                  description="覆盖 angular_proportional_gain（调参用，需重启生效）。空 = 用 yaml 值。"),
            DeclareLaunchArgument(
                "output_mode",
                default_value="position",
                choices=["position", "velocity"],
                description="Servo 的输出形式：position（默认，2026-09-24 改回，发位置给 "
                            "forward_command_controller_position）或 velocity。"
                            "为什么是 position：本款机械臂不支持速度控制（厂家驱动的速度接口是死壳），"
                            "位置模式与真机一致。必须与 stage2a 的 arm_control_mode / mujoco_model 一致。",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
