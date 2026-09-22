#!/usr/bin/env python3
"""阶段 2b：用 MoveGroup action 做一次规划 + 执行。

用途：验证「MoveIt 规划 -> moveit_simple_controller_manager -> JTC -> MuJoCo」整条接线。
这一步通过后，MoveIt Servo 要用的那条路（规划场景 + 控制器接口）就是通的。

用法（需先跑 stage2a 与 stage2b）：
    python3 scripts/plan_execute_test.py
    python3 scripts/plan_execute_test.py --target 0.5 -0.8 1.2 0.3 0.6 -0.4
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

JOINTS = [
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
]

# MoveGroup action 的 error_code：1 = SUCCESS
MOVEIT_ERROR_TEXT = {
    1: "SUCCESS",
    -1: "FAILURE",
    -2: "PLANNING_FAILED",
    -3: "INVALID_MOTION_PLAN",
    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -5: "CONTROL_FAILED",
    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -7: "TIMED_OUT",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "NO_IK_SOLUTION",
}


class PlanExecuteTest(Node):
    def __init__(self):
        super().__init__("plan_execute_test")
        self._client = ActionClient(self, MoveGroup, "/move_action")

    def run(self, targets, plan_only):
        self.get_logger().info("等待 /move_action ...")
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("/move_action 不可用——stage2b 起来了吗？")
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = "manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        constraints = Constraints()
        for name, pos in zip(JOINTS, targets):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False

        self.get_logger().info(f"目标: {dict(zip(JOINTS, targets))}")
        send = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("目标被拒绝")
            return False
        self.get_logger().info("目标已接受，等待结果...")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("等待结果超时")
            return False

        code = result.result.error_code.val
        name = MOVEIT_ERROR_TEXT.get(code, f"未知({code})")
        traj = result.result.planned_trajectory.joint_trajectory
        n_pts = len(traj.points)
        dur = traj.points[-1].time_from_start if n_pts else None
        self.get_logger().info(
            f"结果: error_code={code} ({name})  轨迹点数={n_pts}  "
            f"时长={dur.sec if dur else '-'}.{dur.nanosec // 10**6 if dur else '-'}s"
        )
        return code == 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, nargs=6,
                    default=[0.5, -0.8, 1.2, 0.3, 0.6, -0.4],
                    help="六个关节目标（按 JOINTS 顺序）")
    ap.add_argument("--plan-only", action="store_true", help="只规划不执行")
    args = ap.parse_args()

    rclpy.init()
    node = PlanExecuteTest()
    try:
        ok = node.run(args.target, args.plan_only)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("阶段 2b:", "✅ 通过" if ok else "❌ 失败")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
