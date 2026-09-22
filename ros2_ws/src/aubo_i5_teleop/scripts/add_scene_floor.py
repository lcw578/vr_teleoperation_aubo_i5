#!/usr/bin/env python3
"""把地面加进 MoveIt 的 planning scene。

为什么需要：地面只存在于 MJCF 里，**URDF 里没有任何场景几何**。
MoveIt 和 Servo 只看 URDF，所以它们完全不知道地面存在。
实测确认过后果：命令能把末端推到 z<0，MuJoCo 的地面把它顶住，
关节停在半路、状态与命令不一致，而且很容易落进奇异位形。

做法：发布一个 moveit_msgs/CollisionObject 到 /collision_object，
MoveIt 的 planning scene monitor 会把它并进场景，之后 Servo 的
`scene_collision_proximity_threshold` 就能看见它了。

形状用**薄长方体**而不是平面——MoveIt 的碰撞体必须是实体，没有无限平面。

═══ 2026-09-22 修正：上表面不能正好放在 z=0 ═══
原先取 4×4×0.2 m、中心 z=-0.1（上表面正好 z=0，"与 MJCF 地面一致"）。这是错的，
而且造成了整条阶段 3 一直在查的那个"无指令下沉"：

  机器人**就站在 z=0 上**（base_link 底面在 z=0），所以它的部件与这块地面上表面的
  距离恒为 0。Servo 用的不是"接触判定"而是**接近度阈值**
  （servo.yaml 的 scene_collision_proximity_threshold: 0.02 = 2 cm），
  距离 0 永远落在阈值内 → Servo **持续降速/急停** → 输出退化成"保持在当前位置"。
  而在 MuJoCo 里，位置执行器的力矩是 kp·(ctrl - q)：ctrl 跟着 q 走时这项恒为 ~0，
  **弹簧被抵消，重力无人对抗**，机械臂就以约 0.045 rad/s 匀速下沉（对应末端 ~6 cm/s）。

  证据链（都在本次调试记录里）：
   - 关掉 check_collisions 后，零误差目标下的实际位移从 -0.406 m 变成 +0.0034 m；
   - /check_state_validity 显示 ag95_body / wrist2_Link 与 floor 接触（那是下沉到地面之后的后果）；
   - ready 位姿本身 valid=True（所以不是"真碰撞"，而是接近度阈值）。

修正：把上表面降到 **z = -0.03 m**，比 2 cm 阈值再留 1 cm 余量。
代价（明确记录）：planning scene 里的地面比 MuJoCo 的物理地面低 3 cm，
所以 Servo 允许末端下探到 z=-0.03；再往下由 MuJoCo 的地面挡住。
这个折中是必要的——只要地面上表面落在阈值内，遥操就不可用。

用法（在 stage2b 之后）：
    python3 scripts/add_scene_floor.py
"""

import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

FLOOR_ID = "floor"
HALF = 2.0          # 4×4 m
THICK = 0.2         # 厚 0.2 m
TOP_Z = -0.03       # 上表面高度：必须比 scene_collision_proximity_threshold(0.02) 更低，
                    # 否则机器人与地面恒在阈值内，Servo 会永久降速/急停。


class FloorPublisher(Node):
    def __init__(self, period=2.0):
        super().__init__("add_scene_floor")
        self.pub = self.create_publisher(CollisionObject, "/collision_object", 1)
        self.first = True
        # 周期性重发（每 2 秒）而不是只发一次：MoveIt 的 planning scene 会记住物体，
        # 但重发能cover MoveIt 重启的情况，长时间遥操更稳。
        self.timer = self.create_timer(period, self._tick)

    def build(self):
        obj = CollisionObject()
        obj.header.frame_id = "world"     # 与 servo_pose_tracking.yaml 的 planning_frame 一致
        obj.id = FLOOR_ID
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [2 * HALF, 2 * HALF, THICK]
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = TOP_Z - THICK / 2.0   # 上表面落在 TOP_Z
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _tick(self):
        self.pub.publish(self.build())
        if self.first:
            self.first = False
            self.get_logger().info(
                "已发布地面 collision object：4x4x%.1f m，上表面 z=%.2f"
                "（比物理地面低，避免与机器人底座恒在接近度阈值内）" % (THICK, TOP_Z)
            )


def main():
    rclpy.init()
    n = FloorPublisher()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
