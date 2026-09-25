#!/usr/bin/env python3
"""把 MJCF 场景几何（地板 + 底盘 + 升降立柱）加进 MoveIt 的 planning scene。

为什么需要：场景几何只存在于 MJCF 里，**URDF 里没有任何场景几何**。
MoveIt 和 Servo 只看 URDF，所以它们完全不知道这些障碍物存在。
实测确认过后果（2026-09-22）：命令能把末端推到 z<0，MuJoCo 的地面把它顶住，
关节停在半路、状态与命令不一致；2026-09-24 又踩过一次"幽灵地板"
（MJCF 地板降到 -1.2 而这里还是 -0.03 → 规划场景里多出 1.17 m 高的幽灵地板，
机械臂一往下走就碰撞急停）。

2026-09-25 扩展：把底盘与立柱也加进来。此前它们只在 MJCF 里——
"物理挡住、Servo 看不见"的错配（与幽灵地板同类）。当前 8 个测试位姿都在
基座正前方 700–944 mm、碰不到立柱，所以加进来对现有测量无影响（已验证）；
但它让基座附近/后方的位姿在规划场景里也是真实受限的。

做法：发布 moveit_msgs/CollisionObject 到 /collision_object，MoveIt 的
planning scene monitor 把它们并进场景，Servo 的 scene_collision_proximity_threshold
就能看见。形状用**长方体**（MoveIt 碰撞体必须是实体）。每 2 秒重发一次，
覆盖 MoveIt 重启的情况。

⚠️ 尺寸/位置必须与 MJCF 逐字一致（scene_ros2.xml 里都是**半**尺寸，
   SolidPrimitive 用**全**尺寸——改 MJCF 时同步改这里）：
   · floor:          4×4×0.2 m 盒，上表面 z = −1.23（= MJCF 地板 −1.2 再低 3 cm，
                     低于 2 cm 接近度阈值，否则机器人与地面恒在阈值内 → 永久降速）
   · chassis:        1.30×0.84×0.49 m（半尺寸 0.65 0.42 0.245），中心 z = −0.955
   · platform_column:0.25×0.25×0.705 m（半尺寸 0.125 0.125 0.3525），中心 z = −0.3575

用法（在 stage2b 之后）：
    python3 scripts/add_scene_floor.py
"""

import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

# (id, 全尺寸 [x,y,z], 中心 z)。x/y 居中于原点；姿态恒等。
# ⚠️ 与 scene_ros2.xml 的 worldbody 里对应 geom 逐字核对过（2026-09-25）。
OBJECTS = [
    # 上表面 z=−1.23：= MJCF 地板（−1.2）再低 3 cm，避开 2 cm 接近度阈值（见上）
    ("floor", [4.0, 4.0, 0.2], -1.23 - 0.2 / 2.0),
    # MJCF: size="0.65 0.42 0.245" pos="0 0 -0.955"（半尺寸）
    ("chassis", [1.30, 0.84, 0.49], -0.955),
    # MJCF: size="0.125 0.125 0.3525" pos="0 0 -0.3575"（半尺寸）
    ("platform_column", [0.25, 0.25, 0.705], -0.3575),
]


class ScenePublisher(Node):
    def __init__(self, period=2.0):
        super().__init__("add_scene_floor")
        self.pub = self.create_publisher(CollisionObject, "/collision_object", 1)
        self.first = True
        # 周期性重发（每 2 秒）而不是只发一次：MoveIt 重启后场景会清空，重发能自愈。
        self.timer = self.create_timer(period, self._tick)

    def build(self, obj_id, dims, center_z):
        obj = CollisionObject()
        obj.header.frame_id = "world"     # 与 servo_pose_tracking.yaml 的 planning_frame 一致
        obj.id = obj_id
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(dims)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = center_z
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _tick(self):
        for obj_id, dims, cz in OBJECTS:
            self.pub.publish(self.build(obj_id, dims, cz))
        if self.first:
            self.first = False
            desc = ", ".join("%s %.2fx%.2fx%.2f@z=%.3f" % (i, d[0], d[1], d[2], c)
                             for i, d, c in OBJECTS)
            self.get_logger().info("已发布场景 collision objects：%s" % desc)


def main():
    rclpy.init()
    n = ScenePublisher()
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
