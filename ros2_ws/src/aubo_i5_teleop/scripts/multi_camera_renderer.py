#!/usr/bin/env python3
"""多路虚拟相机渲染节点（DATA_PIPELINE_PLAN.md 阶段 B）。

把 scene_ros2_demo.xml 里的三路虚拟相机（D405 腕部 + D435 右/左）渲染成
sensor_msgs/Image + CameraInfo。架构 = 旁路渲染：订阅 /joint_states →
按关节名回填 qpos → mj_forward → EGL 离屏渲染 → 发布。
不碰上游 mujocos_ros2_control（它没有相机能力）。

时间戳：图像 stamp 取最近一帧 /joint_states 的 header stamp——图像与
关节状态天然对齐（录制时同入一个 rosbag 即时间同步）。

D405 特殊处理：腕部相机必须跟随手臂运动，但上游机器人文件不可改（无法把
相机挂进 ag95_base），所以相机元素放 worldbody，本节点每帧用 ag95_base
的实时位姿覆写 d.cam_xpos/cam_xmat（世界位姿 = 基座位姿 × 局部标称位姿）。
局部标称：pos=(0,-0.075,0.10) target=(0,-0.01,0.15) up=基座 -y
（2026-09-25 渲染迭代定稿：两白垫对称入画、夹持区居中）。

针孔近似声明：渲染图 = 中心主点 + 零畸变；CameraInfo 按此发布
（K 由 fovy 反推：fy=(h/2)/tan(fovy/2)）。真机 D405 有 k1=-0.054 畸变与
13px 主点偏移——若 sim-to-real 出现视觉域差，优先在真车端 rectify。

用法：
  MUJOCO_GL=egl python3 multi_camera_renderer.py                 # 常驻（run_demo.sh 自动拉起）
  python3 multi_camera_renderer.py --selftest                    # 离线自测（无需 ROS）
"""

import argparse
import os
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")   # 必须在 import mujoco 之前

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None  # JPEG 流不可用时退化为仅 raw（selftest 不需要 cv2）

SCENE = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2_demo.xml"
RATE_HZ = 30.0

# 三路相机：名字 → (宽, 高, fovy度, frame_id)
CAMERAS = {
    "d405_wrist": (1280, 720, 58.0, "demo_d405_wrist_optical"),
    "d435_right": (640, 480, 42.6, "demo_d435_right_optical"),
    "d435_left": (640, 480, 42.6, "demo_d435_left_optical"),
}

# D405 腕部相机在 ag95_base 系的标称位姿（渲染迭代定稿，改这里即可调机位）
D405_POS_LOCAL = np.array([0.0, -0.075, 0.10])
D405_TARGET_LOCAL = np.array([0.0, -0.01, 0.15])
D405_UP_LOCAL = np.array([0.0, -1.0, 0.0])


class CameraRig:
    """加载场景 + 管理三路渲染（ROS 无关，selftest 与在线共用）。"""

    def __init__(self, scene_path: str):
        import mujoco
        self.mujoco = mujoco
        self.m = mujoco.MjModel.from_xml_path(scene_path)
        # ⚠️ 阴影关闭（2026-09-26 现场诊断）：与 MuJoCo 交互窗口的 GPU 争用 +
        #    阴影渲染，把 30Hz 压到 3.5Hz（实测首场录制）。关阴影后三路合计
        #    2.3ms（带阴影 1.9+0.7+0.7ms），争用下也有 10 倍余量。
        self.m.vis.quality.shadowsize = 0
        self.d = mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        mujoco.mj_forward(self.m, self.d)
        self.base_bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")
        self.cam_ids = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_CAMERA, n) for n in CAMERAS}
        # 关节名 → qpos 地址（/joint_states 按名回填用）
        self.jnt_qadr = {}
        for j in range(self.m.njnt):
            name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)
            self.jnt_qadr[name] = self.m.jnt_qposadr[j]
        self._renderers = {}
        for n, (w, h, _, _) in CAMERAS.items():
            self._renderers[n] = mujoco.Renderer(self.m, height=h, width=w)
        self._joint_state = None  # (names, positions, stamp)

    def apply_joint_state(self, names, positions):
        """按关节名把 /joint_states 回填进 qpos（未出现的关节保持原值）。
        注意：这里只写 qpos 不做 mj_forward——正运动学在 render_all 里每
        渲染帧做一次即可（/joint_states 190Hz×mj_forward 纯属浪费）。"""
        for name, val in zip(names, positions):
            adr = self.jnt_qadr.get(name)
            if adr is not None:
                self.d.qpos[adr] = val

    def _apply_d405_follow(self):
        """D405 世界位姿 = ag95_base 实时位姿 × 局部标称（look-at up=基座 -y）。"""
        mujoco = self.mujoco
        R_b = self.d.xmat[self.base_bid].reshape(3, 3)
        p_b = self.d.xpos[self.base_bid]
        P_w = p_b + R_b @ D405_POS_LOCAL
        T_w = p_b + R_b @ D405_TARGET_LOCAL
        up = R_b @ D405_UP_LOCAL
        f = T_w - P_w
        f /= np.linalg.norm(f)
        x = np.cross(f, up)
        x /= np.linalg.norm(x)
        y = np.cross(x, f)
        cid = self.cam_ids["d405_wrist"]
        self.d.cam_xpos[cid] = P_w
        self.d.cam_xmat[cid] = np.column_stack([x, y, -f]).flatten()

    def render_all(self):
        """渲染三路 → {名字: (h,w,3) uint8}。"""
        self.mujoco.mj_forward(self.m, self.d)   # 每渲染帧一次（详见 apply_joint_state 注）
        self._apply_d405_follow()
        out = {}
        for n, (w, h, _, _) in CAMERAS.items():
            r = self._renderers[n]
            r.update_scene(self.d, camera=self.cam_ids[n])
            out[n] = r.render()
        return out

    def close(self):
        for r in self._renderers.values():
            r.close()


def camera_info(w, h, fovy_deg, frame_id):
    """针孔 CameraInfo：K 由 fovy 反推，主点居中，零畸变（与渲染像素一致）。"""
    from sensor_msgs.msg import CameraInfo
    fy = (h / 2.0) / np.tan(np.radians(fovy_deg / 2.0))
    fx = fy  # MuJoCo 像素为正方形
    info = CameraInfo()
    info.width = w
    info.height = h
    info.distortion_model = "plumb_bob"
    info.d = []
    info.k = [fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.header.frame_id = frame_id
    return info


def selftest(scene_path: str):
    """离线自测：渲染三路各一帧存 /tmp，检查非空与非纯色。无需 ROS。"""
    import time
    rig = CameraRig(scene_path)
    ok = True
    imgs = rig.render_all()
    for n, (w, h, _, _) in CAMERAS.items():
        img = imgs[n]
        std = float(img.std())
        good = img.shape == (h, w, 3) and std > 5.0
        ok &= good
        path = f"/tmp/stageB_selftest_{n}.png"
        from PIL import Image
        Image.fromarray(img).save(path)
        print(f"  {n}: {w}x{h} std={std:.1f} {'✓' if good else '✗ 疑似纯色/空帧'} → {path}")
        time.sleep(0.05)
    rig.close()
    print("selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default=SCENE)
    parser.add_argument("--selftest", action="store_true", help="离线渲染一帧并存图退出（无需 ROS）")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(selftest(args.scene))

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, CompressedImage, JointState

    rclpy.init()

    class MultiCameraNode(Node):
        def __init__(self):
            super().__init__("multi_camera_renderer")
            self.rig = CameraRig(args.scene)
            self.infos = {n: camera_info(w, h, fov, fid)
                          for n, (w, h, fov, fid) in CAMERAS.items()}
            self.pubs = {}
            for n, (w, h, _, fid) in CAMERAS.items():
                self.pubs[n] = (
                    # ⚠️ 只发 JPEG 压缩流（录制主通道）。raw Image（720p 每帧 2.7MB）
                    #    实测把 tick 拖到 287ms（30Hz→3.5Hz 的根因）：rclpy 的 Python
                    #    发布路径序列化+分片扛不住 4.5MB/tick。压缩流 ~15MB/s 无压力。
                    #    rviz/rqt_image_view 直接订 /compressed 话题即可。
                    self.create_publisher(CompressedImage,
                                          f"/camera/{n}/image_rect_color/compressed", 1),
                    self.create_publisher(CameraInfo, f"/camera/{n}/camera_info", 1),
                )
            self.latest_stamp = None
            self.create_subscription(JointState, "/joint_states", self._on_js, 10)
            self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
            self.get_logger().info(
                f"三路虚拟相机渲染中 @{RATE_HZ:.0f}Hz（scene={args.scene}）")

        def _on_js(self, msg: JointState):
            self.latest_stamp = msg.header.stamp
            self.rig.apply_joint_state(msg.name, msg.position)

        def _tick(self):
            if self.latest_stamp is None:
                return  # 还没收到 /joint_states
            imgs = self.rig.render_all()
            for n, img in imgs.items():
                h, w, _ = img.shape
                ok, buf = cv2.imencode(".jpeg", img,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if not ok:
                    continue
                cmsg = CompressedImage()
                cmsg.header.stamp = self.latest_stamp
                cmsg.header.frame_id = CAMERAS[n][3]
                cmsg.format = "jpeg"
                cmsg.data = buf.tobytes()
                self.pubs[n][0].publish(cmsg)
                info = self.infos[n]
                info.header.stamp = self.latest_stamp
                self.pubs[n][1].publish(info)

    node = MultiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.rig.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
