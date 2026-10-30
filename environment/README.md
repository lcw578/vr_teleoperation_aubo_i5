# VR 专用环境

本项目使用 uv 建立的独立 Python 3.10 环境，实际位于
`/data/VR_teleoperation_runtime/.venv`，不再借用 tomato_robot。
ROS 构建产物位于 `/data/VR_teleoperation_runtime/ros2/{build,install,log}`。

日常继续使用已有入口：

```bash
bash /home/lcw/VR_teleoperation/scripts/run_mujoco_teleop.sh
```

MuJoCo、SDK、预览、离线检测和 Route B 的入口都通过
`scripts/teleop_env.sh` 选择解释器。需要 ROS 时只加载 Humble 和新 VR overlay。
旧 `ros2_ws/install` 留作回退，不再是本机推荐入口。

```bash
project-env vr check
project-env vr-ros check
project-env vr python -m unittest discover -s tests -p 'test_*.py'
project-env vr exec env MUJOCO_GL=egl python tests/smoke_mujoco_transport.py --backend physics
```

复现 Python 环境（从仓库根目录运行）：

```bash
uv venv --python /usr/bin/python3 /data/VR_teleoperation_runtime/.venv
uv pip sync --python /data/VR_teleoperation_runtime/.venv/bin/python environment/requirements.lock
bash scripts/build_ros_clean.sh
```

`requirements.lock` 锁定已验收的依赖版本；本地 vr-teleop-kit 仍是该仓库内的
editable 源码。ROS 构建脚本使用干净环境，系统 Python 和明确的包名单。
普通安装用于避开上游模型中失效软链接导致的 symlink-install 错误。
上游 ros_joints_plan 增加了 `find_package(tl-expected REQUIRED)`；完整补丁保存在
本轮验收目录。雷达插件已装入本项目 overlay，向系统 simulate 目录复制插件的
可选步骤没有权限，产生警告；已验收的 ROS 仿真控制器正常。

验收包括 146 项离线测试、MuJoCo 两种后端的三路视频/通信、四个原生绑定构建与
无连接导入，以及 ROS 仿真控制器激活。没有进行真实机器人连接和运动。
