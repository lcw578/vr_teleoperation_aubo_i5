# 遥操运行环境

项目使用 `/data/VR_teleoperation_runtime/.venv` 中的独立 Python 环境。运行时依赖和构建产物保存在 `/data/VR_teleoperation_runtime`，AUBO SDK 通过 `/home/lcw/VR_teleoperation/robot_SDK` 软链接接入 `/data/robot_SDK`。

```bash
source scripts/teleop_env.sh
teleop_env
"$TELEOP_PYTHON" -m unittest tests.test_mujoco_teleop
```

安装锁定依赖时从项目根目录运行

```bash
uv venv --python /usr/bin/python3 /data/VR_teleoperation_runtime/.venv
uv pip sync --python /data/VR_teleoperation_runtime/.venv/bin/python environment/requirements.lock
```

真机原生绑定先使用 `scripts/build_aubo_readonly.sh` 构建只读组件，再使用 `scripts/build_aubo_follow.sh` 构建跟随组件。构建前需要配置匹配的 AUBO SDK，构建结果留在运行时目录，不写入仓库。
