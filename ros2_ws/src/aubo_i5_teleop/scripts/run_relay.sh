#!/usr/bin/env bash
# VR relay 启动：Quest 浏览器 → WebSocket 位姿流 + WebRTC 相机回传。
#
# 两种模式：
#   bash scripts/run_relay.sh            # LAN HTTPS（头显浏览器开 https://10.191.211.150:8443）
#   bash scripts/run_relay.sh usb        # USB（先插线+头显允许调试；PC 再跑 adb reverse）
#
# 相机（可选）：环境变量指定 v4l2 设备才启用，如
#   CAM_TOP=/dev/video0 bash scripts/run_relay.sh
# 仿真阶段想用 MuJoCo 渲染画面充当相机：走 v4l2loopback 或改帧源（后续任务）。
set +u
cd /home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop
PY=/home/lcw/tomato_robot/.venv/bin/python
CERTS=/home/lcw/VR_teleoperation/certs
LAN_IP=10.191.211.150

cleanup() {
  for p in $(pgrep -f "vr_teleop_rela[y]" 2>/dev/null); do kill "$p" 2>/dev/null; done
  echo "relay 已停"
}
trap cleanup EXIT

if [ "$1" = "usb" ]; then
  echo "USB 模式：relay 绑 127.0.0.1:8443"
  echo "  另开终端执行: adb reverse tcp:8443 tcp:8443"
  echo "  头显浏览器开: http://localhost:8443/"
  exec "$PY" -m vr_teleop_kit.relay.server --host 127.0.0.1 --port 8443
else
  echo "LAN HTTPS 模式（自签证书）"
  echo "  头显浏览器开: https://$LAN_IP:8443/    （首次接受'不安全'警告）"
  echo "  PC 局域网 IP: $LAN_IP（变了就改本脚本）"
  exec "$PY" -m vr_teleop_kit.relay.server --host 0.0.0.0 --port 8443 \
    --ssl-keyfile "$CERTS/key.pem" --ssl-certfile "$CERTS/cert.pem"
fi
