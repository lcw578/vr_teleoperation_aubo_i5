#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""L515 实时画面预览（运行于车载电脑本机终端——需要显示器，SSH 里跑不了）。

用法：
  python3 camera_preview.py                    # 自动预览所有在线相机
  python3 camera_preview.py --serials f1150496 # 指定序列号
窗口按 Q 或 Esc 退出。左上角叠加：序列号 / 实测帧率 / 分辨率。
"""

import argparse
import statistics
import sys
import time

try:
    import cv2
    import numpy as np
    import pyrealsense2 as rs
except ImportError as exc:
    print(f"缺少依赖: {exc}")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--serials", default=None, help="逗号分隔；缺省=全部在线相机")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    ctx = rs.context()
    online = [d.get_info(rs.camera_info.serial_number)
              for d in ctx.query_devices()]
    if not online:
        print("没有在线相机")
        sys.exit(1)
    serials = ([s.strip() for s in args.serials.split(",") if s.strip()]
               if args.serials else online)
    missing = [s for s in serials if s not in online]
    if missing:
        print(f"请求的相机不在线: {missing}；在线: {online}")
        sys.exit(1)

    pipes = []
    for s in serials:
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(s)
        cfg.enable_stream(rs.stream.color, args.width, args.height,
                          rs.format.bgr8, 30)
        try:
            pipe.start(cfg)
        except Exception as exc:
            print(f"[{s}] 启动失败: {exc}")
            sys.exit(1)
        pipes.append((s, pipe))
    print(f"预览中: {serials}  （窗口按 Q/Esc 退出）", flush=True)

    stamps = {s: [] for s, _ in pipes}   # 每相机最近 30 帧的主机时刻
    fps = {s: 0.0 for s, _ in pipes}
    last_report = time.monotonic()
    try:
        while True:
            panels = []
            for s, pipe in pipes:
                frames = pipe.poll_for_frames()
                if frames:
                    c = frames.get_color_frame()
                    if c:
                        img = np.asanyarray(c.get_data()).copy()
                        now = time.monotonic()
                        stamps[s].append(now)
                        if len(stamps[s]) > 30:
                            stamps[s].pop(0)
                        if len(stamps[s]) >= 10:
                            iv = [b - a for a, b in
                                  zip(stamps[s], stamps[s][1:])]
                            fps[s] = round(1.0 / statistics.median(iv), 1)
                        cv2.rectangle(img, (0, 0), (330, 58), (0, 0, 0), -1)
                        cv2.putText(img, f"{s}  {fps[s]} fps",
                                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0, 255, 0), 2)
                        cv2.putText(img, f"{img.shape[1]}x{img.shape[0]}",
                                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (255, 255, 255), 1)
                        panels.append(img)
            if panels:
                cv2.imshow("L515 preview (Q to quit)",
                           np.hstack(panels) if len(panels) > 1 else panels[0])
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            try:
                if cv2.getWindowProperty("L515 preview (Q to quit)",
                                         cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break  # 窗口被标题栏 ✕ 关闭：QT 后端在此抛 Null guiReceiver
            if time.monotonic() - last_report > 10:
                last_report = time.monotonic()
                print("fps:", {s: fps[s] for s, _ in pipes}, flush=True)
    finally:
        for _, pipe in pipes:
            try:
                pipe.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()
        print("预览结束。")


if __name__ == "__main__":
    main()
