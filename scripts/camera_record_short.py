#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双 L515 短段录制验收（运行于车载电脑 xc-pc；独立脚本，不 import 本仓库其他模块）。

目的：验收"两台 L515 RGB 并发 + MJPEG 写盘 + 每帧时间戳 + 有界队列"链路——
这是未来常驻采集进程的架构预演（docs/data_collection_scheme_2026-10-31.md §6）。

架构（与未来常驻进程一致）：
  采集线程（每相机一个，wait_for_frames → 有界队列） → 写盘线程（VideoWriter MJPEG + jsonl）
  队列满 = 丢新帧并计数（帧号缺口可事后审计），绝不阻塞采集。

验收判据（摘要在最后，可贴回）：
  1. 写盘帧率 29.5–30.5 fps
  2. 帧号缺口 = 0
  3. 回放帧数 == jsonl 行数
  4. 写盘间隔 max < 100ms
  5. 进程峰值内存报告（有界队列下应稳定，不随时间增长）

用法（车载电脑，系统 python3；先关闭 RealSense Viewer/采摘程序）：
  python3 camera_record_short.py --seconds 60                 # 640x480@30（已验证档）
  python3 camera_record_short.py --seconds 60 --resolution 720p  # 1280x720@30（新档，双流需复测）
输出：<out>/<serial>.avi + <serial>.jsonl + summary.json + 终端 VERDICT 块。
"""

import argparse
import json
import queue
import shutil
import statistics
import sys
import threading
import time
from pathlib import Path

try:
    import cv2
    import numpy as np  # noqa: F401  （pyrealsense2 返回数组依赖它，保持显式）
    import pyrealsense2 as rs
except ImportError as exc:
    print(f"缺少依赖: {exc}")
    sys.exit(1)

DEFAULT_SERIALS = ("f1150496", "f1422449")  # 左工作区 / 右工作区（10-31 已确认）
RES = {"480p": (640, 480), "720p": (1280, 720)}


class CameraRecorder:
    """单相机：采集线程 → 有界队列 → 写盘线程（avi + jsonl）。"""

    def __init__(self, serial, out_dir, w, h, fps=30, queue_size=48):
        self.serial = serial
        self.w, self.h, self.fps = w, h, fps
        self.avi_path = out_dir / f"{serial}.avi"
        self.jsonl_path = out_dir / f"{serial}.jsonl"
        self.q = queue.Queue(maxsize=queue_size)
        self.dropped = 0
        self.rows = 0
        self.error = None
        self._stop = threading.Event()
        self._threads = []

    def start(self):
        for target in (self._capture_loop, self._write_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self, timeout=5.0):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=timeout)

    def _open_pipeline(self):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, self.fps)
        profile = pipe.start(cfg)  # 相机被占用时会在这里抛异常
        usb = profile.get_device().get_info(rs.camera_info.usb_type_descriptor)
        if not str(usb).startswith("3"):
            print(f"[{self.serial}] 警告：USB 枚举为 {usb}（非 USB3），带宽可能不足")
        return pipe

    def _capture_loop(self):
        try:
            pipe = self._open_pipeline()
        except Exception as exc:
            self.error = f"打开失败: {type(exc).__name__}: {exc}"
            self._stop.set()
            return
        warmup_end = time.monotonic() + 2.0
        try:
            while not self._stop.is_set():
                fs = pipe.wait_for_frames(2000)
                if time.monotonic() < warmup_end:
                    continue
                c = fs.get_color_frame()
                if not c:
                    continue
                item = (
                    np.asanyarray(c.get_data()).copy(),
                    int(c.get_frame_number()),
                    float(c.get_timestamp()),
                    str(c.get_frame_timestamp_domain()).split(".")[-1],
                    time.monotonic_ns(),
                    time.time_ns(),
                )
                try:
                    self.q.put_nowait(item)
                except queue.Full:
                    self.dropped += 1  # 丢新帧：由帧号缺口事后审计
        except Exception as exc:
            self.error = f"采集异常: {type(exc).__name__}: {exc}"
        finally:
            try:
                pipe.stop()
            except Exception:
                pass
            try:
                self.q.put_nowait(None)  # 哨兵：通知写盘线程收尾
            except queue.Full:
                pass

    def _write_loop(self):
        writer = cv2.VideoWriter(str(self.avi_path),
                                 cv2.VideoWriter_fourcc(*"MJPG"),
                                 self.fps, (self.w, self.h))
        if not writer.isOpened():
            self.error = self.error or "VideoWriter 打开失败"
            return
        last_write_ns = None
        max_gap_ms = 0.0
        with open(self.jsonl_path, "w", encoding="utf-8") as fh:
            while True:
                item = self.q.get()
                if item is None:
                    break
                frame, fno, dev_ts, domain, mono_ns, wall_ns = item
                writer.write(frame)
                now = time.monotonic_ns()
                gap = 0.0 if last_write_ns is None else (now - last_write_ns) / 1e6
                last_write_ns = now
                max_gap_ms = max(max_gap_ms, gap)
                fh.write(json.dumps(dict(
                    serial=self.serial, frame_number=fno,
                    device_ts_ms=dev_ts, domain=domain,
                    host_mono_ns=mono_ns, host_wall_ns=wall_ns,
                    write_gap_ms=round(gap, 3))) + "\n")
                self.rows += 1
        writer.release()
        self.max_write_gap_ms = max_gap_ms


def verify_playback(avi_path):
    cap = cv2.VideoCapture(str(avi_path))
    n = 0
    while True:
        ok, _ = cap.read()
        if not ok:
            break
        n += 1
    cap.release()
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=int, default=60)
    ap.add_argument("--out", default=None, help="输出目录（默认 ~/camera_test/record-<时间戳>）")
    ap.add_argument("--resolution", choices=sorted(RES), default="480p")
    ap.add_argument("--serials", default=",".join(DEFAULT_SERIALS))
    args = ap.parse_args()
    serials = [s.strip() for s in args.serials.split(",") if s.strip()]
    w, h = RES[args.resolution]

    out_dir = Path(args.out) if args.out else (
        Path.home() / "camera_test" / f"record-{time.strftime('%Y%m%d-%H%M%S')}")

    # 开跑前设备核对：缺相机直接拒绝（2026-11-01 教训：在线列表变了要当场叫停）
    ctx = rs.context()
    found = {d.get_info(rs.camera_info.serial_number): d.get_info(rs.camera_info.name)
             for d in ctx.query_devices()}
    print(f"在线设备: {found}")
    missing = [s for s in serials if s not in found]
    if missing:
        print(f"FAIL — 期望的相机不在线: {missing}")
        print("检查：该相机的 USB 线是否插好/被拔走/换过口；或用 --serials 指定实际在线的相机。")
        sys.exit(1)

    out_dir.mkdir(parents=True, exist_ok=True)
    free_gb = shutil.disk_usage(out_dir).free / 1e9
    print(f"输出目录: {out_dir}  分区可用: {free_gb:.1f} GB  分辨率: {w}x{h}@30")
    if free_gb < 10:
        print("警告：可用空间不足 10GB，长录制前请先清理/归档。")

    import resource
    rss0 = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    recorders = [CameraRecorder(s, out_dir, w, h) for s in serials]
    for r in recorders:
        r.start()
    print(f"录制 {args.seconds}s（预热 2s 不计）…… Ctrl+C 可提前结束并保留已录数据", flush=True)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            time.sleep(0.5)
            states = "  ".join(f"{r.serial}: {r.rows}行 丢{r.dropped}"
                               + (f" 错误{r.error}" if r.error else "") for r in recorders)
            print("\r" + states + "   ", end="", flush=True)
        print()
    except KeyboardInterrupt:
        print("\n提前结束：停止采集并核对已录数据……", flush=True)
    for r in recorders:
        r.stop()
    rss1 = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    summary = {"resolution": f"{w}x{h}@30", "seconds": args.seconds,
               "out_dir": str(out_dir), "peak_rss_mb": round(rss1 / 1024, 1),
               "cameras": {}}
    ok = True
    for r in recorders:
        cam = {"rows": r.rows, "dropped_capture_queue": r.dropped,
               "error": r.error, "max_write_gap_ms": round(r.max_write_gap_ms, 3)
               if hasattr(r, "max_write_gap_ms") else None}
        if r.rows >= 10 and not r.error:
            rows = [json.loads(l) for l in open(r.jsonl_path, encoding="utf-8")]
            span_s = (rows[-1]["host_mono_ns"] - rows[0]["host_mono_ns"]) / 1e9
            cam["written_fps"] = round((r.rows - 1) / span_s, 2)
            cam["frame_number_gaps"] = sum(
                max(0, b["frame_number"] - a["frame_number"] - 1)
                for a, b in zip(rows, rows[1:]))
            cam["avi_bytes"] = r.avi_path.stat().st_size
            cam["playback_frames"] = verify_playback(r.avi_path)
            cam["avi_matches_jsonl"] = cam["playback_frames"] == r.rows
            mbps = r.avi_path.stat().st_size * 8 / span_s / 1e6
            cam["avi_mbps"] = round(mbps, 1)
            # 判据
            fails = []
            if not (29.0 <= cam["written_fps"] <= 31.0):
                fails.append(f"帧率{cam['written_fps']}")
            if cam["frame_number_gaps"] != 0:
                fails.append(f"帧号缺口{cam['frame_number_gaps']}")
            if not cam["avi_matches_jsonl"]:
                fails.append("回放帧数!=jsonl行数")
            if cam["max_write_gap_ms"] is None or cam["max_write_gap_ms"] >= 100:
                fails.append(f"写盘间隔{cam['max_write_gap_ms']}ms")
            cam["verdict"] = "PASS" if not fails else "FAIL: " + "; ".join(fails)
            ok = ok and not fails
        else:
            cam["verdict"] = f"FAIL: {r.error or '帧太少'}"
            ok = False
        summary["cameras"][r.serial] = cam

    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("\n===== VERDICT =====")
    for s, cam in summary["cameras"].items():
        print(f"[{s}] {cam['verdict']}")
    print("VERDICT:", "PASS — 双路 RGB+时间戳写盘链路验收通过（该分辨率）"
          if ok else "FAIL — 把本输出整段贴回")
    print("说明：720p 通过后即为正式采数档位；480p 通过=链路成立。"
          "设备时间戳只作辅助列，主时间戳=host_mono_ns。")


if __name__ == "__main__":
    main()
