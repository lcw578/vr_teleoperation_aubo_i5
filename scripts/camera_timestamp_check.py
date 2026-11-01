#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机时间戳验证（运行于车载电脑 xc-pc；独立脚本，不 import 本仓库其他模块）。

背景（2026-10-31 车载电脑已实测）：
  - 双 L515 并发 640x480@30 稳定 30.024fps、0 丢帧（f1150496=左工作区, f1422449=右工作区）；
  - 时间戳域 = hardware_clock，设备钟比主机慢约 0.4%（15s 差 60ms），不能只用起始偏移对齐。
本脚本要量化并定案：
  1. 主机接收间隔（复核 30fps 与无抖动）；
  2. 设备钟→主机钟的线性映射斜率与拟合残差（量化漂移、验证"线性拟合可救"的程度）；
  3. RS2_OPTION_GLOBAL_TIME_ENABLED 探测：开启后时间戳域是否变为 global_time
     （librealsense 内建设备→主机映射）。【未在 L515 固件 1.5.8.1 上验证过，本脚本即验证】

定案（本脚本的判读基准，出处：docs/data_collection_scheme_2026-10-31.md §6）：
  主时间戳 = 主机接收时刻（time.monotonic_ns），设备时间戳只作辅助列。
  若 global_time 生效，设备辅助列升级为"SDK 已映射到主机钟"，仍不作主时间戳。

用法（车载电脑，系统 python3，需要 pyrealsense2+numpy；先关闭 RealSense Viewer/采摘程序）：
  python3 camera_timestamp_check.py [--seconds-a 20] [--seconds-b 10]
                                   [--serials f1150496,f1422449]
输出：逐相机 JSON 报告 + 最后的 VERDICT 块（可直接贴回）。
"""

import argparse
import json
import statistics
import sys
import time

try:
    import numpy as np
    import pyrealsense2 as rs
except ImportError as exc:
    print(f"缺少依赖: {exc}")
    print("车载电脑已确认安装过 pyrealsense2/numpy；若报错请检查当前 python3 环境。")
    sys.exit(1)

DEFAULT_SERIALS = ("f1150496", "f1422449")


def stream_phase(serial, seconds, tag):
    """采集一段流。返回帧列表与 GlobalTime 选项状态。"""
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    profile = pipe.start(cfg)
    dev = profile.get_device()
    color_sensor = next((s for s in dev.query_sensors()
                         if any(f.stream_type() == rs.stream.color
                                for f in s.get_stream_profiles())), None)
    gt = {"supported": False, "before": None, "after": None, "set_error": None}
    if color_sensor is not None:
        try:
            gt["supported"] = bool(color_sensor.supports(rs.option.global_time_enabled))
            if gt["supported"]:
                gt["before"] = color_sensor.get_option(rs.option.global_time_enabled)
                if tag == "B" and not gt["before"]:
                    color_sensor.set_option(rs.option.global_time_enabled, 1.0)
                    gt["after"] = color_sensor.get_option(rs.option.global_time_enabled)
        except Exception as exc:  # 选项失败不阻断采集，只记录
            gt["set_error"] = f"{type(exc).__name__}: {exc}"
    frames = []
    warmup_end = time.monotonic() + 2.0
    deadline = warmup_end + seconds
    while time.monotonic() < deadline:
        fs = pipe.wait_for_frames(3000)
        recv_ns = time.monotonic_ns()
        if time.monotonic() < warmup_end:
            continue  # 预热期不记录
        c = fs.get_color_frame()
        if not c:
            continue
        frames.append((
            int(c.get_frame_number()),
            float(c.get_timestamp()),
            str(c.get_frame_timestamp_domain()).split(".")[-1],
            recv_ns,
            time.time_ns(),
        ))
    try:
        pipe.stop()
    except Exception:
        pass
    return frames, gt


def analyze(frames):
    if len(frames) < 10:
        return {"error": f"帧太少: {len(frames)}"}
    host_ms = [(f[3] - frames[0][3]) / 1e6 for f in frames]
    dev_ms = [f[1] for f in frames]
    host_iv = [(b - a) for a, b in zip(host_ms, host_ms[1:])]
    domains = sorted({f[2] for f in frames})
    dev_iv = [(b - a) for a, b in zip(dev_ms, dev_ms[1:])
              if frames[0][2] == frames[1][2]]
    out = {
        "frames": len(frames),
        "domains": domains,
        "host_interval_median_ms": round(statistics.median(host_iv), 3),
        "host_interval_max_ms": round(max(host_iv), 3),
        "host_fps": round(1000.0 / statistics.median(host_iv), 2),
        "frame_number_gaps": sum(max(0, b[0] - a[0] - 1) for a, b in zip(frames, frames[1:])),
    }
    # 线性映射 host_mono_s = a*dev_s + b（同域时才拟合）
    if len(domains) == 1:
        x = np.asarray(dev_ms, dtype=float) / 1000.0
        y = np.asarray(host_ms, dtype=float) / 1000.0
        a, b = np.polyfit(x, y, 1)
        resid_ms = (y - (a * x + b)) * 1000.0
        out["fit_slope_a"] = round(float(a), 8)
        out["fit_drift_ppm"] = round(float((a - 1.0) * 1e6), 1)
        out["fit_residual_std_ms"] = round(float(resid_ms.std()), 3)
        out["fit_residual_max_ms"] = round(float(np.abs(resid_ms).max()), 3)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds-a", type=int, default=20, help="A 相位（默认选项）时长")
    ap.add_argument("--seconds-b", type=int, default=10, help="B 相位（开 GlobalTime）时长")
    ap.add_argument("--serials", default=",".join(DEFAULT_SERIALS))
    args = ap.parse_args()
    serials = [s.strip() for s in args.serials.split(",") if s.strip()]

    ctx = rs.context()
    found = {d.get_info(rs.camera_info.serial_number): d.get_info(rs.camera_info.name)
             for d in ctx.query_devices()}
    print(f"在线设备: {found}")
    for s in serials:
        if s not in found:
            print(f"VERDICT: FAIL — 序列号 {s} 不在线（检查 USB/占用）")
            sys.exit(1)

    report = {}
    for s in serials:
        cam = {}
        for tag, secs in (("A", args.seconds_a), ("B", args.seconds_b)):
            frames, gt = stream_phase(s, secs, tag)
            res = analyze(frames)
            res["global_time"] = gt
            cam[tag] = res
        report[s] = cam

    print(json.dumps(report, ensure_ascii=False, indent=2))

    print("\n===== VERDICT =====")
    ok = True
    for s, cam in report.items():
        a, b = cam["A"], cam["B"]
        line = f"[{s}] "
        if "error" in a or "error" in b:
            print(line + "FAIL — 采集异常"); ok = False; continue
        line += (f"fps={a['host_fps']} 主机间隔max={a['host_interval_max_ms']}ms "
                 f"帧号缺口={a['frame_number_gaps']} 域={a['domains']}")
        if "fit_drift_ppm" in a:
            line += (f" 漂移={a['fit_drift_ppm']}ppm "
                     f"拟合残差std={a['fit_residual_std_ms']}ms/max={a['fit_residual_max_ms']}ms")
        gt_b = b["global_time"]
        if gt_b.get("supported"):
            dom_b = b.get("domains", [])
            line += f" | GlobalTime 开启后域={dom_b}"
            if "global_time" in dom_b:
                line += " ⇒ SDK 映射生效（辅助列升级，仍不作主时间戳）"
        else:
            line += " | GlobalTime 不支持（辅助列维持原始设备钟）"
        # 验收判据：主机间隔中位 30-35ms、max<100ms、无帧号缺口
        if not (29.0 <= a["host_interval_median_ms"] <= 36.0
                and a["host_interval_max_ms"] < 100.0
                and a["frame_number_gaps"] == 0):
            ok = False
            line += " ⇒ FAIL"
        print(line)
    print("VERDICT:", "PASS — 主机接收时间作主时间戳的定案成立"
          if ok else "FAIL — 见上；把本输出整段贴回")
    # 复核结论提醒
    print("提醒：漂移 ppm 与拟合残差只说明'设备时间辅助列'的可用性；"
          "主时间戳始终=主机 monotonic，不受本项影响。")


if __name__ == "__main__":
    main()
