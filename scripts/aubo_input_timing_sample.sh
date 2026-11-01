#!/usr/bin/env bash
# 只读采样 /aubo/status 的 input_timing 短窗摘要，用于 Wi-Fi vs USB 传输 A/B。
# 依据：docs/aubo_held_grip_input_gap_2026-10-29.md 的判读四分法与
# teleop/aubo/input_timing.py 的字段（receive_gap_s / source_gap_s /
# seq_gap / relative_delay_change_s）。本脚本不连接 SDK、不发送任何命令，
# 对运行中的遥操会话零干扰（状态页本就被头显 HUD 以 2Hz 轮询；采样 1Hz）。
#
# 用法（与遥操会话并行运行）：
#   bash scripts/aubo_input_timing_sample.sh --url https://127.0.0.1:8443 \
#        --out /tmp/input_timing_wifi.log --duration 600
#   bash scripts/aubo_input_timing_sample.sh --url http://127.0.0.1:8443 \
#        --out /tmp/input_timing_usb.log --duration 600
# 会话结束后汇总（第二行起为摘要）：
#   bash scripts/aubo_input_timing_sample.sh --summarize /tmp/input_timing_usb.log
#
# 注：--url 用 https 时自动带 -k（自签证书）；http 时 -k 被忽略，同一命令两种模式通用。
#     recent_received 窗口 128 帧（72Hz 下约 1.8s），1s 采样重叠覆盖，不会漏 200ms 级空档。
set -euo pipefail

SAMPLER_PY='
import json, sys, time
try:
    d = json.load(sys.stdin)
except Exception:
    print(json.dumps({"status_unreachable": True})); raise SystemExit
it = (d.get("input_timing") or {})
lr = it.get("last_received") or {}
seq = lr.get("seq")
gap = lr.get("receive_gap_s")
sgap = lr.get("source_gap_s")
print(json.dumps({
    "t": time.time(),
    "seq": seq if isinstance(seq, int) else None,
    "gap_s": gap if isinstance(gap, (int, float)) else None,
    "src_gap_s": sgap if isinstance(sgap, (int, float)) else None,
    "fresh": bool(d.get("quest_input_fresh")),
}))
'

SUMMARY_PY='
import json, sys, statistics
rows=[]; unreachable=0; sampled_gaps=[]; src_gaps=[]
for line in open(sys.argv[1]):
    line=line.strip()
    if not line: continue
    r=json.loads(line)
    if r.get("status_unreachable"): unreachable+=1; continue
    rows.append(r)
    if isinstance(r.get("gap_s"),(int,float)): sampled_gaps.append(r["gap_s"])
    if isinstance(r.get("src_gap_s"),(int,float)): src_gaps.append(r["src_gap_s"])
intervals=[]
for a,b in zip(rows, rows[1:]):
    if a.get("seq") is None or b.get("seq") is None or b["t"] <= a["t"]: continue
    dt=b["t"]-a["t"]; dseq=b["seq"]-a["seq"]
    if dt<=0 or dseq<0: continue
    intervals.append((dt, dseq, dseq/dt))
if not intervals:
    print("无有效数据（seq 无推进——会话未收到手柄帧？）")
    raise SystemExit
rates=[r[2] for r in intervals]
med=statistics.median(rates)
ref=max(rates)
deficits=[max(0.0, ref-r[2])*r[0]/ref for r in intervals]
total=sum(deficits)
worst_i=max(range(len(intervals)), key=lambda i: deficits[i])
span=sum(r[0] for r in intervals)
print(f"有效采样间隔: {len(intervals)}  中位帧率: {med:.1f} Hz  参考帧率(最优间隔): {ref:.1f} Hz")
print(f"累计缺帧折合停顿: {total:.3f} s（占采样时长 {100*total/span:.2f}%）")
print(f"最差间隔: 折合停顿 {deficits[worst_i]:.3f} s（间隔时长 {intervals[worst_i][0]:.1f}s）")
print(f"采样到的最大接收间隔(gap_s): {max(sampled_gaps):.3f} s" if sampled_gaps else "采样 gap_s: n/a")
print(f"采样到的最大源间隔(src_gap_s): {max(src_gaps):.3f} s" if src_gaps else "采样 src_gap_s: n/a")
print(f"状态页不可达采样: {unreachable}")
'

URL=""; OUT=""; INTERVAL=1; DURATION=600; SUMMARIZE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --url) URL="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --interval) INTERVAL="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --summarize) SUMMARIZE="$2"; shift 2;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done

if [[ -n "$SUMMARIZE" ]]; then
  python3 -c "$SUMMARY_PY" "$SUMMARIZE"
  exit 0
fi

[[ -n "$URL" && -n "$OUT" ]] || { echo "需要 --url 和 --out（或仅 --summarize）" >&2; exit 2; }
echo "每 ${INTERVAL}s 采样 $URL/aubo/status，持续 ${DURATION}s -> $OUT"
END=$(( $(date +%s) + DURATION ))
: > "$OUT"
while [[ $(date +%s) -lt $END ]]; do
  curl -sk --max-time 3 "$URL/aubo/status" | python3 -c "$SAMPLER_PY" >> "$OUT" || true
  sleep "$INTERVAL"
done
echo "完成。汇总：bash $0 --summarize $OUT"
