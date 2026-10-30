// Small canvas renderer and status formatter for the in-headset recording HUD.
// Kept independent from WebXR/WebGL so its status language can be tested in Node.
(() => {
  "use strict";

  const segmentLabel = (id) => Number.isInteger(id) && id >= 0 ? `第 ${id + 1} 段` : "当前段";

  const formatDuration = (seconds) => {
    const value = Math.max(0, Math.floor(Number.isFinite(seconds) ? seconds : 0));
    const hours = Math.floor(value / 3600);
    const minutes = Math.floor((value % 3600) / 60);
    const remainder = value % 60;
    return hours > 0
      ? `${String(hours).padStart(2, "0")}:${String(minutes).padStart(2, "0")}:${String(remainder).padStart(2, "0")}`
      : `${String(minutes).padStart(2, "0")}:${String(remainder).padStart(2, "0")}`;
  };

  const previousEpisodeLine = (result) => {
    if (!result || !Number.isInteger(result.episode_id)) return "上一段：暂无";
    const label = segmentLabel(result.episode_id);
    if (result.disposition === "saved" && result.flushed) return `上一段：${label}已保存 ✓`;
    if (result.disposition === "invalid" || result.disposition === "interrupted") return `上一段：${label}已标记无效`;
    return `上一段：${label}保存未确认`;
  };

  function model(status, { connection = "connected", activeElapsedSeconds = null } = {}) {
    const prompts = {
      promptLine1: "左摇杆：← 重启 · → 退出循环 · ↑ 结束录制 · ↓ 未分配",
      promptLine2: "右 Grip：松开或超时暂停，重新接合后续录 · 右摇杆按下：标记",
      promptLine3: "左 Y：按住回原位，松开停止（无需右 Grip）",
    };
    const empty = { visible: true, tone: "waiting", title: "录制状态连接中…",
      detail: "正在读取本次会话状态", previous: "", ...prompts };
    if (connection === "unsupported") return { ...empty, visible: false };
    if (!status) {
      return connection === "disconnected"
        ? { ...empty, tone: "error", title: "录制状态连接中断", detail: "尚未收到本次会话的状态" }
        : empty;
    }
    const collection = status.collection;
    if (!collection || typeof collection !== "object") return { ...empty, visible: false };

    const previous = previousEpisodeLine(collection.last_result);
    if (connection === "disconnected") {
      const lastState = collection.active_episode != null
        ? collection.state === "paused"
          ? `${segmentLabel(collection.active_episode)} · 已暂停`
          : `${segmentLabel(collection.active_episode)} · 录制中`
        : collection.state === "saving" || collection.pending_saves > 0
          ? "正在保存"
          : collection.state === "error" ? "录制失败" : "等待录制";
      return { visible: true, tone: "error", title: "录制状态连接中断",
        detail: `最后确认：${lastState}`, previous: "状态可能已过期", ...prompts };
    }
    const home = status.home_return;
    if (home && !collection.error && ["waiting_for_stop", "ready", "moving", "stopping"].includes(home.state)) {
      return { visible: true, tone: "saving", title: home.state === "moving" ? "正在回原位" : "回位准备／停止确认中",
        detail: "保持左 Y 按住 · 松开即请求停止", previous, ...prompts };
    }
    if (!collection.enabled) {
      return { visible: true, tone: "waiting", title: "录制未启用",
        detail: "本次会话未开启轨迹采集", previous: "", ...prompts };
    }
    if (collection.state === "error" || collection.error) {
      const error = String(collection.error || "录制状态异常").replace(/[\r\n]+/g, " ");
      return { visible: true, tone: "error", title: "录制失败",
        detail: error.length > 24 ? `${error.slice(0, 24)}…` : error, previous, ...prompts };
    }
    if (collection.active_episode != null && collection.state === "paused") {
      const reason = ({
        awaiting_grip: "等待首次 Grip 接合",
        input_timeout: "输入超时",
        grip_released: "Grip 已松开",
        restart_loop: "循环正在重启",
        exit_loop: "循环正在结束并保存",
        exit_record: "整个录制会话正在结束并保存",
        episode_finished: "当前段正在保存",
        episode_invalid: "当前段正在标记无效",
        operator_pause: "网页暂停已锁定",
        home_return: "回位期间暂停录制",
      })[collection.pause_reason] || "操作已暂停";
      const closing = ["restart_loop", "exit_loop", "exit_record", "episode_finished", "episode_invalid"]
        .includes(collection.pause_reason);
      return { visible: true, tone: "saving",
        title: `${segmentLabel(collection.active_episode)} · 已暂停`,
        detail: closing ? reason : `${reason}；重新按下右 Grip 并成功接合后继续同一段`, previous, ...prompts };
    }
    if (collection.active_episode != null && collection.state === "recording") {
      const elapsed = activeElapsedSeconds == null ? collection.active_episode_elapsed_s : activeElapsedSeconds;
      return { visible: true, tone: "recording",
        title: `${segmentLabel(collection.active_episode)} · 录制中`,
        detail: `本段时长 ${formatDuration(elapsed)}`, previous, ...prompts };
    }
    if (collection.state === "saving" || collection.pending_saves > 0) {
      const ids = Array.isArray(collection.saving_episodes) ? collection.saving_episodes : [];
      const saving = ids.length ? ids.map(segmentLabel).join("、") : "当前段";
      return { visible: true, tone: "saving", title: `正在保存${saving}…`,
        detail: "等待数据写入确认", previous, ...prompts };
    }
    if (status.session_finished) {
      return { visible: true, tone: "done", title: "采集会话已结束",
        detail: "当前会话不会再开始新段", previous, ...prompts };
    }
    return { visible: true, tone: "waiting",
      title: status.session_finished ? "采集会话已结束" :
        status.session_finish_pending ? "正在结束录制会话" :
        status.episode_waiting_start ? "循环已结束 · 向左推摇杆开始下一段" :
          "待录制 · 向左推摇杆开始循环",
      detail: "准备就绪", previous, ...prompts };
  }

  const colors = {
    recording: "#ff5147",
    saving: "#ffc247",
    waiting: "#70b7ff",
    done: "#62d697",
    error: "#ff5147",
  };

  function draw(ctx, view, width = 1024, height = 320) {
    ctx.clearRect(0, 0, width, height);
    if (!view || !view.visible) return;
    const x = 8, y = 8, w = width - 16, h = height - 16, r = 28;
    ctx.beginPath();
    ctx.moveTo(x + r, y); ctx.lineTo(x + w - r, y); ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r); ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    ctx.lineTo(x + r, y + h); ctx.quadraticCurveTo(x, y + h, x, y + h - r);
    ctx.lineTo(x, y + r); ctx.quadraticCurveTo(x, y, x + r, y); ctx.closePath();
    ctx.fillStyle = "rgba(10, 15, 23, 0.86)"; ctx.fill();
    ctx.strokeStyle = "rgba(255, 255, 255, 0.24)"; ctx.lineWidth = 3; ctx.stroke();

    ctx.beginPath(); ctx.arc(52, 68, 15, 0, Math.PI * 2);
    ctx.fillStyle = colors[view.tone] || colors.waiting; ctx.fill();
    ctx.font = '700 42px "Noto Sans CJK SC", system-ui, sans-serif';
    ctx.textBaseline = "middle"; ctx.fillStyle = "#ffffff";
    ctx.fillText(view.title, 84, 52, width - 120);
    ctx.font = '500 29px "Noto Sans CJK SC", system-ui, sans-serif';
    ctx.fillStyle = "#edf2f8";
    ctx.fillText(view.detail || "", 36, 106, width - 72);
    ctx.font = '500 23px "Noto Sans CJK SC", system-ui, sans-serif';
    ctx.fillStyle = "#e5edf6";
    ctx.fillText(view.promptLine1 || "", 36, 160, width - 72);
    ctx.fillText(view.promptLine2 || "", 36, 198, width - 72);
    ctx.fillText(view.promptLine3 || "", 36, 236, width - 72);
    ctx.fillStyle = "#c2ccd8";
    ctx.fillText(view.previous || "", 36, 280, width - 72);
  }

  window.VRTeleopRecordingHud = Object.freeze({ model, draw, formatDuration });
})();
