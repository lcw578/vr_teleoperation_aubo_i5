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

  function recordingModel(status, { connection = "connected", activeElapsedSeconds = null } = {}) {
    const oneshot = status?.home_return?.trigger === "oneshot";
    const prompts = {
      promptLine1: "左摇杆：← 重启 · → 退出循环 · ↑ 结束录制 · ↓ 未分配",
      promptLine2: "右 Grip：松开暂停／原始采集继续 · 右摇杆按下：事件标记",
      promptLine3: (status?.home_return?.enabled === false ? "左 Y：原位配置不可用" : oneshot
        ? "左 Y：单击回位／再按取消（无需右 Grip）" : "左 Y：按住回位／松开停止（无需右 Grip）")
        + " · 右 A：按住半倍率",
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
          ? `${segmentLabel(collection.active_episode)} · 操作暂停/仍在采集`
          : `${segmentLabel(collection.active_episode)} · 录制中`
        : collection.state === "saving" || collection.pending_saves > 0
          ? "正在保存"
          : collection.state === "error" ? "录制失败" : "等待录制";
      return { visible: true, tone: "error", title: "录制状态连接中断",
        detail: `最后确认：${lastState}`, previous: "状态可能已过期", ...prompts };
    }
    const hardware = status.hardware || {};
    if (status.recording_blocked_reason || status.clutch_state === "fault" || hardware.fatal || hardware.stop_result_unknown) {
      return { visible: true, tone: "error",
        title: hardware.stop_result_unknown ? "停止未确认 · 采集已锁定" : "硬件故障 · 采集已锁定",
        detail: "右 Grip 无法恢复；先确认物理停止并处理故障", previous, ...prompts,
        promptLine2: "当前不能开始新段或继续遥操",
        promptLine3: "处理故障后重新启动会话",
      };
    }
    const home = status.home_return;
    if (home && !collection.error && ["waiting_for_stop", "ready", "moving", "stopping"].includes(home.state)) {
      return { visible: true, tone: "saving", title: home.state === "moving" ? "正在回原位" : "回位准备／停止确认中",
        detail: oneshot ? "自动回位 · 再按左 Y 中止 · 无需右 Grip" : "保持左 Y 按住 · 松开即请求停止", previous, ...prompts };
    }
    if (!collection.enabled && hardware.state) {
      // Free teleoperation needs admission feedback even without recording.
      // Read the existing motion status; this card never grants permission.
      let title = "遥操就绪", detail = "捏住右 Grip 接合", tone = "waiting";
      const phase = hardware.state;
      if (phase === "stop_requested" || phase === "stopping") {
        title = "正在停止并确认静止"; detail = "确认完成后，松开再按 Grip";
      } else if (status.operator_paused) {
        title = "遥操已暂停"; detail = "请先解除网页暂停";
      } else if (hardware.underrun_recovering) {
        title = "供点恢复中 · 暂不接合"; detail = "正在确认静止，随后从零速恢复";
      } else if (phase === "priming" || phase === "arming") {
        title = "冷接合预热中"; detail = "保持 Grip，等待预热完成";
      } else if (phase === "braking") {
        title = "正在制动"; detail = hardware.immediate_hot_clutch
          ? "随时捏住右 Grip，从当前运动状态继续跟随"
          : "停稳并满足反馈条件后才能重新接合";
      } else if (phase === "holding") {
        if (hardware.hot_ready || (hardware.immediate_hot_clutch && hardware.feedback_ready !== false)) {
          title = "已保持 · 可重新接合"; detail = "捏住右 Grip 继续运动";
        } else if (Number.isFinite(hardware.hot_fast) && hardware.hot_fast < 5) {
          title = "保持中 · 等待发送稳定";
          detail = `连续快帧 ${hardware.hot_fast}/5（每次 ≤40ms），达标后自动接合`;
        } else {
          title = "保持中 · 等待反馈与静止确认";
          detail = "条件满足后，保持按下的 Grip 会自动接合";
        }
      } else if (phase === "active" && status.engaged) {
        title = "遥操已接合"; detail = "移动右手柄控制机械臂 · 松开 Grip 制动"; tone = "done";
      } else if (hardware.feedback_ready === false) {
        title = "等待机械臂反馈"; detail = "反馈恢复后，松开再按 Grip";
      } else if (status.xr_input && status.xr_input.right_present === false) {
        title = "等待右手柄输入"; detail = "保持手柄可追踪，松开扳机和 Grip";
      } else if (status.require_release) {
        title = "请先松开右 Grip"; detail = "完全松开后再捏住接合";
      }
      return { visible: true, tone, title, detail, previous: "",
        promptLine1: "右 Grip：捏住接合 · 松开制动保持",
        promptLine2: "等待接合时保持手柄姿态稳定",
        promptLine3: "追踪丢失后：先松开扳机和 Grip，再重新接合" };
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
        home_return: "回位期间自动运动，离线可筛选",
      })[collection.pause_reason] || "操作已暂停";
      const closing = ["restart_loop", "exit_loop", "exit_record", "episode_finished", "episode_invalid"]
        .includes(collection.pause_reason);
      return { visible: true, tone: "saving",
        title: `${segmentLabel(collection.active_episode)} · 操作暂停/仍在采集`,
        detail: closing ? reason : `${reason}；图像、关节与工具原始记录继续写入`, previous, ...prompts };
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

  function model(status, options = {}) {
    const view = recordingModel(status, options);
    const mark = status?.last_mark;
    const connected = !options.connection || options.connection === "connected";
    const precision = status?.precision_mode ? "精细 0.5×" : "普通 1.0×";
    let notice = `遥操倍率：${precision} · 事件标记：暂无`;
    if (mark && Number.isInteger(mark.count)) {
      const at = Number.isFinite(mark.episode_elapsed_s) ? ` · 本段 ${formatDuration(mark.episode_elapsed_s)}` : "";
      notice = `遥操：${precision} · ${mark.recorded ? "已记录标记" : "标记未保存"} #${mark.count}${at}`;
    }
    return { ...view, notice: connected ? notice : "", markRecent: connected &&
      !!mark?.recorded && Number.isFinite(mark.age_s) && mark.age_s < 3 };
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
    ctx.fillStyle = view.markRecent ? "#62ffb3" : "#c2ccd8";
    ctx.fillText(view.notice || "", 36, 140, width - 72);
    ctx.fillStyle = "#e5edf6";
    ctx.fillText(view.promptLine1 || "", 36, 178, width - 72);
    ctx.fillText(view.promptLine2 || "", 36, 212, width - 72);
    ctx.fillText(view.promptLine3 || "", 36, 246, width - 72);
    ctx.fillStyle = "#c2ccd8";
    ctx.fillText(view.previous || "", 36, 280, width - 72);
  }

  window.VRTeleopRecordingHud = Object.freeze({ model, draw, formatDuration });
})();
