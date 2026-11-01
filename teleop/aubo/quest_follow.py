"""Supervised Quest J6 or explicit six-axis commissioning; default has no device I/O."""
import argparse
from contextlib import ExitStack
import fcntl
import getpass
import json
import logging
import math
import os
from pathlib import Path
import socket
import threading
import time

from .feedback import joints
from .follow import J6FollowWorker, load_follow_native
from .offline_session import Candidate
from .probe import json_safe, record_ok
from .quest_preview import ROOT, ControlLoopTiming, make_shadow, preview_velocity_limits, serve_quest
from .runtime import OfflineRuntime
from .diagnostic_stream import DiagnosticStream
from .operator_controls import OperatorButtons, xr_record
from .recording import RawRecorder, recording_path
from .home import DEFAULT_HOME_PATH, HOME_SPEED_RAD_S, HOME_ACCEL_RAD_S2, HOME_TOLERANCE_RAD, load_home


def validate_prepare_profile(prepare, *, six_axis, command_mode="follow", follow_hz=20.):
    """Reject stale bindings and keep optional SDK memory readback distinct from cabinet settings."""
    expected = dict(six_axis=six_axis, command_mode=command_mode,
        target_policy="direct_ik" if six_axis else "j6_rate_limited",
        host_target_rate_limit_deg_s=None if six_axis else J6FollowWorker.speed_deg_s,
        client_speed_deg_s=None if six_axis else J6FollowWorker.speed_deg_s,
        client_acceleration_deg_s2=None if six_axis else J6FollowWorker.client_acceleration_deg_s2,
        sdk_profile_policy="sdk_unmodified" if six_axis else "legacy_j6_explicit",
        sdk_profile_writes=not six_axis,
        following_error_limit_deg=J6FollowWorker.following_error_limit_deg,
        send_period_s=(J6FollowWorker.six_axis_period_s if command_mode == "passthrough"
                       else 1./follow_hz) if six_axis else J6FollowWorker.period_s,
        send_max_gap_s=J6FollowWorker.send_max_gap_s,
        sdk_budget_s=J6FollowWorker.sdk_budget_s)
    if any(key not in prepare or prepare[key] != value for key, value in expected.items()):
        raise RuntimeError("Native/Python commissioning profile mismatch; rebuild follower")
    readback = prepare.get("sdk_profile_readback")
    try:
        if six_axis and readback["source"] == "not_read_six_axis_unmodified":
            if readback["max_velocity_rad_s"] is not None or readback["max_acceleration_rad_s2"] is not None:
                raise ValueError("Unexpected six-axis profile values")
        else:
            if readback["source"] != "sdk_client_memory_not_controller_settings":
                raise ValueError("Incorrect profile source")
            for key in ("max_velocity_rad_s", "max_acceleration_rad_s2"):
                values = readback[key]
                if len(values) != 6 or any(not math.isfinite(v) or v <= 0 for v in values):
                    raise ValueError("Invalid SDK profile values")
    except (TypeError, KeyError, ValueError, OverflowError) as exc:
        raise RuntimeError("Invalid SDK client memory profile readback") from exc
    return readback


class J6QuestRuntime(OfflineRuntime):
    mode = "aubo_j6_supervised_follow"
    motion_available = True

    def __init__(self, shadow, worker, *, recorder=None, home_reference=None, home_error=None,
                 home_trigger='hold', **kwargs):
        super().__init__(shadow, dispatch_fake=False, **kwargs)
        if home_trigger not in ('hold', 'oneshot'):
            raise ValueError("home_trigger must be 'hold' or 'oneshot'")
        self.home_trigger = home_trigger
        self.worker = worker
        self._waiting_stop = False
        self._last_stop_confirmed_s = None
        self._last_grip_state = None
        self._grip_pressed_s = None
        self._last_input_stop_stamp = None
        self._last_input_stop = None
        self.recorder = recorder
        self.buttons = OperatorButtons()
        self._operator_lock = threading.RLock()
        self._operator_barrier = False
        self.operator_paused = False
        self._recording_pause_trigger_s = None
        self.episode_waiting_start = False
        self._episode_transition = None
        self.software_reset_pending = False
        self.session_finished = False
        self.session_finish_pending = False
        self.last_operator_message = None
        self._recording_error_seen = False
        self._last_saved_episode = None
        self.home = dict(enabled=home_reference is not None,
            reference_path=home_reference['path'] if home_reference else None,
            target_q_rad=list(home_reference['q_rad']) if home_reference else None,
            state='idle' if home_reference else 'disabled', error=home_error,
            control_button='left_y_hold', requires_right_grip=False, hold_timeout_s=.2,
            trigger=home_trigger,
            target_speed_deg_s=math.degrees(HOME_SPEED_RAD_S),
            target_acceleration_deg_s2=math.degrees(HOME_ACCEL_RAD_S2),
            requested_monotonic_s=None, completed_monotonic_s=None, max_error_deg=None)
        self._home_offered = False
        self._home_cancel_reason = None
        self._home_started_s = None
        self._home_session = -1
        self._home_button_requested = False
        self._home_quiet_s = self._home_quiet_stamp = None

    def _home_busy(self):
        return self.home['state'] in ('waiting_for_stop', 'ready', 'moving', 'stopping')

    def _home_event(self):
        if self.recorder:
            self.recorder.record('home_return', **self.home)

    def _cancel_home(self, reason, *, failed=False):
        if not self._home_busy():
            return
        self.home.update(state='failed' if failed else 'cancelled', error=str(reason))
        self._home_offered = False
        self._home_button_requested = False
        self._home_cancel_reason = None
        self.submit_fault(str(reason) if failed else 'home_cancelled')
        self.shadow.reject_input('home_cancelled')
        self.last_operator_message = ('回位已中止：'+str(reason)+'；再按一次左 Y 重新开始'
            if self.home_trigger == 'oneshot' else
            '回位已停止：'+str(reason)+'；松开后重新按住左 Y 才能再次回位')
        self._home_event()

    def _advance_home(self):
        if not self._home_busy():
            self._home_cancel_reason = None
            return
        hardware = self.worker.status()
        if (hardware['fatal'] or hardware['stop_result_unknown'] or self.session_finished or
                self.session_finish_pending):
            self._cancel_home('hardware_fault_or_session_finished', failed=True)
            return
        if self._home_cancel_reason is not None:
            self._cancel_home(self._home_cancel_reason)
            return
        phase = self.home['state']
        if (self.home_trigger == 'hold' and
                phase in ('waiting_for_stop', 'ready', 'moving') and
                self._home_button_requested and not self.buttons.home_hold_active(self.clock())):
            self._cancel_home('home_button_released_or_expired')
            return
        if phase == 'waiting_for_stop':
            sample = self._stationary_feedback()
            if sample is None:
                return
            self.submit_fault('home_ready')
            self.shadow.reject_input('home_ready')
            error = max(abs(q-goal) for q, goal in zip(sample.q_rad, self.home['target_q_rad']))
            self.home.update(state='complete' if error <= HOME_TOLERANCE_RAD else 'ready',
                             max_error_deg=math.degrees(error))
            if self.home['state'] == 'complete':
                self.home['completed_monotonic_s'] = self.clock()
                self.last_operator_message = '已在原位，无需运动；请松开再按 Grip 控制'
            else:
                self.last_operator_message = '回位已准备：按住左 Y 回位，松开即停止；无需右 Grip'
            self._home_event()
            return
        if phase == 'ready':
            if self._home_offered and hardware['state'] == 'active':
                self.home['state'] = 'moving'
                self._home_started_s = self.clock()
                self.last_operator_message = '正在回到已保存原位；保持左 Y 按住，松开即停止'
                self._home_event()
            elif self._home_offered and hardware['state'] in ('stop_requested', 'stopping'):
                self._cancel_home(hardware.get('fault') or 'home_start_interrupted')
            return
        if phase == 'moving':
            if hardware['state'] != 'active':
                self._cancel_home(hardware.get('fault') or 'home_interrupted')
                return
            if self.clock()-self._home_started_s > 180.:
                self._cancel_home('home_timeout', failed=True)
                return
            try:
                sample = self.worker.measured_feedback()
            except (RuntimeError, ValueError, TypeError, KeyError, OverflowError):
                self._cancel_home('home_feedback_unavailable', failed=True)
                return
            error = max(abs(q-goal) for q, goal in zip(sample.q_rad, self.home['target_q_rad']))
            self.home['max_error_deg'] = math.degrees(error)
            attempt = hardware.get('last_attempt') or {}
            if attempt.get('control_source') == 'home_return' and attempt.get('outcome') == 'sdk_accepted':
                sent = attempt['sdk_result']['target_rad']
                if max(abs(q-target) for q, target in zip(sample.q_rad, sent)) > math.radians(3.):
                    self._cancel_home('home_tracking_error', failed=True)
                    return
            if error <= HOME_TOLERANCE_RAD:
                if self._home_quiet_s is None:
                    self._home_quiet_s = sample.received_monotonic_s
                if (self._home_quiet_stamp != sample.received_monotonic_s and
                        sample.received_monotonic_s-self._home_quiet_s >= .15):
                    self.home['state'] = 'stopping'
                    self.submit_fault('home_completed')
                    self.shadow.reject_input('home_completed')
                    self.last_operator_message = '已接近原位，正在确认停稳'
                    self._home_event()
                self._home_quiet_stamp = sample.received_monotonic_s
            else:
                self._home_quiet_s = self._home_quiet_stamp = None
            return
        if phase == 'stopping':
            sample = self._stationary_feedback()
            if sample is None or not hardware['stop_confirmed']:
                return
            error = max(abs(q-goal) for q, goal in zip(sample.q_rad, self.home['target_q_rad']))
            if error > HOME_TOLERANCE_RAD:
                self._cancel_home('home_outside_tolerance_after_stop', failed=True)
                return
            self.home.update(state='complete', completed_monotonic_s=self.clock(),
                             max_error_deg=math.degrees(error))
            self._home_offered = False
            self._home_button_requested = False
            self.last_operator_message = '已回到原位并确认停稳；松开再按 Grip 才能重新遥操'
            self._home_event()

    def request_operator_command(self, command):
        """UI/receiver threads publish commands; they never run IK or write files."""
        with self._operator_lock:
            accepted = self.buttons.request(command, self.clock())
            if not accepted:
                self._operator_barrier = True
                self.worker.revoke('operator_event_queue_overflow')
            else:
                self._gate_operator_commands([command])
        return accepted

    def _pause_recording_for_clutch(self, reason, stamp=None):
        """Keep the current raw episode open while the motion clutch is stopped."""
        if not self.recorder:
            return False
        stamp = self.clock() if stamp is None else stamp
        if self.recorder.status()['active_episode'] is None:
            return False
        paused = self.recorder.pause_episode(reason, stamp=stamp)
        if paused:
            self.last_operator_message = '录制已暂停；重新按下右 Grip 并成功接合后继续同一段'
        return paused

    def _resume_recording_after_clutch(self, stamp=None):
        """Resume sampling only after the worker has actually re-armed the clutch."""
        if not self.recorder or not self._last_grip_state or self._home_busy():
            return False
        hardware = self.worker.status()
        if hardware['state'] != 'active' or not self.shadow.session.engaged:
            return False
        at = self.clock() if stamp is None else stamp
        resumed = self.recorder.resume_episode('grip_reengaged', stamp=at)
        if resumed:
            self.last_operator_message = '已重新接合；继续录制当前段'
        return resumed

    def _sync_recording_pause_state(self):
        """Bridge immediate Grip releases and asynchronous watchdog stops."""
        if not self.recorder:
            return
        hardware = self.worker.status()
        trigger = hardware.get('first_stop_trigger')
        if trigger and trigger.get('reason') in ('grip_released', 'input_timeout'):
            trigger_s = trigger.get('host_monotonic_s')
            active_start = self.recorder.status().get('active_episode_started_monotonic_s')
            if (trigger_s != self._recording_pause_trigger_s and active_start is not None
                    and trigger_s is not None and trigger_s >= active_start):
                self._recording_pause_trigger_s = trigger_s
                self._pause_recording_for_clutch(trigger['reason'], trigger_s)
        if hardware.get('state') == 'active':
            self._resume_recording_after_clutch()

    def _gate_operator_commands(self, commands):
        """Revoke at receipt; starting a recording never resumes a held Grip."""
        for command in commands:
            if command == 'home_hold_end':
                if self.home_trigger == 'oneshot':
                    continue
                if not self._home_busy():
                    continue
                self._home_cancel_reason = 'home_button_released'
                reason = 'home_button_released'
            elif command == 'home_hold_start':
                reason = 'home_requested'
            elif command in ('toggle_episode', 'restart_episode') and self.recorder is not None:
                reason = ('episode_finished' if self.recorder.status()['active_episode'] is not None
                          else 'episode_started')
            elif command == 'pause':
                if not self.operator_paused:
                    reason = 'operator_pause'
                else:
                    continue
            elif command == 'exit_loop':
                if (self.recorder is None or
                        self.recorder.status()['active_episode'] is None):
                    continue
                reason = 'episode_finished'
            elif command in ('restart_loop', 'exit_record') and self.recorder is None:
                continue
            elif command in ('invalidate', 'restart_loop', 'exit_loop', 'finish_session', 'exit_record',
                             'software_reset', 'return_home'):
                reason = dict(pause='operator_pause', invalidate='episode_invalid',
                              restart_loop='episode_invalid', exit_loop='episode_finished',
                              finish_session='session_finished', exit_record='session_finished',
                              software_reset='operator_software_reset',
                              return_home='home_requested')[command]
            else:
                continue
            self._operator_barrier = True
            self.worker.revoke(reason)

    def _operator_tick(self):
        with self._operator_lock:
            self._sync_recording_pause_state()
            self._consume_operator_events()
            self._advance_operator_operations()
            self._advance_home()

    def _stationary_feedback(self):
        """Read existing observations only; no SDK query or motion from here."""
        hardware = self.worker.status()
        if (hardware['state'] != 'idle' or hardware['fatal'] or hardware['stop_result_unknown']):
            return None
        snapshot = hardware.get('last_snapshot')
        if not snapshot:
            return None
        state = snapshot['state']
        if (state.get('return_code') != 0 or state.get('data') != 0 or
                not 0 <= self.clock()-state['host_received_monotonic_s'] <= self.worker.feedback_max_age_s):
            return None
        try:
            sample = self.worker.measured_feedback()
        except (RuntimeError, ValueError, TypeError, KeyError, OverflowError):
            return None
        return sample if sample.valid else None

    def _start_episode(self):
        ok = self.recorder.begin()
        if ok:
            self.episode_waiting_start = False
            self._pause_recording_for_clutch('awaiting_grip', self.clock())
            self.submit_fault('episode_started')
        self.last_operator_message = ('新段已开始并暂挂；按下 Grip 且成功接合后开始采样' if ok
                                      else '尚未开始：请等待保存完成，或检查录制错误')
        return ok

    def _advance_operator_operations(self):
        hardware = self.worker.status()
        if hardware['fatal'] or hardware['stop_result_unknown']:
            if self._episode_transition is not None or self.software_reset_pending:
                self.last_operator_message = '操作未完成：会话已结束或硬件故障锁定，未开始新段或完成复位'
            self._episode_transition = None
            self.software_reset_pending = False
            if self.session_finish_pending:
                self.session_finish_pending = False
                self.session_finished = True
                self.last_operator_message = '硬件故障阻止正常结束；当前段会保留为未完成记录'
            return
        sample = None
        if (self._episode_transition is not None or self.software_reset_pending or
                self.session_finish_pending):
            sample = self._stationary_feedback()
        if self.software_reset_pending and sample is not None:
            # Reset mapping only after rest is observed. Keep the same episode,
            # current calibration and hardware fault latches. Never go home.
            self.shadow.feedback(sample.q_rad, received_monotonic_s=sample.received_monotonic_s,
                                 source=sample.source)
            self.submit_fault('operator_software_reset')
            self.shadow.reject_input('operator_software_reset')
            self.software_reset_pending = False
            self.last_operator_message = '软件复位完成；机械臂未回位，请松开再按 Grip 重新对齐'
            if self.recorder:
                self.recorder.record('software_reset', phase='completed', q_rad=list(sample.q_rad),
                    feedback_received_monotonic_s=sample.received_monotonic_s)
        transition = self._episode_transition
        if transition is None:
            if self.session_finish_pending:
                if sample is None:
                    return
                collection = self.recorder.status() if self.recorder else dict(pending_saves=0)
                if self.recorder and self.recorder.error:
                    self.session_finish_pending = False
                    self.session_finished = True
                    self.last_operator_message = '录制错误阻止会话正常保存：'+self.recorder.error
                elif collection['pending_saves'] == 0:
                    self.session_finish_pending = False
                    self.session_finished = True
                    self.last_operator_message = '录制会话已结束并完成保存'
            return
        if self.recorder.error:
            self._episode_transition = None
            if self.session_finish_pending and sample is not None:
                self.session_finish_pending = False
                self.session_finished = True
                self.last_operator_message = '录制错误阻止会话正常保存：'+self.recorder.error
            return
        if not transition['ended']:
            if sample is None:
                return
            if not self.recorder.end(disposition=transition['disposition']):
                self._episode_transition = None
                self.last_operator_message = '当前段未能正常结束，请检查录制状态'
                return
            transition['ended'] = True
        collection = self.recorder.status()
        if collection['pending_saves']:
            return
        self._episode_transition = None
        if transition['restart'] and sample is not None:
            self._start_episode()
        elif transition['restart']:
            # A save acknowledgement does not replace fresh stopped feedback.
            self._episode_transition = transition
        elif transition.get('finish_session'):
            self.session_finished = True
            self.session_finish_pending = False
            self.operator_paused = True
            if self.recorder:
                self.recorder.record('recording_session_end', stamp=self.clock(),
                    disposition='saved', active_episode=None)
            self.last_operator_message = '录制会话已结束；当前段已保存'

    def _consume_operator_events(self):
        self._operator_barrier = False
        for event in self.buttons.drain():
            command, stamp = event['command'], event['host_monotonic_s']
            if command in ('pause', 'toggle_episode', 'restart_episode', 'restart_loop', 'exit_loop',
                           'invalidate', 'software_reset', 'finish_session', 'exit_record'):
                self._cancel_home('operator_'+command)
            if self.recorder:
                self.recorder.record('operator_event', stamp=stamp, command=command)
            with self._operator_lock:
                if command == 'pause':
                    self.operator_paused = not self.operator_paused
                    self.submit_fault('operator_pause' if self.operator_paused else 'operator_resume_requires_release')
                    if self.operator_paused:
                        self._pause_recording_for_clutch('operator_pause', stamp)
                    self.last_operator_message = ('已锁定暂停；解除后仍需松开再按 Grip' if self.operator_paused
                                                  else '暂停已解除；请松开再按 Grip')
                elif command == 'mark':
                    self.last_operator_message = '事件标记已提交' if self.recorder else '未开启录制，未保存标记'
                elif command == 'home_hold_end':
                    if self.home_trigger == 'oneshot':
                        continue
                    self._cancel_home('home_button_released')
                elif command in ('return_home', 'home_hold_start'):
                    if self.home_trigger == 'oneshot' and self.home['state'] == 'moving':
                        self._cancel_home('home_abort_pressed')
                        continue
                    hardware = self.worker.status()
                    if (not self.home['enabled'] or not self.worker.six_axis or self.worker.command_mode != 'follow'):
                        self.last_operator_message = '运动回位不可用：需要有效原位配置和六轴 Follow 模式'
                        continue
                    if (hardware['fatal'] or hardware['stop_result_unknown'] or self.session_finished
                            or self.session_finish_pending):
                        self.last_operator_message = '当前状态不允许回位：请先排除硬件故障或重新启动会话'
                        continue
                    if self.operator_paused or self._episode_transition is not None:
                        self.last_operator_message = '请先解除网页暂停或等待切段完成，再请求回位'
                        continue
                    self.software_reset_pending = False
                    self._home_button_requested = command == 'home_hold_start'
                    self._home_session -= 1
                    self.home.update(state='waiting_for_stop', error=None,
                        requested_monotonic_s=stamp, completed_monotonic_s=None)
                    self._home_offered = False
                    self._home_cancel_reason = None
                    self._home_quiet_s = self._home_quiet_stamp = None
                    self.submit_fault('home_requested')
                    self._pause_recording_for_clutch('home_return', stamp)
                    self.last_operator_message = '已请求回原位；确认停稳后保持左 Y 按住即可回位，无需右 Grip'
                    self._home_event()
                elif command == 'software_reset':
                    if self.session_finished:
                        self.last_operator_message = '本次采集已结束，不能执行复位'
                        continue
                    hardware = self.worker.status()
                    if hardware['fatal'] or hardware['stop_result_unknown']:
                        self.last_operator_message = '硬件故障锁定，软件复位不能清除该故障'
                        continue
                    self.software_reset_pending = True
                    self.worker.revoke('operator_software_reset')
                    self.last_operator_message = '已请求软件复位；等待确认停稳后重新对齐'
                elif command in ('toggle_episode', 'restart_episode', 'restart_loop', 'exit_loop', 'invalidate'):
                    if self.recorder is None:
                        self.last_operator_message = '录制未启用；请使用 --record 和 --task 启动'
                        continue
                    if self.session_finished or self.session_finish_pending:
                        self.last_operator_message = '本次采集已结束，不能开始新段'
                        continue
                    current = self.recorder.status()
                    if self._episode_transition is not None:
                        if command == 'invalidate':
                            self._episode_transition['restart'] = False
                            if not self._episode_transition['ended']:
                                self._episode_transition['disposition'] = 'invalid'
                            self.last_operator_message = '已取消自动重开；当前段结束后等待重新开始'
                            continue
                        if command == 'exit_loop':
                            self._episode_transition.update(restart=False, finish_session=False)
                            if not self._episode_transition['ended']:
                                self._episode_transition['disposition'] = 'saved'
                            self.last_operator_message = '正在停止并保存当前循环；等待操作完成'
                            continue
                        self.last_operator_message = '正在停止并保存当前段，请等待切换完成'
                        continue
                    if current['active_episode'] is not None:
                        self.episode_waiting_start = True
                        invalid = command in ('invalidate', 'restart_loop')
                        self._pause_recording_for_clutch(
                            'restart_loop' if command == 'restart_loop' else
                            'exit_loop' if command == 'exit_loop' else
                            'episode_invalid' if invalid else 'episode_finished', stamp)
                        self.submit_fault('episode_invalid' if invalid else 'episode_finished')
                        self._episode_transition = dict(ended=False,
                            disposition='invalid' if invalid else 'saved',
                            restart=command in ('restart_episode', 'restart_loop'))
                        self.last_operator_message = ('正在停止并保存；完成后自动开始新段，请松开 Grip'
                            if command in ('restart_episode', 'restart_loop')
                            else '正在停止并保存；原始记录保留，完成后显示结果')
                    elif command in ('toggle_episode', 'restart_episode', 'restart_loop'):
                        self._start_episode()
                    else:
                        self.last_operator_message = ('当前循环已退出；没有正在录制的段可保存'
                            if command == 'exit_loop' else '没有正在录制的段；原始记录未删除')
                elif command in ('finish_session', 'exit_record'):
                    if command == 'exit_record' and self.recorder is None:
                        self.last_operator_message = '录制未启用；没有可结束的录制会话'
                        continue
                    self.software_reset_pending = False
                    if self.session_finished or self.session_finish_pending:
                        continue
                    self.session_finish_pending = True
                    self.operator_paused = True
                    if self._episode_transition is not None:
                        self._episode_transition.update(restart=False, finish_session=True)
                        if not self._episode_transition['ended']:
                            self._episode_transition['disposition'] = 'saved'
                    elif self.recorder and self.recorder.status()['active_episode'] is not None:
                        self.episode_waiting_start = True
                        self._pause_recording_for_clutch('exit_record', stamp)
                        self._episode_transition = dict(ended=False, disposition='saved',
                            restart=False, finish_session=True)
                    self.submit_fault('session_finished')
                    if self._episode_transition is None and self.recorder:
                        self.recorder.record('recording_session_end', stamp=stamp,
                            disposition='saved', active_episode=None)
                    self.last_operator_message = '结束录制会话：正在停止并完成保存'
        if self.buttons.error:
            self.operator_paused = True
            self.last_operator_message = self.buttons.error
        if self.recorder and self.recorder.error and not self._recording_error_seen:
            self._recording_error_seen = True
            self.operator_paused = True
            self.submit_fault('recording_failed')
            self.last_operator_message = '录制失败，已请求停止：'+self.recorder.error
        if self.recorder:
            result = self.recorder.status()['last_result']
            if result is not None and result['episode_id'] != self._last_saved_episode:
                self._last_saved_episode = result['episode_id']
                self.last_operator_message = ('本段已保存' if result['disposition'] == 'saved'
                                               else '本段已标记无效，原始记录已保留')

    def submit_fault(self, reason):
        if (self._home_busy() and reason not in ('home_requested', 'home_ready', 'home_completed', 'home_cancelled')):
            self._home_cancel_reason = str(reason)
        self.worker.revoke(reason)
        super().submit_fault(reason)

    def record_control_timing(self, timing):
        self.worker._diagnose("control_loop_timing", timing=timing)

    def submit_xr_frame(self, frame, *, received_monotonic_s=None):
        epoch_before = self._last_epoch
        accepted = super().submit_xr_frame(frame, received_monotonic_s=received_monotonic_s)
        if accepted:
            stamp = self.clock() if received_monotonic_s is None else float(received_monotonic_s)
            self.worker.note_input(stamp)
            grip = frame["controllers"]["right"]["buttons"][1]["p"]
            with self._lock:
                edge = grip != self._last_grip_state
                if edge and grip:
                    self._grip_pressed_s = stamp
                self._last_grip_state = grip
            if edge:
                self.worker._diagnose("grip_edge", pressed=grip,
                    received_monotonic_s=stamp, seq=frame["seq"], epoch=frame["epoch"],
                    hardware_state=self.worker.status()["state"])
            if epoch_before is not None and frame["epoch"] != epoch_before:
                self.worker.revoke("reference_changed")
            with self._operator_lock:
                # Management-button continuity is independent of the right
                # clutch; left Y has its own continuously refreshed permit.
                events = self.buttons.feed(frame, stamp,
                    max_age_s=min(.2, self.shadow.session.policy.input_max_age_s))
                self._gate_operator_commands(events)
                if self.buttons.error:
                    self._operator_barrier = True
                    self.worker.revoke('operator_event_queue_overflow')
                if not grip:
                    if not self._home_busy() and not self.buttons.home_hold_active(stamp):
                        self.worker.revoke('grip_released')
                    self._pause_recording_for_clutch('grip_released', stamp)
            if self.recorder and not self.recorder.status()['recording_paused']:
                self.recorder.record('xr', stamp=stamp, frame=xr_record(frame))
        else:
            self.worker.revoke("invalid_xr_frame")
        return accepted

    def refresh_feedback(self):
        try:
            sample = self.worker.measured_feedback()
            self.shadow.feedback(sample.q_rad, received_monotonic_s=sample.received_monotonic_s,
                                 source=sample.source)
        except (RuntimeError, ValueError, TypeError, KeyError, OverflowError) as exc:
            self.worker.note_feedback_failure(exc)
            self.shadow.session.cache.invalidate("hardware_feedback_unavailable")
            # Serial SDK feedback is intentionally unavailable during a slow
            # stop. The stopping gate already blocks IK and recovery; stale
            # feedback must not become a new transport/device failure.
            if self.worker.status()["state"] not in ("stop_requested", "stopping", "stop_unknown", "fault"):
                self.submit_fault("hardware_feedback_unavailable")

    def step(self):
        self._operator_tick()
        candidate = self._motion_step()
        self._sync_recording_pause_state()
        if self.recorder:
            self.recorder.sample(feedback=self.shadow.session.cache.latest(), hardware=self.worker.status(),
                                 paused=self.operator_paused, candidate=candidate,
                                 control_source='home_return' if self._home_busy() else 'teleop')
        return candidate

    def _home_motion_step(self):
        """Offer a bounded home target under left Y, independently of right Grip."""
        self.watchdog_tick()
        with self._operator_lock:
            if (not self._home_button_requested or
                    (self.home_trigger == 'hold' and
                     not self.buttons.home_hold_active(self.clock())) or
                    self._home_cancel_reason is not None or self._operator_barrier):
                return None
            candidate = Candidate(tuple(self.home['target_q_rad']), self.clock(), self._home_session)
            if not self.worker.offer(candidate, control_source='home_return'):
                return None
            self._home_offered = True
            return candidate

    def _motion_step(self):
        status = self.worker.status()
        if status["fatal"] or status["stop_result_unknown"]:
            self.shadow.reject_input("hardware_fault_latched")
            return None
        if status["state"] in ("stop_requested", "stopping"):
            strict = status["stop_requires_post_confirm_release"]
            if not self._waiting_stop:
                # Clear targets/author anchors once, without manufacturing an
                # input fault on every tick or erasing valid released frames.
                if strict or self.shadow.session.engaged:
                    self.shadow.reject_input("waiting_for_stop_confirmation")
                self._waiting_stop = True
            self.watchdog_tick()
            if self.worker.status()["stop_requires_post_confirm_release"]:
                with self._lock:
                    self._latest_frame = self._latest_frame_received = None
                    self._release_frame = None
            return None
        confirmed = status["stop_confirmed_monotonic_s"]
        if self._waiting_stop or (confirmed is not None and confirmed != self._last_stop_confirmed_s):
            if not status["stop_confirmed"] or confirmed is None:
                self.submit_fault("stop_confirmation_missing")
                return None
            # Consume pending transport faults before deciding whether this
            # confirmation can acknowledge a normal clutch release.
            self.watchdog_tick()
            status = self.worker.status()
            strict = status["stop_requires_post_confirm_release"]
            if not self._waiting_stop and (strict or self.shadow.session.engaged):
                # The SDK thread may complete a short stop between two UI
                # ticks. Apply the same edge gate even if stopping was missed.
                self.shadow.reject_input("waiting_for_stop_confirmation")
            with self._lock:
                early_press = (self._last_grip_state is True and self._grip_pressed_s is not None
                               and self._grip_pressed_s <= confirmed)
                if strict and early_press:
                    self._latest_frame = self._latest_frame_received = None
                    self._release_frame = None
                elif strict:
                    if self._release_frame is not None and self._release_frame[1] <= confirmed:
                        self._release_frame = None
                    if self._latest_frame_received is not None and self._latest_frame_received <= confirmed:
                        self._latest_frame = self._latest_frame_received = None
                else:
                    # The worker has already witnessed the normal release.
                    # Do not replay its old pose through the input-age gate.
                    self._release_frame = None
                    if self._latest_frame_received is not None and self._latest_frame_received <= confirmed:
                        self._latest_frame = self._latest_frame_received = None
            if strict and early_press:
                self.shadow.reject_input("release_grip_after_stop")
            if not strict:
                self.shadow.acknowledge_clutch_stop()
            self._waiting_stop = False
            self._last_stop_confirmed_s = confirmed
            self.worker._diagnose("clutch_stop_gate_open", confirmed_monotonic_s=confirmed,
                requires_post_confirm_release=strict, early_press_discarded=strict and early_press,
                held_press_retained=not strict and early_press,
                fresh_post_confirm_frame_required=True)
        if self.worker.parallel_state_query and not status["feedback_ready"]:
            # Joint push may be fresh while B state is aged. Keep the author
            # clutch/anchor gate closed too, so a held press during this gap
            # cannot accumulate a target and arm when observations return.
            self.shadow.reject_input("hardware_observation_pending")
            with self._lock:
                self._latest_frame = self._latest_frame_received = None
                self._release_frame = None
            return None
        if (self.operator_paused or (self.episode_waiting_start and not self._home_busy()) or self.software_reset_pending
                or self.home['state'] in ('waiting_for_stop', 'stopping')
                or self._operator_barrier or self.session_finished or self.session_finish_pending):
            self.watchdog_tick()
            return None
        if self.home['state'] in ('ready', 'moving'):
            return self._home_motion_step()
        if self.buttons.home_hold_active(self.clock()):
            # Completing a return while Y remains held never starts teleop.
            self.watchdog_tick()
            return None
        step_started = self.clock()
        candidate = super().step()
        if self.clock()-step_started > ControlLoopTiming.stall_s:
            # Do not offer a freshly stamped target produced by a stalled IK
            # tick before the outer loop gets a chance to revoke permission.
            self.submit_fault("preview_control_stall")
            return None
        if candidate is None:
            if status["state"] in ("arming", "active"):
                self.worker.revoke(self.shadow.session.fault or
                                   ("grip_released" if not self.shadow.grip else "not_engaged"))
            return None
        with self._operator_lock:
            if (self.operator_paused or self.episode_waiting_start or self.software_reset_pending
                    or self._home_cancel_reason is not None
                    or self._operator_barrier or self.session_finished or self.session_finish_pending):
                return None
            offered = self.worker.offer(candidate)
        if not offered:
            # A release can win between the status check and offer(). Preserve
            # that normal stop instead of relabeling it as a device/input fault.
            if self.worker.status()["state"] not in ("stop_requested", "stopping"):
                self.shadow.reject_input("hardware_intent_refused")
            return None
        # Cyan = target projected to the selected scope. The actual submitted
        # q is reported separately in hardware.last_attempt.sdk_result.target_rad.
        q = self.worker.project_target(candidate.q_rad)
        return Candidate(q, candidate.generated_monotonic_s, candidate.session)

    def close(self):
        self.worker.revoke("runtime_closing")
        super().close()

    def status(self):
        status = super().status()
        hardware = self.worker.status()
        trigger = hardware["first_stop_trigger"]
        if (trigger is not None and trigger["host_monotonic_s"] != self._last_input_stop_stamp):
            self._last_input_stop_stamp = trigger["host_monotonic_s"]
            if trigger["reason"] != "grip_released":
                # Freeze before later frames/recovery replace the live input
                # state. Only copy the bounded rings once per abnormal stop.
                self._last_input_stop = dict(trigger=trigger,
                    captured_monotonic_s=self.clock(),
                    core_interruption=self.shadow.last_input_interruption,
                    **self.input_timing.snapshot(recent=True))
        if hardware["fatal"] or hardware["stop_result_unknown"]:
            clutch = "fault"
        elif hardware["state"] in ("stop_requested", "stopping"):
            clutch = "stopping"
        elif self.worker.parallel_state_query and not hardware["feedback_ready"]:
            clutch = "waiting_for_feedback"
        elif status["engaged"]:
            clutch = "engaged"
        else:
            clutch = "release_grip" if status["require_release"] else "ready_to_engage"
        if self.episode_waiting_start and clutch not in ('fault', 'stopping'):
            clutch = 'episode_waiting_start'
        collection = self.recorder.status() if self.recorder else dict(enabled=False, state='disabled',
            active_episode=None)
        return dict(status, clutch_state='operator_paused' if self.operator_paused else clutch,
            last_input_stop=self._last_input_stop, operator_paused=self.operator_paused,
            episode_waiting_start=self.episode_waiting_start,
            episode_transition=dict(self._episode_transition) if self._episode_transition else None,
            software_reset_pending=self.software_reset_pending,
            home_return=dict(self.home, button_held=self.buttons.home_hold_active(self.clock())),
            operator_message=self.last_operator_message, session_finished=self.session_finished,
            recording=collection['active_episode'] is not None,
            recording_paused=collection.get('recording_paused', False),
            session_finish_pending=self.session_finish_pending,
            collection=collection)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    execution = p.add_mutually_exclusive_group()
    execution.add_argument("--execute-j6", action="store_true", help="enable bounded J6 hardware commissioning")
    execution.add_argument("--execute-six-axis", action="store_true", help="enable supervised six-axis hardware commissioning")
    execution.add_argument("--inspect-profile", action="store_true",
                           help="login and inspect the six-axis SDK memory profile; no arm, targets or profile writes")
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--user")
    p.add_argument("--runtime")
    p.add_argument("--follow-hz", type=int, choices=(20,25,50,200), default=None,
                   help="explicit six-axis Follow request rate: 20, 25, 50 or 200 Hz; default remains 20 Hz")
    state_query = p.add_mutually_exclusive_group()
    state_query.add_argument("--serial-state-query", dest="parallel_state_query", action="store_false",
                             help="use one SDK connection for motion and serial state queries (default)")
    state_query.add_argument("--parallel-state-query", dest="parallel_state_query", action="store_true",
                             help="opt-in independent read-only SDK connection for six-axis Follow state queries")
    p.set_defaults(parallel_state_query=False)
    p.add_argument("--command-mode", choices=("passthrough", "follow"), default=None,
                   help="six-axis backend; default passthrough; inspect-profile and legacy J6 remain follow")
    p.add_argument("--workspace-alignment", help="explicit calibrated XR-to-workspace JSON")
    p.add_argument('--home-reference', default=str(DEFAULT_HOME_PATH),
                   help='measured joint-home JSON; only used after an explicit home request and fresh Grip engagement')
    p.add_argument('--home-trigger', choices=('hold', 'oneshot'), default='hold',
                   help='left-Y home trigger: hold=press-and-hold (legacy, 200ms freshness gate); '
                        'oneshot=one press latches the return, second press aborts, Grip release still stops')
    p.add_argument("--relay-port", type=int, default=8443)
    p.add_argument("--rotation", choices=("world", "tool_local"), default="world")
    p.add_argument("--preview-speed-deg-s", type=float,
                   help="optional extra candidate cap; default is the accepted simulation baseline")
    p.add_argument("--duration", type=float, default=0.)
    p.add_argument("--diagnostic-output", help="opt-in finite joint diagnostic JSONL under /data; no images")
    p.add_argument('--record', nargs='?', const='auto', help='enable button-managed state/action recording in a new /data directory')
    p.add_argument('--task', help='truthful task description for recorded episodes')
    p.add_argument('--record-fps', type=int, choices=(20,25,50), default=25,
                   help='raw state snapshot rate; independent of SDK send frequency (default 25)')
    p.add_argument("--waypoint-feedback", action="store_true", help="use verified vendor waypoint push for measured joints")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--usb", action="store_true")
    args = p.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0 or not 1 <= args.relay_port <= 65535:
        p.error("Invalid duration, relay port or candidate speed")
    try:
        velocities = preview_velocity_limits(args.preview_speed_deg_s)
    except ValueError as exc:
        p.error(str(exc))
    execute = args.execute_j6 or args.execute_six_axis
    connect = execute or args.inspect_profile
    if args.command_mode is not None and (args.execute_j6 or args.inspect_profile):
        p.error("--command-mode selects the six-axis motion backend only")
    command_mode = args.command_mode or ("passthrough" if not args.execute_j6 and not args.inspect_profile else "follow")
    if args.follow_hz is not None and (not args.execute_six_axis or command_mode != "follow"):
        p.error("--follow-hz requires --execute-six-axis --command-mode follow")
    follow_hz = args.follow_hz or 20
    capture_path = None
    if args.record:
        if not (args.execute_six_axis and command_mode == 'follow' and args.task and args.task.strip()):
            p.error('--record requires --execute-six-axis --command-mode follow and a nonempty --task')
        try:
            capture_path = recording_path(args.record)
        except ValueError as exc:
            p.error(str(exc))
    elif args.task:
        p.error('--task requires --record')
    if args.execute_six_axis and not args.waypoint_feedback:
        p.error("Six-axis commissioning requires --waypoint-feedback")
    if args.parallel_state_query and not (args.execute_six_axis and command_mode == "follow" and args.waypoint_feedback):
        p.error("--parallel-state-query requires --execute-six-axis --command-mode follow --waypoint-feedback")
    from .workspace import load_alignment
    alignment=load_alignment(args.workspace_alignment) if args.workspace_alignment else None
    diagnostic_path = None
    if args.diagnostic_output:
        diagnostic_path = Path(args.diagnostic_output).expanduser().resolve()
        roots = [Path('/data')]
        for part in os.environ.get('AUBO_ALLOWED_ROOTS', '').split(':'):
            if part:
                roots.append(Path(part).expanduser().resolve())
        if (not connect or (execute and not 0 < args.duration <= 300) or
                not any(root in diagnostic_path.parents for root in roots)
                or not diagnostic_path.parent.is_dir()):
            p.error("Diagnostics require explicit execution with duration in (0,300], or profile inspection, and an existing parent directory under /data or AUBO_ALLOWED_ROOTS")
    if not connect:
        if args.host or args.port or args.user:
            p.error("Device parameters require explicit execution")
        print(json.dumps(dict(mode="offline_plan", sdk_loaded=False, motion_available=False,
            execution_scope="default J6 only; --execute-six-axis explicitly enables J1-J6; controller-reported ranges",
            joint_range_source="controller_joint_ranges; no hardcoded manufacturer limit",
            send_hz=1./(J6FollowWorker.six_axis_period_s if command_mode == "passthrough" else J6FollowWorker.six_axis_follow_period_s),
            six_axis_send_hz=1./(J6FollowWorker.six_axis_period_s if command_mode == "passthrough" else J6FollowWorker.six_axis_follow_period_s),
            six_axis_command_mode=command_mode,
            passthrough_profile_interpolation_assumed=False,
            legacy_j6_send_hz=1./J6FollowWorker.period_s,
            active_push_state_query_period_s=J6FollowWorker.active_push_query_period_s,
            idle_state_query_period_s=J6FollowWorker.idle_query_period_s,
            state_query_mode="serial", sdk_connections=1,
            parallel_state_query=False, parallel_state_query_available=True,
            send_max_gap_s=J6FollowWorker.send_max_gap_s,
            input_timeout_s=J6FollowWorker.input_max_age_s,
            grip_release_policy="immediate_revoke_and_pause_recording_until_successful_reengagement",
            waypoint_feedback=args.waypoint_feedback,
            physical_command_speed_deg_s=None,
            sdk_client_speed_deg_s=None,
            six_axis_sdk_profile_policy="sdk_unmodified", six_axis_sdk_profile_writes=False,
            sdk_profile_readback_source="sdk_client_memory_not_controller_settings",
            legacy_j6_profile=dict(speed_deg_s=J6FollowWorker.speed_deg_s,
                acceleration_deg_s2=J6FollowWorker.client_acceleration_deg_s2,
                sdk_profile_policy="legacy_j6_explicit"),
            six_axis_target_policy="direct_ik",
            six_axis_host_target_rate_limit_deg_s=None,
            client_acceleration_deg_s2=None,
            following_error_limit_deg=J6FollowWorker.following_error_limit_deg,
            candidate_speed_deg_s=args.preview_speed_deg_s, recording=False,
            candidate_velocity_limits_deg_s=[math.degrees(v) for v in velocities],core_frequency_hz=200.,
            sdk_request_budget_s=J6FollowWorker.sdk_budget_s,
            stop_completion_budget_s=J6FollowWorker.stop_completion_budget_s,
            stop_feedback_budget_s=J6FollowWorker.stop_feedback_budget_s,
            independent_physical_stop=False, operator_stop_required=True,
            automatic_home=False, automatic_startup=False),ensure_ascii=False,indent=2))
        return 0
    if not args.host or not args.user or args.port is None or not 1 <= args.port <= 65535:
        p.error("Explicit execution requires host, port and user")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    LOG = logging.getLogger("aubo_quest_follow")
    with ExitStack() as stack:
        lock = stack.enter_context((ROOT/".teleop_session.lock").open("a"))
        fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        trace_file = stack.enter_context(diagnostic_path.open('x')) if diagnostic_path else None
        if trace_file:
            trace_file.write(json.dumps(dict(event="session_start", host_monotonic_s=time.monotonic(),
                wall_time_ns=time.time_ns(), pid=os.getpid(),
                mode="profile_inspection" if args.inspect_profile else "follow",
                command_mode=command_mode,
                duration_s=args.duration, dataset_recording=bool(args.record),
                parallel_state_query=args.parallel_state_query,
                max_buffered_events=J6FollowWorker.streaming_history_capacity,
                stream_queue_capacity=8192))+"\n")
            trace_file.flush()
        tls = {}
        if not args.inspect_profile:
            with socket.socket() as probe:
                probe.bind(("127.0.0.1",args.relay_port))
            tls = {} if args.usb else dict(ssl_keyfile=str(ROOT/"certs/key.pem"),ssl_certfile=str(ROOT/"certs/cert.pem"))
            if any(not Path(v).is_file() for v in tls.values()):
                raise RuntimeError("Missing existing TLS certificates")
        native = load_follow_native(args.runtime)
        if follow_hz != 20 and not getattr(native,"supports_follow_rate_selection",False):
            raise RuntimeError("Native binding has no Follow rate selection; rebuild")
        if args.parallel_state_query and not getattr(native,"supports_parallel_state_query",False):
            raise RuntimeError("Native binding has no independent state query support; rebuild")
        if command_mode == "passthrough":
            if not getattr(native,"supports_passthrough",False):
                raise RuntimeError("Native binding has no passthrough support; rebuild")
            client = native.J6Follower(six_axis=True, passthrough=True)
        else:
            client = native.J6Follower(six_axis=args.execute_six_axis or args.inspect_profile,
                **(dict(follow_hz=follow_hz, inspect_profile=args.inspect_profile)
                   if follow_hz != 20 or args.inspect_profile else {}))
        observer = native.RobotStateObserver() if args.parallel_state_query else None
        logged_in = [False, False]
        logout_allowed = [True]
        def logout():
            if logout_allowed[0]:
                # Both workers must be joined before either connection logs out.
                # A exits first; B stays alive until all motion work has ended.
                try:
                    if logged_in[0]:
                        result = client.logout()
                        if not record_ok(result):
                            raise RuntimeError("SDK logout failed")
                finally:
                    if logged_in[1]:
                        result = observer.logout()
                        if not record_ok(result):
                            raise RuntimeError("State SDK logout failed")
        stack.callback(logout)
        password = getpass.getpass("SDK password (not recorded): ")
        try:
            login = client.login(args.host,args.port,args.user,password)
            logged_in[0] = record_ok(login)
            if not logged_in[0]:
                raise RuntimeError("SDK login failed")
            if observer:
                login_b = observer.login(args.host,args.port,args.user,password)
                logged_in[1] = record_ok(login_b)
                if not logged_in[1] or login_b["duration_s"] > J6FollowWorker.sdk_budget_s:
                    raise RuntimeError("State SDK login failed or late")
        finally:
            del password
        if args.waypoint_feedback:
            subscription=client.start_waypoint_feedback()
            if not record_ok(subscription) or subscription['duration_s'] > J6FollowWorker.sdk_budget_s:
                raise RuntimeError('Waypoint feedback subscription failed or late')
            deadline=time.monotonic()+1.
            while not client.waypoint_feedback()['samples']:
                if time.monotonic()>=deadline:raise RuntimeError('Waypoint feedback not received')
                time.sleep(.01)
        if observer:
            from .state_query import validate_state_result
            validate_state_result(observer.query_state(), time.monotonic())
            if not getattr(native, "supports_redundant_waypoint_feedback", False):
                raise RuntimeError("Native binding has no redundant waypoint feedback support; rebuild")
            client.attach_state_observer(observer, bool(args.waypoint_feedback))
        prepare = client.prepare()
        if not record_ok(prepare) or not prepare["prepared"]:
            raise RuntimeError("Follower prepare refused: "+prepare.get("refusal_reason","unknown")+
                               "; SDK client memory profile="+repr(prepare.get("sdk_profile_readback")))
        profile_readback = validate_prepare_profile(prepare, six_axis=args.execute_six_axis or args.inspect_profile,
                                                    command_mode=command_mode,follow_hz=follow_hz)
        if profile_readback["max_velocity_rad_s"] is not None:
            profile_readback["max_velocity_deg_s"] = [math.degrees(v) for v in profile_readback["max_velocity_rad_s"]]
            profile_readback["max_acceleration_deg_s2"] = [math.degrees(v) for v in profile_readback["max_acceleration_rad_s2"]]
        anchor = joints(prepare["anchor_rad"])
        if trace_file:
            trace_file.write(json.dumps(json_safe(dict(event="prepare", result=prepare)))+"\n")
            trace_file.flush()
        if args.inspect_profile:
            report = dict(mode="profile_inspection", motion_available=False,
                sdk_profile_policy=prepare["sdk_profile_policy"], sdk_profile_writes=False,
                sdk_profile_readback=profile_readback, anchor_rad=list(anchor))
            print(json.dumps(report,ensure_ascii=False,indent=2))
            if trace_file:
                trace_file.write(json.dumps(dict(event="session_end", host_monotonic_s=time.monotonic(), **report))+"\n")
                trace_file.flush()
            return 0
        worker = J6FollowWorker(client,anchor,j6_range_rad=prepare["joint_ranges_rad"][5],
                               diagnostics=trace_file is not None,
                               joint_ranges_rad=prepare["joint_ranges_rad"] if args.execute_six_axis else None,
                               waypoint_feedback=args.waypoint_feedback,
                               sdk_profile_readback=profile_readback, command_mode=command_mode,follow_hz=follow_hz,
                               state_observer=observer)
        diagnostic_stream = None
        def close_worker():
            try:
                worker.close()
            except RuntimeError:
                if worker._thread.is_alive() or (worker.state_query and worker.state_query._thread.is_alive()):
                    logout_allowed[0] = False
                    # Do not let destructors perform SDK work under a stuck call.
                    # Deliberately retain native SDK allocations until process exit.
                    client.abandon_cleanup()
                    if observer:
                        observer.abandon_cleanup()
                raise
            finally:
                if trace_file:
                    if diagnostic_stream:
                        diagnostic_stream.close()
                    report = worker.diagnostic_report()
                    events = report.pop("events")
                    if diagnostic_stream:
                        report["stream"] = dict(written_events=diagnostic_stream.written_events,
                            missing_events=diagnostic_stream.missing_events,
                            error=diagnostic_stream.error)
                    if diagnostic_stream is None:
                        for event in events:
                            trace_file.write(json.dumps(json_safe(event), allow_nan=False)+"\n")
                    trace_file.write(json.dumps(json_safe(dict(event="session_end", **report)), allow_nan=False)+"\n")
                    trace_file.flush()
        shadow = make_shadow(anchor,prepare["joint_ranges_rad"],rotation=args.rotation,
                             preview_speed_deg_s=args.preview_speed_deg_s,
                             quest_to_base=alignment["quest_to_base"] if alignment else None,
                             input_timeout_s=worker.input_max_age_s)
        home_reference, home_error = None, None
        if args.execute_six_axis and command_mode == 'follow':
            try:
                home_reference = load_home(args.home_reference, prepare['joint_ranges_rad'])
            except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
                home_error = str(exc)
                LOG.warning('Home return disabled: %s', home_error)
        recorder = None
        if capture_path is not None:
            recorder = RawRecorder(capture_path, task=args.task, fps=args.record_fps,
                metadata=dict(command_mode=command_mode, requested_send_hz=follow_hz,
                    input_timeout_s=worker.input_max_age_s,
                    grip_release_policy='immediate_revoke_and_pause_recording_until_successful_reengagement',
                    state_query_mode='parallel' if args.parallel_state_query else 'serial',
                    rotation=args.rotation, controller_joint_ranges_rad=prepare['joint_ranges_rad'],
                    quest_to_base=shadow.core.config.r_calib,
                    precision_factor=shadow.core.config.precision_factor,
                    operator_buttons=dict(left_stick_left='restart_loop',
                        left_stick_right='exit_loop', left_stick_up='exit_record',
                        left_y_hold='return_home',
                        right_grip_release='pause_recording_until_successful_reengagement',
                        input_timeout='pause_recording_until_successful_reengagement',
                        right_b='unassigned', right_stick_click='mark'),
                    home_reference=home_reference, home_reference_error=home_error,
                    home_target_speed_deg_s=math.degrees(HOME_SPEED_RAD_S),
                    home_target_acceleration_deg_s2=math.degrees(HOME_ACCEL_RAD_S2),
                    home_hold_policy=('left_y_fresh_press_then_hold; release_or_200ms_gap_revokes; right_grip_not_required'
                        if args.home_trigger == 'hold' else
                        'left_y_press_latches_return; second_press_aborts; release_and_200ms_input_gaps_do_not_revoke; '
                        'feedback_or_worker_faults_still_stop; grip_release_stops; right_grip_not_required'),
                    scale_translation=shadow.core.config.scale_translation,
                    scale_rotation=shadow.core.config.scale_rotation,
                    workspace_alignment=alignment))
            # Writer must remain alive through worker shutdown to capture late
            # SDK returns. close_worker runs first on the ExitStack.
            stack.callback(recorder.close)
            stack.callback(lambda:setattr(worker, 'recorder', None))
            worker.recorder = recorder
        stack.callback(close_worker)
        runtime = J6QuestRuntime(shadow,worker,recorder=recorder,home_reference=home_reference,home_error=home_error,
            home_trigger=args.home_trigger,
            control_dt=shadow.core.config.control_dt,
            expected_reference_id=alignment["xr_reference_id"] if alignment else None)
        if args.execute_six_axis:
            runtime.mode = ("aubo_six_axis_supervised_passthrough" if command_mode == "passthrough"
                            else "aubo_six_axis_supervised_follow")
        stack.callback(runtime.close)
        if trace_file:
            diagnostic_stream = DiagnosticStream(trace_file, worker,
                snapshot=lambda:dict(input_stop=runtime._last_input_stop,
                                     hardware=worker.status()))
            diagnostic_stream.start()
        worker.start()
        deadline = time.monotonic()+1.
        while worker.cache.latest() is None:
            if time.monotonic()>deadline:
                raise RuntimeError("Initial feedback unavailable")
            time.sleep(.01)
        worker.cache.require_recent(.25)
        LOG.warning("SUPERVISED %s: controller range deg=%s, target policy=%s, SDK profile policy=%s, profile writes=%s, requested %sHz, waypoint push=%s; motion SDK serial, operator stop required if request blocks",
                    "SIX AXES" if worker.six_axis else "J6 ONLY",
                    worker.status()["joint_ranges_deg"] if worker.six_axis else worker.status()["joint_range_deg"],
                    worker.status()["target_policy"], worker.status()["sdk_profile_policy"], worker.status()["sdk_profile_writes"],
                    1./worker.period_s,args.waypoint_feedback)
        LOG.warning("SDK profile memory readback (not controller/pendant settings): %s", profile_readback)
        LOG.warning("COMMAND MODE=%s; passthrough does not assume FollowMode speed/acceleration interpolation", command_mode)
        LOG.info("Quest valid-input timeout=%.3fs; normal teleop: right Grip release revokes immediately; home return: hold left Y, right Grip not required, Y release/200ms hold gap revokes; queued-pose age gate remains 200ms",
                 worker.input_max_age_s)
        LOG.info("State query mode=%s (%s), SDK connections=%s; policy=%s: active=%sms, idle=%sms; feedback age limit=%sms; fresh query before engagement",
                 worker.status()["state_query_mode"].upper(), worker.status()["state_query_mode_label"],
                 worker.status()["sdk_connections"],
                 worker.status()["state_query_policy"],
                 worker.status()["active_state_query_period_s"]*1000,
                 worker.status()["idle_state_query_period_s"]*1000, worker.feedback_max_age_s*1000)
        if recorder:
            LOG.info('Recording READY (not started): %s; left Y hold=return home without right Grip; left stick left=restart loop, right=exit loop, up=exit record; right Grip release/input timeout pauses current segment until successful re-engagement; right stick click=mark; status /aubo', capture_path)
        def feedback_status():
            status = worker.status()
            return dict(error=status["fault"] if status["fatal"] else None,
                        last_snapshot=status["last_snapshot"])
        serve_quest(args,shadow,runtime,stack,tls,refresh_feedback=runtime.refresh_feedback,
            feedback_status=feedback_status,extra_status=lambda:dict(hardware=worker.status(),
                candidate_sent_to_robot=None,
                candidate_scope=("Six-axis desired target" if worker.six_axis else "J6-projected desired target")+"; actual submission recorded in hardware.last_attempt",
                waypoint_feedback=args.waypoint_feedback,
                preview_speed_deg_s=args.preview_speed_deg_s))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
