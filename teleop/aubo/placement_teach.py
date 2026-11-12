"""Capture an unloaded basket reference via loopback GET; never loads the SDK."""
import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import tempfile
import time

from .feedback import joints
from .home import DEFAULT_HOME_PATH, load_home


MAX_AGE_S = .2
QUIET_S = .6
QUIET_SPAN_RAD = math.radians(.02)
MIN_SAMPLES = 5
MAX_BODY_BYTES = 2 * 1024 * 1024


def read_local_status(port=8443):
    """Only GET the existing local web service; no redirect, proxy or robot I/O."""
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=1.)
    try:
        connection.request('GET', '/aubo/status')
        response = connection.getresponse()
        if response.status != 200:
            raise ValueError('本机状态页面不可用（HTTP %s）' % response.status)
        body = response.read(MAX_BODY_BYTES + 1)
        if len(body) > MAX_BODY_BYTES:
            raise ValueError('状态响应过大，未保存点位')
        return json.loads(body)
    finally:
        connection.close()


def stationary_observation(status, now):
    """Validate actual fresh SDK feedback, never the cyan/candidate target."""
    try:
        if (status['mode'] != 'aubo_six_axis_supervised_follow' or
                status['gripper_installed'] is not False or
                status['waypoint_feedback'] is not True or status['closed'] is not False or
                status['engaged'] is not False or status['session_finished'] is not False or
                status['session_finish_pending'] is not False):
            raise ValueError('需要无夹爪六轴 Follow 会话，松开 Grip 后保持待机')
        hardware = status['hardware']
        if (hardware['state'] != 'idle' or hardware['fatal'] is not False or
                hardware['stop_result_unknown'] is not False or
                hardware['execution_scope'] != 'six_axis' or
                hardware['command_mode'] != 'follow' or
                status['home_return']['state'] not in ('idle', 'complete', 'cancelled')):
            raise ValueError('机械臂尚未处于可记录的静止待机状态')
        snapshot = hardware['last_snapshot']
        jr, sr = snapshot['joints'], snapshot['state']
        if (jr['return_code'] != 0 or jr['source'] != 'sdk_waypoint_push' or
                sr['return_code'] != 0 or sr['data'] != 0):
            raise ValueError('需要实际关节推送和控制柜停止状态')
        q = joints(jr['data'])
        jt, st = (float(r['host_received_monotonic_s']) for r in (jr, sr))
        for stamp in (jt, st):
            if not math.isfinite(stamp) or not 0 <= now-stamp <= MAX_AGE_S:
                raise ValueError('反馈已过期或时钟不匹配，未保存点位')
        bounds = hardware['joint_ranges_deg']
        if len(bounds) != 6:
            raise ValueError('缺少六轴控制柜关节范围')
        ranges = tuple(tuple(math.radians(float(v)) for v in pair) for pair in bounds)
        if any(len(pair) != 2 or not all(math.isfinite(v) for v in pair) or
               pair[0] >= pair[1] or not pair[0] <= angle <= pair[1]
               for pair, angle in zip(ranges, q)):
            raise ValueError('实测关节超出有效控制柜范围')
        return dict(q_rad=list(q), joint_received_monotonic_s=jt,
                    state_received_monotonic_s=st, read_monotonic_s=now,
                    joint_ranges_rad=[list(pair) for pair in ranges])
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('状态字段不完整，未保存点位') from exc


def collect_reference(home_path, *, read_status=read_local_status, clock=time.monotonic,
                      sleep=time.sleep, timeout_s=4.):
    home_path = Path(home_path).expanduser().resolve()
    home_bytes = home_path.read_bytes()
    evidence = []
    started = clock()
    reference = None
    while clock()-started <= timeout_s:
        status = read_status()
        observation = stationary_observation(status, clock())
        if reference is None:
            reference = load_home(home_path, observation['joint_ranges_rad'])
        # The running session must use the same home selected for this route.
        if joints(status['home_return']['target_q_rad']) != reference['q_rad']:
            raise ValueError('会话原位与计划中转位不一致，未保存点位')
        if evidence:
            previous = evidence[-1]
            jt, st = (observation[k] for k in
                      ('joint_received_monotonic_s', 'state_received_monotonic_s'))
            if (jt < previous['joint_received_monotonic_s'] or
                    st < previous['state_received_monotonic_s'] or
                    observation['joint_ranges_rad'] != previous['joint_ranges_rad']):
                raise ValueError('反馈顺序或控制柜范围发生变化，未保存点位')
            if jt == previous['joint_received_monotonic_s']:
                sleep(.05)
                continue
        evidence.append(observation)
        spans = [max(row['q_rad'][axis] for row in evidence) -
                 min(row['q_rad'][axis] for row in evidence) for axis in range(6)]
        if max(spans) > QUIET_SPAN_RAD:
            raise ValueError('观察到关节仍在变化，请停稳后再保存')
        duration = observation['joint_received_monotonic_s']-evidence[0]['joint_received_monotonic_s']
        state_duration = observation['state_received_monotonic_s']-evidence[0]['state_received_monotonic_s']
        if len(evidence) >= MIN_SAMPLES and min(duration, state_duration) >= QUIET_S:
            if home_path.read_bytes() != home_bytes:
                raise ValueError('采集期间原位配置发生变化，未保存点位')
            return dict(schema='aubo_placement_reference_v1',
                role='basket_above', captured_at_utc=datetime.now(timezone.utc).isoformat(),
                capture_method='read_only_existing_session_status',
                joint_source='sdk_waypoint_push', joint_names=[f'joint_{n}' for n in range(1, 7)],
                q_rad=observation['q_rad'], q_deg=[math.degrees(v) for v in observation['q_rad']],
                joint_ranges_rad=observation['joint_ranges_rad'],
                controller_state=0, host_state='idle', tool='bare_flange', gripper_installed=False,
                home_reference=dict(path=str(home_path), sha256=hashlib.sha256(home_bytes).hexdigest(),
                                    q_rad=list(reference['q_rad'])),
                stability=dict(duration_s=duration, state_duration_s=state_duration,
                               unique_joint_samples=len(evidence), joint_span_deg=[math.degrees(v) for v in spans]),
                evidence=evidence, automatic_motion_enabled=False, route_validated=False,
                note='Measured unloaded endpoint only. Route and future gripper clearance require separate validation.')
        sleep(.05)
    raise ValueError('未取得连续新鲜的静止反馈，未保存点位')


def save_new_reference(path, reference):
    """Publish a complete new file atomically; never overwrite a taught point."""
    path = Path(path).expanduser().resolve()
    roots = [Path('/data')]+[Path(p).expanduser().resolve()
                            for p in os.environ.get('AUBO_ALLOWED_ROOTS', '').split(':') if p]
    if not any(root in path.parents for root in roots):
        raise ValueError('点位必须保存在 /data 或 AUBO_ALLOWED_ROOTS 指定目录内')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.basket-', suffix='.tmp', delete=False) as out:
            temporary = Path(out.name)
            json.dump(reference, out, ensure_ascii=False, allow_nan=False, indent=2)
            out.write('\n')
            out.flush()
            os.fsync(out.fileno())
        os.link(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-basket', action='store_true', help='读取已停稳机械臂的篮筐上方点；不发送运动')
    parser.add_argument('--home-reference', default=str(DEFAULT_HOME_PATH))
    parser.add_argument('--port', type=int, default=8443, help='本机 USB 遥操网页端口')
    parser.add_argument('--output', help='新 JSON 文件；默认保存在运行目录 config/placement')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('Invalid local web port')
    if not args.capture_basket:
        print('只读示教工具已就绪。机械臂停在篮筐上方并松开 Grip 后，使用 --capture-basket 保存。')
        return 0
    try:
        reference = collect_reference(args.home_reference, read_status=lambda:read_local_status(args.port))
        reference['source_url'] = f'http://127.0.0.1:{args.port}/aubo/status'
        default = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'),
                       'config', 'placement', 'basket-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json')
        path = save_new_reference(args.output or default, reference)
    except (OSError, ValueError, TypeError, http.client.HTTPException) as exc:
        print('点位未保存：'+str(exc))
        print('请启动原有 USB 遥操会话，移至篮筐上方，松开 Grip 并等待停稳；此工具不会启动或移动机械臂。')
        return 2
    print('篮筐上方点已保存：'+str(path))
    print('实测关节角（度）：'+', '.join(f'{q:.4f}' for q in reference['q_deg']))
    print('已保存端点；自动运动尚未启用，路径仍待检查。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
