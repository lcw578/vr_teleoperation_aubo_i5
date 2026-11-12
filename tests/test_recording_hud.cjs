const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');

const source = fs.readFileSync(path.join(__dirname,
  '../third_party/vr-teleop-kit/src/vr_teleop_kit/relay/web/recording_hud.js'), 'utf8');
const window = {};
vm.runInNewContext(source, { window });
const hud = window.VRTeleopRecordingHud;

assert.strictEqual(hud.formatDuration(0), '00:00');
assert.strictEqual(hud.formatDuration(83.9), '01:23');
assert.strictEqual(hud.formatDuration(3661), '01:01:01');

let view = hud.model(null, {connection:'connecting'});
assert.strictEqual(view.title, '录制状态连接中…');
view = hud.model({collection:{enabled:false,state:'disabled'}}, {});
assert.strictEqual(view.title, '录制未启用');

const completed = {episode_id:2, disposition:'saved', flushed:true};
view = hud.model({collection:{enabled:true,state:'recording',active_episode:3,
  active_episode_elapsed_s:23,saved_episodes:3,last_result:completed}},
  {connection:'connected',activeElapsedSeconds:24});
assert.strictEqual(view.title, '第 4 段 · 录制中');
assert.strictEqual(view.detail, '本段时长 00:24');
assert.strictEqual(view.previous, '上一段：第 3 段已保存 ✓');
assert.strictEqual(view.tone, 'recording');
assert.match(view.promptLine1, /← 重启/);
assert.match(view.promptLine1, /→ 退出循环/);
assert.match(view.promptLine1, /↑ 结束录制/);
assert.match(view.promptLine1, /↓ 未分配/);
assert.match(view.promptLine2, /右 Grip/);
assert.doesNotMatch(view.promptLine1 + view.promptLine2, /左\s*X|左\s*Y/);
assert.match(view.promptLine3, /左 Y：按住回原位/);
assert.match(view.promptLine3, /无需右 Grip/);
view = hud.model({home_return:{state:'moving'}, collection:{enabled:false,state:'disabled'}});
assert.strictEqual(view.title, '正在回原位');
assert.match(view.detail, /左 Y/);

view = hud.model({collection:{enabled:true,state:'saving',active_episode:null,
  pending_saves:1,saving_episodes:[4],last_result:completed}}, {connection:'connected'});
assert.strictEqual(view.title, '正在保存第 5 段…');
assert.strictEqual(view.previous, '上一段：第 3 段已保存 ✓');

view = hud.model({collection:{enabled:true,state:'ready',active_episode:null,
  pending_saves:0,last_result:{episode_id:4,disposition:'invalid',flushed:true}}},
  {connection:'connected'});
assert.strictEqual(view.title, '待录制 · 向左推摇杆开始循环');
assert.strictEqual(view.previous, '上一段：第 5 段已标记无效');

view = hud.model({collection:{enabled:true,state:'recording',active_episode:0,
  pending_saves:0,last_result:null}}, {connection:'disconnected'});
assert.strictEqual(view.title, '录制状态连接中断');
assert.strictEqual(view.detail, '最后确认：第 1 段 · 录制中');
assert.strictEqual(view.previous, '状态可能已过期');
view = hud.model({collection:{enabled:true,state:'paused',active_episode:0,
  pause_reason:'grip_released',active_episode_elapsed_s:12,last_result:null}}, {connection:'connected'});
assert.strictEqual(view.title, '第 1 段 · 已暂停');
assert.match(view.detail, /重新按下右 Grip/);
for (const collection of [
  {enabled:true,state:'paused',active_episode:1,pause_reason:'awaiting_grip'},
  {enabled:true,state:'ready',active_episode:null},
]) {
  view = hud.model({clutch_state:'fault',hardware:{fatal:true,stop_result_unknown:true},
    home_return:{state:'moving'},collection});
  assert.strictEqual(view.title,'停止未确认 · 采集已锁定');
  assert.strictEqual(view.tone,'error');
  assert.match(view.detail,/右 Grip 无法恢复/);
  assert.doesNotMatch(view.detail + view.promptLine2 + view.promptLine3,/重新接合|按住回原位/);
}
view = hud.model({recording_blocked_reason:'硬件故障',collection:{enabled:true,state:'ready'}});
assert.strictEqual(view.title,'硬件故障 · 采集已锁定');
assert.strictEqual(hud.model(null, {connection:'unsupported'}).visible, false);
view = hud.model({collection:{enabled:true,state:'error',error:'disk full',
  last_result:completed}}, {connection:'connected'});
assert.strictEqual(view.title, '录制失败');
assert.strictEqual(view.detail, 'disk full');
view = hud.model({session_finished:true,collection:{enabled:true,state:'saving',
  active_episode:null,pending_saves:1,saving_episodes:[0],last_result:null}},
  {connection:'connected'});
assert.strictEqual(view.title, '正在保存第 1 段…');

const html = fs.readFileSync(path.join(__dirname,
  '../third_party/vr-teleop-kit/src/vr_teleop_kit/relay/web/index.html'), 'utf8');
const scriptSources = [...html.matchAll(/<script src="([^"]+)"/g)].map(match => match[1]);
const hudScriptIndex = scriptSources.findIndex(src => src.startsWith('/static/recording_hud.js?'));
const clientScriptIndex = scriptSources.findIndex(src => src.startsWith('/static/client.js?'));
assert.ok(hudScriptIndex >= 0);
assert.ok(clientScriptIndex >= 0);
assert.ok(hudScriptIndex < clientScriptIndex,
  'HUD formatter must load before the XR client');
assert.match(scriptSources[hudScriptIndex], /\?v=9$/);
assert.match(scriptSources[clientScriptIndex], /\?v=42$/);
const client = fs.readFileSync(path.join(__dirname,
  '../third_party/vr-teleop-kit/src/vr_teleop_kit/relay/web/client.js'), 'utf8');
assert.match(client, /fetch\("\/aubo\/status"/);
assert.match(client, /_renderRecordingHud\(vpose, layer, _time\)/);
assert.match(client, /AbortController\(\)/);

const free = (hardware, extra = {}) => hud.model({
  collection:{enabled:false,state:'disabled'}, hardware, ...extra });
view = free({state:'holding',hot_fast:2,hot_ready:false});
assert.match(view.title,/等待发送稳定/); assert.match(view.detail,/2\/5/);
view = free({state:'holding',hot_fast:9,hot_ready:false});
assert.match(view.title,/等待反馈与静止确认/);
view = free({state:'holding',hot_fast:9,hot_ready:true});
assert.match(view.title,/可重新接合/);
view = free({state:'holding',hot_fast:9,hot_ready:true,underrun_recovering:true});
assert.match(view.title,/供点恢复中/); assert.doesNotMatch(view.title,/可重新接合/);
view = free({state:'priming'}); assert.match(view.title,/预热/);
view = free({state:'braking'}); assert.match(view.title,/制动/);
view = free({state:'braking',immediate_hot_clutch:true}); assert.match(view.detail,/随时/);
view = free({state:'holding',immediate_hot_clutch:true,hot_fast:0});
assert.match(view.title,/可重新接合/); assert.doesNotMatch(view.detail,/快帧/);

view = free({state:'active'}, {engaged:true}); assert.strictEqual(view.title,'遥操已接合');
view = free({state:'idle',feedback_ready:true}, {require_release:true,xr_input:{right_present:false}});
assert.strictEqual(view.title,'等待右手柄输入');
view = free({state:'idle',feedback_ready:false}); assert.strictEqual(view.title,'等待机械臂反馈');
view = free({state:'holding',fatal:true,underrun_recovering:true});
assert.strictEqual(view.tone,'error'); assert.match(view.title,/锁定/);
console.log('Recording HUD and free teleoperation admission states passed.');
