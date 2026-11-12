// Exercise the browser gate used by the production XR sender.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(path.join(__dirname,
  '../third_party/vr-teleop-kit/src/vr_teleop_kit/relay/web/client.js'), 'utf8');
const start = source.indexOf('const createXRInputGate =');
assert(start >= 0);
let depth = 0, end = source.indexOf('{', start);
for (; end < source.length; end++) {
  if (source[end] === '{') depth++;
  if (source[end] === '}' && --depth === 0) break;
}
const create = vm.runInNewContext(source.slice(start, end + 2) + '\ncreateXRInputGate;');
const controllers = (pressed = false, grip = false, axis = 0) => ({right: {
  buttons: [{p: pressed, v: pressed ? 1 : 0}, {p: grip, v: grip ? 1 : 0}], axes: [axis],
}});
function fixture() {
  const gate = create();
  let now = 0;
  gate.reset(now);
  return { gate, sample: (pressed = false, options = {}) => {
    now += options.dt ?? 20;
    return gate.update(now, now - (options.age ?? 0), options.visibility ?? 'visible',
      options.missing ? {} : controllers(pressed, options.grip, options.axis), options.tracked ?? true);
  }};
}
function arm(f) { for (let i = 0; i < 30; i++) f.sample(); assert(f.sample()); }

let f = fixture();
assert(!f.sample(true)); // initially held never authorizes an action
for (let i = 0; i < 50; i++) assert(!f.sample(true));
for (let i = 0; i < 10; i++) assert(!f.sample());
assert(f.sample());
assert(f.sample(true)); // a fresh press after release is accepted
assert(f.sample(true)); // holding is a state; the backend owns edge detection

for (const visibility of ['hidden', 'visible-blurred']) {
  f = fixture(); arm(f);
  for (const pressed of [false, true, false, true, false])
    assert(!f.sample(pressed, {visibility}));
  // Deferred press/release sequences arriving during resume are discarded.
  for (let i = 0; i < 24; i++) assert(!f.sample(i % 2 === 0));
  for (let i = 0; i < 40; i++) assert(!f.sample(true));
  for (let i = 0; i < 10; i++) assert(!f.sample());
  assert(f.sample());
  assert(f.sample(true));
}

f = fixture(); arm(f);
assert(!f.sample(true, {dt: 1000})); // RAF pauses even without visibilitychange
for (let i = 0; i < 20; i++) assert(!f.sample());
for (let i = 0; i < 15; i++) f.sample();
assert(f.sample());
assert(!f.sample(true, {age: 1000})); // cached frame cannot be relabeled as current
for (let i = 0; i < 30; i++) assert(!f.sample(true));

for (const options of [{missing:true}, {tracked:false}]) {
  f = fixture(); arm(f);
  assert(!f.sample(false, options));
  assert(!f.sample(true));
}
f = fixture(); arm(f);
f.gate.reset(620); // session visibility event / reconnect / spatial reset
for (let i = 0; i < 30; i++) f.sample(false, {grip:true,axis:0.8});
assert(f.sample(false, {grip:true,axis:0.8}));
assert(f.sample(true)); // unrelated Grip/axes do not gate Trigger recovery

assert.match(source, /session\.addEventListener\("visibilitychange", suspendXRInput\)/);
assert.match(source, /bindXRInputLifecycle\(xrSession\)/);
assert.match(source, /t_client:_time, input_active:inputActive/);
assert.match(source, /controllers:inputActive \? controllers : \{\}/);
console.log('XR visibility, resume drain, release, tracking loss and delayed-frame gates passed.');
