// Exercise the exact browser pivot-fit functions, not a Python reimplementation.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(path.join(__dirname,'../third_party/vr-teleop-kit/src/vr_teleop_kit/relay/web/client.js'),'utf8');
function extract(name) {
  const start=source.indexOf(`const ${name} =`);
  assert(start>=0,name);
  const body=source.indexOf('{',start);
  let depth=0,end=body;
  for(;end<source.length;end++) {
    if(source[end]==='{') depth++;
    if(source[end]==='}' && --depth===0) break;
  }
  return source.slice(start,end+2);
}
const names=['solve3x3','solvePivot','quatToMat3','rotateVecByQuat'];
const functions=vm.runInNewContext(names.map(extract).join('\n')+`\n({${names.join(',')}});`);
const offset=[.015,-.025,.075], pivot=[.2,1.4,-.3], samples=[];
for(let i=0;i<120;i++) {
  const q=[Math.sin(i*.17),Math.cos(i*.23),Math.sin(i*.31),1];
  const norm=Math.hypot(...q);for(let j=0;j<4;j++) q[j]/=norm;
  const shift=functions.rotateVecByQuat(offset,q);
  samples.push({p:pivot.map((v,j)=>v-shift[j]),R:functions.quatToMat3(q)});
}
const fit=functions.solvePivot(samples);
assert(fit.ok);
assert(fit.rms<1e-12);
fit.o.forEach((v,i)=>assert(Math.abs(v-offset[i])<1e-12));
assert(!functions.solvePivot(samples.slice(0,29)).ok);
const singleAxis=[];
for(let i=0;i<120;i++) {
  const q=[0,0,Math.sin(i*.02),Math.cos(i*.02)];
  const shift=functions.rotateVecByQuat(offset,q);
  singleAxis.push({p:pivot.map((v,j)=>v-shift[j]),R:functions.quatToMat3(q)});
}
assert(!functions.solvePivot(singleAxis).ok);
console.log('Browser wrist pivot: recovered 3D offset; insufficient samples and single-axis capture rejected.');
