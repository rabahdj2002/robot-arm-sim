import test from 'node:test';
import assert from 'node:assert/strict';
import { makeJoint, PRESETS } from '../js/dh.js';
import { encodeConfig, decodeConfig, configFromHash } from '../js/config.js';
import { trajectoryAt, wrapDelta } from '../js/trajectory.js';
import { sampleWorkspace, sampleWorkspaceConfigs, workspaceBounds, workspaceShell } from '../js/workspace.js';

test('config encode/decode round-trips a preset', () => {
  const joints = PRESETS['SCARA (RRP)']();
  const back = decodeConfig(encodeConfig(joints));
  assert.equal(back.length, joints.length);
  back.forEach((j, i) => {
    assert.equal(j.type, joints[i].type);
    assert.ok(Math.abs(j.a - joints[i].a) < 1e-4);
    assert.ok(Math.abs(j.alpha - joints[i].alpha) < 1e-4);
    assert.ok(Math.abs(j.value - joints[i].value) < 1e-4);
  });
});

test('encoded config is URL-safe (no +, /, =)', () => {
  const s = encodeConfig(PRESETS['PUMA-style 6R']());
  assert.ok(!/[+/=]/.test(s));
});

test('configFromHash parses a config URL fragment', () => {
  const joints = [makeJoint('revolute', { a: 0.2 })];
  joints[0].name = 'Base α';
  const hash = `#config=${encodeConfig(joints)}`;
  const back = configFromHash(hash);
  assert.equal(back.length, 1);
  assert.equal(back[0].name, 'Base α');
  assert.equal(configFromHash('#other=1'), null);
  assert.equal(configFromHash('#config=!!garbage'), null);
});

test('wrapDelta takes the shortest path across ±π', () => {
  const j = makeJoint('revolute'); // full ±π range
  const d = wrapDelta(2.9, -2.9, j);
  assert.ok(d > 0 && d < 1, `expected short positive wrap, got ${d}`);
  const d2 = wrapDelta(0, 1.0, j);
  assert.ok(Math.abs(d2 - 1.0) < 1e-9);
});

test('trajectoryAt hits waypoints exactly at segment boundaries', () => {
  const joints = [makeJoint('revolute'), makeJoint('revolute')];
  const wps = [[0, 0], [1, 0.5], [-0.5, 1]];
  const v0 = trajectoryAt(wps, joints, 0, 1.5, false);
  assert.ok(Math.abs(v0[0]) < 1e-9 && Math.abs(v0[1]) < 1e-9);
  const v1 = trajectoryAt(wps, joints, 1.5, 1.5, false);
  assert.ok(Math.abs(v1[0] - 1) < 1e-9 && Math.abs(v1[1] - 0.5) < 1e-9);
  // mid-segment is between the endpoints
  const vm = trajectoryAt(wps, joints, 0.75, 1.5, false);
  assert.ok(vm[0] > 0 && vm[0] < 1);
});

test('trajectoryAt loops back to the first waypoint', () => {
  const joints = [makeJoint('revolute')];
  const wps = [[0], [1]];
  // at t = 2*seg in loop mode we return to waypoint 0
  const v = trajectoryAt(wps, joints, 3.0, 1.5, true);
  assert.ok(Math.abs(v[0]) < 1e-9);
});

test('sampleWorkspace produces reachable positions within bounds', () => {
  const joints = PRESETS['2R Planar Arm']();
  const pts = sampleWorkspace(joints, 500);
  assert.equal(pts.length, 500 * 3);
  const { rMax } = workspaceBounds(pts);
  const reach = joints[0].a + joints[1].a;
  assert.ok(rMax <= reach + 1e-6, `rMax ${rMax} exceeds reach ${reach}`);
  assert.ok(rMax > reach * 0.5, 'samples too clustered');
});

test('workspaceShell builds a closed dome near max reach', () => {
  const joints = PRESETS['2R Planar Arm']();
  const pts = sampleWorkspace(joints, 4000);
  const shell = workspaceShell(pts);
  assert.ok(shell.positions.length > 0);
  assert.ok(shell.indices.length > 0);
  // every dome vertex is a real reach point — never beyond max reach
  const reach = joints[0].a + joints[1].a;
  let vMax = 0;
  for (let i = 0; i < shell.positions.length; i += 3) {
    vMax = Math.max(vMax, Math.hypot(
      shell.positions[i], shell.positions[i + 1], shell.positions[i + 2]));
  }
  assert.ok(vMax <= reach + 1e-6, `dome vertex at ${vMax} exceeds reach`);
  assert.ok(vMax > reach * 0.8, 'dome collapsed inward');
});

test('workspaceShell with zMin=0 never dips under the floor', () => {
  const joints = PRESETS['PUMA-style 6R']();
  const pts = sampleWorkspace(joints, 20000, Math.random, 0);
  for (let i = 2; i < pts.length; i += 3) {
    assert.ok(pts[i] >= 0, `sample z=${pts[i]} under floor`);
  }
  const shell = workspaceShell(pts, 72, 36, 0);
  for (let i = 2; i < shell.positions.length; i += 3) {
    assert.ok(shell.positions[i] >= -1e-9, `dome vertex z=${shell.positions[i]} under floor`);
  }
});

test('workspaceShell refinement pushes dome to the true reach bound', () => {
  const joints = PRESETS['PUMA-style 6R']();
  const { positions, values } = sampleWorkspaceConfigs(joints, 15000, Math.random, 0);
  let sMax = 0;
  for (let i = 0; i < positions.length; i += 3) {
    sMax = Math.max(sMax, Math.hypot(positions[i], positions[i + 1], positions[i + 2]));
  }
  const refined = workspaceShell(positions, 72, 36, 0, joints, values);
  let vMax = 0;
  for (let i = 0; i < refined.positions.length; i += 3) {
    vMax = Math.max(vMax, Math.hypot(
      refined.positions[i], refined.positions[i + 1], refined.positions[i + 2]));
  }
  // refinement finds extremes MC missed — but never a physically impossible one
  const reachBound = joints.reduce((s, j) => s + Math.abs(j.a) + Math.abs(j.d), 0);
  assert.ok(vMax >= sMax, `refined ${vMax} < sampled ${sMax}`);
  assert.ok(vMax <= reachBound + 1e-6, `refined ${vMax} exceeds reach bound ${reachBound}`);
});
