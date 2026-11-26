import test from 'node:test';
import assert from 'node:assert/strict';
import { makeJoint, PRESETS } from '../js/dh.js';
import { encodeConfig, decodeConfig, configFromHash } from '../js/config.js';
import { trajectoryAt, wrapDelta } from '../js/trajectory.js';
import { sampleWorkspace, workspaceBounds } from '../js/workspace.js';

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
