import test from 'node:test';
import assert from 'node:assert/strict';
import {
  mat4Identity, mat4Multiply, dhTransform, forwardKinematics,
  getPosition, getRPY, degreesOfFreedom, dhTable, makeJoint,
  JOINT_TYPES,
} from '../js/dh.js';

const pos = (m) => getPosition(m);
const near = (a, b, eps = 1e-9) => Math.abs(a - b) < eps;
const vecNear = (a, b, eps = 1e-9) => a.every((v, i) => near(v, b[i], eps));

test('identity matrix', () => {
  const I = mat4Identity();
  assert.deepEqual(I[0], [1, 0, 0, 0]);
  assert.deepEqual(mat4Multiply(I, I), I);
});

test('zero DH params -> identity transform', () => {
  assert.deepEqual(dhTransform(0, 0, 0, 0), mat4Identity());
});

test('pure z translation', () => {
  const T = dhTransform(0, 0.5, 0, 0);
  assert.ok(vecNear(pos(T), [0, 0, 0.5]));
});

test('revolute joint rotates the frame', () => {
  const T = dhTransform(Math.PI / 2, 0, 0.2, 0);
  // link along local x, rotated 90° about z -> position (0, a, 0)
  assert.ok(vecNear(pos(T), [0, 0.2, 0], 1e-12));
});

test('2R planar arm: zero config reaches a1+a2 along x', () => {
  const joints = [makeJoint('revolute', { a: 0.3 }), makeJoint('revolute', { a: 0.25 })];
  const { endEffector, transforms } = forwardKinematics(joints);
  assert.equal(transforms.length, 3);
  assert.ok(vecNear(pos(endEffector), [0.55, 0, 0], 1e-12));
});

test('2R planar arm: rotate shoulder 90° reaches along y', () => {
  const j1 = makeJoint('revolute', { a: 0.3, value: Math.PI / 2 });
  const j2 = makeJoint('revolute', { a: 0.25 });
  const p = pos(forwardKinematics([j1, j2]).endEffector);
  assert.ok(near(p[0], 0, 1e-12));
  assert.ok(near(p[1], 0.55, 1e-12));
});

test('2R arm: fold elbow 180° collapses reach to |a1-a2|', () => {
  const j1 = makeJoint('revolute', { a: 0.3 });
  const j2 = makeJoint('revolute', { a: 0.25, value: Math.PI });
  const p = pos(forwardKinematics([j1, j2]).endEffector);
  assert.ok(vecNear(p, [0.05, 0, 0], 1e-12));
});

test('prismatic joint extends along z', () => {
  const j = makeJoint('prismatic', { d: 0.1, a: 0, value: 0.05 });
  const p = pos(forwardKinematics([j]).endEffector);
  assert.ok(vecNear(p, [0, 0, 0.15], 1e-12));
});

test('alpha twist: alpha=pi/2 bends chain into next plane', () => {
  // Frame1 z-axis points along world y after alpha=pi/2; a link a along x stays in-plane.
  const j1 = makeJoint('revolute', { a: 0, d: 0.2, alpha: Math.PI / 2 });
  const j2 = makeJoint('revolute', { a: 0.3 });
  const p = pos(forwardKinematics([j1, j2]).endEffector);
  assert.ok(near(p[0], 0.3, 1e-12));
  assert.ok(near(p[1], 0, 1e-12));
  assert.ok(near(p[2], 0.2, 1e-12));
});

test('rotation part stays orthonormal under arbitrary config', () => {
  const joints = [
    makeJoint('revolute', { a: 0.2, d: 0.1, alpha: 0.7, value: 0.4 }),
    makeJoint('revolute', { a: 0.15, d: 0.05, alpha: -0.9, value: -0.8 }),
    makeJoint('prismatic', { a: 0.1, value: 0.07 }),
  ];
  const R = forwardKinematics(joints).endEffector;
  for (let c = 0; c < 3; c++) {
    const n = Math.hypot(R[0][c], R[1][c], R[2][c]);
    assert.ok(near(n, 1, 1e-12), `column ${c} unit norm`);
  }
  const det =
    R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
    R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
    R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
  assert.ok(near(det, 1, 1e-12));
});

test('DOF counts only actuated joints', () => {
  const joints = [
    makeJoint('revolute'), makeJoint('prismatic'),
    { type: 'fixed', theta: 0, d: 0.1, a: 0.1, alpha: 0, value: 0 },
  ];
  assert.equal(degreesOfFreedom(joints), 2);
});

test('getRPY round-trips a yaw', () => {
  const T = dhTransform(0.7, 0, 0, 0);
  const [r, p, y] = getRPY(T);
  assert.ok(near(r, 0, 1e-12));
  assert.ok(near(p, 0, 1e-12));
  assert.ok(near(y, 0.7, 1e-12));
});

test('dhTable marks the joint variable', () => {
  const rows = dhTable([makeJoint('revolute'), makeJoint('prismatic')]);
  assert.equal(rows[0].variable, 'θ');
  assert.equal(rows[1].variable, 'd');
});

test('empty chain: FK returns identity, DOF 0, empty DH table', () => {
  const { transforms, endEffector } = forwardKinematics([]);
  assert.equal(transforms.length, 1);
  assert.deepEqual(endEffector, mat4Identity());
  assert.equal(degreesOfFreedom([]), 0);
  assert.deepEqual(dhTable([]), []);
});

test('single joint chain: FK = that joint transform', () => {
  const j = makeJoint('revolute', { a: 0.2, value: Math.PI / 2 });
  const { endEffector } = forwardKinematics([j]);
  assert.ok(vecNear(pos(endEffector), [0, 0.2, 0], 1e-12));
});

test('chain length scales: FK valid for 1..8 joints', () => {
  for (let n = 1; n <= 8; n++) {
    const joints = Array.from({ length: n }, (_, i) =>
      makeJoint(i % 2 ? 'prismatic' : 'revolute', { a: 0.1, d: 0.05, value: 0.1 * i }));
    const { transforms, endEffector } = forwardKinematics(joints);
    assert.equal(transforms.length, n + 1);
    assert.equal(degreesOfFreedom(joints), n);
    pos(endEffector).forEach((v) => assert.ok(Number.isFinite(v)));
  }
});

test('removing a mid-chain joint changes downstream frames correctly', () => {
  const j1 = makeJoint('revolute', { a: 0.3 });              // +0.3 x
  const j2 = makeJoint('prismatic', { a: 0, d: 0.1, value: 0.1 }); // +0.2 z total
  const j3 = makeJoint('revolute', { a: 0.25 });             // +0.25 x
  const full = pos(forwardKinematics([j1, j2, j3]).endEffector);
  assert.ok(vecNear(full, [0.55, 0, 0.2], 1e-12));
  const minusJ2 = pos(forwardKinematics([j1, j3]).endEffector);
  assert.ok(vecNear(minusJ2, [0.55, 0, 0], 1e-12)); // z contribution gone
});

test('splice-out first joint shifts whole chain', () => {
  const j1 = makeJoint('revolute', { a: 0.3, value: Math.PI / 2 });
  const j2 = makeJoint('revolute', { a: 0.25 });
  // j1 rotated 90°: downstream link extends along +y -> EE at (0, 0.55, 0)
  const withBase = pos(forwardKinematics([j1, j2]).endEffector);
  assert.ok(vecNear(withBase, [0, 0.55, 0], 1e-12));
  const without = pos(forwardKinematics([j2]).endEffector);
  assert.ok(vecNear(without, [0.25, 0, 0], 1e-12));
});
