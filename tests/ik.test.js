import test from 'node:test';
import assert from 'node:assert/strict';
import { forwardKinematics, getPosition, makeJoint } from '../js/dh.js';
import { solveIK, positionJacobian } from '../js/ik.js';

const near = (a, b, eps) => Math.abs(a - b) < eps;
const reach2R = (a1 = 0.3, a2 = 0.25) => [
  makeJoint('revolute', { a: a1 }),
  makeJoint('revolute', { a: a2 }),
];

test('IK solves a reachable 2R target in the plane', () => {
  const joints = reach2R();
  const res = solveIK(joints, [0.3, 0.4, 0]);
  assert.ok(res.converged, `failed to converge, err=${res.error}`);
  const p = getPosition(forwardKinematics(joints, res.values).endEffector);
  assert.ok(near(p[0], 0.3, 1e-3));
  assert.ok(near(p[1], 0.4, 1e-3));
});

test('IK respects joint limits', () => {
  const joints = reach2R();
  joints[1].min = -0.5;
  joints[1].max = 0.5;
  const res = solveIK(joints, [0.3, 0.4, 0]);
  res.values.forEach((v, i) => {
    assert.ok(v <= joints[i].max + 1e-9 && v >= joints[i].min - 1e-9);
  });
});

test('IK reports non-convergence for unreachable target', () => {
  const joints = reach2R(0.1, 0.1);
  const res = solveIK(joints, [5, 5, 5]);
  assert.equal(res.converged, false);
  assert.ok(res.error > 0.1);
});

test('IK on 6R arm finds a pose', () => {
  const joints = [
    makeJoint('revolute', { a: 0, d: 0.25, alpha: -Math.PI / 2 }),
    makeJoint('revolute', { a: 0.22 }),
    makeJoint('revolute', { a: 0, alpha: Math.PI / 2 }),
    makeJoint('revolute', { a: 0, d: 0.22, alpha: -Math.PI / 2 }),
    makeJoint('revolute', { a: 0, alpha: Math.PI / 2 }),
    makeJoint('revolute', { a: 0, d: 0.1 }),
  ];
  const res = solveIK(joints, [0.15, 0.15, 0.4], { maxIterations: 500 });
  assert.ok(res.error < 0.01, `residual ${res.error}`);
});

test('position Jacobian has correct shape and finite values', () => {
  const joints = reach2R();
  const J = positionJacobian(joints, [0.3, -0.2]);
  assert.equal(J.length, 3);
  assert.equal(J[0].length, 2);
  J.flat().forEach((v) => assert.ok(Number.isFinite(v)));
});

test('revolute joints wrap across the ±π boundary instead of stalling', () => {
  const joints = reach2R();
  joints[0].value = 3.1; // parked at the +π boundary
  // target sits on the far side — solver must pass "through" ±π
  const res = solveIK(joints, [-0.4, -0.35, 0]);
  assert.ok(res.converged, `failed to converge, err=${res.error}`);
  const p = getPosition(forwardKinematics(joints, res.values).endEffector);
  assert.ok(near(p[0], -0.4, 1e-3));
  assert.ok(near(p[1], -0.35, 1e-3));
});

test('IK recovers from a fully wrong-side base rotation', () => {
  const joints = reach2R();
  joints[0].value = Math.PI; // facing opposite the target
  const res = solveIK(joints, [0.4, 0.3, 0]);
  assert.ok(res.converged, `failed to converge, err=${res.error}`);
  const p = getPosition(forwardKinematics(joints, res.values).endEffector);
  assert.ok(near(p[0], 0.4, 1e-3));
  assert.ok(near(p[1], 0.3, 1e-3));
});

test('IK restarts escape a bad local minimum (6R)', () => {
  const joints = [
    makeJoint('revolute', { a: 0, d: 0.25, alpha: -Math.PI / 2, value: 2.8 }),
    makeJoint('revolute', { a: 0.22, value: -2.0 }),
    makeJoint('revolute', { a: 0, alpha: Math.PI / 2, value: 1.5 }),
    makeJoint('revolute', { a: 0, d: 0.22, alpha: -Math.PI / 2 }),
    makeJoint('revolute', { a: 0, alpha: Math.PI / 2 }),
    makeJoint('revolute', { a: 0, d: 0.1 }),
  ];
  const res = solveIK(joints, [-0.3, -0.2, 0.35], { maxIterations: 300 });
  assert.ok(res.error < 0.01, `residual ${res.error}`);
});

test('IK readjusts a base joint parked on the wrong side', () => {
  // 3R articulated: vertical base + two planar links
  const joints = [
    makeJoint('revolute', { a: 0, d: 0.2, alpha: Math.PI / 2, value: 2.8 }), // parked wrong
    makeJoint('revolute', { a: 0.25 }),
    makeJoint('revolute', { a: 0.2 }),
  ];
  const res = solveIK(joints, [-0.3, -0.15, 0.15]);
  assert.ok(res.converged, `failed to converge, err=${res.error}`);
  const p = getPosition(forwardKinematics(joints, res.values).endEffector);
  assert.ok(near(p[0], -0.3, 1e-3));
  assert.ok(near(p[1], -0.15, 1e-3));
  assert.ok(near(p[2], 0.15, 1e-3));
});

test('IK flips elbow branch when target needs the other side', () => {
  const joints = reach2R();
  joints[1].value = 2.5; // elbow bent the "wrong" way for this target
  const res = solveIK(joints, [0.45, 0.1, 0]);
  assert.ok(res.converged, `failed to converge, err=${res.error}`);
  const p = getPosition(forwardKinematics(joints, res.values).endEffector);
  assert.ok(near(p[0], 0.45, 1e-3));
  assert.ok(near(p[1], 0.1, 1e-3));
});

test('solved values always respect restricted joint limits', () => {
  const joints = reach2R();
  joints[0].min = -1.0; joints[0].max = 1.0; // real sub-2π limit
  const res = solveIK(joints, [0.3, 0.4, 0]);
  assert.ok(res.values[0] <= 1.0 + 1e-9 && res.values[0] >= -1.0 - 1e-9);
});

test('Jacobian column matches analytic direction for planar arm', () => {
  const joints = reach2R();
  // At q1=0, moving joint 1 should move EE purely in +y (dpos/dq1 = (0, a1+a2, 0))
  const J = positionJacobian(joints, [0, 0]);
  assert.ok(near(J[0][0], 0, 1e-6));
  assert.ok(near(J[1][0], 0.55, 1e-5));
  assert.ok(near(J[2][0], 0, 1e-6));
});
