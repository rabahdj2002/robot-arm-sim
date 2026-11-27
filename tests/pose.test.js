import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  forwardKinematics, getPosition, getRPY, matFromRPY, PRESETS, JOINT_TYPES,
  makeJoint, GRIP_Z,
} from '../js/dh.js';
import { solveIK, geometricJacobian, singularity } from '../js/ik.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) < tol, `${msg}: ${a} vs ${b}`);

test('matFromRPY inverts getRPY', () => {
  for (const [r, p, y] of [[0.3, -0.5, 1.2], [0, 0, 0], [-1.1, 0.8, -2.4]]) {
    const [r2, p2, y2] = getRPY(matFromRPY(r, p, y));
    near(r2, r, 1e-9, 'roll');
    near(p2, p, 1e-9, 'pitch');
    near(y2, y, 1e-9, 'yaw');
  }
});

test('geometricJacobian matches finite differences', () => {
  const joints = PRESETS['PUMA-style 6R']();
  const values = [0.4, -0.6, 0.9, 0.2, -1.0, 0.7];
  const J = geometricJacobian(joints, values);
  const eps = 1e-6;
  const ee = (v) => getPosition(forwardKinematics(joints, v).endEffector);
  for (let i = 0; i < joints.length; i++) {
    const vp = values.slice(); vp[i] += eps;
    const vm = values.slice(); vm[i] -= eps;
    const pp = ee(vp), pm = ee(vm);
    for (let r = 0; r < 3; r++) {
      const fd = (pp[r] - pm[r]) / (2 * eps);
      near(J[r][i], fd, 1e-5, `J[${r}][${i}] analytic vs finite-diff`);
    }
  }
});

test('singularity: full extension is near-singular, bent is not', () => {
  const joints = PRESETS['2R Planar Arm']();
  // elbow fully stretched — EE at the reach boundary, J cols parallel
  const stretched = singularity(joints, [0.3, 0]);
  const bent = singularity(joints, [0.3, 1.4]);
  assert.ok(stretched.condPos < bent.condPos,
    `stretched ${stretched.condPos} not < bent ${bent.condPos}`);
  assert.ok(stretched.condPos < 0.1, 'full extension should read near-singular');
  assert.equal(stretched.dirPos.length, 3, 'stuck direction is a 3-vector');
});

test('singularity: 1-DOF arm keeps its one motion direction', () => {
  const joints = [makeJoint(JOINT_TYPES.REVOLUTE, { a: 0.2 })];
  const s = singularity(joints, [0.5]);
  // achievable subspace has one direction and it is fully conditioned
  near(s.condPos, 1, 1e-6, '1-DOF condPos');
});

test('solveIK with rpy reaches position AND orientation on 6R', () => {
  const joints = PRESETS['PUMA-style 6R']();
  // pick a reachable pose by FK-ing a known config, then solve back to it
  const src = [0.5, -0.7, 0.8, 0.3, -0.6, 0.9];
  const T = forwardKinematics(joints, src).endEffector;
  const pos = getPosition(T);
  const rpy = getRPY(T);
  const res = solveIK(joints, pos, { rpy, restarts: 10, budgetMs: 300 });
  assert.ok(res.converged, `did not converge: err ${res.error}, oriErr ${res.oriError}`);
  const T2 = forwardKinematics(joints, res.values).endEffector;
  let dmax = 0;
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) dmax = Math.max(dmax, Math.abs(T[i][j] - T2[i][j]));
  assert.ok(dmax < 0.02, `rotation matrix off by ${dmax}`);
});

test('solveIK without rpy still converges position-only', () => {
  const joints = PRESETS['2R Planar Arm']();
  const res = solveIK(joints, [0.3, 0.2, 0]);
  assert.ok(res.converged);
  assert.equal(res.oriError, undefined);
});

test('solveIK rpy on underactuated chain does not crash', () => {
  const joints = PRESETS['2R Planar Arm']();
  const res = solveIK(joints, [0.3, 0.2, 0], { rpy: [0, 0, 0], budgetMs: 80 });
  assert.ok(res.values.length === 2); // may not converge — just don't explode
});

test('TCP grip offset: solving flange target puts pads on the marker', () => {
  const joints = PRESETS['PUMA-style 6R']();
  // reachable TCP pose from a known config
  const src = [0.5, -0.7, 0.8, 0.3, -0.6, 0.9];
  const T = forwardKinematics(joints, src).endEffector;
  const R0 = matFromRPY(...getRPY(T));
  const tcpTarget = [0, 1, 2].map((i) => T[i][3] + T[i][2] * GRIP_Z);
  const rpy = getRPY(T);
  // UI does this: solve the flange at tcpTarget - toolZ * GRIP_Z
  const flangeT = tcpTarget.map((v, i) => v - R0[i][2] * GRIP_Z);
  const res = solveIK(joints, flangeT, { rpy, restarts: 10, budgetMs: 300 });
  assert.ok(res.converged, `did not converge: ${res.error}`);
  // TCP of the solution must land back on the marker position
  const T2 = forwardKinematics(joints, res.values).endEffector;
  const tcp = [0, 1, 2].map((i) => T2[i][3] + T2[i][2] * GRIP_Z);
  const dist = Math.hypot(...tcp.map((v, i) => v - tcpTarget[i]));
  assert.ok(dist < 0.005, `TCP missed target by ${dist} m`);
});
