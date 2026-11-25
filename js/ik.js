/**
 * Numerical inverse kinematics — damped least-squares (Levenberg–Marquardt)
 * on the position Jacobian, with restart support. Position-only IK: works
 * for any serial chain.
 */
import { forwardKinematics, getPosition, JOINT_TYPES } from './dh.js';

const TWO_PI = Math.PI * 2;

function solve3x3(A, b) {
  // Gaussian elimination with partial pivoting
  const M = [A[0].concat(b[0]), A[1].concat(b[1]), A[2].concat(b[2])];
  for (let col = 0; col < 3; col++) {
    let piv = col;
    for (let r = col + 1; r < 3; r++)
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    [M[col], M[piv]] = [M[piv], M[col]];
    const d = M[col][col] || 1e-12;
    for (let c = col; c < 4; c++) M[col][c] /= d;
    for (let r = 0; r < 3; r++) {
      if (r === col) continue;
      const f = M[r][col];
      for (let c = col; c < 4; c++) M[r][c] -= f * M[col][c];
    }
  }
  return [M[0][3], M[1][3], M[2][3]];
}

function endPosition(joints, values) {
  return getPosition(forwardKinematics(joints, values).endEffector);
}

/**
 * Revolute joints whose range spans a full rotation are unbounded — wrap
 * into [min, max) so the solver can pass "through" ±π instead of stalling
 * on the clamp. Joints with a real sub-2π range are clamped as physical
 * limits (same for prismatic joints, which can never wrap).
 */
function wrapOrClamp(v, j) {
  const span = j.max - j.min;
  if (j.type === JOINT_TYPES.REVOLUTE && span >= TWO_PI - 1e-9) {
    return j.min + ((((v - j.min) % span) + span) % span);
  }
  return Math.min(j.max, Math.max(j.min, v));
}

/** 3×n position Jacobian via central finite differences. */
export function positionJacobian(joints, values, eps = 1e-6) {
  const n = joints.length;
  const J = [[], [], []];
  for (let i = 0; i < n; i++) {
    const vp = values.slice(), vm = values.slice();
    vp[i] += eps; vm[i] -= eps;
    const pp = endPosition(joints, vp);
    const pm = endPosition(joints, vm);
    for (let r = 0; r < 3; r++) J[r][i] = (pp[r] - pm[r]) / (2 * eps);
  }
  return J;
}

/** Single DLS descent from a starting configuration. */
function iterate(joints, target, startValues, { maxIterations, tolerance, damping }) {
  const n = joints.length;
  let values = startValues.slice();

  for (let iter = 0; iter < maxIterations; iter++) {
    const pos = endPosition(joints, values);
    const err = [target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]];
    const errNorm = Math.hypot(...err);
    if (errNorm < tolerance) {
      return { values, converged: true, iterations: iter, error: errNorm };
    }

    const J = positionJacobian(joints, values);

    // DLS: dq = J^T (J J^T + λ²I)^-1 err — the inner system is only 3×3
    const JJt = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let r = 0; r < 3; r++)
      for (let c = 0; c < 3; c++) {
        for (let k = 0; k < n; k++) JJt[r][c] += J[r][k] * J[c][k];
        if (r === c) JJt[r][c] += damping * damping;
      }
    const w = solve3x3(JJt, err);

    // Step size limiting keeps big error steps stable
    const step = [];
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let r = 0; r < 3; r++) s += J[r][i] * w[r];
      step.push(s);
    }
    const maxStep = Math.max(...step.map(Math.abs), 1e-9);
    const scale = Math.min(1, 0.5 / maxStep); // cap joint velocity per iteration

    values = values.map((v, i) => wrapOrClamp(v + step[i] * scale, joints[i]));
  }
  const pos = endPosition(joints, values);
  const err = Math.hypot(target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]);
  return { values, converged: false, iterations: maxIterations, error: err };
}

const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const vcross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const vnorm = (a) => Math.hypot(a[0], a[1], a[2]);
const vscale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const vclamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/**
 * Cyclic Coordinate Descent — geometric IK. Each joint (tip to base) is
 * rotated about its axis to align the joint->EE and joint->target vectors.
 * Escapes wrong-branch configurations that pure gradient descent can't.
 */
function ccdIterate(joints, target, startValues, { maxIterations, tolerance }) {
  const n = joints.length;
  let values = startValues.slice();

  for (let iter = 0; iter < maxIterations; iter++) {
    const ee0 = endPosition(joints, values);
    const errNorm = vnorm(vsub(target, ee0));
    if (errNorm < tolerance) {
      return { values, converged: true, iterations: iter, error: errNorm };
    }
    for (let i = n - 1; i >= 0; i--) {
      const { transforms } = forwardKinematics(joints, values);
      const jp = getPosition(transforms[i]);
      const z = [transforms[i][0][2], transforms[i][1][2], transforms[i][2][2]];
      const ee = getPosition(transforms[n]);
      const vE = vsub(ee, jp), vT = vsub(target, jp);
      if (joints[i].type === JOINT_TYPES.PRISMATIC) {
        values[i] = wrapOrClamp(values[i] + (vdot(vT, z) - vdot(vE, z)), joints[i]);
      } else {
        // rotate about the joint axis so vE aligns with vT
        const pE = vsub(vE, vscale(z, vdot(vE, z)));
        const pT = vsub(vT, vscale(z, vdot(vT, z)));
        const nE = vnorm(pE), nT = vnorm(pT);
        if (nE < 1e-9 || nT < 1e-9) continue;
        const cosA = vclamp(vdot(pE, pT) / (nE * nT), -1, 1);
        const s = vdot(z, vcross(pE, pT)) >= 0 ? 1 : -1;
        values[i] = wrapOrClamp(values[i] + s * Math.acos(cosA) * 0.8, joints[i]);
      }
    }
  }
  const err = vnorm(vsub(target, endPosition(joints, values)));
  return { values, converged: false, iterations: maxIterations, error: err };
}

/** Deterministic pseudo-random seed spread across each joint's range. */
function seedValues(joints, r) {
  return joints.map((j, i) => {
    const h = (Math.sin(r * 12.9898 + i * 78.233) * 43758.5453) % 1;
    return j.min + (h < 0 ? h + 1 : h) * (j.max - j.min);
  });
}

/**
 * Solve position IK. Returns { values, converged, iterations, error }.
 *
 * Strategy: descend from the current configuration first (smooth tracking
 * for live solving). On failure, restart from structured seeds — flipping
 * the base joint 180° (the classic wrong-side trap), then deterministic
 * pseudo-random configurations — and keep the best result.
 */
export function solveIK(joints, target, options = {}) {
  const {
    maxIterations = 200,
    tolerance = 1e-4,
    damping = 0.05,
    restarts = 6,
    budgetMs = 60,
  } = options;
  const opts = { maxIterations, tolerance, damping };
  const cur = joints.map((j) => j.value);

  // Seed the base revolute joint directly at the target azimuth (and its
  // opposite) — the cheap way out of the classic "base parked on the
  // wrong side" trap instead of hoping random restarts find the flip.
  const az = Math.atan2(target[1], target[0]);
  const reseedBase = (q) =>
    joints.map((j, i) =>
      i === 0 && j.type === JOINT_TYPES.REVOLUTE ? wrapOrClamp(q, j) : j.value);
  const seeds = [
    cur,
    reseedBase(az),
    reseedBase(az + Math.PI),
    reseedBase(cur[0] + Math.PI),
    joints.map((j) => wrapOrClamp(-j.value, j)),
  ];
  for (let r = 0; r < restarts; r++) seeds.push(seedValues(joints, r));

  const deadline = Date.now() + budgetMs;
  let best = null;
  for (const seed of seeds) {
    // CCD first — geometric and cheap; DLS refines when it misses
    for (const solve of [ccdIterate, iterate]) {
      const res = solve(joints, target, seed.slice(), opts);
      if (res.converged) return res;
      if (!best || res.error < best.error) best = res;
    }
    if (Date.now() > deadline) break;
  }
  return best;
}
