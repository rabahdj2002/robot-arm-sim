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
  } = options;
  const opts = { maxIterations, tolerance, damping };

  let best = iterate(joints, target, joints.map((j) => j.value), opts);
  if (best.converged) return best;

  // structured restarts: base flip, then sign-flipped configuration
  const seeds = [
    joints.map((j, i) => wrapOrClamp(j.value + (i === 0 ? Math.PI : 0), j)),
    joints.map((j) => wrapOrClamp(-j.value, j)),
  ];
  for (let r = 0; r < restarts; r++) seeds.push(seedValues(joints, r));

  for (const seed of seeds) {
    const res = iterate(joints, target, seed, opts);
    if (res.converged) return res;
    if (res.error < best.error) best = res;
  }
  return best;
}
