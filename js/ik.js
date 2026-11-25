/**
 * Numerical inverse kinematics — damped least-squares (Levenberg–Marquardt)
 * on the position Jacobian. Position-only IK: works for any serial chain.
 */
import { forwardKinematics, getPosition } from './dh.js';

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

/** 3×n position Jacobian via central finite differences. */
export function positionJacobian(joints, values, eps = 1e-6) {
  const n = joints.length;
  const J = [[], [], []];
  const base = endPosition(joints, values);
  for (let i = 0; i < n; i++) {
    const vp = values.slice(), vm = values.slice();
    vp[i] += eps; vm[i] -= eps;
    const pp = endPosition(joints, vp);
    const pm = endPosition(joints, vm);
    for (let r = 0; r < 3; r++) J[r][i] = (pp[r] - pm[r]) / (2 * eps);
  }
  return J;
}

/**
 * Solve position IK. Mutates nothing — returns { values, converged, iterations, error }.
 * Respects each joint's [min, max] limits via clamping.
 */
export function solveIK(joints, target, options = {}) {
  const { maxIterations = 200, tolerance = 1e-4, damping = 0.05 } = options;
  const n = joints.length;
  let values = joints.map((j) => j.value);

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

    values = values.map((v, i) => {
      const j = joints[i];
      return Math.min(j.max, Math.max(j.min, v + step[i] * scale));
    });
  }
  const err = Math.hypot(
    target[0] - endPosition(joints, values)[0],
    target[1] - endPosition(joints, values)[1],
    target[2] - endPosition(joints, values)[2],
  );
  return { values, converged: false, iterations: maxIterations, error: err };
}
