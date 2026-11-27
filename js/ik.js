/**
 * Numerical inverse kinematics — damped least-squares (Levenberg–Marquardt)
 * on the position Jacobian, with restart support. Position-only IK: works
 * for any serial chain.
 */
import { forwardKinematics, getPosition, matFromRPY, JOINT_TYPES } from './dh.js';

const TWO_PI = Math.PI * 2;

/** Solve A·x = b for an n×n system — Gaussian elimination, partial pivoting. */
function solveNxN(A, b) {
  const n = A.length;
  const M = A.map((row, i) => row.concat(b[i]));
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++)
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    [M[col], M[piv]] = [M[piv], M[col]];
    const d = M[col][col] || 1e-12;
    for (let c = col; c <= n; c++) M[col][c] /= d;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col];
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return M.map((row) => row[n]);
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

/**
 * Analytic 6×n geometric Jacobian from one FK pass — rows 0-2 are the
 * linear-velocity part, rows 3-5 the angular part. Revolute joint i:
 * Jp = z_{i-1} × (p_ee − p_{i-1}), Jω = z_{i-1}. Prismatic: Jp = z_{i-1},
 * Jω = 0. ~30× cheaper than finite differences and exact.
 */
export function geometricJacobian(joints, values) {
  const n = joints.length;
  const { transforms } = forwardKinematics(joints, values);
  const ee = getPosition(transforms[n]);
  const J = [new Array(n), new Array(n), new Array(n),
             new Array(n), new Array(n), new Array(n)];
  for (let i = 0; i < n; i++) {
    const T = transforms[i];
    const z = [T[0][2], T[1][2], T[2][2]];
    const p = getPosition(T);
    if (joints[i].type === JOINT_TYPES.PRISMATIC) {
      for (let r = 0; r < 3; r++) { J[r][i] = z[r]; J[3 + r][i] = 0; }
    } else {
      const d = [ee[0] - p[0], ee[1] - p[1], ee[2] - p[2]];
      const c = [z[1] * d[2] - z[2] * d[1],
                 z[2] * d[0] - z[0] * d[2],
                 z[0] * d[1] - z[1] * d[0]];
      for (let r = 0; r < 3; r++) { J[r][i] = c[r]; J[3 + r][i] = z[r]; }
    }
  }
  return J;
}

/** 3×n position Jacobian (top rows of the geometric Jacobian). */
export function positionJacobian(joints, values) {
  return geometricJacobian(joints, values).slice(0, 3);
}

/**
 * Singular values of a symmetric m×m matrix by Jacobi sweeps.
 * Returns {values: [...desc], vectors: [columns]} — good enough for
 * 3×3 / 6×6 singularity analysis.
 */
function symEig(A) {
  const m = A.length;
  const a = A.map((r) => r.slice());
  let V = A.map((r, i) => r.map((_, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 24; sweep++) {
    let off = 0;
    for (let p = 0; p < m; p++)
      for (let q = p + 1; q < m; q++) off += a[p][q] * a[p][q];
    if (off < 1e-20) break;
    for (let p = 0; p < m; p++) {
      for (let q = p + 1; q < m; q++) {
        if (Math.abs(a[p][q]) < 1e-14) continue;
        const t = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        const s = Math.sign(t || 1) / (Math.abs(t) + Math.sqrt(1 + t * t));
        const c = 1 / Math.sqrt(1 + s * s);
        const sn = s * c;
        for (let k = 0; k < m; k++) {
          const akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - sn * akq;
          a[k][q] = sn * akp + c * akq;
        }
        for (let k = 0; k < m; k++) {
          const apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - sn * aqk;
          a[q][k] = sn * apk + c * aqk;
        }
        for (let k = 0; k < m; k++) {
          const vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - sn * vkq;
          V[k][q] = sn * vkp + c * vkq;
        }
      }
    }
  }
  const order = a.map((_, i) => i).sort((x, y) => a[y][y] - a[x][x]);
  return {
    values: order.map((i) => a[i][i]),
    vectors: order.map((i) => V.map((row) => row[i])),
  };
}

const _jSig = (J) => {
  // singular values of J = sqrt of eigenvalues of J·Jᵀ (small square side)
  const m = J.length;
  const n = J[0].length;
  const JJt = [];
  for (let r = 0; r < m; r++) {
    JJt.push(new Array(m).fill(0));
    for (let c = 0; c < m; c++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += J[r][k] * J[c][k];
      JJt[r][c] = s;
    }
  }
  const { values, vectors } = symEig(JJt);
  const sig = values.map((v) => Math.sqrt(Math.max(0, v)));
  // Mobility margin: σ_min/σ_max over the achievable subspace — the top
  // min(m,n) singular values only. The m−n structurally-zero sigmas of an
  // underactuated chain (n<m) are dropped, but a genuinely degenerate
  // direction (rank < min(m,n)) still shows as a real ~0.
  const k = Math.min(m, n) - 1;
  return { sigMin: sig[k], sigMax: sig[0], dirMin: vectors[k] };
};

/**
 * Singularity analysis at a configuration: σmin/σmax (condition) for the
 * position Jacobian, and for the full 6×n pose Jacobian when n ≥ 6.
 * dirPos/dirPose = the near-stuck EE directions (left singular vectors).
 */
export function singularity(joints, values) {
  const J = geometricJacobian(joints, values);
  const pos = _jSig(J.slice(0, 3));
  const out = { condPos: pos.sigMax > 0 ? pos.sigMin / pos.sigMax : 0, dirPos: pos.dirMin };
  if (joints.length >= 6) {
    const pose = _jSig(J);
    out.condPose = pose.sigMax > 0 ? pose.sigMin / pose.sigMax : 0;
    out.dirPose = pose.dirMin;
  }
  return out;
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
    const w = solveNxN(JJt, err);

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

/**
 * Orientation error (axis-angle, rad) rotating current EE frame Rc toward
 * target Rt — the CLIK half-cross-product residual, exact to first order.
 */
function oriError(Rc, Rt) {
  return [
    0.5 * (Rc[1][0] * Rt[2][0] - Rc[2][0] * Rt[1][0] +
           Rc[1][1] * Rt[2][1] - Rc[2][1] * Rt[1][1] +
           Rc[1][2] * Rt[2][2] - Rc[2][2] * Rt[1][2]),
    0.5 * (Rc[2][0] * Rt[0][0] - Rc[0][0] * Rt[2][0] +
           Rc[2][1] * Rt[0][1] - Rc[0][1] * Rt[2][1] +
           Rc[2][2] * Rt[0][2] - Rc[0][2] * Rt[2][2]),
    0.5 * (Rc[0][0] * Rt[1][0] - Rc[1][0] * Rt[0][0] +
           Rc[0][1] * Rt[1][1] - Rc[1][1] * Rt[0][1] +
           Rc[0][2] * Rt[1][2] - Rc[1][2] * Rt[0][2]),
  ];
}

/**
 * DLS descent on the full 6D pose error — position (m) + orientation
 * (axis-angle rad, scaled by oriWeight so units are comparable).
 */
function iteratePose(joints, target, Rt, startValues, { maxIterations, tolerance, oriTolerance, damping, oriWeight }) {
  const n = joints.length;
  let values = startValues.slice();

  for (let iter = 0; iter < maxIterations; iter++) {
    const T = forwardKinematics(joints, values).endEffector;
    const pos = getPosition(T);
    const eP = [target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]];
    const eO = oriError(T, Rt);
    const errNorm = Math.hypot(...eP);
    const oriNorm = Math.hypot(...eO);
    if (errNorm < tolerance && oriNorm < oriTolerance) {
      return { values, converged: true, iterations: iter, error: errNorm, oriError: oriNorm };
    }

    const J = geometricJacobian(joints, values);
    // 6D residual: position + orientation scaled into comparable units
    const e = eP.concat(eO.map((x) => x * oriWeight));
    const Jr = J.map((row, r) => (r < 3 ? row : row.map((x) => x * oriWeight)));

    const JJt = Jr.map((row, r) => row.map((_, c) => {
      let s = r === c ? damping * damping : 0;
      for (let k = 0; k < n; k++) s += Jr[r][k] * Jr[c][k];
      return s;
    }));
    const w = solveNxN(JJt, e);

    const step = [];
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let r = 0; r < 6; r++) s += Jr[r][i] * w[r];
      step.push(s);
    }
    const maxStep = Math.max(...step.map(Math.abs), 1e-9);
    const scale = Math.min(1, 0.4 / maxStep);
    values = values.map((v, i) => wrapOrClamp(v + step[i] * scale, joints[i]));
  }
  const T = forwardKinematics(joints, values).endEffector;
  const pos = getPosition(T);
  return {
    values, converged: false, iterations: maxIterations,
    error: Math.hypot(target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]),
    oriError: Math.hypot(...oriError(T, Rt)),
  };
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
 *
 * options.validate(values) — optional predicate (e.g. collision check).
 * The first converged AND valid solution is returned immediately; a
 * converged-but-invalid solution is only a fallback for when every
 * converged branch fails validation.
 */
export function solveIK(joints, target, options = {}) {
  const {
    maxIterations = 200,
    tolerance = 1e-4,
    oriTolerance = 0.02,   // rad — ~1.1°
    oriWeight = 0.3,       // scales rad residual into metres-comparable units
    damping = 0.05,
    restarts = 6,
    budgetMs = 60,
    validate = null,
    rpy = null,            // [roll,pitch,yaw] target orientation, or null
    seed = null,           // initial joint vector; defaults to current pose
  } = options;
  const opts = { maxIterations, tolerance, oriTolerance, damping, oriWeight };
  const Rt = rpy ? matFromRPY(rpy[0], rpy[1], rpy[2]) : null;
  const cur = joints.map((j) => j.value);

  // Seed the base revolute joint directly at the target azimuth (and its
  // opposite) — the cheap way out of the classic "base parked on the
  // wrong side" trap instead of hoping random restarts find the flip.
  const az = Math.atan2(target[1], target[0]);
  const reseedBase = (q) =>
    joints.map((j, i) =>
      i === 0 && j.type === JOINT_TYPES.REVOLUTE ? wrapOrClamp(q, j) : j.value);
  const seeds = [
    seed ?? cur,
    reseedBase(az),
    reseedBase(az + Math.PI),
    reseedBase(cur[0] + Math.PI),
    joints.map((j) => wrapOrClamp(-j.value, j)),
  ];
  for (let r = 0; r < restarts; r++) seeds.push(seedValues(joints, r));

  const deadline = Date.now() + budgetMs;
  let best = null;          // best error, any validity
  let bestInvalid = null;   // converged but failed validate()
  // With an orientation target, "converged" must mean the full pose —
  // CCD only solves position, so it becomes a pre-seeding step feeding
  // iteratePose instead of a valid terminal solver.
  const solvers = Rt
    ? [(jj, tt, s) => iteratePose(jj, tt, Rt, ccdIterate(jj, tt, s, opts).values, opts)]
    : [ccdIterate, (jj, tt, s) => iterate(jj, tt, s, opts)];
  for (const seed of seeds) {
    for (const solve of solvers) {
      const res = solve(joints, target, seed.slice(), opts);
      if (res.converged) {
        if (!validate || validate(res.values)) return res;
        if (!bestInvalid) bestInvalid = res;
      }
      if (!best || res.error < best.error) best = res;
    }
    if (Date.now() > deadline) break;
  }
  // Every converged branch was invalid — surface one so the caller can
  // explain *why* it's blocked instead of pretending out-of-reach.
  return bestInvalid || best;
}
