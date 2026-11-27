/**
 * Denavit–Hartenberg kinematics — framework-free math core.
 *
 * Convention: classic DH, A_i = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)
 * Joint variable: θ for revolute joints, d for prismatic joints.
 * Matrices are nested arrays, row-major: m[row][col].
 */

export const JOINT_TYPES = { REVOLUTE: 'revolute', PRISMATIC: 'prismatic' };

export function mat4Identity() {
  return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
}

export function mat4Multiply(A, B) {
  const C = mat4Identity();
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++) {
      C[i][j] = 0;
      for (let k = 0; k < 4; k++) C[i][j] += A[i][k] * B[k][j];
    }
  return C;
}

/** Standard DH transformation matrix. */
export function dhTransform(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  // `0 - x` normalizes away -0 so identity cases compare equal
  return [
    [ct, 0 - st * ca, st * sa, a * ct],
    [st, ct * ca, 0 - ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}

/** Effective DH params for a joint (base value + joint variable). */
export function jointParams(joint, value = null) {
  const q = value === null ? joint.value : value;
  return {
    theta: joint.theta + (joint.type === JOINT_TYPES.REVOLUTE ? q : 0),
    d: joint.d + (joint.type === JOINT_TYPES.PRISMATIC ? q : 0),
    a: joint.a,
    alpha: joint.alpha,
  };
}

/**
 * Forward kinematics for a serial chain.
 * Returns { transforms, endEffector } — transforms[0] is the base frame,
 * transforms[i] is the frame after joint i (1-indexed), last is the tool frame.
 */
export function forwardKinematics(joints, values = null, base = null) {
  let T = base || mat4Identity();
  const transforms = [T];
  joints.forEach((j, i) => {
    const q = values ? values[i] : null;
    const p = jointParams(j, q);
    T = mat4Multiply(T, dhTransform(p.theta, p.d, p.a, p.alpha));
    transforms.push(T);
  });
  return { transforms, endEffector: T };
}

export function getPosition(m) {
  return [m[0][3], m[1][3], m[2][3]];
}

/** Extract roll-pitch-yaw (XYZ order) from a rotation matrix. */
export function getRPY(m) {
  const r20 = m[2][0];
  const pitch = Math.asin(Math.max(-1, Math.min(1, -r20)));
  const roll = Math.atan2(m[2][1], m[2][2]);
  const yaw = Math.atan2(m[1][0], m[0][0]);
  return [roll, pitch, yaw];
}

/** Rotation matrix from roll-pitch-yaw (XYZ order) — inverse of getRPY. */
export function matFromRPY(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  // R = Rz(yaw) · Ry(pitch) · Rx(roll)
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

/**
 * Mobility of the chain (Grübler–Kutzbach reduced to a serial arm of
 * 1-DOF joints — every non-fixed joint contributes 1).
 */
export function degreesOfFreedom(joints) {
  return joints.filter((j) => j.type !== 'fixed').length;
}

/** Rows for the on-screen DH table; `variable` marks the joint variable. */
export function dhTable(joints) {
  return joints.map((j, i) => {
    const p = jointParams(j);
    return {
      joint: i + 1,
      name: j.name || `J${i + 1}`,
      type: j.type,
      theta: p.theta,
      d: p.d,
      a: p.a,
      alpha: p.alpha,
      variable: j.type === JOINT_TYPES.REVOLUTE ? 'θ' : j.type === JOINT_TYPES.PRISMATIC ? 'd' : '–',
    };
  });
}

let jointCounter = 0;
export function makeJoint(type = JOINT_TYPES.REVOLUTE, params = {}) {
  return {
    id: ++jointCounter,
    name: params.name || `J${jointCounter}`,
    type,
    theta: params.theta ?? 0,
    d: params.d ?? 0,
    a: params.a ?? 0.15,
    alpha: params.alpha ?? 0,
    value: params.value ?? 0,
    min: params.min ?? (type === JOINT_TYPES.PRISMATIC ? 0 : -Math.PI),
    max: params.max ?? (type === JOINT_TYPES.PRISMATIC ? 0.2 : Math.PI),
  };
}

/** Ready-made arms shown in the preset picker. */
export const PRESETS = {
  '2R Planar Arm': () => [
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'Shoulder', a: 0.3 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'Elbow', a: 0.25 }),
  ],
  '3R Articulated': () => [
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'Waist', a: 0, d: 0.2, alpha: Math.PI / 2 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'Shoulder', a: 0.25 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'Elbow', a: 0.2 }),
  ],
  'SCARA (RRP)': () => [
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'Base', a: 0.25, d: 0.25 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'Elbow', a: 0.2 }),
    makeJoint(JOINT_TYPES.PRISMATIC, { name: 'Z-axis', a: 0, d: 0.05, min: 0, max: 0.15 }),
  ],
  'PUMA-style 6R': () => [
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'J1 Waist', a: 0, d: 0.25, alpha: -Math.PI / 2 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'J2 Shoulder', a: 0.22 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'J3 Elbow', a: 0, alpha: Math.PI / 2 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'J4 Wrist1', a: 0, d: 0.22, alpha: -Math.PI / 2 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'J5 Wrist2', a: 0, alpha: Math.PI / 2 }),
    makeJoint(JOINT_TYPES.REVOLUTE, { name: 'J6 Flange', a: 0, d: 0.1 }),
  ],
};
