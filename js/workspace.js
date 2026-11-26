/**
 * Reachable-workspace sampling — Monte-Carlo FK over the joint ranges.
 * Pure math, no three.js — returns positions the renderer plots as a cloud.
 */
import { forwardKinematics, getPosition } from './dh.js';

/**
 * Sample n random configurations within each joint's [min, max] and return
 * the end-effector positions in DH space as a flat Float32Array [x,y,z]*n.
 */
export function sampleWorkspace(joints, n = 12000, rng = Math.random) {
  const out = new Float32Array(n * 3);
  const values = new Array(joints.length);
  for (let s = 0; s < n; s++) {
    for (let i = 0; i < joints.length; i++) {
      values[i] = joints[i].min + rng() * (joints[i].max - joints[i].min);
    }
    const p = getPosition(forwardKinematics(joints, values).endEffector);
    out[s * 3] = p[0];
    out[s * 3 + 1] = p[1];
    out[s * 3 + 2] = p[2];
  }
  return out;
}

/** Radial reach bounds of the workspace (max |xy| and z range) — for tests/UI. */
export function workspaceBounds(positions) {
  let rMax = 0, zMin = Infinity, zMax = -Infinity;
  for (let s = 0; s < positions.length; s += 3) {
    const r = Math.hypot(positions[s], positions[s + 1]);
    if (r > rMax) rMax = r;
    if (positions[s + 2] < zMin) zMin = positions[s + 2];
    if (positions[s + 2] > zMax) zMax = positions[s + 2];
  }
  return { rMax, zMin, zMax };
}
