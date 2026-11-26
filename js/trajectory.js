/**
 * Joint-space trajectory playback — smoothstep blending through waypoints.
 * Pure math, no three.js — same wrap-aware shortest path as the IK follower.
 */
import { JOINT_TYPES } from './dh.js';

const TWO_PI = Math.PI * 2;

/** Shortest signed delta from a to b, wrap-aware for unbounded revolute joints. */
export function wrapDelta(a, b, j) {
  const span = j.max - j.min;
  let d = b - a;
  if (j.type === JOINT_TYPES.REVOLUTE && span >= TWO_PI - 1e-9) {
    d = ((d + Math.PI) % TWO_PI + TWO_PI) % TWO_PI - Math.PI;
  }
  return d;
}

const smoothstep = (t) => t * t * (3 - 2 * t);

/**
 * Given waypoints = [values0, values1, ...] and per-segment duration `segSec`,
 * return the joint values at absolute trajectory time `t` seconds.
 * Each segment eases from its start pose to the next (zero velocity at waypoints).
 */
export function trajectoryAt(waypoints, joints, t, segSec = 1.5, loop = true) {
  const n = waypoints.length;
  if (n === 0) return null;
  if (n === 1) return waypoints[0].slice();
  const total = segSec * (loop ? n : n - 1);
  let tt = t;
  if (loop) tt = ((t % total) + total) % total;
  else tt = Math.min(t, total - 1e-9);
  const seg = Math.min(n - 1, Math.floor(tt / segSec));
  const u = smoothstep((tt - seg * segSec) / segSec);
  const a = waypoints[seg];
  const b = waypoints[(seg + 1) % n];
  return joints.map((j, i) => a[i] + wrapDelta(a[i], b[i], j) * u);
}
