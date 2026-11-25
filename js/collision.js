/**
 * Collision checking: floor plane (z >= 0) and link self-intersection.
 * Links are the segments between consecutive DH frame origins.
 */
import { forwardKinematics, getPosition } from './dh.js';

export const LINK_RADIUS = 0.022; // matches the rendered link thickness

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/** Minimum distance between two 3D segments (Ericson, RTCD §5.1.9). */
export function segSegDist(p1, q1, p2, q2) {
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  const c = dot(d1, r), b = dot(d1, d2);
  const EPS = 1e-12;

  let s, t;
  if (a < EPS && e < EPS) return Math.hypot(...r); // both degenerate
  if (a < EPS) { s = 0; t = clamp(f / e, 0, 1); }
  else if (e < EPS) { t = 0; s = clamp(-c / a, 0, 1); }
  else {
    const den = a * e - b * b;
    s = den > EPS ? clamp((b * f - c * e) / den, 0, 1) : 0;
    t = (b * s + f) / e;
    if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
    else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
  }
  return Math.hypot(r[0] + d1[0] * s - d2[0] * t,
                    r[1] + d1[1] * s - d2[1] * t,
                    r[2] + d1[2] * s - d2[2] * t);
}

/**
 * Check a configuration for collisions.
 * Returns null when clear, or { type: 'floor', link } /
 * { type: 'self', linkA, linkB } describing the first violation.
 *
 * - Floor: a link's centerline dips below z = -floorTol (centerline check so
 *   links resting at z = 0 aren't permanently "in collision")
 * - Self: distance between non-adjacent link segments < 2×link radius.
 *   Adjacent links (share a joint) are exempt; a degenerate (zero-length)
 *   middle link means the outer pair also shares a joint — exempt too.
 */
export function checkCollisions(joints, values = null, options = {}) {
  const { floorTol = 0.005, selfTol = 2 * LINK_RADIUS } = options;
  const { transforms } = forwardKinematics(joints, values);
  const pts = transforms.map(getPosition);
  const n = joints.length;
  if (n === 0) return null;

  const links = [];
  for (let i = 0; i < n; i++) links.push([pts[i], pts[i + 1]]);

  for (let i = 0; i < n; i++)
    if (Math.min(links[i][0][2], links[i][1][2]) < -floorTol)
      return { type: 'floor', link: i };

  const len = (l) => Math.hypot(l[1][0] - l[0][0], l[1][1] - l[0][1], l[1][2] - l[0][2]);
  for (let i = 0; i < n; i++)
    for (let j = i + 2; j < n; j++) {
      if (len(links[i]) < 1e-6 || len(links[j]) < 1e-6) continue;
      if (j === i + 2 && len(links[i + 1]) < 1e-6) continue;
      if (segSegDist(links[i][0], links[i][1], links[j][0], links[j][1]) < selfTol)
        return { type: 'self', linkA: i, linkB: j };
    }
  return null;
}

/** Human-readable description of a collision result. */
export function describeCollision(c) {
  if (!c) return '';
  return c.type === 'floor'
    ? `link ${c.link + 1} hits the floor`
    : `links ${c.linkA + 1} and ${c.linkB + 1} intersect`;
}
