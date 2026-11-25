/**
 * Collision checking: floor plane (z >= 0), link self-intersection, and
 * joint-body contact. Links are the segments between consecutive DH frame
 * origins; joints are treated as spheres at each frame origin.
 */
import { forwardKinematics, getPosition } from './dh.js';

export const LINK_RADIUS = 0.022; // matches the rendered link thickness
const JOINT_RADIUS = 0.05;        // approx. joint mesh size

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/** Closest approach between two 3D segments; returns { d, s, t } (Ericson §5.1.9). */
function segSegClosest(p1, q1, p2, q2) {
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  const c = dot(d1, r), b = dot(d1, d2);
  const EPS = 1e-12;

  let s, t;
  if (a < EPS && e < EPS) return { d: Math.hypot(...r), s: 0, t: 0 };
  if (a < EPS) { s = 0; t = clamp(f / e, 0, 1); }
  else if (e < EPS) { t = 0; s = clamp(-c / a, 0, 1); }
  else {
    const den = a * e - b * b;
    s = den > EPS ? clamp((b * f - c * e) / den, 0, 1) : 0;
    t = (b * s + f) / e;
    if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
    else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
  }
  return {
    d: Math.hypot(r[0] + d1[0] * s - d2[0] * t,
                  r[1] + d1[1] * s - d2[1] * t,
                  r[2] + d1[2] * s - d2[2] * t),
    s, t,
  };
}

export function segSegDist(p1, q1, p2, q2) {
  return segSegClosest(p1, q1, p2, q2).d;
}

/** Distance from point to segment. */
function pointSegDist(p, p1, q1) {
  const d1 = sub(q1, p1);
  const a = dot(d1, d1);
  const t = a < 1e-12 ? 0 : clamp(dot(sub(p, p1), d1) / a, 0, 1);
  return dist(p, [p1[0] + d1[0] * t, p1[1] + d1[1] * t, p1[2] + d1[2] * t]);
}

/**
 * Check a configuration for collisions. Returns null when clear, or:
 *   { type: 'floor', link }             — link centerline below z = -floorTol
 *   { type: 'self', linkA, linkB }      — two link segments intersect
 *   { type: 'joint', link, joint }      — a link hits a joint body
 *   { type: 'joint', jointA, jointB }   — two joint bodies overlap
 *
 * Exemptions that model real robot structure:
 * - adjacent links may touch only near their shared joint (s>0.75 & t<0.25)
 * - coincident frame points are the same physical joint location and are
 *   exempt only while all links between them are degenerate (zero-length)
 */
export function checkCollisions(joints, values = null, options = {}) {
  const {
    floorTol = 0.005,
    selfTol = 2 * LINK_RADIUS,
    jointTol = JOINT_RADIUS + LINK_RADIUS,   // link vs joint body
    jointJointTol = 2 * JOINT_RADIUS,         // joint vs joint body
  } = options;
  const { transforms } = forwardKinematics(joints, values);
  const pts = transforms.map(getPosition);
  const n = joints.length;
  if (n === 0) return null;

  const links = [];
  for (let i = 0; i < n; i++) links.push([pts[i], pts[i + 1]]);
  const len = (l) => dist(l[0], l[1]);
  const degenerate = (i) => len(links[i]) < 1e-6;

  // 1. floor
  for (let i = 0; i < n; i++)
    if (Math.min(links[i][0][2], links[i][1][2]) < -floorTol)
      return { type: 'floor', link: i };

  // 2. adjacent links — contact is only legal near the shared joint
  for (let i = 0; i < n - 1; i++) {
    if (degenerate(i) || degenerate(i + 1)) continue;
    const { d, s, t } = segSegClosest(links[i][0], links[i][1], links[i + 1][0], links[i + 1][1]);
    if (d < selfTol && !(s > 0.75 && t < 0.25))
      return { type: 'self', linkA: i, linkB: i + 1 };
  }

  // 3. joint vs joint (non-adjacent bodies occupying the same space)
  for (let j = 0; j < n; j++)
    for (let k = j + 2; k <= n; k++) {
      let allDeg = true, span = 0;
      for (let m = j; m < k; m++) {
        span += len(links[m]);
        if (!degenerate(m)) allDeg = false;
      }
      // all-degenerate run = same frame; short intermediates = structurally
      // close joints that cannot avoid proximity — both exempt
      if (allDeg || span < jointJointTol) continue;
      if (dist(pts[j], pts[k]) < jointJointTol)
        return { type: 'joint', jointA: j, jointB: k };
    }

  // 4. link vs non-endpoint joint bodies
  for (let i = 0; i < n; i++) {
    if (degenerate(i)) continue;
    for (let j = 0; j <= n; j++) {
      if (j === i || j === i + 1) continue;
      // a joint tethered to link i's end by a shorter-than-body link is in
      // structural proximity — it can never get far enough away to matter
      if (j === i + 2 && len(links[i + 1]) < jointTol) continue;
      if (pointSegDist(pts[j], links[i][0], links[i][1]) >= jointTol) continue;
      // exempt a point coincident with an endpoint inside the structural
      // neighborhood — further out it means the chain folded onto the link
      const nearEnd = Math.min(dist(pts[j], pts[i]), dist(pts[j], pts[i + 1]));
      if (nearEnd < 1e-6 && j <= i + 2) continue;
      return { type: 'joint', link: i, joint: j, jointCount: n };
    }
  }

  // 5. non-adjacent link vs link
  for (let i = 0; i < n; i++)
    for (let j = i + 2; j < n; j++) {
      if (degenerate(i) || degenerate(j)) continue;
      if (j === i + 2 && degenerate(i + 1)) continue; // shared joint
      if (segSegDist(links[i][0], links[i][1], links[j][0], links[j][1]) < selfTol)
        return { type: 'self', linkA: i, linkB: j };
    }
  return null;
}

/** Human-readable description of a collision result. */
export function describeCollision(c) {
  if (!c) return '';
  switch (c.type) {
    case 'floor': return `link ${c.link + 1} hits the floor`;
    case 'self': return `links ${c.linkA + 1} and ${c.linkB + 1} intersect`;
    case 'joint':
      if (c.jointA !== undefined)
        return `joints ${c.jointA + 1} and ${c.jointB + 1} collide`;
      return `link ${c.link + 1} hits ${c.joint === c.jointCount ? 'the tool' : `joint ${c.joint + 1}`}`;
    default: return 'collision';
  }
}
