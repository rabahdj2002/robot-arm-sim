/**
 * Shareable arm configurations — encode a joint chain into a URL hash.
 * `#config=<base64url>` carries everything needed to rebuild the arm:
 * names, types, DH params, limits and current values. No backend needed.
 */
import { makeJoint, JOINT_TYPES } from './dh.js';

const SHORT = { name: 'n', type: 't', theta: 'h', d: 'd', a: 'a', alpha: 'p', min: 'm', max: 'M', value: 'v' };

function toB64Url(str) {
  const bytes = new TextEncoder().encode(str);
  let bin = '';
  bytes.forEach((b) => (bin += String.fromCharCode(b)));
  return btoa(bin).replaceAll('+', '-').replaceAll('/', '_').replace(/=+$/, '');
}

function fromB64Url(b64) {
  const bin = atob(b64.replaceAll('-', '+').replaceAll('_', '/'));
  const bytes = Uint8Array.from(bin, (c) => c.charCodeAt(0));
  return new TextDecoder().decode(bytes);
}

/** Serialize the current chain into a compact base64url string. */
export function encodeConfig(joints) {
  const slim = joints.map((j) => {
    const o = {};
    for (const [k, s] of Object.entries(SHORT)) {
      const v = j[k];
      o[s] = typeof v === 'number' ? +v.toFixed(5) : v;
    }
    return o;
  });
  return toB64Url(JSON.stringify({ v: 1, j: slim }));
}

/** Decode a config string back into joint objects (makeJoint-shaped). */
export function decodeConfig(b64) {
  const data = JSON.parse(fromB64Url(b64));
  if (data.v !== 1 || !Array.isArray(data.j)) throw new Error('bad config version');
  return data.j.map((s) => {
    const type = s.t === JOINT_TYPES.PRISMATIC ? JOINT_TYPES.PRISMATIC : JOINT_TYPES.REVOLUTE;
    const j = makeJoint(type);
    j.name = String(s.n ?? j.name);
    j.theta = +s.h || 0;
    j.d = +s.d || 0;
    j.a = +s.a || 0;
    j.alpha = +s.p || 0;
    if (typeof s.m === 'number') j.min = s.m;
    if (typeof s.M === 'number') j.max = s.M;
    j.value = Math.min(j.max, Math.max(j.min, +s.v || 0));
    return j;
  });
}

/** Read and decode a config from a URL hash like `#config=abc123`. */
export function configFromHash(hash) {
  const m = /^#?config=(.+)$/.exec(hash || '');
  if (!m) return null;
  try {
    return decodeConfig(m[1]);
  } catch {
    return null;
  }
}

/** Build a shareable URL for the given joints using the page's base URL. */
export function shareUrl(joints, base = location.origin + location.pathname) {
  return `${base}#config=${encodeConfig(joints)}`;
}
