/**
 * Reachable-workspace sampling — Monte-Carlo FK over the joint ranges.
 * Pure math, no three.js — returns positions the renderer plots as a cloud.
 */
import { forwardKinematics, getPosition, JOINT_TYPES } from './dh.js';

/**
 * Sample n random configurations within each joint's [min, max] and return
 * the end-effector positions in DH space as a flat Float32Array [x,y,z]*n.
 */
export function sampleWorkspace(joints, n = 12000, rng = Math.random, zMin = -Infinity) {
  return sampleWorkspaceConfigs(joints, n, rng, zMin).positions;
}

/**
 * Like sampleWorkspace but also returns the joint values behind each kept
 * sample — workspaceShell uses them as seeds for boundary refinement.
 */
export function sampleWorkspaceConfigs(joints, n = 12000, rng = Math.random, zMin = -Infinity) {
  const pos = [];
  const vals = [];
  const values = new Array(joints.length);
  for (let s = 0; s < n; s++) {
    for (let i = 0; i < joints.length; i++) {
      values[i] = joints[i].min + rng() * (joints[i].max - joints[i].min);
    }
    const p = getPosition(forwardKinematics(joints, values).endEffector);
    if (p[2] < zMin) continue;
    pos.push(p[0], p[1], p[2]);
    vals.push(...values);
  }
  return { positions: new Float32Array(pos), values: new Float32Array(vals) };
}

/**
 * Push a seed configuration outward along one direction ray by coordinate
 * descent: perturb each joint inside its limits and keep moves that grow
 * the EE's distance along the ray (p·dir) while the EE stays within
 * `angTol` of the ray and above `zMin`.
 * Returns { r, values } with r = refined distance along the ray, or -1
 * if no reachable point lands near the ray (unreachable direction).
 */
function refineReach(joints, seed, dir, cosTol, zMin) {
  const J = joints.length;
  const v = Float64Array.from(seed);
  const evalAt = (vv) => {
    const p = getPosition(forwardKinematics(joints, vv).endEffector);
    const r2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
    if (r2 < 1e-12) return { t: 0, ok: false };
    const t = (p[0] * dir[0] + p[1] * dir[1] + p[2] * dir[2]) / Math.sqrt(r2);
    return { t: p[0] * dir[0] + p[1] * dir[1] + p[2] * dir[2], ok: p[2] >= zMin && t >= cosTol };
  };
  let e = evalAt(v);
  let best = e.ok ? e.t : -1;
  for (let step = 0.5; step > 0.012; step *= 0.5) {
    for (let sweep = 0; sweep < 2; sweep++) {
      let improved = false;
      for (let j = 0; j < J; j++) {
        for (const s of [step, -step]) {
          const t = v[j] + s;
          if (t < joints[j].min || t > joints[j].max) continue;
          const prev = v[j];
          v[j] = t;
          e = evalAt(v);
          if (e.ok && e.t > best) { best = e.t; improved = true; }
          else v[j] = prev;
        }
      }
      if (!improved) break;
    }
  }
  return { r: best, values: v };
}

/**
 * Rotate a seed config by `azDelta` about the base axis when joint 0 is
 * revolute — rotating J1 rigidly rotates the whole distal chain, so this
 * transplants a solution proven in one azimuth sector into another.
 * Picks the 2π-equivalent angle inside joint limits; returns the seed
 * unchanged when rotation can't fit (e.g. limited waist → genuinely
 * asymmetric workspace).
 */
function rotateSeed(joints, cfg, azDelta) {
  if (azDelta === 0 || joints[0].type !== JOINT_TYPES.REVOLUTE) return cfg;
  const j = joints[0];
  let a = cfg[0] + azDelta;
  a = Math.atan2(Math.sin(a), Math.cos(a));
  for (const c of [a, a + 2 * Math.PI, a - 2 * Math.PI]) {
    if (c >= j.min - 1e-9 && c <= j.max + 1e-9) {
      const out = cfg.slice();
      out[0] = c;
      return out;
    }
  }
  return cfg;
}

/**
 * Build a dome-like outer shell around sampled workspace points.
 *
 * Bin every sample by direction (azimuth × elevation seen from the base),
 * keep the max reached radius per direction cell, lightly smooth the radius
 * field, then triangulate the direction grid into a surface. Cells with no
 * samples become holes — unreachable directions stay visibly open.
 *
 * zMin clamps the shell to a floor plane (DH z): directions whose rays
 * can never rise above zMin are never filled, and boundary vertices are
 * clamped down onto the floor so the dome visibly sits on it.
 *
 * When `joints` + `values` (seed configs from sampleWorkspaceConfigs)
 * are provided, every cell radius is refined by coordinate descent to
 * the true farthest reachable point along that direction — the dome
 * then hugs the real envelope instead of the luckiest MC sample.
 *
 * Returns { positions: Float32Array (DH space), indices: Uint32Array }.
 */
export function workspaceShell(positions, azBins = 72, elBins = 36, zMin = -Infinity, joints = null, values = null) {
  const rMax = new Float32Array(azBins * elBins).fill(-1);
  const seedIdx = new Int32Array(azBins * elBins).fill(-1);
  const TWO_PI = Math.PI * 2;
  const n = positions.length / 3;

  for (let s = 0; s < n; s++) {
    const x = positions[s * 3], y = positions[s * 3 + 1], z = positions[s * 3 + 2];
    const rxy = Math.hypot(x, y);
    if (rxy < 1e-9) continue;
    const az = Math.atan2(y, x);          // [-π, π]
    const el = Math.atan2(z, rxy);        // [-π/2, π/2]
    const ai = Math.min(azBins - 1, Math.floor(((az + Math.PI) / TWO_PI) * azBins));
    const ei = Math.min(elBins - 1, Math.floor(((el + Math.PI / 2) / Math.PI) * elBins));
    const r = Math.hypot(x, y, z);
    const idx = ai * elBins + ei;
    if (r > rMax[idx]) { rMax[idx] = r; seedIdx[idx] = s; }
  }

  // directions whose rays can never climb above the floor
  const subFloor = new Array(elBins);
  for (let ei = 0; ei < elBins; ei++) {
    const elC = -Math.PI / 2 + ((ei + 0.5) / elBins) * Math.PI;
    subFloor[ei] = zMin >= 0 && Math.sin(elC) <= 0;
  }
  const dirOf = (ai, ei) => {
    const az = -Math.PI + ((ai + 0.5) / azBins) * TWO_PI;
    const el = -Math.PI / 2 + ((ei + 0.5) / elBins) * Math.PI;
    return [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
  };

  let r = rMax;
  const J = joints ? joints.length : 0;
  if (joints && values) {
    // angular tolerance = center→corner of a direction cell, so a sample
    // anywhere inside its own cell counts as "on the ray"
    const cosTol = Math.cos(Math.hypot(Math.PI / azBins, Math.PI / (2 * elBins)));
    const cfg = new Array(azBins * elBins).fill(null);

    // refine every sampled cell to the true reach along its direction
    for (let ai = 0; ai < azBins; ai++) {
      for (let ei = 0; ei < elBins; ei++) {
        const idx = ai * elBins + ei;
        const si = seedIdx[idx];
        if (si < 0) continue;
        const out = refineReach(joints, values.subarray(si * J, si * J + J), dirOf(ai, ei), cosTol, zMin);
        if (out.r >= 0) { r[idx] = Math.max(r[idx], out.r); cfg[idx] = out.values; }
      }
    }

    // diffusion rounds: every cell re-refines from the best available
    // config among itself and its 4-neighbors — good solutions spread
    // across the grid so each direction converges to the true reach,
    // and reachable-but-unsampled cells get a real seed instead of a fill
    for (let round = 0; round < 6; round++) {
      const next = r.slice();
      let changed = false;
      for (let ai = 0; ai < azBins; ai++) {
        for (let ei = 0; ei < elBins; ei++) {
          const idx = ai * elBins + ei;
          if (subFloor[ei]) continue;
          let seed = cfg[idx], br = r[idx], srcAi = ai;
          for (const [ai2, ei2] of [[ai + 1, ei], [ai - 1, ei], [ai, ei - 1], [ai, ei + 1]]) {
            if (ei2 < 0 || ei2 >= elBins) continue;
            const ka = ((ai2 % azBins) + azBins) % azBins;
            const k = ka * elBins + ei2;
            if (cfg[k] && r[k] > br) { seed = cfg[k]; br = r[k]; srcAi = ka; }
          }
          if (!seed) continue;
          // transplant the best neighbor's solution into this azimuth —
          // rotating J1 rigidly rotates the chain, so a proven config
          // reaches the same radius everywhere the waist range allows
          const azDelta = Math.atan2(
            Math.sin((ai - srcAi) * TWO_PI / azBins),
            Math.cos((ai - srcAi) * TWO_PI / azBins));
          const out = refineReach(joints, rotateSeed(joints, seed, azDelta), dirOf(ai, ei), cosTol, zMin);
          if (out.r > next[idx]) { next[idx] = out.r; cfg[idx] = out.values; changed = true; }
        }
      }
      r = next;
      if (!changed) break;
    }
  }

  // last-resort fill for cells refinement couldn't reach: an empty cell
  // with ≥2 occupied neighbors inherits their mean radius — mainly gives
  // thin workspaces (e.g. a planar arm's flat annulus) rows to triangulate
  for (let pass = 0; pass < 2; pass++) {
    const next = r.slice();
    for (let ai = 0; ai < azBins; ai++) {
      for (let ei = 0; ei < elBins; ei++) {
        const idx = ai * elBins + ei;
        if (r[idx] >= 0) continue;
        if (subFloor[ei]) continue;
        const nbr = [
          ((ai + 1) % azBins) * elBins + ei,
          ((ai - 1 + azBins) % azBins) * elBins + ei,
          ei > 0 ? ai * elBins + ei - 1 : -1,
          ei < elBins - 1 ? ai * elBins + ei + 1 : -1,
          ei > 0 ? ((ai + 1) % azBins) * elBins + ei - 1 : -1,
          ei > 0 ? ((ai - 1 + azBins) % azBins) * elBins + ei - 1 : -1,
          ei < elBins - 1 ? ((ai + 1) % azBins) * elBins + ei + 1 : -1,
          ei < elBins - 1 ? ((ai - 1 + azBins) % azBins) * elBins + ei + 1 : -1,
        ];
        let sum = 0, n = 0;
        for (const k of nbr) {
          if (k >= 0 && r[k] >= 0) { sum += r[k]; n++; }
        }
        if (n >= 2) next[idx] = sum / n;
      }
    }
    r = next;
  }

  // vertices: occupied cells become dome vertices on their direction ray
  const verts = [];
  const vid = new Int32Array(azBins * elBins).fill(-1);
  for (let ai = 0; ai < azBins; ai++) {
    for (let ei = 0; ei < elBins; ei++) {
      const idx = ai * elBins + ei;
      if (r[idx] < 0) continue;
      const az = -Math.PI + ((ai + 0.5) / azBins) * TWO_PI;
      const el = -Math.PI / 2 + ((ei + 0.5) / elBins) * Math.PI;
      const ce = Math.cos(el);
      verts.push(r[idx] * ce * Math.cos(az), r[idx] * ce * Math.sin(az),
        Math.max(r[idx] * Math.sin(el), zMin));
      vid[idx] = verts.length / 3 - 1;
    }
  }

  // tangential vertex smoothing: slide each vertex toward its neighbors'
  // centroid along its own radius-sphere, then renormalize back to the
  // refined radius — softens bin-edge stair steps without ever changing
  // the claimed reach in any direction
  for (let pass = 0; pass < 2; pass++) {
    const copy = verts.slice();
    for (let ai = 0; ai < azBins; ai++) {
      for (let ei = 0; ei < elBins; ei++) {
        const idx = ai * elBins + ei;
        const vi = vid[idx];
        if (vi < 0) continue;
        let cx = 0, cy = 0, cz = 0, n = 0;
        for (const k of [
          ((ai + 1) % azBins) * elBins + ei,
          ((ai - 1 + azBins) % azBins) * elBins + ei,
          ei > 0 ? ai * elBins + ei - 1 : -1,
          ei < elBins - 1 ? ai * elBins + ei + 1 : -1,
        ]) {
          const vj = k >= 0 ? vid[k] : -1;
          if (vj < 0) continue;
          cx += copy[vj * 3]; cy += copy[vj * 3 + 1]; cz += copy[vj * 3 + 2];
          n++;
        }
        if (!n) continue;
        const o = vi * 3;
        let x = copy[o] + 0.5 * (cx / n - copy[o]);
        let y = copy[o + 1] + 0.5 * (cy / n - copy[o + 1]);
        let z = copy[o + 2] + 0.5 * (cz / n - copy[o + 2]);
        const len = Math.hypot(x, y, z);
        if (len < 1e-12) continue;
        const rv = Math.hypot(copy[o], copy[o + 1], copy[o + 2]) / len;
        x *= rv; y *= rv; z *= rv;
        verts[o] = x; verts[o + 1] = y; verts[o + 2] = Math.max(z, zMin);
      }
    }
  }

  // triangles: quad between 4 neighbor cells (azimuth wraps)
  const tris = [];
  const at = (ai, ei) => ((ai % azBins) + azBins) % azBins * elBins + Math.max(0, Math.min(elBins - 1, ei));
  const inEl = (ei) => ei >= 0 && ei < elBins;
  for (let ai = 0; ai < azBins; ai++) {
    for (let ei = 0; ei < elBins; ei++) {
      if (!inEl(ei + 1)) continue;
      const v00 = vid[at(ai, ei)], v10 = vid[at(ai + 1, ei)];
      const v01 = vid[at(ai, ei + 1)], v11 = vid[at(ai + 1, ei + 1)];
      if (v00 < 0 || v10 < 0 || v01 < 0 || v11 < 0) continue;
      tris.push(v00, v10, v11, v00, v11, v01);
    }
  }
  return { positions: new Float32Array(verts), indices: new Uint32Array(tris) };
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
