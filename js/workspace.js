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

/**
 * Build a dome-like outer shell around sampled workspace points.
 *
 * Bin every sample by direction (azimuth × elevation seen from the base),
 * keep the max reached radius per direction cell, lightly smooth the radius
 * field, then triangulate the direction grid into a surface. Cells with no
 * samples become holes — unreachable directions stay visibly open.
 *
 * Returns { positions: Float32Array (DH space), indices: Uint32Array }.
 */
export function workspaceShell(positions, azBins = 72, elBins = 36) {
  const rMax = new Float32Array(azBins * elBins).fill(-1);
  const TWO_PI = Math.PI * 2;

  for (let s = 0; s < positions.length; s += 3) {
    const x = positions[s], y = positions[s + 1], z = positions[s + 2];
    const rxy = Math.hypot(x, y);
    if (rxy < 1e-9) continue;
    const az = Math.atan2(y, x);          // [-π, π]
    const el = Math.atan2(z, rxy);        // [-π/2, π/2]
    const ai = Math.min(azBins - 1, Math.floor(((az + Math.PI) / TWO_PI) * azBins));
    const ei = Math.min(elBins - 1, Math.floor(((el + Math.PI / 2) / Math.PI) * elBins));
    const r = Math.hypot(x, y, z);
    const idx = ai * elBins + ei;
    if (r > rMax[idx]) rMax[idx] = r;
  }

  // fill gaps: an empty cell with ≥2 occupied neighbors inherits their
  // mean radius — closes sampling holes and gives thin workspaces
  // (e.g. a planar arm's flat annulus) enough rows to triangulate
  let r = rMax;
  for (let pass = 0; pass < 4; pass++) {
    const next = r.slice();
    for (let ai = 0; ai < azBins; ai++) {
      for (let ei = 0; ei < elBins; ei++) {
        const idx = ai * elBins + ei;
        if (r[idx] >= 0) continue;
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

  // smooth the radius field: 3 passes over the 8-neighborhood
  // (wraps in azimuth; empty cells are skipped, not averaged in)
  for (let pass = 0; pass < 3; pass++) {
    const next = r.slice();
    for (let ai = 0; ai < azBins; ai++) {
      for (let ei = 0; ei < elBins; ei++) {
        const idx = ai * elBins + ei;
        if (r[idx] < 0) continue;
        let sum = r[idx], n = 1;
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
        for (const k of nbr) {
          if (k >= 0 && r[k] >= 0) { sum += r[k]; n++; }
        }
        next[idx] = sum / n;
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
      verts.push(r[idx] * ce * Math.cos(az), r[idx] * ce * Math.sin(az), r[idx] * Math.sin(el));
      vid[idx] = verts.length / 3 - 1;
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
