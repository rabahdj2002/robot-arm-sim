# Robot Arm Kinematics Simulator

Interactive, browser-based serial robot arm simulator built on the **Denavit–Hartenberg (DH) convention**. Define an arbitrary chain of revolute and prismatic joints and get real-time forward kinematics, the generated DH table, mobility count, and numerical inverse kinematics — all in the 3D viewport.

**Live demo:** https://rabahdj2002.github.io/robot-arm-sim/ · **Docs:** https://rabahdj2002.github.io/robot-arm-sim-docs/ · **Tests:** [`tests/`](tests/)

## Features

- **Free-form chain editor** — add/remove revolute and prismatic joints, edit every DH parameter (`θ₀`, `d`, `a`, `α`) per joint
- **Real-time forward kinematics** — joint sliders drive the pose; end-effector position (XYZ) and orientation (RPY) update live
- **Inverse kinematics** — set a target point and solve with damped least-squares + cyclic coordinate descent restarts; joint limits respected, and the solver prefers a collision-free branch when several configurations reach the same target
- **Degrees of freedom** — live mobility count (Grübler–Kutzbach reduced for serial chains)
- **Generated DH table** — the effective parameter table of the current configuration, marked with each joint's variable
- **Collision limits** — floor-plane and link self-intersection checked in real time (segment–segment distance on link centerlines); joint sliders bisect at the boundary for a "hit the wall" feel, IK solutions that would collide are rejected with a description, and offending links tint red — toggleable
- **Frame axes overlay** — toggleable RGB axes at every frame to inspect DH frame assignment
- **Presets** — 2R planar, 3R articulated, SCARA (RRP), PUMA-style 6R
- **No build step** — static site, three.js via CDN importmap

## Math

**Standard DH transform**

```
A_i = Rot_z(θ_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(α_i)
```

The joint variable `q_i` is `θ_i` for revolute and `d_i` for prismatic joints. End-effector pose: `T_0^n = A_1 · A_2 · … · A_n`.

**IK (position only):** iterative damped least squares — `Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e` — where `J` is the 3×n position Jacobian computed by central finite differences, with a cyclic coordinate descent second engine and structured restarts (azimuth seeds, branch flips, pseudo-randoms). A `validate(q)` predicate lets the collision check steer the solver toward a collision-free branch. Converges for any serial chain (best-effort with residual reporting) — closed-form IK does not exist for arbitrary DH configurations. Full detail: [docs site](https://rabahdj2002.github.io/robot-arm-sim-docs/).

## Run locally

Any static file server works (ES modules require http, not `file://`):

```bash
npx serve .          # or
python -m http.server 8000
```

Then open http://localhost:8000 (or the port `serve` reports).

## Tests

The math core (`js/dh.js`, `js/ik.js`) is framework-free and unit-tested with Node's built-in runner:

```bash
node --test tests/
```

## Roadmap

- [ ] Orientation-aware IK (6D task space)
- [ ] Drag-the-target manipulation in the viewport
- [ ] Multiple coordinated arms / closed-loop chains
- [ ] Trajectory playback and joint-space path planning
- [ ] URDF import/export
- [ ] Symbolic FK equation display (LaTeX)

## Tech

Vanilla ES modules + [three.js](https://threejs.org) (CDN). No bundler, no backend — deploys straight to GitHub Pages.
