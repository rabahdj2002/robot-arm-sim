/**
 * Control panel — joint editor, sliders, FK readouts, DH table, IK target.
 */
import {
  forwardKinematics, getPosition, getRPY, degreesOfFreedom, dhTable,
  makeJoint, JOINT_TYPES, PRESETS,
} from './dh.js';
import { solveIK } from './ik.js';
import { checkCollisions, describeCollision } from './collision.js';
import { trajectoryAt, wrapDelta } from './trajectory.js';
import { shareUrl } from './config.js';

const fmt = (x) => (x >= 0 ? '+' : '') + x.toFixed(4);
const fmtDeg = (rad) => fmt((rad * 180) / Math.PI) + '°';

export class Panel {
  /**
   * @param {object} hooks
   *   hooks.onChange()         — joints or values changed -> re-render arm + readouts
   *   hooks.onRebuild()        — joint list structure changed -> rebuild arm
   *   hooks.onTarget(pos|null) — update the IK target marker
   */
  constructor(root, joints, hooks) {
    this.root = root;
    this.joints = joints;
    this.hooks = hooks;
    this.target = null;
    this.collisionEnabled = true;
    this.lastCollision = null;
    this._animGoal = null;   // IK solution being eased toward
    this._animRaf = null;
    this._els = [];          // {slider, num} per joint for in-place sync
    this.waypoints = [];     // recorded joint poses for trajectory playback
    this._trajRaf = null;
    this._trajOn = false;
    this.render();
  }

  render() {
    this.root.innerHTML = `
      <div class="sec">
        <h3>Robot Config</h3>
        <select id="preset" class="full">
          <option value="">— Load a preset —</option>
          ${Object.keys(PRESETS).map((k) => `<option>${k}</option>`).join('')}
        </select>
        <div class="row">
          <button id="add-rev">+ Revolute</button>
          <button id="add-pris">+ Prismatic</button>
        </div>
        <div class="row">
          <button id="share-cfg">Copy share link</button>
        </div>
      </div>
      <div id="joints"></div>
      <div class="sec">
        <h3>Inverse Kinematics <span class="hint">(DH frame, z = up)</span></h3>
        <div class="q-row">
          <span class="q-label">X (m)</span>
          <input class="tslider" data-ax="0" type="range" min="-0.6" max="0.6" step="0.005" value="0.25">
          <input class="tval" data-ax="0" type="number" step="0.01" value="0.25">
        </div>
        <div class="q-row">
          <span class="q-label">Y (m)</span>
          <input class="tslider" data-ax="1" type="range" min="-0.6" max="0.6" step="0.005" value="0.2">
          <input class="tval" data-ax="1" type="number" step="0.01" value="0.2">
        </div>
        <div class="q-row">
          <span class="q-label">Z (m)</span>
          <input class="tslider" data-ax="2" type="range" min="-0.1" max="0.8" step="0.005" value="0.15">
          <input class="tval" data-ax="2" type="number" step="0.01" value="0.15">
        </div>
        <label class="toggle"><input id="ik-live" type="checkbox" checked> arm follows target</label>
        <label class="toggle"><input id="col-check" type="checkbox" checked> collision limits (floor + self)</label>
        <div class="row">
          <button id="ik-solve" class="primary">Solve IK</button>
          <button id="ik-clear">Hide target</button>
        </div>
        <div id="ik-status" class="status"></div>
      </div>
      <div class="sec">
        <h3>Trajectory <span class="hint" id="traj-count"></span></h3>
        <div class="row">
          <button id="traj-add">+ Waypoint</button>
          <button id="traj-play" class="primary">Play</button>
          <button id="traj-stop">Stop</button>
        </div>
        <div class="row">
          <button id="traj-clear">Clear</button>
          <label class="toggle" style="margin-top:0;flex:1;justify-content:flex-end"><input id="traj-loop" type="checkbox" checked> loop</label>
        </div>
        <label class="toggle"><input id="trail-check" type="checkbox"> EE trail</label>
        <label class="toggle"><input id="ws-check" type="checkbox"> workspace dome</label>
      </div>
      <div class="sec">
        <h3>Forward Kinematics</h3>
        <div class="dof">DOF: <b id="dof"></b></div>
        <div id="col-status" class="status"></div>
        <table class="kv">
          <tr><td>Position (m)</td><td id="ee-pos"></td></tr>
          <tr><td>Orientation (RPY)</td><td id="ee-rpy"></td></tr>
        </table>
      </div>
      <div class="sec">
        <h3>DH Table</h3>
        <table class="dh" id="dh-table"></table>
      </div>`;

    this.root.querySelector('#preset').addEventListener('change', (e) => {
      const name = e.target.value;
      if (name && PRESETS[name]) this.hooks.onPreset(PRESETS[name]());
      e.target.value = '';
    });
    this.root.querySelector('#add-rev').addEventListener('click', () =>
      this.hooks.onAddJoint(makeJoint(JOINT_TYPES.REVOLUTE)));
    this.root.querySelector('#add-pris').addEventListener('click', () =>
      this.hooks.onAddJoint(makeJoint(JOINT_TYPES.PRISMATIC)));

    const shareBtn = this.root.querySelector('#share-cfg');
    shareBtn.addEventListener('click', () => {
      navigator.clipboard?.writeText(shareUrl(this.joints));
      shareBtn.textContent = 'Link copied';
      setTimeout(() => { shareBtn.textContent = 'Copy share link'; }, 1200);
    });

    // trajectory record / playback
    this._trajUpdateCount = () => {
      this.root.querySelector('#traj-count').textContent =
        this.waypoints.length ? `${this.waypoints.length} wp` : '';
    };
    this.root.querySelector('#traj-add').addEventListener('click', () => {
      this.waypoints.push(this.joints.map((j) => j.value));
      this._trajUpdateCount();
    });
    this.root.querySelector('#traj-play').addEventListener('click', () => this.playTrajectory());
    this.root.querySelector('#traj-stop').addEventListener('click', () => this.stopTrajectory());
    this.root.querySelector('#traj-clear').addEventListener('click', () => {
      this.stopTrajectory();
      this.waypoints = [];
      this._trajUpdateCount();
    });
    this.root.querySelector('#trail-check').addEventListener('change', (e) =>
      this.hooks.onTrail(e.target.checked));
    this.root.querySelector('#ws-check').addEventListener('change', (e) =>
      this.hooks.onWorkspace(e.target.checked));

    this.target = [0.25, 0.2, 0.15];
    const doSolve = () => {
      if (!this.target) return;
      this.stopTrajectory(); // IK supersedes waypoint playback
      // Hand the solver the collision predicate — it keeps retrying other
      // seeds/branches until it finds a converged solution that also
      // doesn't collide, instead of rejecting the first branch it finds.
      const res = solveIK(this.joints, this.target, {
        validate: this.collisionEnabled
          ? (v) => !checkCollisions(this.joints, v)
          : null,
      });
      const coll = this.collisionEnabled
        ? checkCollisions(this.joints, res.values)
        : null;
      const st = this.root.querySelector('#ik-status');
      if (coll) {
        st.textContent = res.converged
          ? `all branches collide — ${describeCollision(coll)}`
          : `blocked — ${describeCollision(coll)}`;
        st.className = 'status warn';
        this.renderJoints();
      } else {
        st.textContent = res.converged
          ? `Converged in ${res.iterations} iters (err ${res.error.toExponential(1)})`
          : `Best effort — residual ${res.error.toFixed(3)} m (out of reach)`;
        st.className = 'status ' + (res.converged ? 'ok' : 'warn');
        // Ease toward the solution — the follower does the actual writes
        // per frame so joints glide instead of snapping.
        this._animGoal = res.values;
        this._pumpAnim();
      }
    };
    // throttle live solves to once per animation frame during slider drags
    const liveSolve = () => {
      if (this._ikRaf || this._trajOn || !this.root.querySelector('#ik-live').checked) return;
      this._ikRaf = requestAnimationFrame(() => {
        this._ikRaf = null;
        doSolve();
      });
    };
    this.root.querySelectorAll('.tslider,.tval').forEach((inp) =>
      inp.addEventListener('input', () => {
        const ax = +inp.dataset.ax;
        this.target[ax] = parseFloat(inp.value) || 0;
        this.root.querySelectorAll(`[data-ax="${ax}"]`).forEach((o) => {
          if (o !== inp) o.value = inp.value;
        });
        this.hooks.onTarget(this.target);
        liveSolve();
      }));
    this._liveSolve = liveSolve;
    this.root.querySelector('#ik-solve').addEventListener('click', doSolve);
    this.root.querySelector('#col-check').addEventListener('change', (e) => {
      this.collisionEnabled = e.target.checked;
      this.updateReadouts();
    });
    this.root.querySelector('#ik-clear').addEventListener('click', () => {
      this.hooks.onTarget(null);
      this.root.querySelector('#ik-status').textContent = '';
    });
    this.hooks.onTarget(this.target); // show marker + axes on load

    this.renderJoints();
    this.updateReadouts();
  }

  /**
   * Glide joints toward the last accepted IK goal — exponential approach
   * in joint space (~0.6 s settle) along the shortest path, wrap-aware
   * across ±π for full-rotation revolute joints. Live-drag solves keep
   * re-targeting the same follower so tracking stays continuous.
   */
  _pumpAnim() {
    if (this._animRaf) return;
    let last = performance.now();
    const step = () => {
      this._animRaf = null;
      const goal = this._animGoal;
      if (!goal || goal.length !== this.joints.length) return;
      const now = performance.now();
      const dt = Math.min(0.05, (now - last) / 1000);
      last = now;
      const k = 1 - Math.exp(-7 * dt);
      let maxD = 0;
      this.joints.forEach((j, i) => {
        const span = j.max - j.min;
        const unbounded = j.type === 'revolute' && span >= 2 * Math.PI - 1e-9;
        const d = wrapDelta(j.value, goal[i], j);
        maxD = Math.max(maxD, Math.abs(d));
        j.value += d * k;
        if (unbounded) j.value = j.min + ((((j.value - j.min) % span) + span) % span);
        const el = this._els[i];
        if (el) { el.slider.value = j.value; el.num.value = j.value.toFixed(3); }
      });
      this.updateReadouts();
      if (maxD > 0.002) {
        this._animRaf = requestAnimationFrame(step);
      } else {
        this.joints.forEach((j, i) => { j.value = goal[i]; });
        this._animGoal = null;
        this.updateReadouts();
        this.renderJoints();
      }
    };
    this._animRaf = requestAnimationFrame(step);
  }

  /** Set the IK target from outside the panel (e.g. viewport drag). */
  setTarget(x, y, z) {
    this.target = [
      Math.min(0.6, Math.max(-0.6, x)),
      Math.min(0.6, Math.max(-0.6, y)),
      Math.min(0.8, Math.max(-0.1, z)),
    ];
    this.root.querySelectorAll('[data-ax]').forEach((o) => {
      o.value = +this.target[+o.dataset.ax].toFixed(3);
    });
    this.hooks.onTarget(this.target);
    this._liveSolve?.();
  }

  playTrajectory() {
    if (this._trajOn || !this.waypoints.length) return;
    this._animGoal = null;
    this._trajOn = true;
    const loop = this.root.querySelector('#traj-loop').checked;
    const segSec = 1.6;
    const total = segSec * (loop ? this.waypoints.length : this.waypoints.length - 1);
    const t0 = performance.now();
    const step = () => {
      this._trajRaf = null;
      const t = (performance.now() - t0) / 1000;
      const vals = trajectoryAt(this.waypoints, this.joints, t, segSec, loop);
      if (!vals || (!loop && t > total)) { this.stopTrajectory(); return; }
      this.joints.forEach((j, i) => {
        j.value = vals[i];
        const el = this._els[i];
        if (el) { el.slider.value = j.value; el.num.value = j.value.toFixed(3); }
      });
      this.updateReadouts();
      this._trajRaf = requestAnimationFrame(step);
    };
    this._trajRaf = requestAnimationFrame(step);
  }

  stopTrajectory() {
    this._trajOn = false;
    if (this._trajRaf) cancelAnimationFrame(this._trajRaf);
    this._trajRaf = null;
  }

  renderJoints() {
    const host = this.root.querySelector('#joints');
    host.innerHTML = '';
    this._els = [];
    this.joints.forEach((j, i) => {
      const card = document.createElement('div');
      card.className = `joint-card t-${j.type === 'prismatic' ? 'pris' : 'rev'}`;
      const chips = j.type === 'prismatic' ? '' : `
        <div class="qchips">
          <button data-snap="0">0°</button><button data-snap="1.5708">90°</button>
          <button data-snap="-1.5708">−90°</button><button data-snap="3.1416">180°</button>
        </div>`;
      card.innerHTML = `
        <div class="jhead">
          <input class="jname" value="${j.name}">
          <select class="jtype">
            <option value="revolute" ${j.type === 'revolute' ? 'selected' : ''}>Revolute</option>
            <option value="prismatic" ${j.type === 'prismatic' ? 'selected' : ''}>Prismatic</option>
          </select>
          <button class="del" title="Remove joint">×</button>
        </div>
        <div class="q-row">
          <span class="q-label">${j.type === 'prismatic' ? 'd* (m)' : 'θ* (rad)'}</span>
          <input class="qslider" type="range" min="${j.min}" max="${j.max}" step="0.001" value="${j.value}">
          <input class="qval" type="number" step="0.01" value="${j.value.toFixed(3)}">
        </div>
        ${chips}
        <div class="params">
          <label>θ₀ <input data-p="theta" type="number" step="0.1" value="${j.theta}"></label>
          <label>d <input data-p="d" type="number" step="0.01" value="${j.d}"></label>
          <label>a <input data-p="a" type="number" step="0.01" value="${j.a}"></label>
          <label>α <input data-p="alpha" type="number" step="0.1" value="${j.alpha}"></label>
        </div>`;

      card.querySelector('.jname').addEventListener('input', (e) => { j.name = e.target.value; this.updateReadouts(); });
      card.querySelector('.jtype').addEventListener('change', (e) => {
        j.type = e.target.value;
        j.min = j.type === 'prismatic' ? 0 : -Math.PI;
        j.max = j.type === 'prismatic' ? 0.2 : Math.PI;
        j.value = 0;
        this.renderJoints();
        this.hooks.onRebuild();
      });
      card.querySelector('.del').addEventListener('click', () => this.hooks.onRemoveJoint(i));

      const slider = card.querySelector('.qslider');
      const num = card.querySelector('.qval');
      this._els[i] = { slider, num };
      const setVal = (v) => {
        this._animGoal = null; // user grabbed a slider — cancel IK glide + playback
        this.stopTrajectory();
        const prev = j.value;
        const wanted = Math.min(j.max, Math.max(j.min, v));
        let blocked = false;
        if (this.collisionEnabled && wanted !== prev) {
          j.value = wanted;
          if (checkCollisions(this.joints)) {
            // bisect back toward prev -> the joint moves until it "hits the wall"
            let lo = prev, hi = wanted;
            for (let k = 0; k < 8; k++) {
              const mid = (lo + hi) / 2;
              j.value = mid;
              if (checkCollisions(this.joints)) hi = mid; else lo = mid;
            }
            j.value = lo;
            blocked = true;
          }
        } else {
          j.value = wanted;
        }
        slider.value = j.value; num.value = j.value.toFixed(3);
        this.updateReadouts();
        if (blocked) {
          const el = this.root.querySelector('#col-status');
          el.textContent = 'move blocked — collision';
          el.className = 'status warn';
        }
      };
      slider.addEventListener('input', (e) => setVal(parseFloat(e.target.value)));
      num.addEventListener('change', (e) => setVal(parseFloat(e.target.value) || 0));
      card.querySelectorAll('[data-snap]').forEach((btn) =>
        btn.addEventListener('click', () => setVal(parseFloat(btn.dataset.snap))));

      card.querySelectorAll('.params input').forEach((inp) =>
        inp.addEventListener('change', (e) => {
          j[inp.dataset.p] = parseFloat(e.target.value) || 0;
          this.updateReadouts();
          this.hooks.onChange();
        }));
      host.appendChild(card);
    });
    this.updateReadouts();
  }

  updateReadouts() {
    const { endEffector } = forwardKinematics(this.joints);
    const [x, y, z] = getPosition(endEffector);
    const [r, p, yw] = getRPY(endEffector);
    const dof = degreesOfFreedom(this.joints);
    this.root.querySelector('#dof').textContent = dof;
    this.root.querySelector('#ee-pos').textContent = `${fmt(x)}, ${fmt(y)}, ${fmt(z)}`;
    this.root.querySelector('#ee-rpy').textContent = `${fmtDeg(r)}, ${fmtDeg(p)}, ${fmtDeg(yw)}`;
    // HUD overlay lives outside the panel root
    const hudDof = document.getElementById('hud-dof');
    if (hudDof) hudDof.textContent = dof;
    const hudEe = document.getElementById('hud-ee');
    if (hudEe) hudEe.textContent = `${fmt(x)}, ${fmt(y)}, ${fmt(z)}`;

    const rows = dhTable(this.joints);
    this.root.querySelector('#dh-table').innerHTML =
      '<tr><th>J</th><th>θ</th><th>d</th><th>a</th><th>α</th><th>var</th></tr>' +
      rows.map((t) =>
        `<tr><td>${t.joint}</td><td>${t.theta.toFixed(2)}</td><td>${t.d.toFixed(2)}</td>` +
        `<td>${t.a.toFixed(2)}</td><td>${t.alpha.toFixed(2)}</td><td>${t.variable}</td></tr>`).join('');

    // collision badge + notify renderer (tints links, refreshes pose)
    const coll = this.collisionEnabled ? checkCollisions(this.joints) : null;
    this.lastCollision = coll;
    const el = this.root.querySelector('#col-status');
    if (coll) {
      el.textContent = '⚠ ' + describeCollision(coll);
      el.className = 'status warn';
    } else if (el.textContent.startsWith('⚠')) {
      el.textContent = '';
    }
    this.hooks.onChange(coll);
  }
}
