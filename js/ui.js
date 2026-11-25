/**
 * Control panel — joint editor, sliders, FK readouts, DH table, IK target.
 */
import {
  forwardKinematics, getPosition, getRPY, degreesOfFreedom, dhTable,
  makeJoint, JOINT_TYPES, PRESETS,
} from './dh.js';
import { solveIK } from './ik.js';

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
      </div>
      <div id="joints"></div>
      <div class="sec">
        <h3>Inverse Kinematics</h3>
        <div class="grid3">
          <label>X <input id="ik-x" type="number" step="0.01" value="0.25"></label>
          <label>Y <input id="ik-y" type="number" step="0.01" value="0.20"></label>
          <label>Z <input id="ik-z" type="number" step="0.01" value="0.15"></label>
        </div>
        <div class="row">
          <button id="ik-solve">Solve IK</button>
          <button id="ik-clear">Clear target</button>
        </div>
        <div id="ik-status" class="status"></div>
      </div>
      <div class="sec">
        <h3>Forward Kinematics</h3>
        <div class="dof">DOF: <b id="dof"></b></div>
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

    const readTarget = () => [
      parseFloat(this.root.querySelector('#ik-x').value) || 0,
      parseFloat(this.root.querySelector('#ik-y').value) || 0,
      parseFloat(this.root.querySelector('#ik-z').value) || 0,
    ];
    this.root.querySelector('#ik-solve').addEventListener('click', () => {
      this.target = readTarget();
      this.hooks.onTarget(this.target);
      const res = solveIK(this.joints, this.target);
      this.joints.forEach((j, i) => { j.value = res.values[i]; });
      const st = this.root.querySelector('#ik-status');
      st.textContent = res.converged
        ? `Converged in ${res.iterations} iters (err ${res.error.toExponential(1)})`
        : `Best effort — residual ${res.error.toFixed(3)} m (target may be out of reach)`;
      st.className = 'status ' + (res.converged ? 'ok' : 'warn');
      this.renderJoints();
      this.hooks.onChange();
    });
    this.root.querySelector('#ik-clear').addEventListener('click', () => {
      this.target = null;
      this.hooks.onTarget(null);
      this.root.querySelector('#ik-status').textContent = '';
    });
    ['ik-x', 'ik-y', 'ik-z'].forEach((id) =>
      this.root.querySelector('#' + id).addEventListener('input', () => {
        this.target = readTarget();
        this.hooks.onTarget(this.target);
      }));

    this.renderJoints();
    this.updateReadouts();
  }

  renderJoints() {
    const host = this.root.querySelector('#joints');
    host.innerHTML = '';
    this.joints.forEach((j, i) => {
      const card = document.createElement('div');
      card.className = 'joint-card';
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
      const setVal = (v) => {
        j.value = Math.min(j.max, Math.max(j.min, v));
        slider.value = j.value; num.value = j.value.toFixed(3);
        this.updateReadouts();
        this.hooks.onChange();
      };
      slider.addEventListener('input', (e) => setVal(parseFloat(e.target.value)));
      num.addEventListener('change', (e) => setVal(parseFloat(e.target.value) || 0));

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
    this.root.querySelector('#dof').textContent = degreesOfFreedom(this.joints);
    this.root.querySelector('#ee-pos').textContent = `${fmt(x)}, ${fmt(y)}, ${fmt(z)}`;
    this.root.querySelector('#ee-rpy').textContent = `${fmtDeg(r)}, ${fmtDeg(p)}, ${fmtDeg(yw)}`;

    const rows = dhTable(this.joints);
    this.root.querySelector('#dh-table').innerHTML =
      '<tr><th>J</th><th>θ</th><th>d</th><th>a</th><th>α</th><th>var</th></tr>' +
      rows.map((t) =>
        `<tr><td>${t.joint}</td><td>${t.theta.toFixed(2)}</td><td>${t.d.toFixed(2)}</td>` +
        `<td>${t.a.toFixed(2)}</td><td>${t.alpha.toFixed(2)}</td><td>${t.variable}</td></tr>`).join('');
  }
}
