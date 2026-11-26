import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RobotArm } from './arm.js';
import { Panel } from './ui.js';
import { PRESETS } from './dh.js';
import { configFromHash } from './config.js';
import { sampleWorkspaceConfigs, workspaceShell } from './workspace.js';

const canvas = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0e18);
scene.fog = new THREE.Fog(0x0a0e18, 1.6, 4.5);

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
camera.position.set(0.55, 0.45, 0.75);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.2, 0);

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a2030, 1.0));
const dir = new THREE.DirectionalLight(0xffffff, 1.6);
dir.position.set(1.5, 2.5, 1.2);
scene.add(dir);
// cool rim light for depth separation from the dark floor
const rim = new THREE.DirectionalLight(0x4da3ff, 0.7);
rim.position.set(-1.5, 1.2, -1.5);
scene.add(rim);

const grid = new THREE.GridHelper(2, 20, 0x2a3350, 0x1d2438);
scene.add(grid);

const arm = new RobotArm(scene);
// shared-link config takes precedence over the default preset
let joints = configFromHash(location.hash) || PRESETS['PUMA-style 6R']();
arm.build(joints);

const panel = new Panel(document.getElementById('panel'), joints, {
  onChange: (coll) => arm.update(joints.map((j) => j.value), coll),
  onRebuild: () => {
    arm.build(joints);
    panel.updateReadouts();
  },
  onAddJoint: (j) => {
    joints.push(j);
    arm.build(joints);
    panel.renderJoints();
  },
  onRemoveJoint: (i) => {
    joints.splice(i, 1);
    arm.build(joints);
    panel.renderJoints();
  },
  onPreset: (newJoints) => {
    joints.length = 0;
    newJoints.forEach((j) => joints.push(j));
    panel.joints = joints;
    arm.build(joints);
    panel.renderJoints();
  },
  onTarget: (pos) => arm.setTarget(pos),
  onTrail: (v) => arm.setTrail(v),
  onWorkspace: (v) => {
    if (!v) { arm.setWorkspace(null); return; }
    const { positions, values } = sampleWorkspaceConfigs(joints, 40000, Math.random, 0);
    const shell = workspaceShell(positions, 72, 36, 0, joints, values);
    const world = new Float32Array(shell.positions.length);
    const tmp = new THREE.Vector3();
    arm.group.updateMatrixWorld(true);
    for (let i = 0; i < shell.positions.length; i += 3) {
      tmp.set(shell.positions[i], shell.positions[i + 1], shell.positions[i + 2]);
      arm.group.localToWorld(tmp);
      world[i] = tmp.x; world[i + 1] = tmp.y; world[i + 2] = tmp.z;
    }
    arm.setWorkspace({ positions: world, indices: shell.indices });
  },
});

document.getElementById('frames-toggle').addEventListener('change', (e) => arm.setShowFrames(e.target.checked));

const HOME_POS = [0.55, 0.45, 0.75], HOME_TGT = [0, 0.2, 0];
document.getElementById('reset-view').addEventListener('click', () => {
  camera.position.set(...HOME_POS);
  controls.target.set(...HOME_TGT);
  controls.update();
});

// --- drag the IK target marker in the viewport ---
const ray = new THREE.Raycaster();
const ptr = new THREE.Vector2();
const dragPlane = new THREE.Plane();
let dragging = false;

const setPtr = (e) =>
  ptr.set((e.offsetX / canvas.clientWidth) * 2 - 1, -(e.offsetY / canvas.clientHeight) * 2 + 1);

canvas.addEventListener('pointerdown', (e) => {
  if (!arm.targetMarker.visible) return;
  setPtr(e);
  ray.setFromCamera(ptr, camera);
  if (!ray.intersectObject(arm.targetMarker, false).length) return;
  dragging = true;
  controls.enabled = false;
  canvas.setPointerCapture(e.pointerId);
  // drag on a camera-facing plane through the marker
  dragPlane.setFromNormalAndCoplanarPoint(
    camera.getWorldDirection(new THREE.Vector3()).negate(),
    arm.targetMarker.position,
  );
});

canvas.addEventListener('pointermove', (e) => {
  setPtr(e);
  ray.setFromCamera(ptr, camera);
  if (!dragging) {
    canvas.style.cursor =
      arm.targetMarker.visible && ray.intersectObject(arm.targetMarker, false).length
        ? 'grab' : '';
    return;
  }
  const hit = new THREE.Vector3();
  if (ray.ray.intersectPlane(dragPlane, hit)) {
    const dh = arm.worldToDH(hit);
    panel.setTarget(dh.x, dh.y, dh.z);
  }
});

window.addEventListener('pointerup', () => {
  if (dragging) { dragging = false; controls.enabled = true; }
});

// debug/test handle (harmless in prod — enables e2e checks)
window.__sim = { arm, panel, camera, controls, scene };

function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * renderer.getPixelRatio() || canvas.height !== h * renderer.getPixelRatio()) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}

renderer.setAnimationLoop(() => {
  resize();
  controls.update();
  renderer.render(scene, camera);
});

// debug/e2e handle
window.__app = { arm, panel, joints };
