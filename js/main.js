import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RobotArm } from './arm.js';
import { Panel } from './ui.js';
import { PRESETS } from './dh.js';

const canvas = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x11151f);

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
camera.position.set(0.55, 0.45, 0.75);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.2, 0);

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a2030, 1.0));
const dir = new THREE.DirectionalLight(0xffffff, 1.6);
dir.position.set(1.5, 2.5, 1.2);
scene.add(dir);

const grid = new THREE.GridHelper(2, 20, 0x2a3350, 0x1d2438);
scene.add(grid);

const arm = new RobotArm(scene);
let joints = PRESETS['PUMA-style 6R']();
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
});

document.getElementById('frames-toggle').addEventListener('change', (e) => arm.setShowFrames(e.target.checked));

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
