/**
 * three.js renderer for a DH serial chain.
 * The arm is rebuilt from joint definitions; each update() applies FK and
 * places link meshes between consecutive frame origins.
 */
import * as THREE from 'three';
import { forwardKinematics, getPosition, JOINT_TYPES } from './dh.js';

const JOINT_COLORS = { revolute: 0xe05555, prismatic: 0x4da3ff, fixed: 0x888888 };

function makeLinkMesh(from, to, radius = 0.022, color = 0xd8dde8) {
  const dir = new THREE.Vector3().subVectors(to, from);
  const len = dir.length();
  const geo = new THREE.CylinderGeometry(radius, radius, Math.max(len, 1e-6), 12);
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.55, roughness: 0.4 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.copy(from).addScaledVector(dir, 0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  return mesh;
}

function makeJointMesh(type) {
  const group = new THREE.Group();
  const color = JOINT_COLORS[type] ?? 0x888888;
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.4, roughness: 0.5 });
  if (type === JOINT_TYPES.REVOLUTE) {
    const cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.05, 20), mat);
    cyl.rotation.x = Math.PI / 2; // axis of motion = local z
    group.add(cyl);
  } else {
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.05), mat);
    group.add(box);
  }
  return group;
}

export class RobotArm {
  constructor(scene) {
    this.scene = scene;
    this.group = new THREE.Group();
    this.group.rotation.x = -Math.PI / 2; // DH z-axis -> world up
    this.scene.add(this.group);
    this.showFrames = true;
    this.targetMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.015, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0x44ff88 }),
    );
    this.targetMarker.visible = false;
    this.scene.add(this.targetMarker);
  }

  /** Rebuild all meshes from joint definitions. */
  build(joints) {
    this.joints = joints;
    this.group.clear();
    // base pedestal
    const base = new THREE.Mesh(
      new THREE.CylinderGeometry(0.07, 0.09, 0.03, 24),
      new THREE.MeshStandardMaterial({ color: 0x333a4a, metalness: 0.6, roughness: 0.4 }),
    );
    base.position.y = -0.015;
    this.group.add(base);
    this.update(joints.map((j) => j.value));
  }

  /** Recompute FK and reposition every joint/link mesh. */
  update(values) {
    if (!this.joints) return;
    const { transforms } = forwardKinematics(this.joints, values);
    const points = transforms.map((t) => {
      const p = getPosition(t);
      return new THREE.Vector3(p[0], p[1], p[2]);
    });

    // clear dynamic children (keep base = first child)
    while (this.group.children.length > 1) this.group.remove(this.group.children[1]);

    this.joints.forEach((j, i) => {
      const framePoint = points[i]; // joint i sits at frame i's origin (acts about z_{i-1})
      const jm = makeJointMesh(j.type);
      jm.position.copy(framePoint);
      // orient the joint's motion axis to z_{i-1} (frame i's z)
      const m = new THREE.Matrix4().set(
        transforms[i][0][0], transforms[i][0][1], transforms[i][0][2], transforms[i][0][3],
        transforms[i][1][0], transforms[i][1][1], transforms[i][1][2], transforms[i][1][3],
        transforms[i][2][0], transforms[i][2][1], transforms[i][2][2], transforms[i][2][3],
        0, 0, 0, 1,
      );
      jm.quaternion.setFromRotationMatrix(m);
      this.group.add(jm);

      if (this.showFrames) {
        const axes = new THREE.AxesHelper(0.08);
        axes.applyMatrix4(m);
        this.group.add(axes);
      }

      const link = makeLinkMesh(points[i], points[i + 1]);
      this.group.add(link);
    });

    // end-effector sphere
    const tip = new THREE.Mesh(
      new THREE.SphereGeometry(0.02, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0xffc247, metalness: 0.3, roughness: 0.4 }),
    );
    tip.position.copy(points[points.length - 1]);
    this.group.add(tip);
  }

  setTarget(pos) {
    this.targetMarker.visible = pos !== null;
    if (pos) this.targetMarker.position.set(pos[0], pos[1], pos[2]);
  }

  setShowFrames(v) {
    this.showFrames = v;
    if (this.joints) this.update(this.joints.map((j) => j.value));
  }
}
