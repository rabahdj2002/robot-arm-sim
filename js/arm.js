/**
 * three.js renderer for a DH serial chain.
 * The arm is rebuilt from joint definitions; each update() applies FK and
 * places link meshes between consecutive frame origins.
 */
import * as THREE from 'three';
import { forwardKinematics, getPosition, matFromRPY, JOINT_TYPES } from './dh.js';
import { singularity } from './ik.js';

const JOINT_COLORS = { revolute: 0xe05555, prismatic: 0x4da3ff, fixed: 0x888888 };

/** Release GPU resources for a removed subtree (prevents leaks on rebuild). */
function disposeDeep(obj) {
  obj.traverse((c) => {
    c.geometry?.dispose();
    if (c.material) {
      const mats = Array.isArray(c.material) ? c.material : [c.material];
      mats.forEach((m) => m.dispose());
    }
  });
}

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
      new THREE.SphereGeometry(0.026, 16, 16), // bigger than the EE tip so it stays visible on contact
      new THREE.MeshBasicMaterial({ color: 0x44ff88 }),
    );
    this.targetMarker.visible = false;
    this.scene.add(this.targetMarker);
    // axis arrows showing the DH frame directions at the target
    this.targetAxes = new THREE.AxesHelper(0.12);
    this.targetAxes.visible = false;
    this.scene.add(this.targetAxes);

    // singularity "stuck direction" arrow at the EE tip (magenta = near)
    this.stuckArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.14, 0xff3fd8, 0.035, 0.02);
    this.stuckArrow.visible = false;
    this.scene.add(this.stuckArrow);

    // EE trail — ring buffer of world-space tip positions, fading line
    this._trailMax = 1500;
    this._trailPts = [];
    this._trailGeo = new THREE.BufferGeometry();
    this._trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(this._trailMax * 3), 3));
    this._trailGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(this._trailMax * 3), 3));
    this._trailGeo.setDrawRange(0, 0);
    this.trail = new THREE.Line(
      this._trailGeo,
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9, blending: THREE.AdditiveBlending }),
    );
    this.trail.frustumCulled = false;
    this.trail.visible = false;
    this.scene.add(this.trail);

    // Workspace envelope — translucent dome shell (fresnel rim glow)
    this._wsGeo = new THREE.BufferGeometry();
    this.workspaceDome = new THREE.Mesh(
      this._wsGeo,
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        blending: THREE.AdditiveBlending,
        uniforms: {
          baseColor: { value: new THREE.Color(0x0e4a66) },
          rimColor: { value: new THREE.Color(0x4fd8ff) },
        },
        vertexShader: `
          varying vec3 vN;
          varying vec3 vV;
          void main() {
            vN = normalize(normalMatrix * normal);
            vec4 mv = modelViewMatrix * vec4(position, 1.0);
            vV = -mv.xyz;
            gl_Position = projectionMatrix * mv;
          }`,
        fragmentShader: `
          uniform vec3 baseColor;
          uniform vec3 rimColor;
          varying vec3 vN;
          varying vec3 vV;
          void main() {
            float f = pow(1.0 - abs(dot(normalize(vN), normalize(vV))), 2.0);
            vec3 c = mix(baseColor, rimColor, f);
            gl_FragColor = vec4(c, 0.10 + 0.55 * f);
          }`,
      }),
    );
    this.workspaceDome.frustumCulled = false;
    this.workspaceDome.visible = false;
    this.scene.add(this.workspaceDome);
  }

  /** Rebuild all meshes from joint definitions. */
  build(joints, collision = null) {
    this.joints = joints;
    disposeDeep(this.group);
    this.group.clear();
    // base pedestal
    const base = new THREE.Mesh(
      new THREE.CylinderGeometry(0.07, 0.09, 0.03, 24),
      new THREE.MeshStandardMaterial({ color: 0x333a4a, metalness: 0.6, roughness: 0.4 }),
    );
    base.rotation.x = Math.PI / 2; // cylinder axis -> local z -> world up
    base.position.z = -0.015;
    this.group.add(base);
    this._links = [];
    this.update(joints.map((j) => j.value), collision);
  }

  /** Recompute FK and reposition every joint/link mesh. */
  update(values, collision = null) {
    if (!this.joints) return;
    const { transforms } = forwardKinematics(this.joints, values);
    const points = transforms.map((t) => {
      const p = getPosition(t);
      return new THREE.Vector3(p[0], p[1], p[2]);
    });

    // clear dynamic children (keep base = first child)
    while (this.group.children.length > 1) {
      const c = this.group.children[1];
      this.group.remove(c);
      disposeDeep(c);
    }
    this._links = [];

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
      this._links.push(link);
      this.group.add(link);
    });

    // tint colliding links red
    const hit = new Set();
    const nL = this._links.length;
    if (collision) {
      if (collision.type === 'floor') hit.add(collision.link);
      else if (collision.type === 'self') { hit.add(collision.linkA); hit.add(collision.linkB); }
      else { // joint-body contact -> tint the links attached to it
        if (collision.link !== undefined) hit.add(collision.link);
        if (collision.joint !== undefined && collision.joint < nL) hit.add(collision.joint);
        if (collision.jointA !== undefined) {
          hit.add(Math.min(collision.jointA, nL - 1));
          hit.add(Math.min(collision.jointB, nL - 1));
        }
      }
    }
    this._links.forEach((l, i) =>
      l.material.color.setHex(hit.has(i) ? 0xff5555 : 0xd8dde8));

    // end-effector sphere
    const tip = new THREE.Mesh(
      new THREE.SphereGeometry(0.02, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0xffc247, metalness: 0.3, roughness: 0.4 }),
    );
    tip.position.copy(points[points.length - 1]);
    this.group.add(tip);

    // parallel-jaw gripper at the tool frame — jaws along +z, open along x
    const gmat = new THREE.MeshStandardMaterial({ color: 0x9aa7bd, metalness: 0.6, roughness: 0.35 });
    const gripper = new THREE.Group();
    const palm = new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.036, 0.018), gmat);
    palm.position.z = 0.012;
    gripper.add(palm);
    for (const s of [-1, 1]) {
      const jaw = new THREE.Mesh(new THREE.BoxGeometry(0.007, 0.012, 0.034), gmat);
      jaw.position.set(s * 0.013, 0, 0.037);
      const pad = new THREE.Mesh(new THREE.BoxGeometry(0.007, 0.016, 0.008), gmat);
      pad.position.set(s * 0.011, 0, 0.052);
      gripper.add(jaw, pad);
    }
    const eeT = transforms[transforms.length - 1];
    const eeM = new THREE.Matrix4().set(
      eeT[0][0], eeT[0][1], eeT[0][2], eeT[0][3],
      eeT[1][0], eeT[1][1], eeT[1][2], eeT[1][3],
      eeT[2][0], eeT[2][1], eeT[2][2], eeT[2][3],
      0, 0, 0, 1,
    );
    gripper.applyMatrix4(eeM);
    this.group.add(gripper);

    // singularity arrow: shows the weakest EE motion direction when the
    // arm is close to losing it (amber < 0.15, magenta < 0.05)
    const sg = singularity(this.joints, values);
    if (sg.condPos < 0.15 && points.length > 1) {
      const dir = new THREE.Vector3(...sg.dirPos);
      this.stuckArrow.position.copy(
        this.group.localToWorld(points[points.length - 1].clone()));
      this.stuckArrow.setDirection(this.group.localToWorld(dir.clone()).sub(
        this.group.localToWorld(new THREE.Vector3())).normalize());
      this.stuckArrow.setColor(sg.condPos < 0.05 ? 0xff3fd8 : 0xffb347);
      this.stuckArrow.visible = true;
    } else {
      this.stuckArrow.visible = false;
    }

    // append tip position to the trail (world space)
    if (this.trail.visible && this._lastTip !== undefined) {
      const wp = this.group.localToWorld(points[points.length - 1].clone());
      const last = this._trailPts[this._trailPts.length - 1];
      if (!last || last.distanceToSquared(wp) > 4e-6) {
        this._trailPts.push(wp);
        if (this._trailPts.length > this._trailMax) this._trailPts.shift();
        const pos = this._trailGeo.attributes.position.array;
        const col = this._trailGeo.attributes.color.array;
        const n = this._trailPts.length;
        for (let k = 0; k < n; k++) {
          pos[k * 3] = this._trailPts[k].x;
          pos[k * 3 + 1] = this._trailPts[k].y;
          pos[k * 3 + 2] = this._trailPts[k].z;
          const f = (k / Math.max(1, n - 1)) ** 1.5; // fade old -> bright new
          col[k * 3] = 0.1 * f;
          col[k * 3 + 1] = 0.75 * f;
          col[k * 3 + 2] = 0.9 * f;
        }
        this._trailGeo.setDrawRange(0, n);
        this._trailGeo.attributes.position.needsUpdate = true;
        this._trailGeo.attributes.color.needsUpdate = true;
      }
    }
    this._lastTip = points[points.length - 1];
  }

  setTrail(v) {
    this.trail.visible = v;
    if (v) { this._trailPts = []; this._trailGeo.setDrawRange(0, 0); }
  }

  /** Show/hide the reachable-workspace dome. shell = {positions(world), indices}. */
  setWorkspace(shell) {
    if (!shell) { this.workspaceDome.visible = false; return; }
    this._wsGeo.setAttribute('position', new THREE.BufferAttribute(shell.positions, 3));
    this._wsGeo.setIndex(new THREE.BufferAttribute(shell.indices, 1));
    this._wsGeo.computeVertexNormals();
    this.workspaceDome.visible = true;
  }

  /** Convert a world-space point into the arm's DH coordinate space. */
  worldToDH(v) {
    this.group.updateMatrixWorld(true);
    return this.group.worldToLocal(v.clone());
  }

  setTarget(pos, rpy = null) {
    this.targetMarker.visible = pos !== null;
    this.targetAxes.visible = pos !== null;
    if (pos) {
      // pos is in DH space; the marker lives in the scene, so convert
      // through the arm group's transform (DH z -> world up)
      this.group.updateMatrixWorld(true);
      const wp = this.group.localToWorld(new THREE.Vector3(pos[0], pos[1], pos[2]));
      this.targetMarker.position.copy(wp);
      this.targetAxes.position.copy(wp);
      // with an orientation target, the axes show the demanded RPY;
      // without one they mirror the DH base frame
      if (rpy) {
        const Rdh = matFromRPY(rpy[0], rpy[1], rpy[2]);
        const m = new THREE.Matrix4().set(
          Rdh[0][0], Rdh[0][1], Rdh[0][2], 0,
          Rdh[1][0], Rdh[1][1], Rdh[1][2], 0,
          Rdh[2][0], Rdh[2][1], Rdh[2][2], 0,
          0, 0, 0, 1);
        this.targetAxes.quaternion.setFromRotationMatrix(
          m.premultiply(new THREE.Matrix4().makeRotationX(-Math.PI / 2)));
      } else {
        this.targetAxes.quaternion.copy(this.group.quaternion);
      }
    }
  }

  setShowFrames(v) {
    this.showFrames = v;
    if (this.joints) this.update(this.joints.map((j) => j.value));
  }
}
