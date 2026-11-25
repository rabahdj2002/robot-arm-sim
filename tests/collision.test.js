import test from 'node:test';
import assert from 'node:assert/strict';
import { segSegDist, checkCollisions, describeCollision } from '../js/collision.js';
import { makeJoint, PRESETS } from '../js/dh.js';

const near = (a, b, eps) => Math.abs(a - b) < eps;

test('segSegDist: identical segments -> 0', () => {
  assert.equal(segSegDist([0,0,0],[1,0,0],[0,0,0],[1,0,0]), 0);
});

test('segSegDist: parallel offset segments', () => {
  const d = segSegDist([0,0,0],[1,0,0],[0,0.5,0],[1,0.5,0]);
  assert.ok(near(d, 0.5, 1e-9));
});

test('segSegDist: crossing perpendicular -> 0', () => {
  const d = segSegDist([-1,0,0],[1,0,0],[0,-1,0],[0,1,0]);
  assert.ok(d < 1e-9);
});

test('segSegDist: skew segments', () => {
  // x-axis segment vs y-axis segment lifted 0.3 in z and shifted in x
  const d = segSegDist([0,0,0],[1,0,0],[0.5,-1,0.3],[0.5,1,0.3]);
  assert.ok(near(d, 0.3, 1e-9));
});

test('segSegDist: degenerate segment (point)', () => {
  const d = segSegDist([0,0,0],[0,0,0],[0,0,1],[0,0,1]);
  assert.ok(near(d, 1, 1e-9));
});

test('all presets are collision-free at default pose', () => {
  for (const [name, make] of Object.entries(PRESETS)) {
    assert.equal(checkCollisions(make()), null, `${name} should be collision-free`);
  }
});

test('empty chain is collision-free', () => {
  assert.equal(checkCollisions([]), null);
});

test('floor: link dipping below z=0 is caught', () => {
  const joints = [
    makeJoint('revolute', { a: 0, d: 0.2, alpha: Math.PI / 2 }), // frame1 at z=0.2
    makeJoint('revolute', { a: 0.3, value: -Math.PI / 2 }),      // swings link below floor
  ];
  const c = checkCollisions(joints);
  assert.equal(c?.type, 'floor');
  assert.equal(c.link, 1);
});

test('floor: same chain swung upward is clean', () => {
  const joints = [
    makeJoint('revolute', { a: 0, d: 0.2, alpha: Math.PI / 2 }),
    makeJoint('revolute', { a: 0.3, value: Math.PI / 2 }),
  ];
  assert.equal(checkCollisions(joints), null);
});

test('planar arm resting at floor level (z=0) is not flagged', () => {
  const joints = [makeJoint('revolute', { a: 0.3 }), makeJoint('revolute', { a: 0.25 })];
  assert.equal(checkCollisions(joints), null);
});

test('self: folded 3R chain reports link overlap', () => {
  const joints = [
    makeJoint('revolute', { a: 0.3 }),
    makeJoint('revolute', { a: 0.3, value: Math.PI }),
    makeJoint('revolute', { a: 0.3 }),
  ];
  const c = checkCollisions(joints);
  assert.equal(c?.type, 'self');
});

test('self: straight collinear chain is clean (adjacency not flagged)', () => {
  const joints = [
    makeJoint('revolute', { a: 0.3 }),
    makeJoint('revolute', { a: 0.3 }),
    makeJoint('revolute', { a: 0.3 }),
  ];
  assert.equal(checkCollisions(joints), null);
});

test('self: degenerate middle link does not false-positive', () => {
  const joints = [
    makeJoint('revolute', { a: 0.3 }),
    makeJoint('revolute', { a: 0, d: 0 }),  // zero-length link between
    makeJoint('revolute', { a: 0.3 }),
  ];
  assert.equal(checkCollisions(joints), null);
});

test('values override lets callers test candidate poses', () => {
  const joints = [
    makeJoint('revolute', { a: 0, d: 0.2, alpha: Math.PI / 2 }),
    makeJoint('revolute', { a: 0.3 }),
  ];
  assert.equal(checkCollisions(joints, [0, -Math.PI / 2])?.type, 'floor');
  assert.equal(checkCollisions(joints, [0, 0]), null);
});

test('describeCollision names the links', () => {
  assert.match(describeCollision({ type: 'floor', link: 2 }), /link 3.*floor/);
  assert.match(describeCollision({ type: 'self', linkA: 0, linkB: 4 }), /1.*5/);
});
