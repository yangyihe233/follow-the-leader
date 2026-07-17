import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

const EPS = 1e-10;
const DEG = Math.PI / 180;
const viewport = document.getElementById('viewport');

const ui = {
  mode: document.getElementById('modeSelect'),
  reset: document.getElementById('resetBtn'),
  refine: document.getElementById('refineBtn'),
  p2x: document.getElementById('p2x'),
  p2y: document.getElementById('p2y'),
  p2z: document.getElementById('p2z'),
  yaw: document.getElementById('yawRange'),
  yawOut: document.getElementById('yawOut'),
  pitch: document.getElementById('pitchRange'),
  pitchOut: document.getElementById('pitchOut'),
  smin: document.getElementById('sminInput'),
  smax: document.getElementById('smaxInput'),
  autoRefine: document.getElementById('autoRefine'),
  showGeometry: document.getElementById('showGeometry'),
  bodyLength: document.getElementById('bodyLength'),
  progress: document.getElementById('progressRange'),
  progressOut: document.getElementById('progressOut'),
  play: document.getElementById('playBtn'),
  restart: document.getElementById('restartBtn'),
  csv: document.getElementById('csvBtn'),
  png: document.getElementById('pngBtn'),
  status: document.getElementById('statusBox'),
};

const state = {
  mode: 'J_lineArc',
  P0: new THREE.Vector3(),
  P1: new THREE.Vector3(),
  P2: new THREE.Vector3(),
  T0: new THREE.Vector3(0, 0, 1),
  T2: new THREE.Vector3(0, 1, 0),
  planeN: new THREE.Vector3(0, 1, 0),
  smin: 45,
  smax: 150,
  wLen: 0.05,
  hFD: 1e-5,
  tol: 1e-9,
  maxGN: 220,
  refineIters: 80,
  realtimeIters: 4,
  stepCap: 8,
  lambda0: 1e-3,
  lambdaMax: 1e6,
  pathPoints: [],
  cumulative: [],
  totalLength: 0,
  playing: false,
  lastAnimationTime: performance.now(),
  selectedHandle: null,
  lastSolve: { status: 'not solved', iters: 0, rnorm: NaN },
};

const presets = {
  J_lineArc: {
    P0: [0, 0, 0], T0: [0, 0, 1], P2: [0, 100, 160], T2: [0, 1, 0.1],
    P1: [0, 0, 90], planeN: [0, 1, 0],
  },
  C_coplanar: {
    P0: [0, 0, 0], T0: [0, 0, 1], P2: [-100, -100, 160], T2: [-1.5, 0, 0.1],
    P1: [-20, -20, 90], planeN: [1, -1, 0],
  },
  C_free3d: {
    P0: [0, 0, 0], T0: [0, 0, 1], P2: [-100, 80, 160], T2: [-3, 1, 0.3],
    P1: [-20, -1, 95], planeN: [0, 1, 0],
  },
  S_coplanar: {
    P0: [0, 0, 0], T0: [0, 0, 1], P2: [0, 30, 220], T2: [0, -0.8, 0.5],
    P1: [0, 30, 120], planeN: [1, 0, 0],
  },
  S_free3d: {
    P0: [0, 0, 0], T0: [0, 0, 1], P2: [80, 10, 210], T2: [0.10, -0.50, 0.3],
    P1: [40, 35, 110], planeN: [0, 1, 0],
  },
};

// ---------- Three.js scene ----------
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xedf4f2);
scene.fog = new THREE.Fog(0xedf4f2, 550, 1000);

const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 3000);
camera.up.set(0, 0, 1);
camera.position.set(360, -390, 300);

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.appendChild(renderer.domElement);

const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.dampingFactor = 0.06;
orbit.target.set(0, 0, 100);

scene.add(new THREE.HemisphereLight(0xffffff, 0x48615b, 2.2));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.0);
keyLight.position.set(250, -180, 400);
scene.add(keyLight);

const grid = new THREE.GridHelper(500, 20, 0xa2b5af, 0xcdd9d5);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(70));

const line1 = makeLine(0xd5503f, 4);
const line2 = makeLine(0x2e68c7, 4);
const fullPathLine = makeLine(0x6f8580, 1.5, 0.36);
const bodyLine = makeLine(0x00a99d, 8);
scene.add(fullPathLine, line1, line2, bodyLine);

const p0Mesh = makePoint(0x15211e, 6);
const p1Mesh = makePoint(0x23a46f, 8);
const p2Mesh = makePoint(0xe28b27, 8);
p1Mesh.userData.handle = 'P1';
p2Mesh.userData.handle = 'P2';
scene.add(p0Mesh, p1Mesh, p2Mesh);

const center1 = makePoint(0xf2c94c, 4.5);
const center2 = makePoint(0x55d6d0, 4.5);
scene.add(center1, center2);
const radius1a = makeLine(0x4d5a56, 1, 0.55);
const radius1b = makeLine(0x4d5a56, 1, 0.55);
const radius2a = makeLine(0x4d5a56, 1, 0.55);
const radius2b = makeLine(0x4d5a56, 1, 0.55);
scene.add(radius1a, radius1b, radius2a, radius2b);

const tipMesh = makePoint(0xffffff, 6);
tipMesh.material.emissive = new THREE.Color(0x00a99d);
tipMesh.material.emissiveIntensity = 0.75;
scene.add(tipMesh);

const arrowT0 = new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(), 28, 0x9a2f2f, 7, 4);
const arrowT1 = new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(), 28, 0xa13cc1, 7, 4);
const arrowT2 = new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(), 28, 0x008fa4, 7, 4);
scene.add(arrowT0, arrowT1, arrowT2);

const designPlane = new THREE.Mesh(
  new THREE.PlaneGeometry(430, 430),
  new THREE.MeshBasicMaterial({ color: 0x3d9f88, transparent: true, opacity: 0.055, side: THREE.DoubleSide, depthWrite: false })
);
scene.add(designPlane);

const transform = new TransformControls(camera, renderer.domElement);
transform.setMode('translate');
transform.setSize(0.72);
scene.add(transform);
transform.addEventListener('dragging-changed', (event) => {
  orbit.enabled = !event.value;
  if (!event.value && ui.autoRefine.checked) {
    solveP1(state.refineIters);
    redraw();
  }
});
transform.addEventListener('objectChange', () => {
  if (transform.object === p1Mesh) {
    state.P1.copy(p1Mesh.position);
    applyP1Constraint();
    solveP1(state.realtimeIters);
  } else if (transform.object === p2Mesh) {
    state.P2.copy(p2Mesh.position);
    applyP2Constraint();
    solveP1(state.realtimeIters);
  }
  syncUIFromState(false);
  redraw();
});

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
renderer.domElement.addEventListener('pointerdown', (event) => {
  if (transform.dragging) return;
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects([p1Mesh, p2Mesh], false);
  if (hits.length) {
    transform.attach(hits[0].object);
    state.selectedHandle = hits[0].object.userData.handle;
    redrawStatus();
  }
});

// ---------- Geometry and optimization ----------
function vec(values) {
  return new THREE.Vector3(values[0], values[1], values[2]);
}

function isLineArcMode() { return state.mode === 'J_lineArc'; }
function isCoplanarMode() { return state.mode.includes('coplanar'); }

function projectPointToPlane(point, origin, normal) {
  const n = normal.clone().normalize();
  return point.clone().sub(n.multiplyScalar(point.clone().sub(origin).dot(n)));
}

function projectVectorToPlane(vector, normal) {
  const n = normal.clone().normalize();
  const projected = vector.clone().sub(n.multiplyScalar(vector.dot(n)));
  if (projected.length() < 1e-8) return pickPerpendicular(normal);
  return projected.normalize();
}

function pickPerpendicular(vector) {
  const v = vector.clone().normalize();
  const t = Math.abs(v.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
  return t.sub(v.clone().multiplyScalar(t.dot(v))).normalize();
}

function applyP1Constraint() {
  if (isLineArcMode()) {
    const t = state.T0.clone().normalize();
    let s = state.P1.clone().sub(state.P0).dot(t);
    s = clamp(s, state.smin, state.smax);
    state.P1.copy(state.P0).addScaledVector(t, s);
  } else if (isCoplanarMode()) {
    state.P1.copy(projectPointToPlane(state.P1, state.P0, state.planeN));
  }
}

function applyP2Constraint() {
  if (isCoplanarMode()) {
    state.P2.copy(projectPointToPlane(state.P2, state.P0, state.planeN));
    state.T2.copy(projectVectorToPlane(state.T2, state.planeN));
  }
}

function circleFromStartTangentToPoint(A, tangent, B) {
  const u = tangent.clone().normalize();
  const d = B.clone().sub(A);
  const dPerp = d.clone().sub(u.clone().multiplyScalar(d.dot(u)));

  if (dPerp.length() < 1e-9) {
    const w = pickPerpendicular(u);
    return { center: null, radius: Infinity, theta: 0, u, w, normal: u.clone().cross(w).normalize(), endTangent: u };
  }

  const w = dPerp.normalize();
  const normal = u.clone().cross(w).normalize();
  const x = d.dot(u);
  const y = d.dot(w);

  if (Math.abs(y) < 1e-9) {
    return { center: null, radius: Infinity, theta: 0, u, w, normal, endTangent: u };
  }

  const radius = (x * x + y * y) / (2 * y);
  const theta = 2 * Math.atan2(y, x);
  const center = A.clone().addScaledVector(w, radius);
  const endTangent = u.clone().multiplyScalar(Math.cos(theta)).addScaledVector(w, Math.sin(theta)).normalize();
  return { center, radius, theta, u, w, normal, endTangent };
}

function buildArcForward(A, tangent, B, count) {
  const circle = circleFromStartTangentToPoint(A, tangent, B);
  const points = [];
  if (!Number.isFinite(circle.radius) || Math.abs(circle.theta) < EPS) {
    for (let i = 0; i < count; i += 1) points.push(A.clone().lerp(B, i / Math.max(1, count - 1)));
    return points;
  }
  for (let i = 0; i < count; i += 1) {
    const phi = circle.theta * i / Math.max(1, count - 1);
    points.push(
      A.clone()
        .addScaledVector(circle.u, circle.radius * Math.sin(phi))
        .addScaledVector(circle.w, circle.radius * (1 - Math.cos(phi)))
    );
  }
  return points;
}

function buildPoints(count = 280) {
  const n1 = Math.round(count / 2);
  const n2 = count - n1;
  let first;
  if (isLineArcMode()) {
    first = [];
    for (let i = 0; i < n1; i += 1) first.push(state.P0.clone().lerp(state.P1, i / Math.max(1, n1 - 1)));
  } else {
    first = buildArcForward(state.P0, state.T0, state.P1, n1);
  }
  const second = buildArcForward(state.P2, state.T2.clone().negate(), state.P1, n2).reverse();
  return { first, second, all: [...first, ...second.slice(1)] };
}

function arcLengths(P1 = state.P1) {
  let s1;
  if (isLineArcMode()) {
    s1 = P1.distanceTo(state.P0);
  } else {
    const c1 = circleFromStartTangentToPoint(state.P0, state.T0, P1);
    s1 = Number.isFinite(c1.radius) && Math.abs(c1.theta) > EPS ? Math.abs(c1.radius * c1.theta) : state.P0.distanceTo(P1);
  }
  const c2 = circleFromStartTangentToPoint(state.P2, state.T2.clone().negate(), P1);
  const s2 = Number.isFinite(c2.radius) && Math.abs(c2.theta) > EPS ? Math.abs(c2.radius * c2.theta) : state.P2.distanceTo(P1);
  return { s1, s2 };
}

function distanceToInterval(s) {
  if (s < state.smin) return state.smin - s;
  if (s > state.smax) return s - state.smax;
  return 0;
}

function residual(P1) {
  let T1end;
  let T2forward;
  if (isLineArcMode()) {
    const backward = circleFromStartTangentToPoint(state.P2, state.T2.clone().negate(), P1);
    T1end = state.T0.clone().normalize();
    T2forward = backward.endTangent.clone().negate();
  } else {
    const first = circleFromStartTangentToPoint(state.P0, state.T0, P1);
    const backward = circleFromStartTangentToPoint(state.P2, state.T2.clone().negate(), P1);
    T1end = first.endTangent;
    T2forward = backward.endTangent.clone().negate();
  }
  const tanResidual = T1end.clone().sub(T2forward);
  const { s1, s2 } = arcLengths(P1);
  const r = [tanResidual.x, tanResidual.y, tanResidual.z, state.wLen * distanceToInterval(s1), state.wLen * distanceToInterval(s2)];
  if (r.some((value) => !Number.isFinite(value))) return { r: [1e6, 1e6, 1e6, 1e6, 1e6], T1end, T2forward, s1, s2 };
  return { r, T1end, T2forward, s1, s2 };
}

function parameterization() {
  if (isLineArcMode()) {
    return { basis: [state.T0.clone().normalize()], origin: state.P0.clone() };
  }
  if (isCoplanarMode()) {
    const n = state.planeN.clone().normalize();
    let e1 = state.T0.clone().sub(n.clone().multiplyScalar(state.T0.dot(n)));
    if (e1.length() < 1e-8) e1 = pickPerpendicular(n); else e1.normalize();
    const e2 = n.clone().cross(e1).normalize();
    return { basis: [e1, e2], origin: state.P0.clone() };
  }
  return {
    basis: [new THREE.Vector3(1,0,0), new THREE.Vector3(0,1,0), new THREE.Vector3(0,0,1)],
    origin: new THREE.Vector3(),
  };
}

function pointFromQ(q, basis, origin) {
  const p = origin.clone();
  basis.forEach((b, i) => p.addScaledVector(b, q[i]));
  return p;
}

function qFromPoint(point, basis, origin) {
  const d = point.clone().sub(origin);
  return basis.map((b) => d.dot(b));
}

function clampQ(q) {
  if (isLineArcMode()) q[0] = clamp(q[0], state.smin, state.smax);
  return q;
}

function solveP1(maxIter = state.refineIters) {
  const { basis, origin } = parameterization();
  let q = clampQ(qFromPoint(state.P1, basis, origin));
  let lambda = state.lambda0;
  let lastNorm = Infinity;
  let status = 'max_iter';
  let iterations = 0;

  for (let iter = 0; iter < Math.max(0, maxIter); iter += 1) {
    iterations = iter + 1;
    const currentP = pointFromQ(q, basis, origin);
    const r = residual(currentP).r;
    const rnorm = norm(r);
    lastNorm = rnorm;
    if (rnorm < state.tol) { status = 'converged'; iterations = iter; break; }

    const m = q.length;
    const J = Array.from({ length: r.length }, () => Array(m).fill(0));
    for (let k = 0; k < m; k += 1) {
      const qPlus = q.slice();
      qPlus[k] += state.hFD;
      const rPlus = residual(pointFromQ(clampQ(qPlus), basis, origin)).r;
      for (let row = 0; row < r.length; row += 1) J[row][k] = (rPlus[row] - r[row]) / state.hFD;
    }

    const H = Array.from({ length: m }, () => Array(m).fill(0));
    const g = Array(m).fill(0);
    for (let a = 0; a < m; a += 1) {
      for (let b = 0; b < m; b += 1) {
        for (let row = 0; row < r.length; row += 1) H[a][b] += J[row][a] * J[row][b];
        if (a === b) H[a][b] += lambda;
      }
      for (let row = 0; row < r.length; row += 1) g[a] += J[row][a] * r[row];
    }

    let dq = solveLinear(H, g.map((value) => -value));
    if (!dq || dq.some((value) => !Number.isFinite(value))) {
      dq = g.map((value, i) => -value / Math.max(Math.abs(H[i][i]), 1e-8));
    }
    const stepNorm = norm(dq);
    if (stepNorm > state.stepCap) dq = dq.map((value) => value * state.stepCap / stepNorm);

    const f0 = 0.5 * dot(r, r);
    const directional = dot(g, dq);
    let alpha = 1;
    let accepted = false;
    let qTry = q.slice();
    let f1 = f0;

    while (alpha > 1e-6) {
      qTry = clampQ(q.map((value, i) => value + alpha * dq[i]));
      const rTry = residual(pointFromQ(qTry, basis, origin)).r;
      f1 = 0.5 * dot(rTry, rTry);
      if (f1 <= f0 + 1e-4 * alpha * directional) { accepted = true; break; }
      alpha *= 0.5;
    }

    if (!accepted) {
      lambda = Math.min(lambda * 10, state.lambdaMax);
    } else {
      q = qTry;
      lambda = f1 < f0 ? Math.max(lambda / 3, 1e-12) : Math.min(lambda * 3, state.lambdaMax);
    }

    if (norm(dq) * alpha < 1e-9) { status = 'stalled'; break; }
  }

  state.P1.copy(pointFromQ(q, basis, origin));
  applyP1Constraint();
  lastNorm = norm(residual(state.P1).r);
  if (lastNorm < state.tol) status = 'converged';
  state.lastSolve = { status, iters: iterations, rnorm: lastNorm };
  return state.lastSolve;
}

function solveLinear(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < n; row += 1) if (Math.abs(M[row][col]) > Math.abs(M[pivot][col])) pivot = row;
    if (Math.abs(M[pivot][col]) < 1e-12) return null;
    [M[col], M[pivot]] = [M[pivot], M[col]];
    const divisor = M[col][col];
    for (let j = col; j <= n; j += 1) M[col][j] /= divisor;
    for (let row = 0; row < n; row += 1) {
      if (row === col) continue;
      const factor = M[row][col];
      for (let j = col; j <= n; j += 1) M[row][j] -= factor * M[col][j];
    }
  }
  return M.map((row) => row[n]);
}

// ---------- Drawing and FTL replay ----------
function makeLine(color, width = 2, opacity = 1) {
  return new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color, linewidth: width, transparent: opacity < 1, opacity })
  );
}

function makePoint(color, radius) {
  return new THREE.Mesh(
    new THREE.SphereGeometry(radius, 24, 16),
    new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.05 })
  );
}

function updateLine(line, points) {
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(points.length ? points : [new THREE.Vector3(), new THREE.Vector3()]);
  line.visible = points.length > 1;
}

function updateArrow(arrow, origin, direction, length = 28) {
  const d = direction.clone().normalize();
  arrow.position.copy(origin);
  arrow.setDirection(d);
  arrow.setLength(length, 7, 4);
}

function updateDesignPlane() {
  designPlane.visible = isCoplanarMode() && ui.showGeometry.checked;
  if (!designPlane.visible) return;
  const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), state.planeN.clone().normalize());
  designPlane.quaternion.copy(q);
  designPlane.position.copy(state.P0);
}

function cachePath(points) {
  state.pathPoints = points.map((p) => p.clone());
  state.cumulative = [0];
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    total += points[i].distanceTo(points[i - 1]);
    state.cumulative.push(total);
  }
  state.totalLength = total;
}

function getPlannerPathData() {
  const geometry = buildPoints();
  return {
    mode: state.mode,
    P0: state.P0.toArray(),
    P1: state.P1.toArray(),
    P2: state.P2.toArray(),
    first: geometry.first.map((point) => point.toArray()),
    second: geometry.second.map((point) => point.toArray()),
    all: geometry.all.map((point) => point.toArray()),
    totalLength: state.totalLength,
  };
}

function publishPlannerPath() {
  if (!window.ftlPlanner) return;
  window.dispatchEvent(new CustomEvent('ftl-path-updated', { detail: getPlannerPathData() }));
}

function pointAtArcLength(s) {
  if (!state.pathPoints.length) return new THREE.Vector3();
  if (s <= 0) return state.pathPoints[0].clone();
  if (s >= state.totalLength) return state.pathPoints.at(-1).clone();
  let lo = 0, hi = state.cumulative.length - 1;
  while (lo + 1 < hi) {
    const mid = Math.floor((lo + hi) / 2);
    if (state.cumulative[mid] < s) lo = mid; else hi = mid;
  }
  const s0 = state.cumulative[lo];
  const s1 = state.cumulative[hi];
  const t = (s - s0) / Math.max(EPS, s1 - s0);
  return state.pathPoints[lo].clone().lerp(state.pathPoints[hi], t);
}

function bodyPointsAtProgress(progress) {
  const headS = clamp(progress, 0, 1) * state.totalLength;
  const bodyLength = Math.max(1, Number(ui.bodyLength.value) || 140);
  const tailS = Math.max(0, headS - bodyLength);
  const result = [pointAtArcLength(tailS)];
  for (let i = 0; i < state.cumulative.length; i += 1) {
    if (state.cumulative[i] > tailS && state.cumulative[i] < headS) result.push(state.pathPoints[i].clone());
  }
  result.push(pointAtArcLength(headS));
  return result;
}

function updateFTL() {
  const progress = Number(ui.progress.value) / 1000;
  const bodyPoints = bodyPointsAtProgress(progress);
  updateLine(bodyLine, bodyPoints);
  tipMesh.position.copy(bodyPoints.at(-1) || state.P0);
  ui.progressOut.value = `${Math.round(progress * 100)}%`;
}

function updateCentersAndRadii() {
  const show = ui.showGeometry.checked;
  const c1 = isLineArcMode() ? null : circleFromStartTangentToPoint(state.P0, state.T0, state.P1);
  const c2 = circleFromStartTangentToPoint(state.P2, state.T2.clone().negate(), state.P1);

  center1.visible = show && !!c1?.center;
  radius1a.visible = center1.visible;
  radius1b.visible = center1.visible;
  if (center1.visible) {
    center1.position.copy(c1.center);
    updateLine(radius1a, [c1.center, state.P0]);
    updateLine(radius1b, [c1.center, state.P1]);
  }

  center2.visible = show && !!c2.center;
  radius2a.visible = center2.visible;
  radius2b.visible = center2.visible;
  if (center2.visible) {
    center2.position.copy(c2.center);
    updateLine(radius2a, [c2.center, state.P1]);
    updateLine(radius2b, [c2.center, state.P2]);
  }
}

function redraw() {
  const geometry = buildPoints();
  updateLine(line1, geometry.first);
  updateLine(line2, geometry.second);
  updateLine(fullPathLine, geometry.all);
  cachePath(geometry.all);
  publishPlannerPath();

  p0Mesh.position.copy(state.P0);
  p1Mesh.position.copy(state.P1);
  p2Mesh.position.copy(state.P2);
  if (transform.object === p1Mesh) transform.object.position.copy(state.P1);
  if (transform.object === p2Mesh) transform.object.position.copy(state.P2);

  const parts = residual(state.P1);
  updateArrow(arrowT0, state.P0, state.T0);
  updateArrow(arrowT1, state.P1, parts.T1end);
  updateArrow(arrowT2, state.P2, state.T2);
  arrowT0.visible = arrowT1.visible = arrowT2.visible = ui.showGeometry.checked;
  updateCentersAndRadii();
  updateDesignPlane();
  updateFTL();
  redrawStatus(parts);
}

function redrawStatus(parts = residual(state.P1)) {
  const s1State = parts.s1 >= state.smin && parts.s1 <= state.smax ? 'OK' : 'OUT';
  const s2State = parts.s2 >= state.smin && parts.s2 <= state.smax ? 'OK' : 'OUT';
  ui.status.textContent = [
    `Mode      : ${state.mode}`,
    `Selected  : ${state.selectedHandle || 'click P1 / P2'}`,
    `P1        : [${fmt(state.P1.x)}, ${fmt(state.P1.y)}, ${fmt(state.P1.z)}]`,
    `P2        : [${fmt(state.P2.x)}, ${fmt(state.P2.y)}, ${fmt(state.P2.z)}]`,
    `s1 / s2   : ${fmt(parts.s1)} (${s1State}) / ${fmt(parts.s2)} (${s2State})`,
    `Residual  : ${state.lastSolve.rnorm.toExponential(3)}`,
    `Solver    : ${state.lastSolve.status}, ${state.lastSolve.iters} iter`,
    `Path      : ${fmt(state.totalLength)} mm, ${state.pathPoints.length} samples`,
  ].join('\n');
}

function fitCamera() {
  const box = new THREE.Box3().setFromPoints([state.P0, state.P1, state.P2, ...state.pathPoints]);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const span = Math.max(size.x, size.y, size.z, 120);
  orbit.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(1.45 * span, -1.65 * span, 1.25 * span));
  camera.near = Math.max(0.1, span / 1000);
  camera.far = span * 15;
  camera.updateProjectionMatrix();
  orbit.update();
}

// ---------- UI ----------
function applyPreset(mode, fit = true) {
  state.mode = mode;
  const p = presets[mode];
  state.P0.copy(vec(p.P0));
  state.P1.copy(vec(p.P1));
  state.P2.copy(vec(p.P2));
  state.T0.copy(vec(p.T0)).normalize();
  state.T2.copy(vec(p.T2)).normalize();
  state.planeN.copy(vec(p.planeN)).normalize();
  if (isCoplanarMode()) {
    state.T2.copy(projectVectorToPlane(state.T2, state.planeN));
    state.P1.copy(projectPointToPlane(state.P1, state.P0, state.planeN));
    state.P2.copy(projectPointToPlane(state.P2, state.P0, state.planeN));
  }
  applyP1Constraint();
  transform.detach();
  state.selectedHandle = null;
  solveP1(state.maxGN);
  syncUIFromState();
  redraw();
  if (fit) fitCamera();
}

function syncUIFromState(syncAngles = true) {
  ui.mode.value = state.mode;
  ui.p2x.value = fmt(state.P2.x, 1);
  ui.p2y.value = fmt(state.P2.y, 1);
  ui.p2z.value = fmt(state.P2.z, 1);
  ui.smin.value = state.smin;
  ui.smax.value = state.smax;
  if (syncAngles) {
    const yaw = Math.atan2(state.T2.y, state.T2.x) / DEG;
    const pitch = Math.asin(clamp(state.T2.z, -1, 1)) / DEG;
    ui.yaw.value = Math.round(yaw);
    ui.pitch.value = Math.round(pitch);
  }
  ui.yawOut.value = `${ui.yaw.value}°`;
  ui.pitchOut.value = `${ui.pitch.value}°`;
}

function updateT2FromAngles() {
  const yaw = Number(ui.yaw.value) * DEG;
  const pitch = Number(ui.pitch.value) * DEG;
  state.T2.set(Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)).normalize();
  if (isCoplanarMode()) state.T2.copy(projectVectorToPlane(state.T2, state.planeN));
  solveP1(state.refineIters);
  syncUIFromState();
  redraw();
}

ui.mode.addEventListener('change', () => applyPreset(ui.mode.value));
ui.reset.addEventListener('click', () => applyPreset(state.mode));
ui.refine.addEventListener('click', () => { solveP1(state.refineIters); syncUIFromState(); redraw(); });
[ui.p2x, ui.p2y, ui.p2z].forEach((input) => input.addEventListener('change', () => {
  state.P2.set(Number(ui.p2x.value), Number(ui.p2y.value), Number(ui.p2z.value));
  applyP2Constraint();
  solveP1(state.refineIters);
  syncUIFromState();
  redraw();
}));
ui.yaw.addEventListener('input', updateT2FromAngles);
ui.pitch.addEventListener('input', updateT2FromAngles);
ui.smin.addEventListener('change', updateLengthBounds);
ui.smax.addEventListener('change', updateLengthBounds);
ui.showGeometry.addEventListener('change', redraw);
ui.bodyLength.addEventListener('input', updateFTL);
ui.progress.addEventListener('input', () => { state.playing = false; ui.play.textContent = 'Play FTL'; updateFTL(); });
ui.play.addEventListener('click', () => {
  if (Number(ui.progress.value) >= 1000) ui.progress.value = 0;
  state.playing = !state.playing;
  state.lastAnimationTime = performance.now();
  ui.play.textContent = state.playing ? 'Pause' : 'Play FTL';
});
ui.restart.addEventListener('click', () => {
  state.playing = false;
  ui.play.textContent = 'Play FTL';
  ui.progress.value = 0;
  updateFTL();
});
ui.csv.addEventListener('click', exportCSV);
ui.png.addEventListener('click', exportPNG);

function updateLengthBounds() {
  const smin = Math.max(1, Number(ui.smin.value) || 45);
  const smax = Math.max(smin + 1, Number(ui.smax.value) || 150);
  state.smin = smin;
  state.smax = smax;
  applyP1Constraint();
  solveP1(state.refineIters);
  syncUIFromState();
  redraw();
}

function exportCSV() {
  const rows = ['x_mm,y_mm,z_mm,type'];
  state.pathPoints.forEach((p) => rows.push(`${p.x},${p.y},${p.z},trajectory`));
  rows.push(`${state.P1.x},${state.P1.y},${state.P1.z},P1`);
  rows.push(`${state.P2.x},${state.P2.y},${state.P2.z},P2`);
  downloadBlob(rows.join('\n'), 'ftl_shape_trajectory.csv', 'text/csv');
}

function exportPNG() {
  renderer.render(scene, camera);
  const a = document.createElement('a');
  a.download = 'ftl_shape_planning.png';
  a.href = renderer.domElement.toDataURL('image/png');
  a.click();
}

function downloadBlob(content, filename, type) {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

function resize() {
  const width = viewport.clientWidth;
  const height = viewport.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / Math.max(1, height);
  camera.updateProjectionMatrix();
}

function animate(now) {
  requestAnimationFrame(animate);
  resize();
  orbit.update();

  if (state.playing) {
    const dt = Math.min(0.05, (now - state.lastAnimationTime) / 1000);
    state.lastAnimationTime = now;
    const next = Number(ui.progress.value) + dt * 220;
    if (next >= 1000) {
      ui.progress.value = 1000;
      state.playing = false;
      ui.play.textContent = 'Play FTL';
    } else {
      ui.progress.value = next;
    }
    updateFTL();
  }
  renderer.render(scene, camera);
}

function clamp(value, min, max) { return Math.min(max, Math.max(min, value)); }
function norm(values) { return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0)); }
function dot(a, b) { return a.reduce((sum, value, i) => sum + value * b[i], 0); }
function fmt(value, digits = 2) { return Number.isFinite(value) ? Number(value).toFixed(digits) : 'NaN'; }

window.ftlPlanner = {
  getPathData: getPlannerPathData,
  sync: publishPlannerPath,
};
window.dispatchEvent(new Event('ftl-planner-ready'));
window.addEventListener('resize', resize);
applyPreset(state.mode);
requestAnimationFrame(animate);
