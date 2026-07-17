import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const EPS = 1e-9;
const D = 11;
const viewport = document.getElementById('simulationViewport');

const ui = {
  pathName: document.getElementById('simPathName'),
  pathMeta: document.getElementById('simPathMeta'),
  sync: document.getElementById('simSyncBtn'),
  waypointCount: document.getElementById('simWaypointCount'),
  waypointOut: document.getElementById('simWaypointOut'),
  iterations: document.getElementById('simIterations'),
  iterationsOut: document.getElementById('simIterationsOut'),
  lower: document.getElementById('simLowerBound'),
  upper: document.getElementById('simUpperBound'),
  delta: document.getElementById('simDelta'),
  nearestStart: document.getElementById('simNearestStart'),
  showFrames: document.getElementById('simShowFrames'),
  showProjections: document.getElementById('simShowProjections'),
  reset: document.getElementById('simResetBtn'),
  run: document.getElementById('simRunBtn'),
  status: document.getElementById('simStatus'),
  frame: document.getElementById('simFrameRange'),
  frameOut: document.getElementById('simFrameOut'),
  speed: document.getElementById('simSpeed'),
  speedOut: document.getElementById('simSpeedOut'),
  play: document.getElementById('simPlayBtn'),
  export: document.getElementById('simExportBtn'),
  stageBadge: document.getElementById('simStageBadge'),
  frameTitle: document.getElementById('simFrameTitle'),
  tipMetric: document.getElementById('simTipMetric'),
  shapeMetric: document.getElementById('simShapeMetric'),
  timeMetric: document.getElementById('simTimeMetric'),
  framesMetric: document.getElementById('simFramesMetric'),
  rods: [1, 2, 3, 4, 5, 6].map((index) => document.getElementById(`rodL${index}`)),
};

const sim = {
  path: null,
  frames: [],
  currentFrame: 0,
  playing: false,
  busy: false,
  lastTime: performance.now(),
  generation: 0,
  axisRange: 140,
  metrics: null,
};

// ---------- Scene ----------
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b1513);
scene.fog = new THREE.Fog(0x0b1513, 520, 1050);

const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 3000);
camera.up.set(0, 0, 1);
camera.position.set(330, -390, 300);

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.appendChild(renderer.domElement);

const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.dampingFactor = 0.06;
orbit.target.set(0, 0, 100);

scene.add(new THREE.HemisphereLight(0xe9fff8, 0x13241f, 2.4));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
keyLight.position.set(250, -180, 420);
scene.add(keyLight);
const rimLight = new THREE.DirectionalLight(0x5ee1c0, 0.8);
rimLight.position.set(-250, 250, 180);
scene.add(rimLight);

const grid = new THREE.GridHelper(520, 20, 0x34534b, 0x1b302b);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

const targetArc1 = makeLine(0xa7b7b1, 2, 0.72);
const targetArc2 = makeLine(0x6f8981, 2, 0.82);
const robotSeg1 = makeTube(0x3f8ed0, 4.8);
const robotSeg2 = makeTube(0xbd3e52, 4.8);
const projectionLines = [
  makeLine(0xa9b8b3, 2, 0.30),
  makeLine(0xa9b8b3, 2, 0.30),
  makeLine(0x8fa29c, 2, 0.24),
  makeLine(0x8fa29c, 2, 0.24),
];
scene.add(targetArc1, targetArc2, robotSeg1, robotSeg2, ...projectionLines);

const basePoint = makePoint(0xe7f4f0, 4.5);
const transitionPoint = makePoint(0x60c7a8, 5.5);
const endpoint = makePoint(0xf09a57, 5.5);
const currentMid = makePoint(0x5aa0dd, 5.2);
const currentTip = makePoint(0xe55368, 6.0);
const targetMid = makePoint(0x74dfb6, 4.5);
const targetTip = makePoint(0xffc26b, 4.5);
scene.add(basePoint, transitionPoint, endpoint, currentMid, currentTip, targetMid, targetTip);

const baseFrame = new THREE.AxesHelper(18);
const middleFrame = new THREE.AxesHelper(18);
const tipFrame = new THREE.AxesHelper(18);
scene.add(baseFrame, middleFrame, tipFrame);

// ---------- Path synchronization ----------
function arrayToVec(values) {
  return new THREE.Vector3(Number(values[0]), Number(values[1]), Number(values[2]));
}

function setPath(pathData, fit = true) {
  if (!pathData?.first?.length || !pathData?.second?.length) return;
  sim.generation += 1;
  sim.path = {
    mode: pathData.mode || 'custom',
    P0: arrayToVec(pathData.P0),
    P1: arrayToVec(pathData.P1),
    P2: arrayToVec(pathData.P2),
    first: pathData.first.map(arrayToVec),
    second: pathData.second.map(arrayToVec),
    all: pathData.all.map(arrayToVec),
  };

  updateLine(targetArc1, sim.path.first);
  updateLine(targetArc2, sim.path.second);
  basePoint.position.copy(sim.path.P0);
  transitionPoint.position.copy(sim.path.P1);
  endpoint.position.copy(sim.path.P2);
  sim.axisRange = calculateAxisRange(sim.path.all);

  ui.pathName.textContent = humanMode(sim.path.mode);
  ui.pathMeta.textContent = `${sim.path.all.length} samples · ${format(polylineLength(sim.path.all), 1)} mm`;
  clearPlan('Reference path synchronized. Run the three-stage planner to generate robot motion.');
  showInitialRobot();
  if (fit) fitCamera();
}

function syncFromPlanner(fit = true) {
  if (!window.ftlPlanner?.getPathData) {
    ui.status.textContent = 'The shape-planner module is not ready yet.';
    return;
  }
  setPath(window.ftlPlanner.getPathData(), fit);
}

window.addEventListener('ftl-path-updated', (event) => setPath(event.detail, false));
window.addEventListener('ftl-planner-ready', () => syncFromPlanner(true));

// ---------- Constant-curvature forward model ----------
function segmentKinematics(lengths, samples = 50) {
  const [l1, l2, l3] = lengths;
  const sumL = Math.max(EPS, l1 + l2 + l3);
  const ds = sumL / 3;
  const term = Math.max(0, l1*l1 + l2*l2 + l3*l3 - (l1*l2 + l2*l3 + l3*l1));
  const kappa = Math.max(2 * Math.sqrt(term) / (D * sumL), 1e-6);
  const phi = Math.atan2(3 * (l2 - l3), -Math.sqrt(3) * (l2 + l3 - 2*l1));
  const thetaEnd = kappa * ds;
  const points = [];

  for (let index = 0; index < samples; index += 1) {
    const theta = thetaEnd * index / Math.max(1, samples - 1);
    points.push(new THREE.Vector3(
      (1 / kappa) * (1 - Math.cos(theta)) * Math.cos(phi),
      (1 / kappa) * (1 - Math.cos(theta)) * Math.sin(phi),
      (1 / kappa) * Math.sin(theta),
    ));
  }

  const rotation = new THREE.Matrix4()
    .makeRotationZ(phi)
    .multiply(new THREE.Matrix4().makeRotationY(thetaEnd))
    .multiply(new THREE.Matrix4().makeRotationZ(-phi));

  return {
    points,
    end: points.at(-1).clone(),
    rotation,
    kappa,
    phi,
    theta: thetaEnd,
    length: ds,
  };
}

function forwardKinematics(lengths, samples = 50) {
  const segment1 = segmentKinematics(lengths.slice(0, 3), samples);
  const segment2Local = segmentKinematics(lengths.slice(3, 6), samples);
  const p01 = segment1.end.clone();
  const segment2Points = segment2Local.points.map((point) => point.clone().applyMatrix4(segment1.rotation).add(p01));
  const rotation02 = segment1.rotation.clone().multiply(segment2Local.rotation);

  return {
    segment1: segment1.points,
    segment2: segment2Points,
    p01,
    p02: segment2Points.at(-1).clone(),
    R01: segment1.rotation,
    R02: rotation02,
  };
}

// ---------- Three-stage planner ----------
function varianceResiduals(values, weight) {
  const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
  const scale = Math.sqrt(weight / Math.max(1, values.length - 1));
  return values.map((value) => (value - mean) * scale);
}

function vectorResidual(actual, target, weight = 1) {
  const scale = Math.sqrt(weight);
  return [
    (actual.x - target.x) * scale,
    (actual.y - target.y) * scale,
    (actual.z - target.z) * scale,
  ];
}

function smoothResidual(current, previous, weight) {
  const scale = Math.sqrt(weight);
  return current.map((value, index) => (value - previous[index]) * scale);
}

function stage1Residual(lengths, target, previous) {
  const fk = forwardKinematics(lengths, 26);
  return [
    ...vectorResidual(fk.p02, target),
    ...smoothResidual(lengths, previous, 0.2),
    ...varianceResiduals(lengths.slice(0, 3), 0.02),
    ...varianceResiduals(lengths.slice(3, 6), 0.02),
  ];
}

function stage2Residual(lengths, target1, target2, previous) {
  const fk = forwardKinematics(lengths, 26);
  return [
    ...vectorResidual(fk.p01, target1),
    ...vectorResidual(fk.p02, target2),
    ...smoothResidual(lengths, previous, 0.2),
    ...varianceResiduals(lengths.slice(0, 3), 0.02),
    ...varianceResiduals(lengths.slice(3, 6), 0.02),
  ];
}

function stage3Residual(secondLengths, fixedFirst, target2, previous) {
  const lengths = [...fixedFirst, ...secondLengths];
  const fk = forwardKinematics(lengths, 26);
  return [
    ...vectorResidual(fk.p02, target2),
    ...smoothResidual(secondLengths, previous, 0.2),
    ...varianceResiduals(secondLengths, 0.02),
  ];
}

function boundedLM(start, residualFunction, lower, upper, maxIterations) {
  let x = start.map((value, index) => clamp(value, lower[index], upper[index]));
  let lambda = 1e-2;
  const h = 1e-3;
  const stepCap = 12;

  for (let iteration = 0; iteration < maxIterations; iteration += 1) {
    const residual = residualFunction(x);
    const baseCost = 0.5 * dot(residual, residual);
    if (!Number.isFinite(baseCost) || Math.sqrt(2 * baseCost) < 1e-5) break;

    const rows = residual.length;
    const cols = x.length;
    const jacobian = Array.from({ length: rows }, () => Array(cols).fill(0));
    for (let column = 0; column < cols; column += 1) {
      const trial = x.slice();
      trial[column] = clamp(trial[column] + h, lower[column], upper[column]);
      const actualStep = trial[column] - x[column];
      if (Math.abs(actualStep) < EPS) continue;
      const trialResidual = residualFunction(trial);
      for (let row = 0; row < rows; row += 1) {
        jacobian[row][column] = (trialResidual[row] - residual[row]) / actualStep;
      }
    }

    const hessian = Array.from({ length: cols }, () => Array(cols).fill(0));
    const gradient = Array(cols).fill(0);
    for (let a = 0; a < cols; a += 1) {
      for (let b = 0; b < cols; b += 1) {
        for (let row = 0; row < rows; row += 1) hessian[a][b] += jacobian[row][a] * jacobian[row][b];
        if (a === b) hessian[a][b] += lambda;
      }
      for (let row = 0; row < rows; row += 1) gradient[a] += jacobian[row][a] * residual[row];
    }

    let step = solveLinear(hessian, gradient.map((value) => -value));
    if (!step || step.some((value) => !Number.isFinite(value))) {
      step = gradient.map((value, index) => -value / Math.max(Math.abs(hessian[index][index]), 1e-8));
    }
    const stepNorm = norm(step);
    if (stepNorm > stepCap) step = step.map((value) => value * stepCap / stepNorm);

    let alpha = 1;
    let accepted = false;
    let candidate = x;
    let candidateCost = baseCost;
    while (alpha > 1e-5) {
      candidate = x.map((value, index) => clamp(value + alpha * step[index], lower[index], upper[index]));
      const candidateResidual = residualFunction(candidate);
      candidateCost = 0.5 * dot(candidateResidual, candidateResidual);
      if (candidateCost < baseCost) {
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }

    if (accepted) {
      x = candidate;
      lambda = Math.max(lambda / 2.5, 1e-9);
      if (norm(step) * alpha < 1e-5 || Math.abs(baseCost - candidateCost) < 1e-7) break;
    } else {
      lambda = Math.min(lambda * 8, 1e8);
      if (lambda >= 1e8) break;
    }
  }
  return x;
}

async function runSimulation() {
  if (sim.busy) return;
  if (!sim.path) syncFromPlanner(false);
  if (!sim.path) return;

  const lowerValue = Math.max(1, Number(ui.lower.value) || 45);
  const upperValue = Math.max(lowerValue + 1, Number(ui.upper.value) || 200);
  const delta = Math.max(0.5, Number(ui.delta.value) || 8);
  const maxIterations = Math.max(1, Number(ui.iterations.value) || 18);
  const waypointCount = Math.max(14, Number(ui.waypointCount.value) || 32);
  const generation = sim.generation;

  ui.lower.value = lowerValue;
  ui.upper.value = upperValue;
  ui.delta.value = delta;
  sim.busy = true;
  sim.playing = false;
  ui.run.disabled = true;
  ui.play.disabled = true;
  ui.export.disabled = true;
  ui.run.textContent = 'Planning…';
  clearMetrics();

  const startTime = performance.now();
  const firstCount = Math.max(7, Math.round(waypointCount / 2));
  const secondCount = Math.max(7, waypointCount - firstCount + 1);
  const arc1 = resamplePolyline(sim.path.first, firstCount);
  const arc2 = resamplePolyline(sim.path.second, secondCount);
  const fullPath = [...arc1, ...arc2.slice(1)];
  let lengths = Array(6).fill(lowerValue);
  const frames = [];
  let completedSteps = 0;

  const updatePlanningPreview = async (frame, message) => {
    frames.push(frame);
    completedSteps += 1;
    sim.frames = frames;
    sim.currentFrame = frames.length - 1;
    renderFrame(frame, frames.length - 1, false);
    ui.status.textContent = `${message}\nGenerated ${frames.length} motion frames.`;
    // Keep the live planning preview readable instead of flashing through all frames.
    await delayBrowser(70);
    if (generation !== sim.generation) throw new Error('Reference path changed during planning.');
  };

  try {
    // Stage 1: p02 tracks arc 1.
    const initialFK = forwardKinematics(lengths, 36);
    const stage1Start = ui.nearestStart.checked ? nearestIndex(arc1, initialFK.p02) : 0;
    for (let index = stage1Start; index < arc1.length; index += 1) {
      const target2 = arc1[index];
      const previous = lengths.slice();
      const lower = previous.map((value) => Math.max(lowerValue, value - delta));
      const upper = previous.map((value) => Math.min(upperValue, value + delta));
      lengths = boundedLM(lengths, (candidate) => stage1Residual(candidate, target2, previous), lower, upper, maxIterations);
      const frame = makeFrame(lengths, 1, null, target2, fullPath, `p02 → arc 1 · ${index + 1}/${arc1.length}`);
      await updatePlanningPreview(frame, `Stage 1 · full tip p02 follows arc 1 (${index + 1}/${arc1.length})`);
    }

    // Stage 2: p01 completes arc 1 while p02 enters arc 2.
    const stage1EndFK = forwardKinematics(lengths, 36);
    const arc1StartIndex = nearestIndex(arc1, stage1EndFK.p01);
    const arc1Remain = arc1.slice(arc1StartIndex);
    let lastArc2Index = 0;
    for (let index = 0; index < arc1Remain.length; index += 1) {
      const target1 = arc1Remain[index];
      const target2Index = Math.min(index, arc2.length - 1);
      const target2 = arc2[target2Index];
      lastArc2Index = target2Index;
      const previous = lengths.slice();
      const lower = previous.map((value) => Math.max(lowerValue, value - delta));
      const upper = previous.map((value) => Math.min(upperValue, value + delta));
      lengths = boundedLM(lengths, (candidate) => stage2Residual(candidate, target1, target2, previous), lower, upper, maxIterations);
      const frame = makeFrame(lengths, 2, target1, target2, fullPath, `p01 → arc 1 · p02 → arc 2 · ${index + 1}/${arc1Remain.length}`);
      await updatePlanningPreview(frame, `Stage 2 · p01 and p02 track both primitives (${index + 1}/${arc1Remain.length})`);
    }

    // Stage 3: freeze first segment, finish arc 2 with L4–L6.
    const fixedFirst = lengths.slice(0, 3);
    for (let index = lastArc2Index + 1; index < arc2.length; index += 1) {
      const target2 = arc2[index];
      const previousSecond = lengths.slice(3, 6);
      const lower = previousSecond.map((value) => Math.max(lowerValue, value - delta));
      const upper = previousSecond.map((value) => Math.min(upperValue, value + delta));
      const solvedSecond = boundedLM(
        previousSecond,
        (candidate) => stage3Residual(candidate, fixedFirst, target2, previousSecond),
        lower,
        upper,
        maxIterations,
      );
      lengths = [...fixedFirst, ...solvedSecond];
      const frame = makeFrame(lengths, 3, sim.path.P1, target2, fullPath, `L1–L3 fixed · p02 → arc 2 · ${index + 1}/${arc2.length}`);
      await updatePlanningPreview(frame, `Stage 3 · second segment finishes arc 2 (${index + 1}/${arc2.length})`);
    }

    if (!frames.length) {
      frames.push(makeFrame(lengths, 0, null, sim.path.P0, fullPath, 'Initial configuration'));
    }

    const elapsed = performance.now() - startTime;
    sim.frames = frames;
    sim.metrics = summarizeMetrics(frames, elapsed);
    sim.currentFrame = 0;
    configurePlayback();
    renderFrame(frames[0], 0, true);
    renderMetrics();
    ui.status.textContent = [
      'Motion plan complete.',
      `Stage 1 start index: ${stage1Start + 1}/${arc1.length}`,
      `Frames: ${frames.length}`,
      `Planning time: ${format(elapsed / 1000, 2)} s`,
    ].join('\n');
  } catch (error) {
    ui.status.textContent = `Planning stopped: ${error.message}`;
    clearPlan(ui.status.textContent);
  } finally {
    sim.busy = false;
    ui.run.disabled = false;
    ui.run.textContent = 'Run simulation';
  }
}

function makeFrame(lengths, stage, target1, target2, fullPath, label) {
  const fk = forwardKinematics(lengths, 50);
  const robotShape = [...fk.segment1, ...fk.segment2.slice(1)];
  const tipError = target2 ? fk.p02.distanceTo(target2) : 0;
  const shapeError = mean(robotShape.map((point) => pointToPolylineDistance(point, fullPath)));
  return {
    lengths: lengths.slice(),
    stage,
    target1: target1?.clone() || null,
    target2: target2?.clone() || null,
    label,
    segment1: fk.segment1.map((point) => point.clone()),
    segment2: fk.segment2.map((point) => point.clone()),
    p01: fk.p01.clone(),
    p02: fk.p02.clone(),
    R01: fk.R01.clone(),
    R02: fk.R02.clone(),
    tipError,
    shapeError,
  };
}

// ---------- Drawing and playback ----------
function renderFrame(frame, index, updateSlider = true) {
  if (!frame) return;
  updateLine(robotSeg1, frame.segment1);
  updateLine(robotSeg2, frame.segment2);
  currentMid.position.copy(frame.p01);
  currentTip.position.copy(frame.p02);
  targetMid.visible = !!frame.target1;
  if (frame.target1) targetMid.position.copy(frame.target1);
  targetTip.visible = !!frame.target2;
  if (frame.target2) targetTip.position.copy(frame.target2);

  setFrameTransform(baseFrame, new THREE.Vector3(), new THREE.Matrix4().identity());
  setFrameTransform(middleFrame, frame.p01, frame.R01);
  setFrameTransform(tipFrame, frame.p02, frame.R02);
  baseFrame.visible = middleFrame.visible = tipFrame.visible = ui.showFrames.checked;
  updateProjections(frame);

  ui.stageBadge.textContent = frame.stage ? `STAGE ${frame.stage}` : 'INITIAL';
  ui.frameTitle.textContent = frame.label;
  ui.frameOut.value = `${index + 1} / ${sim.frames.length}`;
  frame.lengths.forEach((value, rodIndex) => {
    ui.rods[rodIndex].value = `L${rodIndex + 1} ${format(value, 2)} mm`;
  });

  if (updateSlider) {
    ui.frame.value = index;
    sim.currentFrame = index;
  }
}

function updateProjections(frame) {
  const show = ui.showProjections.checked;
  projectionLines.forEach((line) => { line.visible = show; });
  if (!show) return;
  const a = sim.axisRange;
  updateLine(projectionLines[0], frame.segment1.map((point) => new THREE.Vector3(a, point.y, point.z)));
  updateLine(projectionLines[1], frame.segment2.map((point) => new THREE.Vector3(a, point.y, point.z)));
  updateLine(projectionLines[2], frame.segment1.map((point) => new THREE.Vector3(point.x, -a, point.z)));
  updateLine(projectionLines[3], frame.segment2.map((point) => new THREE.Vector3(point.x, -a, point.z)));
}

function showInitialRobot() {
  const lower = Math.max(1, Number(ui.lower.value) || 45);
  const initial = makeFrame(Array(6).fill(lower), 0, null, null, sim.path?.all || [], 'Initial rod configuration');
  updateLine(robotSeg1, initial.segment1);
  updateLine(robotSeg2, initial.segment2);
  currentMid.position.copy(initial.p01);
  currentTip.position.copy(initial.p02);
  targetMid.visible = false;
  targetTip.visible = false;
  setFrameTransform(baseFrame, new THREE.Vector3(), new THREE.Matrix4().identity());
  setFrameTransform(middleFrame, initial.p01, initial.R01);
  setFrameTransform(tipFrame, initial.p02, initial.R02);
  baseFrame.visible = middleFrame.visible = tipFrame.visible = ui.showFrames.checked;
  initial.lengths.forEach((value, index) => { ui.rods[index].value = `L${index + 1} ${format(value, 2)} mm`; });
  updateProjections(initial);
}

function configurePlayback() {
  ui.frame.disabled = false;
  ui.frame.min = 0;
  ui.frame.max = Math.max(0, sim.frames.length - 1);
  ui.frame.value = 0;
  ui.play.disabled = false;
  ui.export.disabled = false;
  ui.play.textContent = 'Play';
  sim.playing = false;
}

function clearPlan(message = 'No motion plan. Run the simulator.') {
  sim.frames = [];
  sim.metrics = null;
  sim.currentFrame = 0;
  sim.playing = false;
  ui.frame.disabled = true;
  ui.frame.min = 0;
  ui.frame.max = 0;
  ui.frame.value = 0;
  ui.frameOut.value = '— / —';
  ui.play.disabled = true;
  ui.export.disabled = true;
  ui.play.textContent = 'Play';
  ui.stageBadge.textContent = 'IDLE';
  ui.frameTitle.textContent = 'No motion plan';
  ui.status.textContent = message;
  clearMetrics();
}

function clearMetrics() {
  ui.tipMetric.textContent = '—';
  ui.shapeMetric.textContent = '—';
  ui.timeMetric.textContent = '—';
  ui.framesMetric.textContent = '—';
}

function summarizeMetrics(frames, elapsed) {
  const tip = frames.map((frame) => frame.tipError);
  const shape = frames.map((frame) => frame.shapeError);
  return {
    tipMean: mean(tip),
    tipMax: Math.max(...tip),
    shapeMean: mean(shape),
    shapeMax: Math.max(...shape),
    elapsed,
    frames: frames.length,
  };
}

function renderMetrics() {
  if (!sim.metrics) return;
  ui.tipMetric.textContent = `${format(sim.metrics.tipMean, 2)} / ${format(sim.metrics.tipMax, 2)} mm`;
  ui.shapeMetric.textContent = `${format(sim.metrics.shapeMean, 2)} / ${format(sim.metrics.shapeMax, 2)} mm`;
  ui.timeMetric.textContent = `${format(sim.metrics.elapsed / 1000, 2)} s`;
  ui.framesMetric.textContent = String(sim.metrics.frames);
}

function updatePlayback(now) {
  if (!sim.playing || !sim.frames.length) return;
  const deltaSeconds = Math.min(0.05, (now - sim.lastTime) / 1000);
  sim.lastTime = now;
  const speed = Number(ui.speed.value) || 1;
  // Base playback is intentionally slow enough to inspect body-shape evolution.
  const next = sim.currentFrame + deltaSeconds * 8 * speed;
  if (next >= sim.frames.length - 1) {
    sim.currentFrame = sim.frames.length - 1;
    sim.playing = false;
    ui.play.textContent = 'Play';
  } else {
    sim.currentFrame = next;
  }
  const frameIndex = Math.floor(sim.currentFrame);
  renderFrame(sim.frames[frameIndex], frameIndex, true);
}

function exportPlan() {
  if (!sim.frames.length) return;
  const rows = ['frame,stage,L1_mm,L2_mm,L3_mm,L4_mm,L5_mm,L6_mm,q1_mm,q2_mm,q3_mm,q4_mm,q5_mm,q6_mm,tip_error_mm,shape_error_mm'];
  sim.frames.forEach((frame, index) => {
    const q = computeQ(frame.lengths);
    rows.push([
      index + 1,
      frame.stage,
      ...frame.lengths.map((value) => value.toFixed(8)),
      ...q.map((value) => value.toFixed(8)),
      frame.tipError.toFixed(8),
      frame.shapeError.toFixed(8),
    ].join(','));
  });
  downloadBlob(rows.join('\n'), 'three_stage_ftl_L_plan.csv', 'text/csv');
}

function computeQ(L) {
  const q11 = L[0] - 45;
  const q12 = L[1] - 45;
  const q13 = L[2] - 45;
  const q14 = (L[0] + L[1]) / 2 - 45;
  const q15 = (L[1] + L[2]) / 2 - 45;
  const q16 = (L[0] + L[2]) / 2 - 45;
  const q24 = L[4] - 45;
  const q25 = L[5] - 45;
  const q26 = L[3] - 45;
  return [q11, q12, q13, q14 + q24, q15 + q25, q16 + q26];
}

// ---------- Geometry and numeric helpers ----------
function makeLine(color, width = 2, opacity = 1) {
  return new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color, linewidth: width, transparent: opacity < 1, opacity }),
  );
}

// WebGL commonly ignores LineBasicMaterial.linewidth. Use a true 3D tube for
// the robot body so segment thickness is consistent across browsers.
function makeTube(color, radius = 4.8) {
  const seedCurve = new THREE.LineCurve3(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, 0.001),
  );
  const mesh = new THREE.Mesh(
    new THREE.TubeGeometry(seedCurve, 2, radius, 12, false),
    new THREE.MeshStandardMaterial({
      color,
      roughness: 0.34,
      metalness: 0.05,
      emissive: color,
      emissiveIntensity: 0.08,
    }),
  );
  mesh.userData.isRobotTube = true;
  mesh.userData.radius = radius;
  return mesh;
}

function makePoint(color, radius) {
  return new THREE.Mesh(
    new THREE.SphereGeometry(radius, 22, 15),
    new THREE.MeshStandardMaterial({ color, roughness: 0.38, metalness: 0.04 }),
  );
}

function updateLine(line, points) {
  if (line.userData?.isRobotTube) {
    updateTube(line, points);
    return;
  }
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(points.length ? points : [new THREE.Vector3(), new THREE.Vector3()]);
  line.visible = points.length > 1;
}

function updateTube(mesh, points) {
  mesh.geometry.dispose();
  if (points.length < 2) {
    mesh.visible = false;
    return;
  }
  const curve = new THREE.CatmullRomCurve3(
    points.map((point) => point.clone()),
    false,
    'centripetal',
  );
  mesh.geometry = new THREE.TubeGeometry(
    curve,
    Math.max(40, points.length * 2),
    mesh.userData.radius,
    14,
    false,
  );
  mesh.visible = true;
}

function setFrameTransform(helper, position, rotation) {
  helper.position.copy(position);
  helper.quaternion.setFromRotationMatrix(rotation);
}

function polylineLength(points) {
  let length = 0;
  for (let index = 1; index < points.length; index += 1) length += points[index].distanceTo(points[index - 1]);
  return length;
}

function resamplePolyline(points, count) {
  if (points.length < 2 || count <= 2) return [points[0].clone(), points.at(-1).clone()];
  const cumulative = [0];
  for (let index = 1; index < points.length; index += 1) cumulative.push(cumulative.at(-1) + points[index].distanceTo(points[index - 1]));
  const total = cumulative.at(-1);
  const output = [];
  for (let sample = 0; sample < count; sample += 1) {
    const target = total * sample / Math.max(1, count - 1);
    let high = 1;
    while (high < cumulative.length && cumulative[high] < target) high += 1;
    high = Math.min(high, cumulative.length - 1);
    const low = Math.max(0, high - 1);
    const fraction = (target - cumulative[low]) / Math.max(EPS, cumulative[high] - cumulative[low]);
    output.push(points[low].clone().lerp(points[high], fraction));
  }
  return output;
}

function nearestIndex(points, target) {
  let best = 0;
  let bestDistance = Infinity;
  points.forEach((point, index) => {
    const distance = point.distanceToSquared(target);
    if (distance < bestDistance) {
      bestDistance = distance;
      best = index;
    }
  });
  return best;
}

function pointToPolylineDistance(point, path) {
  if (!path.length) return 0;
  if (path.length === 1) return point.distanceTo(path[0]);
  let best = Infinity;
  for (let index = 1; index < path.length; index += 1) {
    best = Math.min(best, pointToSegmentDistance(point, path[index - 1], path[index]));
  }
  return best;
}

function pointToSegmentDistance(point, start, end) {
  const segment = end.clone().sub(start);
  const denominator = segment.lengthSq();
  if (denominator < EPS) return point.distanceTo(start);
  const t = clamp(point.clone().sub(start).dot(segment) / denominator, 0, 1);
  return point.distanceTo(start.clone().addScaledVector(segment, t));
}

function calculateAxisRange(points) {
  let maxAbs = 120;
  points.forEach((point) => { maxAbs = Math.max(maxAbs, Math.abs(point.x), Math.abs(point.y)); });
  return Math.ceil((maxAbs + 30) / 10) * 10;
}

function fitCamera() {
  if (!sim.path) return;
  const initial = forwardKinematics(Array(6).fill(Math.max(1, Number(ui.lower.value) || 45)), 30);
  const points = [...sim.path.all, ...initial.segment1, ...initial.segment2];
  const box = new THREE.Box3().setFromPoints(points);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const span = Math.max(size.x, size.y, size.z, 130);
  orbit.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(1.5 * span, -1.75 * span, 1.35 * span));
  camera.near = Math.max(0.1, span / 1000);
  camera.far = span * 18;
  camera.updateProjectionMatrix();
  orbit.update();
}

function solveLinear(A, b) {
  const n = b.length;
  const matrix = A.map((row, index) => [...row, b[index]]);
  for (let column = 0; column < n; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < n; row += 1) {
      if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) pivot = row;
    }
    if (Math.abs(matrix[pivot][column]) < 1e-12) return null;
    [matrix[column], matrix[pivot]] = [matrix[pivot], matrix[column]];
    const divisor = matrix[column][column];
    for (let index = column; index <= n; index += 1) matrix[column][index] /= divisor;
    for (let row = 0; row < n; row += 1) {
      if (row === column) continue;
      const factor = matrix[row][column];
      for (let index = column; index <= n; index += 1) matrix[row][index] -= factor * matrix[column][index];
    }
  }
  return matrix.map((row) => row[n]);
}

function humanMode(mode) {
  const labels = {
    J_lineArc: 'J · line–arc path',
    C_coplanar: 'C · coplanar biarc',
    C_free3d: 'C · spatial biarc',
    S_coplanar: 'S · coplanar biarc',
    S_free3d: 'S · spatial biarc',
  };
  return labels[mode] || mode;
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
  updatePlayback(now);
  renderer.render(scene, camera);
}

function nextBrowserFrame() {
  return new Promise((resolve) => requestAnimationFrame(resolve));
}

function delayBrowser(milliseconds) {
  return new Promise((resolve) => window.setTimeout(resolve, milliseconds));
}

function downloadBlob(content, filename, type) {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

function clamp(value, minimum, maximum) { return Math.min(maximum, Math.max(minimum, value)); }
function dot(a, b) { return a.reduce((sum, value, index) => sum + value * b[index], 0); }
function norm(values) { return Math.sqrt(dot(values, values)); }
function mean(values) { return values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0; }
function format(value, digits = 2) { return Number.isFinite(value) ? Number(value).toFixed(digits) : '—'; }

// ---------- UI ----------
ui.sync.addEventListener('click', () => syncFromPlanner(true));
ui.run.addEventListener('click', runSimulation);
ui.reset.addEventListener('click', () => {
  sim.generation += 1;
  clearPlan('Simulation reset. The current reference path is still available.');
  showInitialRobot();
  fitCamera();
});
ui.frame.addEventListener('input', () => {
  sim.playing = false;
  ui.play.textContent = 'Play';
  const index = Number(ui.frame.value);
  sim.currentFrame = index;
  renderFrame(sim.frames[index], index, false);
});
ui.play.addEventListener('click', () => {
  if (!sim.frames.length) return;
  if (Math.floor(sim.currentFrame) >= sim.frames.length - 1) sim.currentFrame = 0;
  sim.playing = !sim.playing;
  sim.lastTime = performance.now();
  ui.play.textContent = sim.playing ? 'Pause' : 'Play';
});
ui.export.addEventListener('click', exportPlan);
ui.showFrames.addEventListener('change', () => {
  if (sim.frames.length) renderFrame(sim.frames[Math.floor(sim.currentFrame)], Math.floor(sim.currentFrame), false);
  else showInitialRobot();
});
ui.showProjections.addEventListener('change', () => {
  if (sim.frames.length) renderFrame(sim.frames[Math.floor(sim.currentFrame)], Math.floor(sim.currentFrame), false);
  else showInitialRobot();
});
ui.waypointCount.addEventListener('input', () => { ui.waypointOut.value = ui.waypointCount.value; });
ui.iterations.addEventListener('input', () => { ui.iterationsOut.value = ui.iterations.value; });
ui.speed.addEventListener('input', () => { ui.speedOut.value = `${Number(ui.speed.value).toFixed(2).replace(/0$/, '')}×`; });
ui.lower.addEventListener('change', () => { if (!sim.frames.length) showInitialRobot(); });
window.addEventListener('resize', resize);

ui.waypointOut.value = ui.waypointCount.value;
ui.iterationsOut.value = ui.iterations.value;
ui.speedOut.value = `${Number(ui.speed.value).toFixed(2).replace(/0$/, '')}×`;
if (window.ftlPlanner?.getPathData) syncFromPlanner(true);
requestAnimationFrame(animate);
