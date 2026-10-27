/* Three.js is served locally; the control station has no CDN dependency. */
import * as THREE from '/vendor/three/three.module.min.js';
import { OrbitControls } from '/vendor/three-examples/controls/OrbitControls.js';
import { TransformControls } from '/vendor/three-examples/controls/TransformControls.js';

const $ = (id) => document.getElementById(id);
const canvas = $('map');
const ctx = canvas.getContext('2d');
const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
const pidHistory = [];
const held = new Set();
let state = null;
let route = null;
let selectedId = null;
let selectedNodeId = null;
let pointer = null;
let configRevision = null;
let telemetryReceivedAt = 0;
let lastUpdate = performance.now();
let frames = 0;
let webgl = null;
let renderingPaused = false;
let transformMode = 'translate';
let roomSignature = '';
let currentView = '3d';
let nodeOptionSignature = '';
let autoRouteSignature = '';
let autoRouteTimer = null;
let autoRouteRevision = 0;
const labelWorldPosition = new THREE.Vector3();

const numeric = (id) => Number($(id).value);
const finite = (value) => Number.isFinite(Number(value));

function toast(title, message = '', tone = 'info') {
  const item = document.createElement('div');
  item.className = `toast ${tone}`;
  item.innerHTML = `<div><strong>${escapeHtml(title)}</strong><small>${escapeHtml(message)}</small></div>`;
  $('toasts').append(item);
  setTimeout(() => item.remove(), 4400);
}

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>'"]/g, (character) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;',
  })[character]);
}

async function request(url, options = {}) {
  const response = await fetch(url, { headers: { 'Content-Type': 'application/json' }, ...options });
  if (!response.ok) {
    const detail = (await response.text()).replaceAll('"', '');
    throw new Error(detail || `${response.status} ${response.statusText}`);
  }
  return response.headers.get('content-type')?.includes('application/json') ? response.json() : null;
}

function obstacleList() { return state?.scene?.scene?.obstacles ?? []; }
function selected() { return obstacleList().find((obstacle) => obstacle.id === selectedId); }
function nodeList() { return state?.graph?.nodes ?? []; }
function selectedNode() { return nodeList().find((node) => Number(node.id) === Number(selectedNodeId)); }
function currentPose() {
  const telemetry = state?.telemetry ?? {};
  return telemetry.navigation_pose ?? telemetry.odometry_pose ?? telemetry.odometry ?? telemetry.pose ?? telemetry.position;
}

function disposeObject(object) {
  object.traverse((child) => {
    child.geometry?.dispose();
    const materials = child.material ? (Array.isArray(child.material) ? child.material : [child.material]) : [];
    materials.forEach((material) => { material.map?.dispose(); material.dispose(); });
  });
}

function clearGroup(group) {
  while (group.children.length) {
    const object = group.children.pop();
    disposeObject(object);
  }
}

function makeSegment(a, b, radius, color, y = 0.045) {
  const start = new THREE.Vector3(a[0], y, a[1]);
  const end = new THREE.Vector3(b[0], y, b[1]);
  const delta = end.clone().sub(start);
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, delta.length(), 10),
    new THREE.MeshStandardMaterial({ color, roughness: .7, metalness: .05 }),
  );
  mesh.position.copy(start).add(end).multiplyScalar(.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
  return mesh;
}

function makeRack(obstacle) {
  const group = new THREE.Group();
  const width = Number(obstacle.sx);
  const depth = Number(obstacle.sy);
  const height = Math.max(.8, Number(obstacle.sz ?? obstacle.z ?? 1.5));
  const selectedRack = obstacle.id === selectedId;
  const frameMaterial = new THREE.MeshStandardMaterial({ color: selectedRack ? 0x16c784 : 0x4a4a52, roughness: .58, metalness: .12 });
  const shelfMaterial = new THREE.MeshStandardMaterial({ color: selectedRack ? 0x0b8f62 : 0x6f5340, roughness: .76 });
  [0.12, height * .48, height - .12].forEach((level) => {
    const shelf = new THREE.Mesh(new THREE.BoxGeometry(width, .08, depth), shelfMaterial);
    shelf.position.y = level - height / 2;
    group.add(shelf);
  });
  [-1, 1].forEach((xSide) => [-1, 1].forEach((zSide) => {
    const post = new THREE.Mesh(new THREE.BoxGeometry(.055, height, .055), frameMaterial);
    post.position.set(xSide * (width / 2 - .04), 0, zSide * (depth / 2 - .04));
    group.add(post);
  }));
  const sign = new THREE.Mesh(new THREE.BoxGeometry(Math.min(width * .5, 1.35), .22, .05), frameMaterial);
  sign.position.set(0, height / 2 + .15, 0);
  group.add(sign);
  group.position.set(Number(obstacle.x), height / 2, Number(obstacle.y));
  group.rotation.y = Number(obstacle.rot ?? 0);
  group.userData = { obstacleId: obstacle.id, base: { sx: width, sy: depth, sz: height } };
  group.traverse((child) => { child.userData.obstacleId = obstacle.id; });
  return group;
}

function makeNodeLabel(node, active) {
  const label = document.createElement('canvas');
  label.width = 512; label.height = 112;
  const context = label.getContext('2d');
  context.fillStyle = active ? 'rgba(11,143,98,.94)' : 'rgba(10,13,16,.90)';
  context.fillRect(2, 2, 508, 108);
  context.strokeStyle = active ? '#66e6b6' : '#7fb8ff';
  context.lineWidth = 4; context.strokeRect(2, 2, 508, 108);
  context.fillStyle = '#f4f7f8';
  context.font = '600 34px Arial, sans-serif';
  context.textBaseline = 'middle';
  context.fillText(`${node.id}  ${String(node.name).slice(0, 22)}`, 22, 56);
  const texture = new THREE.CanvasTexture(label);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }));
  sprite.scale.set(1.22, .27, 1);
  sprite.position.set(0, .28, 0);
  sprite.userData.screenHeightPx = 28;
  sprite.userData.screenAspect = 1.22 / .27;
  sprite.renderOrder = 20;
  return sprite;
}

function makeNode(node) {
  const active = Number(node.id) === Number(selectedNodeId);
  const group = new THREE.Group();
  const marker = new THREE.Mesh(
    new THREE.CylinderGeometry(active ? .105 : .075, active ? .105 : .075, .06, 16),
    new THREE.MeshBasicMaterial({ color: active ? 0x16c784 : 0x9eabb8 }),
  );
  group.add(marker, makeNodeLabel(node, active));
  group.position.set(Number(node.x_m), .065, Number(node.y_m));
  group.userData = { nodeId: Number(node.id) };
  group.traverse((child) => { child.userData.nodeId = Number(node.id); });
  return group;
}

function makeRulerLabel(text, scale = 1, plaque = false) {
  const label = document.createElement('canvas');
  label.width = 512; label.height = 96;
  const context = label.getContext('2d');
  if (plaque) {
    context.fillStyle = 'rgba(9,9,11,.86)';
    context.fillRect(2, 2, 508, 92);
    context.strokeStyle = 'rgba(190,194,202,.5)';
    context.lineWidth = 2;
    context.strokeRect(2, 2, 508, 92);
  }
  context.fillStyle = '#e7e7e9';
  context.font = plaque ? '600 30px Consolas, monospace' : '600 27px Consolas, monospace';
  context.textAlign = 'center'; context.textBaseline = 'middle';
  context.fillText(text, 256, 50);
  const texture = new THREE.CanvasTexture(label);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }));
  const labelWidth = Math.max(.62, Math.min(3.5, text.length * (plaque ? .105 : .075)));
  sprite.scale.set(labelWidth * scale, (plaque ? .3 : .2) * scale, 1);
  sprite.userData.screenHeightPx = (plaque ? 24 : 18) * scale;
  sprite.userData.screenAspect = labelWidth / (plaque ? .3 : .2);
  sprite.renderOrder = 18;
  return sprite;
}

function addLine(group, points, color = 0xa4a7ad, opacity = .82) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map(([x, y, z]) => new THREE.Vector3(x, y, z)));
  group.add(new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthWrite: false })));
}

function makeFloorGrid(width, depth) {
  const group = new THREE.Group();
  const major = [];
  const minor = [];
  for (let index = 0; index <= Math.floor(width * 2 + 1e-6); index += 1) {
    const x = index / 2;
    const target = index % 2 === 0 ? major : minor;
    target.push([x, .008, 0], [x, .008, depth]);
  }
  for (let index = 0; index <= Math.floor(depth * 2 + 1e-6); index += 1) {
    const z = index / 2;
    const target = index % 2 === 0 ? major : minor;
    target.push([0, .008, z], [width, .008, z]);
  }
  // Exact far boundaries are still drawn when a room dimension is not a
  // multiple of 0.5 m. This mirrors the positioning server's floor plan.
  major.push([width, .008, 0], [width, .008, depth], [0, .008, depth], [width, .008, depth]);
  addLine(group, minor, 0xffffff, .10);
  addLine(group, major, 0xffffff, .28);
  return group;
}

function makeFloorLabel(text, position, rotation = 0, size = .19) {
  const label = document.createElement('canvas');
  const context = label.getContext('2d');
  context.font = '600 40px "JetBrains Mono", Consolas, monospace';
  const padding = 10;
  label.width = Math.ceil(context.measureText(text).width) + padding * 2;
  label.height = 48;
  const draw = label.getContext('2d');
  draw.font = '600 40px "JetBrains Mono", Consolas, monospace';
  draw.textBaseline = 'middle';
  draw.fillStyle = '#ffffff';
  draw.fillText(text, padding, label.height / 2);
  const texture = new THREE.CanvasTexture(label);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.minFilter = THREE.LinearFilter;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry((label.width / 64) * size, (label.height / 64) * size),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false, side: THREE.DoubleSide }),
  );
  mesh.rotation.x = -Math.PI / 2;
  mesh.rotation.z = rotation;
  mesh.position.copy(position);
  mesh.renderOrder = 18;
  return mesh;
}

function makeEdgeRuler(from, to, roomCentre, labelRotation = 0) {
  const group = new THREE.Group();
  const length = from.distanceTo(to);
  const direction = new THREE.Vector3().subVectors(to, from).normalize();
  const outward = new THREE.Vector3(-direction.z, 0, direction.x);
  const midpoint = from.clone().addScaledVector(direction, length / 2);
  if (outward.dot(roomCentre.clone().sub(midpoint)) > 0) outward.negate();
  const rulerFrom = from.clone().addScaledVector(outward, .30);
  const rulerTo = to.clone().addScaledVector(outward, .30);
  addLine(group, [[rulerFrom.x, .02, rulerFrom.z], [rulerTo.x, .02, rulerTo.z]], 0xffffff, .9);

  const tickCount = Math.floor(length * 2 + 1e-6);
  for (let index = 0; index <= tickCount; index += 1) {
    const value = index / 2;
    const point = rulerFrom.clone().addScaledVector(direction, value);
    const major = index % 2 === 0;
    const tickLength = major ? .22 : .11;
    const tickStart = point.clone().addScaledVector(outward, .02);
    const tickEnd = point.clone().addScaledVector(outward, tickLength);
    addLine(group, [[tickStart.x, .02, tickStart.z], [tickEnd.x, .02, tickEnd.z]], 0xffffff, major ? 1 : .75);
    if (major && index > 0) {
      const labelPosition = point.clone().addScaledVector(outward, tickLength + .16);
      labelPosition.y = .012;
      group.add(makeFloorLabel(`${value} m`, labelPosition, labelRotation));
    }
  }
  if (length - tickCount / 2 > .01) {
    const tickEnd = rulerTo.clone().addScaledVector(outward, .22);
    addLine(group, [[rulerTo.x, .02, rulerTo.z], [tickEnd.x, .02, tickEnd.z]], 0xffffff, 1);
    const labelPosition = rulerTo.clone().addScaledVector(outward, .55);
    labelPosition.y = .012;
    group.add(makeFloorLabel(`${length.toFixed(2)} m`, labelPosition, labelRotation, .17));
  }
  return group;
}

function makeAxisArrow(direction, color) {
  const group = new THREE.Group();
  const length = .45;
  addLine(group, [[0, .025, 0], [direction.x * length, .025, direction.z * length]], color, .95);
  const head = new THREE.Mesh(new THREE.ConeGeometry(.035, .11, 12), new THREE.MeshBasicMaterial({ color }));
  head.position.copy(direction).multiplyScalar(length);
  head.position.y = .025;
  head.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
  group.add(head);
  return group;
}

function makeRoomRulers(width, depth, height) {
  const group = new THREE.Group();
  const centre = new THREE.Vector3(width / 2, 0, depth / 2);
  group.add(makeEdgeRuler(new THREE.Vector3(0, .02, 0), new THREE.Vector3(width, .02, 0), centre, 0));
  group.add(makeEdgeRuler(new THREE.Vector3(0, .02, depth), new THREE.Vector3(width, .02, depth), centre, 0));
  group.add(makeEdgeRuler(new THREE.Vector3(0, .02, 0), new THREE.Vector3(0, .02, depth), centre, -Math.PI / 2));
  group.add(makeEdgeRuler(new THREE.Vector3(width, .02, 0), new THREE.Vector3(width, .02, depth), centre, -Math.PI / 2));

  const xAxis = makeAxisArrow(new THREE.Vector3(1, 0, 0), 0xff6b6b);
  const yAxis = makeAxisArrow(new THREE.Vector3(0, 0, 1), 0x6bff9c);
  group.add(xAxis, yAxis);
  const originDot = new THREE.Mesh(new THREE.SphereGeometry(.04, 14, 12), new THREE.MeshBasicMaterial({ color: 0xd8e0ff }));
  originDot.position.set(0, .025, 0);
  group.add(originDot);
  const origin = makeRulerLabel('0,0', .82);
  origin.position.set(-.14, .16, -.34);
  group.add(origin);
  const xLabel = makeRulerLabel('X →', .68);
  xLabel.position.set(.61, .16, .10);
  group.add(xLabel);
  const yLabel = makeRulerLabel('Y →', .68);
  yLabel.position.set(.10, .16, .61);
  group.add(yLabel);

  const widthLabel = makeRulerLabel(`width X ${width.toFixed(2)} m · grid 1 m`, 1, true);
  widthLabel.position.set(width / 2, .20, -1.05);
  group.add(widthLabel);
  const depthLabel = makeRulerLabel(`depth Y ${depth.toFixed(2)} m`, 1, true);
  depthLabel.position.set(-1.05, .20, depth / 2);
  group.add(depthLabel);
  const heightLabel = makeRulerLabel(`height ${height.toFixed(2)} m`, .9, true);
  heightLabel.position.set(width + .25, height, -.25);
  group.add(heightLabel);
  group.userData.decorative = true;
  return group;
}

function createRobot() {
  const robot = new THREE.Group();
  const dark = new THREE.MeshStandardMaterial({ color: 0x17252a, roughness: .55, metalness: .2 });
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x62d2b8, roughness: .45, metalness: .08 });
  const basketMaterial = new THREE.MeshStandardMaterial({ color: 0xa9d9cf, wireframe: true, transparent: true, opacity: .75 });
  const base = new THREE.Mesh(new THREE.BoxGeometry(.52, .16, .38), dark);
  base.position.y = .17;
  robot.add(base);
  const basket = new THREE.Mesh(new THREE.BoxGeometry(.42, .42, .32, 2, 2, 2), basketMaterial);
  basket.position.set(.02, .43, 0);
  robot.add(basket);
  [-1, 1].forEach((side) => {
    const wheel = new THREE.Mesh(new THREE.CylinderGeometry(.105, .105, .065, 18), dark);
    wheel.rotation.x = Math.PI / 2;
    wheel.position.set(-.06, .12, side * .22);
    robot.add(wheel);
  });
  const heading = new THREE.Mesh(new THREE.ConeGeometry(.07, .2, 12), new THREE.MeshBasicMaterial({ color: 0xf5b85d }));
  heading.rotation.z = -Math.PI / 2;
  heading.position.set(.38, .26, 0);
  robot.add(heading);
  robot.visible = false;
  return robot;
}

function initWebgl() {
  const viewport = $('map3d');
  try {
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(devicePixelRatio || 1, innerWidth < 760 ? 1 : 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setClearColor(0x0b0e1a);
    renderer.domElement.setAttribute('aria-hidden', 'true');
    viewport.prepend(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0e1a);
    scene.fog = new THREE.Fog(0x0b0e1a, 30, 120);
    const camera = new THREE.PerspectiveCamera(50, 1, .05, 400);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = !reducedMotion;
    controls.dampingFactor = .075;
    controls.minDistance = .8;
    controls.maxDistance = 60;
    controls.maxPolarAngle = Math.PI * .495;
    controls.screenSpacePanning = false;

    scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a2033, .85));
    const sun = new THREE.DirectionalLight(0xffffff, 1.5);
    sun.position.set(6, 12, 7);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    scene.add(sun);
    const staticWorld = new THREE.Group();
    const graphWorld = new THREE.Group();
    const routeWorld = new THREE.Group();
    const robot = createRobot();
    const transform = new TransformControls(camera, renderer.domElement);
    transform.setSize(.72);
    scene.add(staticWorld, graphWorld, routeWorld, robot, transform.getHelper());
    transform.addEventListener('dragging-changed', (event) => { controls.enabled = !event.value; });
    transform.addEventListener('objectChange', syncSelectionFromTransform);
    transform.addEventListener('mouseUp', commitSelectionTransform);

    webgl = { viewport, renderer, scene, camera, controls, transform, staticWorld, graphWorld, routeWorld, robot, sun, rackObjects: new Map(), nodeObjects: new Map(), raycaster: new THREE.Raycaster(), pointer: new THREE.Vector2(), staticSignature: '', graphSignature: '', routeSignature: '', firstFrame: true };
    renderer.domElement.addEventListener('click', selectObjectFrom3d);
    resetCamera();

    const observer = new ResizeObserver(() => resize3d());
    observer.observe(viewport);
    renderer.domElement.addEventListener('webglcontextlost', (event) => {
      event.preventDefault();
      showWebglFallback('Konteks WebGL terhenti. Gunakan Map editor atau muat ulang halaman.');
    });
    animate3d();
  } catch (error) {
    showWebglFallback(error.message);
  }
}

function showWebglFallback(message) {
  $('scene-loading').classList.add('done');
  $('webgl-fallback').classList.remove('hidden');
  const paragraph = $('webgl-fallback').querySelector('p');
  if (message) paragraph.textContent = `WebGL tidak dapat dimulai: ${message}. Seluruh kontrol misi tetap tersedia.`;
}

function resize3d() {
  if (!webgl) return;
  const width = webgl.viewport.clientWidth;
  const height = webgl.viewport.clientHeight;
  if (!width || !height) return;
  webgl.renderer.setSize(width, height, false);
  webgl.camera.aspect = width / height;
  webgl.camera.updateProjectionMatrix();
}

function resetCamera() {
  if (!webgl) return;
  const width = state?.map?.width_m ?? 12;
  const depth = state?.map?.height_m ?? 8;
  const height = Number(state?.scene?.scene?.room?.height ?? 2.7);
  const radius = Math.hypot(width, depth, height) / 2;
  const viewportWidth = Math.max(webgl.viewport.clientWidth, 1);
  const viewportHeight = Math.max(webgl.viewport.clientHeight, 1);
  const aspect = viewportWidth / viewportHeight;
  const verticalFov = THREE.MathUtils.degToRad(webgl.camera.fov);
  const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * aspect);
  const distance = radius / Math.sin(Math.min(verticalFov, horizontalFov) / 2) * 1.28;
  const centre = new THREE.Vector3(width / 2, height / 2, depth / 2);
  const direction = new THREE.Vector3(1, .95, 1.15).normalize();
  webgl.camera.position.copy(centre).addScaledVector(direction, distance);
  webgl.controls.target.copy(centre);
  const span = Math.max(width, depth, 1);
  webgl.scene.fog.near = Math.max(span * .9, distance * .9);
  webgl.scene.fog.far = Math.max(webgl.scene.fog.near + span * 2.5, distance * 3.2);
  webgl.controls.minDistance = Math.max(.8, span * .12);
  webgl.controls.maxDistance = Math.max(60, span * 5);
  webgl.controls.update();
  setCameraButton('perspective');
}

function topCamera() {
  if (!webgl) return;
  const width = state?.map?.width_m ?? 12;
  const depth = state?.map?.height_m ?? 8;
  const height = Number(state?.scene?.scene?.room?.height ?? 2.7);
  const radius = Math.hypot(width, depth, height) / 2;
  const viewportWidth = Math.max(webgl.viewport.clientWidth, 1);
  const viewportHeight = Math.max(webgl.viewport.clientHeight, 1);
  const verticalFov = THREE.MathUtils.degToRad(webgl.camera.fov);
  const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * (viewportWidth / viewportHeight));
  const distance = radius / Math.sin(Math.min(verticalFov, horizontalFov) / 2) * 1.12;
  webgl.camera.position.set(width / 2, distance, depth / 2 + .001);
  webgl.controls.target.set(width / 2, 0, depth / 2);
  webgl.controls.update();
  setCameraButton('top');
}

function zoomCamera(factor) {
  if (!webgl) return;
  const offset = webgl.camera.position.clone().sub(webgl.controls.target).multiplyScalar(factor);
  const distance = Math.max(webgl.controls.minDistance, Math.min(webgl.controls.maxDistance, offset.length()));
  webgl.camera.position.copy(webgl.controls.target).add(offset.setLength(distance));
  webgl.controls.update();
}

function setCameraButton(kind) {
  $('camera-perspective').classList.toggle('on', kind === 'perspective');
  $('camera-top').classList.toggle('on', kind === 'top');
  $('camera-perspective').setAttribute('aria-pressed', String(kind === 'perspective'));
  $('camera-top').setAttribute('aria-pressed', String(kind === 'top'));
}

function setTransformMode(mode) {
  const nodeActive = Boolean(selectedNode());
  transformMode = nodeActive ? 'translate' : mode;
  document.querySelectorAll('[data-transform]').forEach((button) => {
    const unavailable = nodeActive && button.dataset.transform !== 'translate';
    button.disabled = unavailable;
    button.classList.toggle('on', button.dataset.transform === transformMode);
  });
  if (!webgl) return;
  webgl.transform.setMode(transformMode);
  webgl.transform.setSpace(transformMode === 'translate' ? 'world' : 'local');
  webgl.transform.showX = true;
  // Nodes live on the floor plane, and racks derive their centre Z from their
  // height. Translation therefore only exposes the horizontal X/Z axes.
  webgl.transform.showY = !nodeActive && transformMode === 'scale';
  webgl.transform.showZ = true;
  if (transformMode === 'rotate') {
    webgl.transform.showX = false;
    webgl.transform.showY = true;
    webgl.transform.showZ = false;
  }
}

function attachSelectedObject() {
  if (!webgl) return;
  const object = selectedNode()
    ? webgl.nodeObjects.get(Number(selectedNodeId))
    : webgl.rackObjects.get(selectedId);
  if (!object) {
    webgl.transform.detach();
    $('transformbar').classList.add('hidden');
    return;
  }
  webgl.transform.attach(object);
  setTransformMode(transformMode);
  $('transform-selection').textContent = selectedNode()
    ? `NODE ${selectedNode().id} · ${selectedNode().name}`
    : selected()?.label ?? selectedId;
  $('transformbar').classList.remove('hidden');
}

function selectObjectFrom3d(event) {
  if (!webgl || !state || webgl.transform.dragging || webgl.transform.axis) return;
  const rect = webgl.renderer.domElement.getBoundingClientRect();
  webgl.pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
  webgl.raycaster.setFromCamera(webgl.pointer, webgl.camera);
  const selectable = [...webgl.rackObjects.values(), ...webgl.nodeObjects.values()];
  const hit = webgl.raycaster.intersectObjects(selectable, true).find((item) => item.object.userData.obstacleId || item.object.userData.nodeId);
  selectedId = hit?.object.userData.obstacleId ?? null;
  selectedNodeId = hit?.object.userData.nodeId ?? null;
  openTab('map');
  refreshSelected();
  webgl.staticSignature = '';
  rebuildStatic3d();
}

function syncSelectionFromTransform() {
  if (!webgl?.transform.object) return;
  const object = webgl.transform.object;
  if (object.userData.nodeId) {
    const node = nodeList().find((item) => Number(item.id) === Number(object.userData.nodeId));
    if (!node) return;
    object.position.y = .065;
    node.x_m = Math.max(0, Math.min(state.map.width_m, object.position.x));
    node.y_m = Math.max(0, Math.min(state.map.height_m, object.position.z));
    object.position.x = node.x_m; object.position.z = node.y_m;
    webgl.graphSignature = '';
    webgl.routeSignature = '';
    rebuildGraph3d();
    rebuildRoute3d();
    refreshSelected(); drawEditor();
    return;
  }
  const obstacle = obstacleList().find((item) => item.id === object.userData.obstacleId);
  if (!obstacle) return;
  const base = object.userData.base;
  obstacle.x = object.position.x;
  obstacle.y = object.position.z;
  obstacle.rot = object.rotation.y;
  obstacle.sx = Math.max(.1, Math.abs(base.sx * object.scale.x));
  obstacle.sy = Math.max(.1, Math.abs(base.sy * object.scale.z));
  obstacle.sz = Math.max(.1, Math.abs(base.sz * object.scale.y));
  obstacle.z = obstacle.sz / 2;
  object.position.y = obstacle.z;
  refreshSelected();
  drawEditor();
}

async function commitSelectionTransform() {
  syncSelectionFromTransform();
  try {
    if (selectedNode()) {
      await saveGraph();
      toast('Node tersimpan', `${selectedNode()?.name ?? selectedNodeId} diperbarui dari scene 3D.`);
    } else {
      await saveScene();
      toast('Rack tersimpan', `${selected()?.label ?? selectedId} diperbarui dari scene 3D.`);
    }
  } catch (error) {
    toast('Transform ditolak', error.message, 'error');
    await load();
  }
}

function rebuildStatic3d() {
  if (!webgl || !state) return;
  const signature = JSON.stringify([state.map.width_m, state.map.height_m, state.scene.scene.obstacles, state.graph, selectedId, selectedNodeId]);
  if (signature === webgl.staticSignature) return;
  webgl.staticSignature = signature;
  webgl.transform.detach();
  webgl.rackObjects.clear();
  webgl.nodeObjects.clear();
  clearGroup(webgl.staticWorld);
  const world = webgl.staticWorld;
  const width = state.map.width_m;
  const depth = state.map.height_m;

  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(width, .06, depth),
    new THREE.MeshStandardMaterial({ color: 0x24242b, roughness: .95, metalness: .05 }),
  );
  floor.position.set(width / 2, -.03, depth / 2);
  floor.receiveShadow = true;
  world.add(floor);

  world.add(makeFloorGrid(width, depth));

  world.add(makeRoomRulers(width, depth, Number(state.scene.scene.room.height ?? 2.7)));

  const zoneMaterial = new THREE.MeshBasicMaterial({ color: 0x7d1723, transparent: true, opacity: .18, side: THREE.DoubleSide });
  const zoneWidth = Math.min(1.25, width * .12);
  const zoneDepth = Math.min(.75, depth * .12);
  [[width * .12, depth * .22, zoneWidth, zoneDepth], [width * .82, depth * .22, zoneWidth, zoneDepth]].forEach(([x, z, sx, sz]) => {
    const zone = new THREE.Mesh(new THREE.PlaneGeometry(sx, sz), zoneMaterial.clone());
    zone.rotation.x = -Math.PI / 2;
    zone.position.set(x, .012, z);
    world.add(zone);
  });

  const roomHeight = Number(state.scene.scene.room.height ?? 2.7);
  const wallMaterial = new THREE.MeshStandardMaterial({
    color: 0x3a3a46,
    transparent: true,
    opacity: .15,
    side: THREE.DoubleSide,
    roughness: 1,
    depthWrite: false,
  });
  [
    [width, roomHeight, .06, width / 2, roomHeight / 2, 0],
    [width, roomHeight, .06, width / 2, roomHeight / 2, depth],
    [.06, roomHeight, depth, 0, roomHeight / 2, depth / 2],
    [.06, roomHeight, depth, width, roomHeight / 2, depth / 2],
  ].forEach(([sx, sy, sz, x, y, z]) => {
    const wall = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), wallMaterial);
    wall.position.set(x, y, z);
    world.add(wall);
  });

  obstacleList().forEach((obstacle) => {
    const rack = makeRack(obstacle);
    rack.traverse((child) => { if (child.isMesh) child.castShadow = true; });
    webgl.rackObjects.set(obstacle.id, rack);
    world.add(rack);
  });
  state.graph.nodes.forEach((node) => {
    const marker = makeNode(node);
    webgl.nodeObjects.set(Number(node.id), marker);
    world.add(marker);
  });
  attachSelectedObject();
}

function rebuildGraph3d() {
  if (!webgl || !state) return;
  const signature = JSON.stringify([state.graph.edges, state.graph.nodes.map((node) => [node.id, node.x_m, node.y_m])]);
  if (signature === webgl.graphSignature) return;
  webgl.graphSignature = signature;
  clearGroup(webgl.graphWorld);
  const byId = Object.fromEntries(state.graph.nodes.map((node) => [node.id, node]));
  state.graph.edges.forEach((edge) => {
    const a = byId[edge.from];
    const b = byId[edge.to];
    if (a && b) webgl.graphWorld.add(makeSegment([a.x_m, a.y_m], [b.x_m, b.y_m], .014, 0x474750, .025));
  });
}

function rebuildRoute3d() {
  if (!webgl || !state) return;
  const byId = Object.fromEntries(state.graph.nodes.map((node) => [node.id, node]));
  const signature = JSON.stringify((route?.nodes ?? []).map((nodeId) => [nodeId, byId[nodeId]?.x_m, byId[nodeId]?.y_m]));
  if (signature === webgl.routeSignature) return;
  webgl.routeSignature = signature;
  clearGroup(webgl.routeWorld);
  if (!route?.nodes?.length) return;
  route.nodes.slice(1).forEach((nodeId, index) => {
    const a = byId[route.nodes[index]];
    const b = byId[nodeId];
    if (a && b) webgl.routeWorld.add(makeSegment([a.x_m, a.y_m], [b.x_m, b.y_m], .035, 0xf5b85d, .075));
  });
  route.nodes.forEach((nodeId) => {
    const node = byId[nodeId];
    if (!node) return;
    const dot = new THREE.Mesh(new THREE.SphereGeometry(.075, 12, 8), new THREE.MeshBasicMaterial({ color: 0xffd99b }));
    dot.position.set(node.x_m, .075, node.y_m);
    webgl.routeWorld.add(dot);
  });
}

function syncRobot() {
  if (!webgl) return;
  const pose = currentPose();
  const visible = pose && finite(pose.x_m ?? pose.x) && finite(pose.y_m ?? pose.y);
  webgl.robot.visible = Boolean(visible);
  if (!visible) return;
  webgl.robot.position.set(Number(pose.x_m ?? pose.x), 0, Number(pose.y_m ?? pose.y));
  webgl.robot.rotation.y = -Number(pose.heading_rad ?? pose.heading ?? 0);
}

function updateScenePresentation() {
  if (!webgl || !state) return;
  const width = Number(state.map.width_m);
  const depth = Number(state.map.height_m);
  const span = Math.max(width, depth, 1);
  const distance = webgl.camera.position.distanceTo(webgl.controls.target);
  const near = Math.max(span * .9, distance * .9);
  webgl.scene.fog.near = near;
  webgl.scene.fog.far = Math.max(near + span * 2.5, distance * 3.2);

  const viewportHeight = Math.max(webgl.renderer.domElement.clientHeight, 1);
  webgl.camera.updateMatrixWorld();
  webgl.staticWorld.traverse((object) => {
    if (!object.isSprite || !object.userData.screenHeightPx) return;
    const viewPosition = object.getWorldPosition(labelWorldPosition).applyMatrix4(webgl.camera.matrixWorldInverse);
    const worldPerPixel = 2 * Math.tan(THREE.MathUtils.degToRad(webgl.camera.fov) / 2)
      * Math.max(-viewPosition.z, .05) / viewportHeight;
    const height = object.userData.screenHeightPx * worldPerPixel;
    object.scale.set(height * object.userData.screenAspect, height, 1);
  });
}

function animate3d() {
  if (!webgl) return;
  requestAnimationFrame(animate3d);
  if (!reducedMotion) webgl.controls.update();
  updateScenePresentation();
  if (!renderingPaused) webgl.renderer.render(webgl.scene, webgl.camera);
  if (webgl.firstFrame) {
    webgl.firstFrame = false;
    requestAnimationFrame(() => $('scene-loading').classList.add('done'));
  }
}

function mToPx(x, y) {
  const pad = 54;
  const sx = (canvas.width - pad * 2) / state.map.width_m;
  const sy = (canvas.height - pad * 2) / state.map.height_m;
  return [pad + x * sx, canvas.height - pad - y * sy];
}

function pxToM(x, y) {
  const pad = 54;
  const sx = (canvas.width - pad * 2) / state.map.width_m;
  const sy = (canvas.height - pad * 2) / state.map.height_m;
  return [(x - pad) / sx, (canvas.height - pad - y) / sy];
}

function drawEditor() {
  if (!state) return;
  const { width, height } = canvas;
  const pad = 54;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#111920';
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = '#18242d';
  ctx.fillRect(pad, pad, width - pad * 2, height - pad * 2);
  ctx.strokeStyle = '#263944';
  ctx.lineWidth = 1;
  for (let x = 0; x <= state.map.width_m; x += 1) {
    const [px] = mToPx(x, 0);
    ctx.beginPath(); ctx.moveTo(px, pad); ctx.lineTo(px, height - pad); ctx.stroke();
  }
  for (let y = 0; y <= state.map.height_m; y += 1) {
    const [, py] = mToPx(0, y);
    ctx.beginPath(); ctx.moveTo(pad, py); ctx.lineTo(width - pad, py); ctx.stroke();
  }
  ctx.strokeStyle = '#526875';
  ctx.lineWidth = 2;
  ctx.strokeRect(pad, pad, width - pad * 2, height - pad * 2);
  ctx.fillStyle = '#8e9ba7';
  ctx.font = '12px Inter, sans-serif';
  ctx.fillText('smart_market_map · origin bottom-left · metres', pad, 30);

  const byId = Object.fromEntries(state.graph.nodes.map((node) => [node.id, node]));
  ctx.strokeStyle = '#3e5b6a';
  ctx.lineWidth = 2;
  state.graph.edges.forEach((edge) => {
    const a = byId[edge.from]; const b = byId[edge.to];
    const [ax, ay] = mToPx(a.x_m, a.y_m); const [bx, by] = mToPx(b.x_m, b.y_m);
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
  });
  if (route?.nodes?.length) {
    ctx.strokeStyle = '#f5b85d'; ctx.lineWidth = 6; ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.beginPath();
    route.nodes.forEach((id, index) => { const [x, y] = mToPx(byId[id].x_m, byId[id].y_m); index ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
  }
  obstacleList().forEach((obstacle) => {
    const [x1, y1] = mToPx(obstacle.x - obstacle.sx / 2, obstacle.y - obstacle.sy / 2);
    const [x2, y2] = mToPx(obstacle.x + obstacle.sx / 2, obstacle.y + obstacle.sy / 2);
    const active = obstacle.id === selectedId;
    ctx.fillStyle = active ? '#684f2f' : '#3b4b56';
    ctx.fillRect(x1, y2, x2 - x1, y1 - y2);
    ctx.strokeStyle = active ? '#f5b85d' : '#7d929e'; ctx.lineWidth = active ? 3 : 2;
    ctx.strokeRect(x1, y2, x2 - x1, y1 - y2);
    ctx.fillStyle = active ? '#ffe0aa' : '#d8e0e5'; ctx.font = '600 12px Inter, sans-serif';
    ctx.fillText(obstacle.label ?? obstacle.id, x1 + 8, y2 + 18);
    if (active) { ctx.fillStyle = '#f5b85d'; ctx.fillRect(x2 - 7, y1 - 7, 14, 14); }
  });
  state.graph.nodes.forEach((node) => {
    const [x, y] = mToPx(node.x_m, node.y_m);
    const active = Number(node.id) === Number(selectedNodeId);
    ctx.fillStyle = active ? '#0b8f62' : '#7fb8ff'; ctx.beginPath(); ctx.arc(x, y, active ? 9 : 6, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = active ? '#d8fff0' : '#dcecff'; ctx.font = '11px SFMono-Regular, Consolas, monospace'; ctx.fillText(`${node.id} · ${node.name}`, x + 10, y - 8);
  });
  const pose = currentPose();
  if (pose && finite(pose.x_m ?? pose.x) && finite(pose.y_m ?? pose.y)) {
    const [x, y] = mToPx(Number(pose.x_m ?? pose.x), Number(pose.y_m ?? pose.y));
    ctx.fillStyle = '#6ad6a0'; ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = '#e8fff4'; ctx.lineWidth = 2; ctx.stroke();
  }
}

function pick(mx, my) {
  return obstacleList().slice().reverse().find((obstacle) => {
    const bounds = [obstacle.x - obstacle.sx / 2, obstacle.y - obstacle.sy / 2, obstacle.x + obstacle.sx / 2, obstacle.y + obstacle.sy / 2];
    return mx >= bounds[0] && mx <= bounds[2] && my >= bounds[1] && my <= bounds[3];
  });
}

function pickNode(mx, my) {
  const tolerance = Math.max(state.map.width_m, state.map.height_m) * .022;
  return nodeList().slice().reverse().find((node) => Math.hypot(node.x_m - mx, node.y_m - my) <= tolerance);
}

function refreshSelected() {
  const obstacle = selected();
  const node = selectedNode();
  if (selectedId && !obstacle) selectedId = null;
  if (selectedNodeId != null && !node) selectedNodeId = null;
  const fields = ['ob-name', 'ob-x', 'ob-y', 'ob-width', 'ob-depth', 'ob-height', 'ob-rotation', 'ob-atten', 'ob-products'];
  fields.forEach((id) => { $(id).disabled = !obstacle; });
  $('save-obstacle').disabled = !obstacle;
  $('delete-obstacle').disabled = !obstacle;
  $('obstacle-empty').classList.toggle('hidden', Boolean(obstacle));
  $('obstacle-fields').classList.toggle('hidden', !obstacle);
  const setValue = (id, value) => { if (document.activeElement !== $(id)) $(id).value = value; };
  if (obstacle) {
    setValue('ob-name', obstacle.label ?? obstacle.id);
    setValue('ob-x', Number(obstacle.x).toFixed(2));
    setValue('ob-y', Number(obstacle.y).toFixed(2));
    setValue('ob-width', Number(obstacle.sx).toFixed(2));
    setValue('ob-depth', Number(obstacle.sy).toFixed(2));
    setValue('ob-height', Number(obstacle.sz ?? 1.5).toFixed(2));
    setValue('ob-rotation', (Number(obstacle.rot ?? 0) * 180 / Math.PI).toFixed(1));
    setValue('ob-atten', Number(obstacle.atten ?? 1));
    $('ob-atten-value').textContent = Number(obstacle.atten ?? 1).toFixed(2);
    setValue('ob-products', (obstacle.products ?? []).map((product) => `${product.name}@${product.node_id}`).join(', '));
  }

  ['node-name', 'node-type', 'node-x', 'node-y'].forEach((id) => { $(id).disabled = !node; });
  $('save-node').disabled = !node;
  $('delete-node').disabled = !node;
  $('node-empty').classList.toggle('hidden', Boolean(node));
  $('node-fields').classList.toggle('hidden', !node);
  if (node) {
    setValue('node-id', node.id);
    setValue('node-name', node.name);
    setValue('node-type', node.type ?? 'waypoint');
    setValue('node-x', Number(node.x_m).toFixed(2));
    setValue('node-y', Number(node.y_m).toFixed(2));
    const connectionCount = state.graph.edges.filter((edge) => Number(edge.from) === Number(node.id) || Number(edge.to) === Number(node.id)).length;
    $('node-connections').textContent = `${connectionCount} VISIBLE CONNECTION${connectionCount === 1 ? '' : 'S'}`;
  }
  const selection = node || obstacle;
  $('transformbar').classList.toggle('hidden', !selection || !webgl || currentView !== '3d');
  if (selection) $('transform-selection').textContent = node ? `NODE ${node.id} · ${node.name}` : obstacle.label ?? obstacle.id;
  setTransformMode(transformMode);
}

async function saveScene() {
  state = await request('/api/scene', { method: 'POST', body: JSON.stringify(state.scene) });
  if (webgl) { webgl.staticSignature = ''; webgl.graphSignature = ''; webgl.routeSignature = ''; }
  autoRouteSignature = '';
  syncNodeOptions();
  scheduleAutoRoute();
  drawEditor();
}

async function saveGraph() {
  state = await request('/api/graph', { method: 'POST', body: JSON.stringify(state.graph) });
  nodeOptionSignature = '';
  autoRouteSignature = '';
  if (webgl) { webgl.staticSignature = ''; webgl.graphSignature = ''; webgl.routeSignature = ''; }
  syncNodeOptions();
  scheduleAutoRoute();
  refreshSelected(); drawEditor(); rebuildStatic3d(); rebuildGraph3d(); rebuildRoute3d();
}

function pointBlocked(x, y) {
  return obstacleList().some((obstacle) => {
    const angle = -Number(obstacle.rot ?? 0);
    const dx = x - Number(obstacle.x); const dy = y - Number(obstacle.y);
    const localX = dx * Math.cos(angle) - dy * Math.sin(angle);
    const localY = dx * Math.sin(angle) + dy * Math.cos(angle);
    return Math.abs(localX) <= Number(obstacle.sx) / 2 && Math.abs(localY) <= Number(obstacle.sy) / 2;
  });
}

function fitNodeGridForRoom(width, depth) {
  const nodes = nodeList();
  if (!nodes.length) return [];
  const ordered = [...nodes].sort((first, second) => Number(first.id) - Number(second.id));
  const columns = Math.ceil(Math.sqrt(ordered.length));
  const rows = Math.ceil(ordered.length / columns);
  const marginX = Math.min(1.5, width * .1);
  const marginY = Math.min(1.5, depth * .2);
  const xs = columns === 1 ? [width / 2] : Array.from({ length: columns }, (_, column) => (
    marginX + column * (width - 2 * marginX) / (columns - 1)
  ));
  const idealYs = rows === 1 ? [depth / 2] : Array.from({ length: rows }, (_, row) => (
    marginY + row * (depth - 2 * marginY) / (rows - 1)
  ));
  const step = Math.max(.05, Math.min(width, depth) / 80);
  const rowSpacing = Math.max(.22, Math.min(width, depth) * .025);
  const fittedYs = [];
  const rowIsFree = (y) => y >= 0 && y <= depth
    && fittedYs.every((previous) => Math.abs(y - previous) >= rowSpacing)
    && xs.every((x) => !pointBlocked(x, y));

  idealYs.forEach((idealY) => {
    let fittedY = null;
    const maximumOffset = Math.max(idealY, depth - idealY);
    for (let offset = 0; offset <= maximumOffset + step && fittedY == null; offset += step) {
      const candidates = offset === 0 ? [idealY] : [idealY + offset, idealY - offset];
      fittedY = candidates.find(rowIsFree) ?? null;
    }
    if (fittedY == null) throw new Error('Ruangan tidak memiliki area bebas yang cukup untuk grid node.');
    fittedYs.push(fittedY);
  });

  const positions = new Map(ordered.map((node, index) => [Number(node.id), {
    x_m: xs[index % columns], y_m: fittedYs[Math.floor(index / columns)],
  }]));
  return nodes.map((node) => ({ ...node, ...positions.get(Number(node.id)) }));
}

async function addNode() {
  if (!state) return;
  const id = Math.max(0, ...nodeList().map((node) => Number(node.id))) + 1;
  let position = null;
  for (let y = .5; y < state.map.height_m && !position; y += .5) {
    for (let x = .5; x < state.map.width_m; x += .5) {
      if (!pointBlocked(x, y) && nodeList().every((node) => Math.hypot(node.x_m - x, node.y_m - y) > .25)) { position = [x, y]; break; }
    }
  }
  if (!position) { toast('Node tidak dapat ditambahkan', 'Tidak ada posisi kosong yang valid.', 'error'); return; }
  state.graph.nodes.push({ id, name: `node-${id}`, type: 'waypoint', x_m: position[0], y_m: position[1] });
  selectedId = null; selectedNodeId = id; openTab('map');
  try {
    await saveGraph();
    toast('Node ditambahkan', `Node ${id} siap dipindahkan atau diberi nama.`);
  } catch (error) {
    toast('Node tidak dapat ditambahkan', error.message, 'error');
    await load();
  }
}

async function resetNodePositions() {
  if (!nodeList().length) { toast('Tidak ada node', 'Tambahkan node navigasi terlebih dahulu.', 'warning'); return; }
  $('reset-nodes').disabled = true;
  try {
    const candidate = { ...state.graph, nodes: fitNodeGridForRoom(Number(state.map.width_m), Number(state.map.height_m)) };
    state = await request('/api/graph', { method: 'POST', body: JSON.stringify(candidate) });
    route = null;
    nodeOptionSignature = '';
    autoRouteSignature = '';
    if (webgl) { webgl.staticSignature = ''; webgl.graphSignature = ''; webgl.routeSignature = ''; }
    render();
    toast('Posisi node di-reset', `${nodeList().length} node disesuaikan otomatis dengan ruangan.`);
  } catch (error) {
    toast('Reset node ditolak', error.message, 'error');
    await load();
  } finally {
    $('reset-nodes').disabled = false;
  }
}

async function addObstacle() {
  if (!state) return;
  const id = `rack-${Date.now()}`;
  const width = Math.min(1.2, state.map.width_m * .2);
  const depth = Math.min(.6, state.map.height_m * .15);
  const height = Math.min(1.5, Number(state.scene.scene.room.height ?? 2.7));
  state.scene.scene.obstacles.push({ id, label: `Rack ${obstacleList().length + 1}`, x: state.map.width_m / 2, y: state.map.height_m / 2, z: height / 2, sx: width, sy: depth, sz: height, rot: 0, atten: 1, products: [] });
  selectedId = id; selectedNodeId = null;
  openTab('map');
  try {
    await saveScene();
    refreshSelected();
    rebuildStatic3d();
    toast('Rack ditambahkan', 'Gunakan gizmo 3D atau input Inspector untuk mengaturnya.');
  } catch (error) {
    state.scene.scene.obstacles = obstacleList().filter((obstacle) => obstacle.id !== id);
    selectedId = null;
    toast('Rack tidak dapat ditambahkan', error.message, 'error');
  }
}

async function resizeRoom() {
  $('save-room').disabled = true;
  const width = numeric('room-width');
  const depth = numeric('room-depth');
  const height = numeric('room-height');
  const originalGraph = JSON.parse(JSON.stringify(state.graph));
  const shrinking = width < Number(state.map.width_m) || depth < Number(state.map.height_m);
  let graphPrefitted = false;
  let roomSaved = false;
  $('save-room').textContent = 'VALIDATING…';
  try {
    // Keep resize compatible with a dashboard process that was started before
    // server-side auto-fit existed: shrink the graph first, then the room.
    if (shrinking) {
      const candidate = { ...state.graph, nodes: fitNodeGridForRoom(width, depth) };
      state = await request('/api/graph', { method: 'POST', body: JSON.stringify(candidate) });
      graphPrefitted = true;
    }
    state = await request('/api/room', { method: 'POST', body: JSON.stringify({ width_m: width, depth_m: depth, room_height_m: height }) });
    roomSaved = true;
    if (!shrinking) {
      const candidate = { ...state.graph, nodes: fitNodeGridForRoom(width, depth) };
      state = await request('/api/graph', { method: 'POST', body: JSON.stringify(candidate) });
    }
    route = null;
    roomSignature = '';
    if (webgl) { webgl.staticSignature = ''; webgl.graphSignature = ''; webgl.routeSignature = ''; }
    render(); resetCamera();
    toast('Room size tersimpan', `${state.map.width_m} × ${state.map.height_m} × ${state.scene.scene.room.height} m`);
  } catch (error) {
    if (graphPrefitted && !roomSaved) {
      try { state = await request('/api/graph', { method: 'POST', body: JSON.stringify(originalGraph) }); }
      catch (restoreError) { console.error('Graph rollback failed', restoreError); }
    }
    toast('Room resize ditolak', error.message, 'error');
  } finally {
    $('save-room').disabled = false;
    $('save-room').textContent = 'SAVE ROOM SIZE';
  }
}

canvas.addEventListener('pointerdown', (event) => {
  if (!state) return;
  const rect = canvas.getBoundingClientRect();
  const [mx, my] = pxToM((event.clientX - rect.left) * canvas.width / rect.width, (event.clientY - rect.top) * canvas.height / rect.height);
  const obstacle = pick(mx, my);
  const node = pickNode(mx, my);
  canvas.setPointerCapture(event.pointerId);
  if (node) {
    selectedNodeId = Number(node.id); selectedId = null;
    pointer = { mode: 'node-move', mx, my, original: { ...node } };
    openTab('map');
  } else if (obstacle) {
    selectedId = obstacle.id;
    selectedNodeId = null;
    const nearHandle = Math.abs(mx - (obstacle.x + obstacle.sx / 2)) < .18 && Math.abs(my - (obstacle.y - obstacle.sy / 2)) < .18;
    pointer = { mode: nearHandle ? 'resize' : 'move', mx, my, original: { ...obstacle } };
    openTab('map');
  } else {
    selectedId = null; selectedNodeId = null;
    pointer = { mode: 'create', mx, my };
  }
  refreshSelected(); drawEditor();
});

canvas.addEventListener('pointermove', (event) => {
  if (!pointer) return;
  const rect = canvas.getBoundingClientRect();
  const [mx, my] = pxToM((event.clientX - rect.left) * canvas.width / rect.width, (event.clientY - rect.top) * canvas.height / rect.height);
  const obstacle = selected();
  const node = selectedNode();
  if (pointer.mode === 'move' && obstacle) {
    obstacle.x = Math.max(obstacle.sx / 2, Math.min(state.map.width_m - obstacle.sx / 2, pointer.original.x + mx - pointer.mx));
    obstacle.y = Math.max(obstacle.sy / 2, Math.min(state.map.height_m - obstacle.sy / 2, pointer.original.y + my - pointer.my));
  } else if (pointer.mode === 'resize' && obstacle) {
    obstacle.sx = Math.max(.1, Math.min(state.map.width_m, 2 * (mx - obstacle.x + pointer.original.sx / 2)));
    obstacle.sy = Math.max(.1, Math.min(state.map.height_m, 2 * (obstacle.y - my + pointer.original.sy / 2)));
  } else if (pointer.mode === 'node-move' && node) {
    node.x_m = Math.max(0, Math.min(state.map.width_m, pointer.original.x_m + mx - pointer.mx));
    node.y_m = Math.max(0, Math.min(state.map.height_m, pointer.original.y_m + my - pointer.my));
  }
  drawEditor();
});

canvas.addEventListener('pointerup', async (event) => {
  if (!pointer) return;
  const rect = canvas.getBoundingClientRect();
  const [mx, my] = pxToM((event.clientX - rect.left) * canvas.width / rect.width, (event.clientY - rect.top) * canvas.height / rect.height);
  const operation = pointer.mode;
  if (operation === 'create' && Math.abs(mx - pointer.mx) > .1 && Math.abs(my - pointer.my) > .1) {
    const x = Math.min(mx, pointer.mx); const y = Math.min(my, pointer.my);
    const sx = Math.abs(mx - pointer.mx); const sy = Math.abs(my - pointer.my);
    const id = `rack-${Date.now()}`;
    state.scene.scene.obstacles.push({ id, label: 'Rak baru', x: x + sx / 2, y: y + sy / 2, z: .75, sx, sy, sz: 1.5, rot: 0, atten: 1, products: [] });
    selectedId = id; selectedNodeId = null; openTab('map'); refreshSelected();
  }
  pointer = null;
  try {
    if (operation === 'node-move') { await saveGraph(); toast('Node tersimpan', 'Posisi dan koneksi diperbarui.'); }
    else { await saveScene(); toast('Scene tersimpan', 'Perubahan rak berhasil disimpan.'); }
  } catch (error) { toast('Perubahan tidak dapat disimpan', error.message, 'error'); await load(); }
});

function pointText(point) {
  if (!point || !finite(point.x_m ?? point.x) || !finite(point.y_m ?? point.y)) return 'N/A';
  return `${Number(point.x_m ?? point.x).toFixed(2)}, ${Number(point.y_m ?? point.y).toFixed(2)}`;
}

function drawPidChart() {
  const chart = $('pid-chart');
  const context = chart.getContext('2d');
  const { width, height } = chart;
  context.fillStyle = '#0c1319'; context.fillRect(0, 0, width, height);
  context.strokeStyle = '#26333e'; context.lineWidth = 1;
  for (let row = 1; row < 4; row += 1) { const y = row * height / 4; context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.stroke(); }
  if (pidHistory.length < 2) { context.fillStyle = '#65727d'; context.font = '11px Inter, sans-serif'; context.fillText('Menunggu wheel telemetry…', 14, height / 2); return; }
  const maximum = Math.max(20, ...pidHistory.flatMap((values) => values.map((value) => Math.abs(value))));
  ['#7fb8ff', '#6ad6c3', '#f5b85d', '#d995c5'].forEach((color, column) => {
    context.strokeStyle = color; context.lineWidth = column % 2 ? 2 : 1; context.beginPath();
    pidHistory.forEach((row, index) => { const x = index * width / (pidHistory.length - 1); const y = height / 2 - row[column] / maximum * height * .43; index ? context.lineTo(x, y) : context.moveTo(x, y); });
    context.stroke();
  });
}

function setTelemetry() {
  const telemetry = state.telemetry ?? {};
  const navState = String(telemetry.mode ?? telemetry.state ?? 'IDLE');
  const routeDistance = telemetry.route_distance_m;
  const quality = telemetry.quality ?? telemetry.uwb_quality;
  const odometry = telemetry.odometry_pose ?? telemetry.odometry;
  const uwb = telemetry.uwb_position ?? telemetry.uwb;
  const navigation = telemetry.navigation_pose ?? telemetry.pose;
  $('odo').textContent = pointText(odometry);
  $('uwb').textContent = pointText(uwb);
  $('nav').textContent = pointText(navigation);
  $('metric-state').textContent = navState.replaceAll('_', ' ');
  $('metric-source').textContent = telemetry.source ?? state.mode;
  $('metric-pose').textContent = pointText(navigation ?? currentPose());
  $('metric-route').textContent = finite(routeDistance) ? `${Number(routeDistance).toFixed(2)} m` : (route ? `${route.distance_m.toFixed(2)} m` : '—');
  $('metric-quality').textContent = finite(quality) ? `${Math.round(Number(quality) * 100)}%` : 'N/A';
  $('metric-age').textContent = state.connection.connected && state.connection.last_rx_age_ms != null ? `${state.connection.last_rx_age_ms} ms ago` : 'belum ada sampel';

  const statusClass = /ERROR|ESTOP|LOST/.test(navState) ? 'error' : /NAVIGATING|READY|ARRIVED/.test(navState) ? 'ready' : /WAIT|PLANNING/.test(navState) ? 'warning' : '';
  $('navigation').className = `state-block ${statusClass}`;
  $('navigation').innerHTML = `<span class="state-indicator"></span><div><small>Status</small><strong>${escapeHtml(navState.replaceAll('_', ' '))}</strong><p>${route ? `Preview ${route.nodes.join(' → ')}` : 'Route belum aktif'}</p></div>`;

  const left = telemetry.left ?? {}; const right = telemetry.right ?? {};
  if ([left.target_rpm, left.actual_rpm, right.target_rpm, right.actual_rpm].some(finite)) {
    const wheelRow = (label, wheel) => `<div class="wheel-row"><span>${label}</span><strong>${finite(wheel.target_rpm) ? Number(wheel.target_rpm).toFixed(1) : '—'} → ${finite(wheel.actual_rpm) ? Number(wheel.actual_rpm).toFixed(1) : '—'} RPM</strong><em>${finite(wheel.error_rpm) ? `Δ ${Number(wheel.error_rpm).toFixed(1)}` : 'N/A'}</em></div>`;
    $('wheel-data').innerHTML = wheelRow('LEFT', left) + wheelRow('RIGHT', right);
  } else $('wheel-data').innerHTML = '<p>Wheel telemetry belum tersedia.</p>';
  if ([left.target_rpm, left.actual_rpm, right.target_rpm, right.actual_rpm].every((value) => Number.isFinite(value))) {
    pidHistory.push([left.target_rpm, left.actual_rpm, right.target_rpm, right.actual_rpm]);
    if (pidHistory.length > 120) pidHistory.shift();
  }
  drawPidChart();
}

function setHardwareAvailability() {
  const hardwareMode = state.mode === 'hardware';
  const connected = hardwareMode && state.connection.connected;
  $('port').disabled = !hardwareMode || connected;
  $('baud').disabled = !hardwareMode || connected;
  $('refresh-ports').disabled = !hardwareMode;
  $('connect').disabled = !hardwareMode || connected || !$('port').value;
  $('disconnect').disabled = !connected;
  $('send-destination').disabled = !connected || !route;
  $('start-manual').disabled = !connected;
  $('apply-position-source').disabled = !connected;
  $('set-pose').disabled = !connected;
  $('teleop-lock').classList.toggle('unlocked', connected);
  document.querySelectorAll('[data-command], [data-pid]').forEach((button) => { button.disabled = !connected; });
  document.querySelectorAll('[data-drive]').forEach((button) => { button.disabled = !connected; });
}

function render() {
  if (!state) return;
  const connected = Boolean(state.connection.connected);
  $('mode').value = state.mode;
  $('status').classList.toggle('connected', connected);
  $('status').innerHTML = connected ? '<i></i> ESP32 connected' : `<i></i> ${state.mode === 'simulation' ? 'Simulation ready' : 'ESP32 disconnected'}`;
  $('connection-card').classList.toggle('connected', connected);
  $('connection-mini').classList.toggle('online', connected);
  $('connection-mini').innerHTML = connected ? '<i></i> ONLINE' : '<i></i> OFFLINE';
  $('connection-title').textContent = connected ? `Terhubung ke ${state.connection.port}` : 'ESP32 tidak terhubung';
  $('connection').textContent = connected ? `${state.connection.baud_rate} baud · RX ${state.connection.last_rx_age_ms ?? '—'} ms` : (state.mode === 'simulation' ? 'Mode simulasi aktif' : 'Pilih port untuk mulai');
  $('events').innerHTML = state.events.length ? state.events.map((event) => `<li><b>${escapeHtml(event.code)}</b> ${escapeHtml(event.message)}</li>`).join('') : '<li>Belum ada event.</li>';
  const nextRoomSignature = `${state.map.width_m}:${state.map.height_m}:${state.scene.scene.room.height ?? 2.7}`;
  if (nextRoomSignature !== roomSignature) {
    roomSignature = nextRoomSignature;
    $('room-width').value = state.map.width_m;
    $('room-depth').value = state.map.height_m;
    $('room-height').value = state.scene.scene.room.height ?? 2.7;
  }

  const configuration = state.device_config;
  if (configuration && configuration.revision !== configRevision) {
    configRevision = configuration.revision;
    ['left', 'right', 'heading'].forEach((side) => ['kp', 'ki', 'kd'].forEach((gain) => {
      if (configuration[side]?.[gain] !== undefined) $(`${side}-${gain}`).value = configuration[side][gain];
    }));
    if (Number.isInteger(configuration.position_source)) $('position-source').value = ['encoder_odometry', 'external_mock_uwb', 'real_uwb'][configuration.position_source] ?? 'real_uwb';
  }
  syncNodeOptions(); scheduleAutoRoute();
  setHardwareAvailability(); setTelemetry(); refreshSelected(); drawEditor();
  rebuildStatic3d(); rebuildGraph3d(); rebuildRoute3d(); syncRobot();
  const stale = state.mode === 'hardware' && (!telemetryReceivedAt || performance.now() - telemetryReceivedAt > 1500);
  $('stale-warning').classList.toggle('visible', stale);
  frames += 1;
  const now = performance.now();
  if (now - lastUpdate > 1000) {
    $('footer').textContent = `UI ${frames} Hz · Telemetry ${state.connection.rx_lines ?? 0} RX · Config r${configRevision ?? '—'} · Protocol v${state.protocol_version}`;
    frames = 0; lastUpdate = now;
  }
}

function syncNodeOptions() {
  if (!state) return;
  const signature = JSON.stringify(nodeList().map((node) => [node.id, node.name, node.type]));
  if (signature === nodeOptionSignature) return;
  nodeOptionSignature = signature;
  const start = $('route-start'); const destination = $('route-destination');
  const previousStart = Number(start.value); const previousDestination = Number(destination.value);
  const options = nodeList().map((node) => `<option value="${node.id}">${node.id} · ${escapeHtml(node.name ?? node.type)}</option>`).join('');
  start.innerHTML = options; destination.innerHTML = options;
  const ids = nodeList().map((node) => Number(node.id));
  if (!ids.length) return;
  start.value = String(ids.includes(previousStart) ? previousStart : ids[0]);
  const preferredDestination = nodeList().find((node) => node.type === 'destination')?.id ?? ids.at(-1);
  destination.value = String(ids.includes(previousDestination) ? previousDestination : preferredDestination);
  if (ids.length > 1 && start.value === destination.value) destination.value = String(ids.find((id) => String(id) !== start.value));
}

function scheduleAutoRoute() {
  if (!state) return;
  const start = Number($('route-start').value); const destination = Number($('route-destination').value);
  if (!Number.isInteger(start) || !Number.isInteger(destination)) {
    clearTimeout(autoRouteTimer);
    route = null; $('route-result').textContent = 'TAMBAHKAN NODE UNTUK MEMBENTUK JALUR';
    if (webgl) webgl.routeSignature = '';
    rebuildRoute3d(); drawEditor();
    return;
  }
  const signature = JSON.stringify([start, destination, state.graph.nodes, state.graph.edges, obstacleList()]);
  if (signature === autoRouteSignature) return;
  clearTimeout(autoRouteTimer);
  autoRouteSignature = signature;
  const revision = ++autoRouteRevision;
  route = null;
  if (webgl) webgl.routeSignature = '';
  rebuildRoute3d(); drawEditor();
  $('route-result').textContent = 'MENGHITUNG JALUR TERCEPAT…';
  autoRouteTimer = setTimeout(async () => {
    try {
      const planned = await request('/api/route', { method: 'POST', body: JSON.stringify({ start, destination }) });
      if (revision !== autoRouteRevision) return;
      route = planned;
      const byId = Object.fromEntries(nodeList().map((node) => [node.id, node]));
      const path = route.nodes.map((id) => `${id} ${byId[id]?.name ?? ''}`.trim()).join(' → ');
      $('route-result').textContent = `${path} · ${route.distance_m.toFixed(2)} M`;
    } catch (error) {
      if (revision !== autoRouteRevision) return;
      route = null;
      $('route-result').textContent = `TIDAK ADA JALUR · ${error.message}`;
    }
    if (webgl) webgl.routeSignature = '';
    rebuildRoute3d(); drawEditor(); setHardwareAvailability();
  }, 120);
}

async function load() {
  state = await request('/api/state');
  nodeOptionSignature = '';
  render();
}

async function control(type, extra = {}) {
  try {
    const response = await request('/api/control', { method: 'POST', body: JSON.stringify({ type, ...extra }) });
    if (type !== 'motor.manual') toast('Perintah dikirim', type);
    return response;
  } catch (error) {
    if (type !== 'motor.manual' || state?.connection?.connected) toast('Perintah gagal', error.message, 'error');
    return null;
  }
}

async function ports() {
  try {
    const list = await request('/api/ports');
    $('port').innerHTML = '<option value="">Pilih port COM</option>' + list.map((port) => `<option value="${escapeHtml(port.device)}">${escapeHtml(port.device)} · ${escapeHtml(port.description)}</option>`).join('');
    if (!list.length) toast('Port tidak ditemukan', 'Hubungkan ESP32 lalu tekan Refresh.', 'warning');
    setHardwareAvailability();
  } catch (error) { toast('Gagal membaca port', error.message, 'error'); }
}

function openTab(name) {
  document.querySelectorAll('[data-tab]').forEach((button) => {
    const active = button.dataset.tab === name;
    button.setAttribute('aria-selected', String(active));
    button.classList.toggle('active', active);
  });
  document.querySelectorAll('.tab-panel').forEach((panel) => {
    const active = panel.id === `panel-${name}`;
    panel.hidden = !active; panel.classList.toggle('active', active);
  });
}

function showView(kind) {
  const editor = kind === 'editor';
  currentView = editor ? 'editor' : '3d';
  $('map3d').classList.toggle('hidden', editor);
  canvas.classList.toggle('hidden', !editor);
  $('editor-help').classList.toggle('hidden', !editor);
  $('view-3d').classList.toggle('active', !editor);
  $('view-editor').classList.toggle('active', editor);
  $('view-3d').setAttribute('aria-pressed', String(!editor));
  $('view-editor').setAttribute('aria-pressed', String(editor));
  if (editor) { $('transformbar').classList.add('hidden'); drawEditor(); openTab('map'); }
  else { resize3d(); resetCamera(); attachSelectedObject(); refreshSelected(); }
}

function commandFromKeys() {
  const linear = numeric('linear'); const turn = numeric('turn');
  let velocity = 0; let angular = 0;
  if (held.has('w')) velocity += linear;
  if (held.has('s')) velocity -= linear;
  if (held.has('a')) angular += turn;
  if (held.has('d')) angular -= turn;
  if (held.has('q')) { velocity = 0; angular += turn; }
  if (held.has('e')) { velocity = 0; angular -= turn; }
  return [velocity, angular];
}

function holdDrive(key, active) {
  if (!state?.connection?.connected || state.mode !== 'hardware') return;
  active ? held.add(key) : held.delete(key);
  document.querySelector(`[data-drive="${key}"]`)?.classList.toggle('active', active);
  if (!held.size) control('motor.manual', { linear_mps: 0, angular_radps: 0, heartbeat: true });
}

document.querySelectorAll('[data-tab]').forEach((button) => button.addEventListener('click', () => openTab(button.dataset.tab)));
document.querySelectorAll('[data-open-tab]').forEach((button) => button.addEventListener('click', () => {
  openTab(button.dataset.openTab);
  if (innerWidth <= 1180) $(`panel-${button.dataset.openTab}`).scrollIntoView({ behavior: reducedMotion ? 'auto' : 'smooth', block: 'start' });
}));
$('view-3d').onclick = () => showView('3d');
$('view-editor').onclick = () => showView('editor');
$('fallback-editor').onclick = () => showView('editor');
$('reset-view').onclick = resetCamera;
$('camera-perspective').onclick = resetCamera;
$('camera-top').onclick = topCamera;
$('zoom-in').onclick = () => zoomCamera(.84);
$('zoom-out').onclick = () => zoomCamera(1.18);
$('render-toggle').onclick = () => {
  renderingPaused = !renderingPaused;
  $('render-toggle').textContent = renderingPaused ? '▶ RESUME' : 'Ⅱ PAUSE';
  $('render-toggle').classList.toggle('on', renderingPaused);
};
document.querySelectorAll('[data-open-editor]').forEach((button) => button.onclick = () => showView('editor'));
document.querySelectorAll('[data-add-obstacle]').forEach((button) => button.onclick = addObstacle);
$('add-obstacle').onclick = addObstacle;
document.querySelectorAll('[data-transform]').forEach((button) => button.onclick = () => setTransformMode(button.dataset.transform));
$('save-room').onclick = resizeRoom;
$('add-node').onclick = addNode;
$('reset-nodes').onclick = resetNodePositions;
$('refresh-ports').onclick = ports;
$('port').onchange = setHardwareAvailability;
$('mode').onchange = async () => {
  try { await request(`/api/mode/${$('mode').value}`, { method: 'POST' }); await load(); if (state.mode === 'hardware') await ports(); toast('Mode diubah', $('mode').value.toUpperCase()); }
  catch (error) { toast('Mode tidak dapat diubah', error.message, 'error'); }
};
$('connect').onclick = async () => {
  try { await request('/api/connect', { method: 'POST', body: JSON.stringify({ port: $('port').value, baud_rate: numeric('baud') }) }); toast('ESP32 terhubung', $('port').value); await load(); }
  catch (error) { toast('Koneksi gagal', error.message, 'error'); }
};
$('disconnect').onclick = async () => { await request('/api/disconnect', { method: 'POST' }); toast('ESP32 diputus', 'Motor command dinonaktifkan.'); await load(); };
$('apply-position-source').onclick = () => control('position.source', { source: $('position-source').value });
$('set-pose').onclick = () => control('odometry.set_pose', { x_m: numeric('pose-x'), y_m: numeric('pose-y'), heading_rad: numeric('pose-heading') });
$('route-start').onchange = () => { autoRouteSignature = ''; scheduleAutoRoute(); };
$('route-destination').onchange = () => { autoRouteSignature = ''; scheduleAutoRoute(); };
$('send-destination').onclick = () => control('navigation.destination', { node_id: Number($('route-destination').value) });
document.querySelectorAll('[data-command]').forEach((button) => button.onclick = () => control(button.dataset.command));
document.querySelectorAll('[data-pid]').forEach((button) => button.onclick = () => control('pid.set', { motor: button.dataset.pid, kp: numeric(`${button.dataset.pid}-kp`), ki: numeric(`${button.dataset.pid}-ki`), kd: numeric(`${button.dataset.pid}-kd`) }));
$('linear').oninput = () => { $('linear-value').textContent = `${$('linear').value} m/s`; };
$('turn').oninput = () => { $('turn-value').textContent = `${$('turn').value} rad/s`; };
$('ob-atten').oninput = () => { $('ob-atten-value').textContent = Number($('ob-atten').value).toFixed(2); };
$('start-manual').onclick = () => control('motor.manual', { linear_mps: 0, angular_radps: 0, heartbeat: true });
$('save-node').onclick = async () => {
  const node = selectedNode();
  if (!node) return;
  node.name = $('node-name').value.trim();
  node.type = $('node-type').value;
  node.x_m = numeric('node-x'); node.y_m = numeric('node-y');
  try { await saveGraph(); toast('Node diperbarui', `${node.id} · ${node.name}`); }
  catch (error) { toast('Node tidak tersimpan', error.message, 'error'); await load(); }
};
$('delete-node').onclick = async () => {
  const node = selectedNode();
  if (!node) return;
  const label = `${node.id} · ${node.name}`;
  webgl?.transform.detach();
  state.graph.nodes = nodeList().filter((item) => Number(item.id) !== Number(node.id));
  selectedNodeId = null;
  try { await saveGraph(); toast('Node dihapus', label, 'warning'); }
  catch (error) { toast('Node tidak dapat dihapus', error.message, 'error'); await load(); }
};
$('save-obstacle').onclick = async () => {
  try {
    const obstacle = selected();
    obstacle.label = $('ob-name').value.trim() || obstacle.id;
    obstacle.x = numeric('ob-x'); obstacle.y = numeric('ob-y');
    obstacle.sx = numeric('ob-width'); obstacle.sy = numeric('ob-depth');
    obstacle.sz = numeric('ob-height'); obstacle.z = obstacle.sz / 2;
    obstacle.rot = numeric('ob-rotation') * Math.PI / 180;
    obstacle.atten = numeric('ob-atten');
    obstacle.products = $('ob-products').value.split(',').map((value) => value.trim()).filter(Boolean).map((entry, index) => {
      const split = entry.lastIndexOf('@');
      if (split < 1) throw new Error('Produk harus memakai format nama@node');
      const name = entry.slice(0, split).trim(); const node_id = Number(entry.slice(split + 1));
      if (!name || !Number.isInteger(node_id)) throw new Error('Produk harus memakai format nama@node');
      return { id: `${obstacle.id}-product-${index + 1}`, name, node_id };
    });
    await saveScene();
    rebuildStatic3d();
    toast('Rak diperbarui', obstacle.label);
  } catch (error) { toast('Rak tidak tersimpan', error.message, 'error'); }
};
$('delete-obstacle').onclick = async () => {
  const label = selected()?.label ?? selectedId;
  webgl?.transform.detach();
  state.scene.scene.obstacles = obstacleList().filter((obstacle) => obstacle.id !== selectedId);
  selectedId = null; refreshSelected();
  try { await saveScene(); rebuildStatic3d(); toast('Rak dihapus', label, 'warning'); }
  catch (error) { toast('Rak tidak dapat dihapus', error.message, 'error'); await load(); }
};

document.querySelectorAll('[data-drive]').forEach((button) => {
  const key = button.dataset.drive;
  button.addEventListener('pointerdown', (event) => { event.preventDefault(); button.setPointerCapture(event.pointerId); holdDrive(key, true); });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((name) => button.addEventListener(name, () => holdDrive(key, false)));
});
window.addEventListener('keydown', (event) => {
  const key = event.key.toLowerCase();
  if (key === 'h' && !['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName)) { event.preventDefault(); resetCamera(); return; }
  if (event.code === 'Space') { if (state?.connection?.connected) { event.preventDefault(); control('estop'); } return; }
  if ('wasdqe'.includes(key) && state?.connection?.connected && state.mode === 'hardware') { held.add(key); document.querySelector(`[data-drive="${key}"]`)?.classList.add('active'); event.preventDefault(); }
});
window.addEventListener('keyup', (event) => holdDrive(event.key.toLowerCase(), false));
window.addEventListener('blur', () => {
  if (!held.size) return;
  held.clear(); document.querySelectorAll('[data-drive]').forEach((button) => button.classList.remove('active'));
  control('motor.manual', { linear_mps: 0, angular_radps: 0 });
});
setInterval(() => {
  if (!held.size) return;
  const [linear_mps, angular_radps] = commandFromKeys();
  control('motor.manual', { linear_mps, angular_radps, heartbeat: true });
}, 100);
setInterval(() => { $('clock').textContent = new Date().toLocaleTimeString('id-ID', { hour12: false }); }, 1000);

function connectWebsocket() {
  const socket = new WebSocket(`ws://${location.host}/ws`);
  socket.onmessage = (event) => {
    state = JSON.parse(event.data);
    if (state.connection?.connected && state.connection.last_rx_age_ms != null) telemetryReceivedAt = performance.now() - state.connection.last_rx_age_ms;
    render();
  };
  socket.onclose = () => setTimeout(connectWebsocket, 1400);
  socket.onerror = () => socket.close();
}

initWebgl();
load().then(() => {
  const query = new URLSearchParams(location.search);
  const initialTab = query.get('tab');
  if (['mission', 'drive', 'diagnostics', 'map'].includes(initialTab)) openTab(initialTab);
  const initialSelection = query.get('select');
  if (initialSelection && obstacleList().some((obstacle) => obstacle.id === initialSelection)) {
    selectedId = initialSelection; selectedNodeId = null;
    refreshSelected();
    rebuildStatic3d();
  }
  const initialNode = Number(query.get('node'));
  if (Number.isInteger(initialNode) && nodeList().some((node) => Number(node.id) === initialNode)) {
    selectedId = null; selectedNodeId = initialNode;
    refreshSelected(); rebuildStatic3d();
  }
  if (query.get('view') === 'editor') showView('editor');
  const boot = $('boot-screen');
  boot.classList.add('done');
  setTimeout(() => boot.remove(), reducedMotion ? 10 : 320);
  if (state.mode === 'hardware') ports();
  connectWebsocket();
}).catch((error) => {
  $('boot-screen').classList.add('done');
  toast('Dashboard gagal dimuat', error.message, 'error');
});
