/**
 * three.js scene renderer synchronised with the Station item tree.
 * Units: mm, Z up. Every item gets a flat Object3D whose matrix is the item's absolute pose.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { Station, Item, ItemType, Frame, Target, Tool, SceneObject, Camera as CameraItem } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { PathItem, Program } from '../core/items/program';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { Component } from '../vc/component';
import { FieldItem, CropRow } from '../agri/items';
import { AssetStore } from './assets';
import { buildGeometry, buildLinkVisuals, makeTriad, makeTargetGlyph, polylineObject, polygonMesh, mat4ToThree, materialFor } from './three-geometry';
import { Mat4, multiply, invert, getPos, fromArray, transl } from '../core/math/pose';
import { actuatedIndices } from '../core/kinematics/chain';
import { Trajectory } from '../core/motion/trajectory';

export interface RendererOptions {
  background?: number;
  showGrid?: boolean;
  showShadows?: boolean;
}

interface Entry {
  item: Item;
  root: THREE.Group;
  /** Structural signature used to decide when to rebuild geometry. */
  sig: string;
  /** Sub-objects that need per-frame updates (robot links). */
  links?: THREE.Group[];
  flangeGroup?: THREE.Group;
  selectionBox?: THREE.BoxHelper;
}

export type SelectCallback = (item: Item | null, additive: boolean) => void;
export type PoseEditCallback = (item: Item, pose: Mat4) => void;

export class SceneRenderer {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly controls: OrbitControls;
  readonly transform: TransformControls;
  private entries = new Map<string, Entry>();
  private dirty = true;
  private structural = new Set<string>();
  private station: Station;
  private raycaster = new THREE.Raycaster();
  private pointer = new THREE.Vector2();
  private grid: THREE.GridHelper;
  private trajectoryLine: THREE.Line | null = null;
  private gizmoTarget: Item | null = null;
  private gizmoProxy = new THREE.Object3D();
  private offTree: Array<() => void> = [];
  private robotColors = ['#e8a33d', '#3d8ee8', '#e85d3d', '#7bc043', '#b25de8', '#3de8c9'];
  onSelect: SelectCallback | null = null;
  onPoseEdit: PoseEditCallback | null = null;
  onGizmoEnd: (() => void) | null = null;
  showFrames = true;
  showTargets = true;
  /** Draw a translucent reach sphere around each robot base. */
  showReach = false;
  showTrajectory = true;
  frameSize = 150;
  private timer = new THREE.Timer();
  private animHandlers = new Set<(dt: number) => void>();
  private labelLayer: HTMLDivElement | null = null;
  private collisionIds = new Set<string>();

  /** Highlight items involved in collisions (red boxes). */
  setCollisionHighlight(ids: string[]): void {
    this.collisionIds = new Set(ids);
    this.dirty = true;
  }

  constructor(readonly canvasHost: HTMLElement, station: Station, readonly assets: AssetStore, opts: RendererOptions = {}) {
    this.station = station;
    THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    this.renderer.shadowMap.enabled = opts.showShadows ?? true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    canvasHost.appendChild(this.renderer.domElement);
    this.scene.background = new THREE.Color(opts.background ?? 0x1e2229);
    this.scene.fog = new THREE.Fog(opts.background ?? 0x1e2229, 60000, 250000);
    this.camera = new THREE.PerspectiveCamera(45, 1, 5, 1e6);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(4000, -4500, 3000);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 500);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.PAN };
    this.transform = new TransformControls(this.camera, this.renderer.domElement);
    this.transform.setSize(0.8);
    this.transform.addEventListener('dragging-changed', (e: any) => { this.controls.enabled = !e.value; });
    this.transform.addEventListener('objectChange', () => this.onGizmoChange());
    this.transform.addEventListener('mouseUp', () => this.onGizmoEnd?.());
    this.scene.add(this.transform.getHelper());
    this.scene.add(this.gizmoProxy);

    // lights
    const hemi = new THREE.HemisphereLight(0xdfe8ff, 0x30363d, 0.9);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(20000, -15000, 30000);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -20000; sun.shadow.camera.right = 20000; sun.shadow.camera.top = 20000; sun.shadow.camera.bottom = -20000;
    sun.shadow.camera.near = 1000; sun.shadow.camera.far = 100000;
    sun.shadow.bias = -0.0005;
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0x88aaff, 0.35);
    fill.position.set(-20000, 20000, 10000);
    this.scene.add(fill);

    // ground + grid
    this.grid = new THREE.GridHelper(20000, 40, 0x3a4250, 0x2b313b);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.visible = opts.showGrid ?? true;
    this.scene.add(this.grid);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(400000, 400000), new THREE.ShadowMaterial({ opacity: 0.25 }));
    ground.receiveShadow = true;
    ground.name = 'ground';
    ground.userData.isGround = true;
    ground.position.z = -1;
    this.scene.add(ground);
    this.scene.add(makeTriad(500, 2));

    this.renderer.domElement.addEventListener('pointerdown', (e) => this.onPointerDown(e));
    this.renderer.domElement.addEventListener('pointerup', (e) => this.onPointerUp(e));
    this.renderer.domElement.addEventListener('dblclick', (e) => this.focusAt(e));
    window.addEventListener('resize', () => this.resize());
    this.assets.onChange(() => this.rebuildAll());
    this.bindStation(station);
    this.resize();
    this.renderer.setAnimationLoop(() => this.frame());
  }

  bindStation(station: Station): void {
    for (const off of this.offTree) off();
    this.offTree = [];
    this.station = station;
    for (const e of this.entries.values()) this.scene.remove(e.root);
    this.entries.clear();
    this.setGizmo(null);
    this.offTree.push(station.events.on('changed', ({ item, what }) => {
      this.dirty = true;
      if (!/^pose$|^joints$|^visible$|^name$|^selection$|^batch$/.test(what)) this.structural.add(item.id);
      if (what === 'batch') this.rebuildAll();
    }));
    this.offTree.push(station.events.on('childAdded', () => { this.dirty = true; this.rebuildAll(); }));
    this.offTree.push(station.events.on('childRemoved', ({ child }) => { this.removeEntry(child); this.dirty = true; }));
    this.offTree.push(station.events.on('selection', () => { this.dirty = true; }));
    this.rebuildAll();
  }

  onAnimate(fn: (dt: number) => void): () => void {
    this.animHandlers.add(fn);
    return () => this.animHandlers.delete(fn);
  }

  private pointerDownPos: [number, number] | null = null;
  private onPointerDown(e: PointerEvent) {
    this.pointerDownPos = [e.clientX, e.clientY];
  }
  private onPointerUp(e: PointerEvent) {
    if (!this.pointerDownPos) return;
    const moved = Math.hypot(e.clientX - this.pointerDownPos[0], e.clientY - this.pointerDownPos[1]);
    this.pointerDownPos = null;
    if (moved > 4 || e.button !== 0 || this.transform.dragging) return;
    const item = this.pick(e.clientX, e.clientY);
    this.onSelect?.(item, e.shiftKey || e.ctrlKey);
  }

  pick(clientX: number, clientY: number): Item | null {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.set(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    this.raycaster.params.Line = { threshold: 40 };
    const objs: THREE.Object3D[] = [];
    for (const e of this.entries.values()) if (e.root.visible) objs.push(e.root);
    const hits = this.raycaster.intersectObjects(objs, true);
    for (const h of hits) {
      let o: THREE.Object3D | null = h.object;
      while (o && !o.userData.itemId) o = o.parent;
      if (o?.userData.itemId) {
        const it = this.station.findById(o.userData.itemId);
        if (it) return it;
      }
    }
    return null;
  }

  private focusAt(e: MouseEvent) {
    const item = this.pick(e.clientX, e.clientY);
    if (item) this.focusItem(item);
  }

  focusItem(item: Item): void {
    const e = this.entries.get(item.id);
    const box = new THREE.Box3();
    if (e) box.setFromObject(e.root);
    if (box.isEmpty()) {
      const p = getPos(item.poseAbs());
      box.setFromCenterAndSize(new THREE.Vector3(p[0], p[1], p[2]), new THREE.Vector3(500, 500, 500));
    }
    const c = box.getCenter(new THREE.Vector3());
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 500);
    const dir = this.camera.position.clone().sub(this.controls.target).normalize();
    this.controls.target.copy(c);
    this.camera.position.copy(c.clone().add(dir.multiplyScalar(size * 1.4)));
    this.dirty = true;
  }

  fitAll(): void {
    const box = new THREE.Box3();
    for (const e of this.entries.values()) if (e.root.visible && e.item.type !== ItemType.MAP) box.expandByObject(e.root);
    if (box.isEmpty()) return;
    const c = box.getCenter(new THREE.Vector3());
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 1000);
    this.controls.target.copy(c);
    this.camera.position.set(c.x + size * 0.7, c.y - size * 0.9, c.z + size * 0.6);
    this.camera.far = Math.max(1e6, size * 20);
    this.camera.updateProjectionMatrix();
    this.dirty = true;
  }

  setView(v: 'top' | 'front' | 'side' | 'iso'): void {
    const t = this.controls.target.clone();
    const d = this.camera.position.distanceTo(t) || 5000;
    const p = v === 'top' ? [0, 0.0001, 1] : v === 'front' ? [0, -1, 0.0001] : v === 'side' ? [1, 0, 0.0001] : [0.6, -0.7, 0.5];
    this.camera.position.set(t.x + p[0] * d, t.y + p[1] * d, t.z + p[2] * d);
    this.dirty = true;
  }

  resize(): void {
    const w = this.canvasHost.clientWidth || 800, h = this.canvasHost.clientHeight || 600;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.dirty = true;
  }

  // -- Gizmo ---------------------------------------------------------------

  setGizmo(item: Item | null, mode: 'translate' | 'rotate' = 'translate'): void {
    this.gizmoTarget = item;
    if (!item || item.type === ItemType.STATION || item.type === ItemType.INSTRUCTION || item.type === ItemType.PROGRAM) {
      this.transform.detach();
      return;
    }
    const m = mat4ToThree(item.poseAbs());
    this.gizmoProxy.matrix.copy(m);
    this.gizmoProxy.matrix.decompose(this.gizmoProxy.position, this.gizmoProxy.quaternion, this.gizmoProxy.scale);
    this.transform.setMode(mode);
    this.transform.attach(this.gizmoProxy);
  }
  setGizmoMode(mode: 'translate' | 'rotate'): void {
    this.transform.setMode(mode);
  }
  setGizmoSpace(space: 'world' | 'local'): void {
    this.transform.setSpace(space);
  }
  private onGizmoChange() {
    if (!this.gizmoTarget) return;
    this.gizmoProxy.updateMatrix();
    const m = fromArray(this.gizmoProxy.matrix.elements);
    this.onPoseEdit?.(this.gizmoTarget, m);
  }

  // -- Trajectory preview --------------------------------------------------

  showTrajectoryPreview(robot: Robot | null, trajs: Trajectory[]): void {
    if (this.trajectoryLine) { this.scene.remove(this.trajectoryLine); this.trajectoryLine = null; }
    if (!robot || !trajs.length) return;
    const base = robot.poseAbs();
    const pts: THREE.Vector3[] = [];
    const colors: number[] = [];
    for (const tr of trajs) {
      const ok = tr.ok;
      for (const s of tr.samples) {
        const p = getPos(multiply(base, s.pose));
        pts.push(new THREE.Vector3(p[0], p[1], p[2]));
        colors.push(ok ? 0.2 : 1, ok ? 0.9 : 0.2, ok ? 0.4 : 0.2);
      }
    }
    if (!pts.length) return;
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    this.trajectoryLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ vertexColors: true }));
    this.trajectoryLine.renderOrder = 5;
    this.scene.add(this.trajectoryLine);
    this.dirty = true;
  }

  // -- Sync ---------------------------------------------------------------

  rebuildAll(): void {
    const live = new Set<string>();
    for (const item of this.station.walk()) {
      if (item === this.station) continue;
      live.add(item.id);
      this.ensureEntry(item);
    }
    for (const id of [...this.entries.keys()]) if (!live.has(id)) this.removeEntry(this.entries.get(id)!.item);
    this.structural.clear();
    this.dirty = true;
  }

  private removeEntry(item: Item) {
    for (const it of item.walk()) {
      const e = this.entries.get(it.id);
      if (e) { this.scene.remove(e.root); this.entries.delete(it.id); }
    }
  }

  private signature(item: Item): string {
    if (item instanceof Robot) return `robot:${item.chain.joints.length}:${item.chain.links.map((l) => l.visuals.length).join(',')}:${item.color}:${this.showReach}`;
    if (item instanceof Tool || item instanceof SceneObject) return `obj:${JSON.stringify((item as SceneObject).geometry)}:${item.color}:${(item as any).curves?.length}:${(item as any).points?.length}`;
    if (item instanceof MobileRobot) return `mob:${JSON.stringify(item.kin.footprint)}:${JSON.stringify(item.geometry)}:${item.color}`;
    if (item instanceof FieldItem) return `field:${JSON.stringify(item.polygon)}:${item.rows().map((r) => r.plants.length + ':' + r.plants.reduce((s, p) => s + p.fruit.filter((f) => f.picked).length, 0)).join(',')}:${item.crop.crop}`;
    if (item instanceof CropRow) return `row`;
    if (item instanceof MapItem) return `map:${item.width}x${item.height}:${item.cells.length}:${hashCells(item.cells)}`;
    if (item instanceof ZoneItem) return `zone:${item.kind}:${JSON.stringify(item.polygon)}`;
    if (item instanceof PathItem) return `path:${item.points.length}:${item.closed}`;
    if (item instanceof Component) return `comp:${item.behaviour.type}:${JSON.stringify(item.geometry)}:${JSON.stringify((item.behaviour as any).path ?? null)}`;
    if (item instanceof Target) return `target:${item.isJointTarget}`;
    if (item instanceof CameraItem) return `cam:${item.fov}`;
    return `item:${item.type}`;
  }

  private ensureEntry(item: Item): Entry {
    let e = this.entries.get(item.id);
    const sig = this.signature(item);
    if (e && e.sig === sig && !this.structural.has(item.id)) return e;
    if (e) this.scene.remove(e.root);
    const root = new THREE.Group();
    root.matrixAutoUpdate = false;
    root.userData.itemId = item.id;
    e = { item, root, sig };
    this.buildItem(item, e);
    this.scene.add(root);
    this.entries.set(item.id, e);
    return e;
  }

  private buildItem(item: Item, e: Entry) {
    const root = e.root;
    if (item instanceof Robot) this.buildRobot(item, e);
    else if (item instanceof Tool) {
      const g = buildGeometry(item.geometry, this.assets, item.color ?? '#5b6470');
      // geometry is defined in flange coordinates: root will be positioned at the flange
      root.add(g);
      if (!item.geometry.length) {
        const tcp = getPos(item.pose());
        const len = Math.hypot(tcp[0], tcp[1], tcp[2]);
        if (len > 1) {
          const cyl = new THREE.Mesh(new THREE.CylinderGeometry(25, 35, len, 16).rotateX(Math.PI / 2), materialFor(item.color ?? '#5b6470'));
          cyl.position.set(tcp[0] / 2, tcp[1] / 2, tcp[2] / 2);
          cyl.lookAt(new THREE.Vector3(tcp[0], tcp[1], tcp[2]));
          root.add(cyl);
        }
        const flangeDisc = new THREE.Mesh(new THREE.CylinderGeometry(45, 45, 12, 24).rotateX(Math.PI / 2), materialFor('#3b4149'));
        root.add(flangeDisc);
      }
      const triad = makeTriad(this.frameSize * 0.8);
      triad.matrixAutoUpdate = false;
      triad.matrix.copy(mat4ToThree(item.pose()));
      triad.userData.isFrame = true;
      root.add(triad);
    } else if (item instanceof SceneObject) {
      const g = buildGeometry(item.geometry, this.assets, item.color ?? '#9aa3ad');
      root.add(g);
      for (const c of item.curves) root.add(polylineObject(c.points, 0xffcc00));
      if (item.points.length > 50) {
        // point cloud (imported .pcd/.ply or vision export): coloured by height
        const n = item.points.length; const pos = new Float32Array(n * 3); const col = new Float32Array(n * 3);
        let zMin = Infinity, zMax = -Infinity; for (const p of item.points) { zMin = Math.min(zMin, p.point[2]); zMax = Math.max(zMax, p.point[2]); }
        item.points.forEach((p, i) => { pos[i * 3] = p.point[0]; pos[i * 3 + 1] = p.point[1]; pos[i * 3 + 2] = p.point[2]; const tz = zMax > zMin ? (p.point[2] - zMin) / (zMax - zMin) : 0.5; col[i * 3] = 0.2 + 0.8 * tz; col[i * 3 + 1] = 0.9 - 0.5 * tz; col[i * 3 + 2] = 1 - tz; });
        const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3));
        root.add(new THREE.Points(g, new THREE.PointsMaterial({ size: 25, vertexColors: true })));
      }
      if (!item.geometry.length && !item.curves.length && !item.points.length) {
        const ph = new THREE.Mesh(new THREE.BoxGeometry(200, 200, 200), materialFor(item.color ?? '#9aa3ad', 0.5));
        root.add(ph);
      }
    } else if (item instanceof Target) {
      const glyph = makeTargetGlyph(this.frameSize * 0.5, item.isJointTarget ? 0x66ccff : 0xffaa00);
      glyph.userData.isTarget = true;
      root.add(glyph);
    } else if (item instanceof Frame) {
      const triad = makeTriad(this.frameSize);
      triad.userData.isFrame = true;
      root.add(triad);
    } else if (item instanceof MobileRobot) this.buildMobile(item, e);
    else if (item instanceof FieldItem) this.buildField(item, e);
    else if (item instanceof CropRow) {
      /* rendered by the field */
    } else if (item instanceof MapItem) this.buildMap(item, e);
    else if (item instanceof ZoneItem) {
      const color = item.kind === 'nogo' ? 0xe03131 : item.kind === 'charging' ? 0x1c7ed6 : item.kind === 'headland' ? 0xf59f00 : item.kind === 'gnss_denied' ? 0x9c36b5 : 0x2f9e44;
      root.add(polygonMesh(item.polygon, color, 0.25, 2));
      root.add(polylineObject(item.polygon, color, true, 3));
    } else if (item instanceof PathItem) {
      root.add(polylineObject(item.points, 0x33aaff, item.closed, 5));
    } else if (item instanceof Component) {
      root.add(buildGeometry(item.geometry, this.assets, item.color ?? '#6c757d'));
      const b = item.behaviour as any;
      if (b.path) root.add(polylineObject(b.path, 0x00d1b2, false, 2));
      if (!item.geometry.length) {
        const ph = new THREE.Mesh(new THREE.BoxGeometry(300, 300, 300), materialFor(item.color ?? '#6c757d', 0.5));
        ph.position.z = 150;
        root.add(ph);
      }
    } else if (item instanceof CameraItem) {
      const cam = new THREE.PerspectiveCamera(item.fov, item.width / item.height, item.near, Math.min(item.far, 3000));
      cam.rotateY(Math.PI); // look down +Z of the item
      const helper = new THREE.CameraHelper(cam);
      root.add(cam);
      root.add(helper);
    } else if (item.type === ItemType.MISSION || item.type === ItemType.FLEET || item.type === ItemType.PROGRAM || item.type === ItemType.INSTRUCTION || item.type === ItemType.FOLDER || item.type === ItemType.NOTES) {
      // no 3D representation
    } else {
      root.add(makeTriad(this.frameSize * 0.6));
    }
  }

  private buildRobot(robot: Robot, e: Entry) {
    const idx = [...this.entries.values()].filter((x) => x.item instanceof Robot).length;
    const color = robot.color ?? this.robotColors[idx % this.robotColors.length];
    const chain = robot.chain;
    const links: THREE.Group[] = [];
    const hasMeshes = chain.links.some((l) => l.visuals.length > 0);
    const reach = Math.max(300, robot.reach);
    const fk = robot.fk(robot.jointsHome());
    for (let i = 0; i < chain.links.length; i++) {
      const g = new THREE.Group();
      g.matrixAutoUpdate = false;
      g.userData.itemId = robot.id;
      const link = chain.links[i];
      if (link.visuals.length) g.add(buildLinkVisuals(link.visuals, this.assets, color));
      else if (!hasMeshes) this.addProceduralLink(g, robot, i, fk, reach, color);
      links.push(g);
      e.root.add(g);
    }
    const flange = new THREE.Group();
    flange.matrixAutoUpdate = false;
    const triad = makeTriad(this.frameSize * 0.6);
    triad.userData.isFrame = true;
    flange.add(triad);
    e.root.add(flange);
    e.links = links;
    e.flangeGroup = flange;
    // base triad
    const baseTriad = makeTriad(this.frameSize);
    baseTriad.userData.isFrame = true;
    e.root.add(baseTriad);
    if (this.showReach) {
      const sphere = new THREE.Mesh(new THREE.SphereGeometry(robot.reach, 32, 20), new THREE.MeshBasicMaterial({ color: 0x4dabf7, transparent: true, opacity: 0.08, depthWrite: false, side: THREE.DoubleSide }));
      const wire = new THREE.Mesh(new THREE.SphereGeometry(robot.reach, 24, 12), new THREE.MeshBasicMaterial({ color: 0x4dabf7, wireframe: true, transparent: true, opacity: 0.15 }));
      // centre the sphere on the shoulder (first joint origin) for a better approximation
      const j0 = chain.joints[0];
      const c = j0 ? [j0.origin[12], j0.origin[13], j0.origin[14]] : [0, 0, 0];
      sphere.position.set(c[0], c[1], c[2]);
      wire.position.set(c[0], c[1], c[2]);
      sphere.raycast = () => {};
      wire.raycast = () => {};
      e.root.add(sphere, wire);
    }
  }

  /** Procedural link geometry: cylinders from the joint pivot to the link end and joint hubs. */
  private addProceduralLink(g: THREE.Group, robot: Robot, linkIndex: number, fk: ReturnType<Robot['fk']>, reach: number, color: string) {
    const chain = robot.chain;
    const n = chain.joints.length;
    const r0 = Math.max(18, reach / 22);
    const radius = r0 * (1 - 0.55 * (linkIndex / Math.max(1, n)));
    const mat = materialFor(color);
    const dark = materialFor('#2f343b');
    const addCyl = (a: THREE.Vector3, b: THREE.Vector3, r: number, m: THREE.Material) => {
      const len = a.distanceTo(b);
      if (len < 1) return;
      const cyl = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 20), m);
      cyl.castShadow = true;
      cyl.position.copy(a).add(b).multiplyScalar(0.5);
      cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
      g.add(cyl);
      // rounded ends
      const s1 = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), m);
      s1.position.copy(b);
      g.add(s1);
    };
    if (linkIndex === 0) {
      // base pedestal: from base to first joint origin
      const j0 = chain.joints[0];
      if (j0) {
        const p = new THREE.Vector3(j0.origin[12], j0.origin[13], j0.origin[14]);
        const base = new THREE.Mesh(new THREE.CylinderGeometry(r0 * 1.6, r0 * 2, Math.max(30, r0), 28).rotateX(Math.PI / 2), dark);
        base.position.z = Math.max(15, r0 / 2);
        g.add(base);
        addCyl(new THREE.Vector3(0, 0, 0), p, r0 * 1.1, mat);
        const hub = new THREE.Mesh(new THREE.CylinderGeometry(r0 * 1.2, r0 * 1.2, r0 * 1.6, 24).rotateX(Math.PI / 2), dark);
        hub.position.copy(p);
        hub.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(j0.axis[0], j0.axis[1], j0.axis[2]).normalize());
        g.add(hub);
      }
      return;
    }
    const jIdx = linkIndex - 1;
    const j = chain.joints[jIdx];
    // pivot of this link in the link frame: inv(post) applied to origin (motion is a rotation about the pivot)
    const post = j.post ?? null;
    const pivot = post ? getPos(invert(post)) : [0, 0, 0];
    const pv = new THREE.Vector3(pivot[0], pivot[1], pivot[2]);
    addCyl(pv, new THREE.Vector3(0, 0, 0), radius, mat);
    const next = chain.joints[jIdx + 1];
    if (next) {
      const t = new THREE.Vector3(next.origin[12], next.origin[13], next.origin[14]);
      addCyl(new THREE.Vector3(0, 0, 0), t, radius * 0.95, mat);
      // hub at the next joint along its axis (expressed in this frame via next.origin rotation)
      const axisLocal = new THREE.Vector3(next.axis[0], next.axis[1], next.axis[2]).applyMatrix4(new THREE.Matrix4().extractRotation(mat4ToThree(next.origin))).normalize();
      const hub = new THREE.Mesh(new THREE.CylinderGeometry(radius * 1.25, radius * 1.25, radius * 2.2, 24).rotateX(Math.PI / 2), dark);
      hub.position.copy(t);
      hub.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axisLocal);
      g.add(hub);
    } else {
      // flange
      const f = new THREE.Vector3(chain.flange[12], chain.flange[13], chain.flange[14]);
      addCyl(new THREE.Vector3(0, 0, 0), f, radius * 0.8, mat);
    }
    if (j.type === 'prismatic') {
      // slider rail along the axis
      const len = Math.max(50, j.upper - j.lower);
      const rail = new THREE.Mesh(new THREE.BoxGeometry(radius * 1.2, radius * 1.2, len), dark);
      const ax = new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize();
      rail.position.copy(pv).add(ax.clone().multiplyScalar(-len / 2));
      rail.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), ax);
      g.add(rail);
    }
    void fk;
  }

  private buildMobile(m: MobileRobot, e: Entry) {
    const root = e.root;
    if (m.geometry.length) root.add(buildGeometry(m.geometry, this.assets, m.color));
    else {
      const [L, W, H] = m.kin.footprint;
      const body = new THREE.Mesh(new THREE.BoxGeometry(L, W, H * 0.6), materialFor(m.color ?? '#2b8a3e'));
      body.position.z = H * 0.5;
      body.castShadow = true;
      root.add(body);
      const wheelR = m.kin.wheelRadius, wheelW = Math.max(60, W * 0.12);
      const wheelMat = materialFor('#22262b');
      const positions = m.kin.drive === 'ackermann' || m.kin.drive === 'tracked' ? [[L * 0.35, W / 2 + wheelW / 2], [L * 0.35, -W / 2 - wheelW / 2], [-L * 0.35, W / 2 + wheelW / 2], [-L * 0.35, -W / 2 - wheelW / 2]] : [[0, W / 2 + wheelW / 2], [0, -W / 2 - wheelW / 2], [L * 0.4, 0], [-L * 0.4, 0]];
      positions.forEach(([x, y], i) => {
        const r = i < 2 || m.kin.drive !== 'differential' ? wheelR : wheelR * 0.5;
        const w = new THREE.Mesh(new THREE.CylinderGeometry(r, r, wheelW, 20), wheelMat);
        w.position.set(x, y, r);
        w.castShadow = true;
        root.add(w);
      });
      // heading arrow
      const arrow = new THREE.Mesh(new THREE.ConeGeometry(W * 0.15, L * 0.25, 12).rotateZ(-Math.PI / 2), materialFor('#ffd43b'));
      arrow.position.set(L / 2 + L * 0.1, 0, H * 0.5);
      root.add(arrow);
    }
    const triad = makeTriad(this.frameSize);
    triad.userData.isFrame = true;
    root.add(triad);
    // path line placeholder (updated per frame)
    const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x51cf66 }));
    pathLine.name = 'path';
    pathLine.matrixAutoUpdate = false;
    root.add(pathLine);
  }

  private buildField(f: FieldItem, e: Entry) {
    const root = e.root;
    if (f.polygon.length >= 3) {
      root.add(polygonMesh(f.polygon, f.indoor ? 0x8d99ae : 0x5c8a3a, 0.35, 0.5));
      root.add(polylineObject(f.polygon, 0xc3fae8, true, 2));
    }
    const rows = f.rows();
    const c = f.crop;
    let nPlants = 0, nFruit = 0;
    for (const r of rows) { nPlants += r.plants.length; for (const p of r.plants) nFruit += p.fruit.filter((x) => x.ripe >= 0.5 && !x.picked).length; }
    if (!nPlants) return;
    const trunk = new THREE.InstancedMesh(new THREE.CylinderGeometry(50, 70, 1, 8).rotateX(Math.PI / 2), materialFor('#6b4f2a'), nPlants);
    const canopyGeo = c.training === 'trellis_2d' || c.training === 'v_trellis' || c.training === 'greenhouse_gutter' ? new THREE.BoxGeometry(1, 1, 1) : new THREE.SphereGeometry(0.5, 10, 8);
    const canopy = new THREE.InstancedMesh(canopyGeo, materialFor(f.indoor ? '#4caf50' : '#3f8f3a', 0.92), nPlants);
    const fruitMesh = nFruit ? new THREE.InstancedMesh(new THREE.SphereGeometry(1, 6, 5), materialFor(c.crop === 'citrus' ? '#ff922b' : c.crop === 'grape' ? '#7048e8' : '#e03131'), Math.min(nFruit, 200000)) : null;
    const m = new THREE.Matrix4();
    let pi = 0, fi = 0;
    for (const r of rows) {
      const [dx, dy] = r.direction();
      const yaw = Math.atan2(dy, dx);
      for (const p of r.plants) {
        const [x, y] = r.pointAt(p.s);
        const trunkH = Math.max(1, c.canopyBase);
        m.compose(new THREE.Vector3(x, y, trunkH / 2), new THREE.Quaternion(), new THREE.Vector3(1, 1, trunkH));
        trunk.setMatrixAt(pi, m);
        const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
        m.compose(new THREE.Vector3(x, y, c.canopyBase + p.height / 2), q, new THREE.Vector3(canopyGeo.type === 'BoxGeometry' ? c.plantSpacing * 0.95 : p.width, canopyGeo.type === 'BoxGeometry' ? p.width * 0.6 : p.width, p.height));
        canopy.setMatrixAt(pi, m);
        pi++;
        if (fruitMesh) for (const fr of p.fruit) {
          if (fr.ripe < 0.5 || fr.picked || fi >= fruitMesh.count) continue;
          const lx = x + dx * fr.p[0] - dy * fr.p[1];
          const ly = y + dy * fr.p[0] + dx * fr.p[1];
          m.compose(new THREE.Vector3(lx, ly, fr.p[2]), new THREE.Quaternion(), new THREE.Vector3(fr.d / 2, fr.d / 2, fr.d / 2));
          fruitMesh.setMatrixAt(fi++, m);
        }
      }
    }
    trunk.instanceMatrix.needsUpdate = true;
    canopy.instanceMatrix.needsUpdate = true;
    trunk.castShadow = canopy.castShadow = true;
    root.add(trunk, canopy);
    if (fruitMesh) { fruitMesh.count = fi; fruitMesh.instanceMatrix.needsUpdate = true; root.add(fruitMesh); }
    // row lines
    for (const r of rows) root.add(polylineObject([r.start, r.end], 0x94d82d, false, 3));
  }

  private buildMap(map: MapItem, e: Entry) {
    const canvas = document.createElement('canvas');
    canvas.width = map.width;
    canvas.height = map.height;
    const ctx = canvas.getContext('2d')!;
    const img = ctx.createImageData(map.width, map.height);
    for (let i = 0; i < map.cells.length; i++) {
      const v = map.cells[i];
      const k = i * 4;
      if (v >= 50) { img.data[k] = 60; img.data[k + 1] = 60; img.data[k + 2] = 70; img.data[k + 3] = 220; }
      else if (v === 255) { img.data[k] = 120; img.data[k + 1] = 120; img.data[k + 2] = 120; img.data[k + 3] = 90; }
      else { img.data[k] = 200; img.data[k + 1] = 215; img.data[k + 2] = 230; img.data[k + 3] = 40; }
    }
    ctx.putImageData(img, 0, 0);
    const tex = new THREE.CanvasTexture(canvas);
    tex.magFilter = THREE.NearestFilter;
    tex.flipY = false;
    const w = map.width * map.resolution, h = map.height * map.resolution;
    const plane = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
    plane.position.set(map.originX + w / 2, map.originY + h / 2, 0.3);
    e.root.add(plane);
  }

  // -- Frame loop ------------------------------------------------------------

  private frame() {
    this.timer.update();
    const dt = Math.min(0.1, this.timer.getDelta());
    for (const fn of this.animHandlers) fn(dt);
    this.controls.update();
    if (this.structural.size) {
      for (const id of [...this.structural]) {
        const it = this.station.findById(id);
        if (it) for (const sub of it.walk()) this.ensureEntry(sub);
      }
      this.structural.clear();
    }
    this.syncTransforms();
    this.renderer.render(this.scene, this.camera);
  }

  /** Force transforms to match the station state now (used by exporters that sample the simulation). */
  syncNow(): void {
    this.syncTransforms();
  }

  /** Scene nodes that move during a simulation: item roots plus robot link groups (for animation export). */
  animationTargets(): Array<{ item: Item; root: THREE.Group; links?: THREE.Group[]; flangeGroup?: THREE.Group }> {
    return [...this.entries.values()].map((e) => ({ item: e.item, root: e.root, links: e.links, flangeGroup: e.flangeGroup }));
  }

  private syncTransforms() {
    const selectedIds = new Set(this.station.selection.map((s) => s.id));
    for (const e of this.entries.values()) {
      const item = e.item;
      let visible = item.visible;
      let p: Item | null = item.parent;
      while (visible && p && p !== this.station) { if (!p.visible) visible = false; p = p.parent; }
      e.root.visible = visible;
      if (!visible) continue;
      if (item instanceof Tool) e.root.matrix.copy(mat4ToThree(item.flangeAbs()));
      else if (item instanceof CropRow) e.root.matrix.identity();
      else e.root.matrix.copy(mat4ToThree(item.poseAbs()));
      if (item instanceof Robot && e.links) {
        const fk = item.fk();
        e.links.forEach((g, i) => g.matrix.copy(mat4ToThree(fk.linkPoses[i] ?? fk.linkPoses[fk.linkPoses.length - 1])));
        e.flangeGroup?.matrix.copy(mat4ToThree(fk.flange));
      }
      if (item instanceof MobileRobot) {
        const line = e.root.getObjectByName('path') as THREE.Line | undefined;
        if (line) {
          const path = item.state.path;
          if (path && path.length > 1) {
            const inv = invert(item.poseAbs());
            const pts = path.map((pt) => { const l = multiply(inv, transl(pt[0], pt[1], 0)); return new THREE.Vector3(l[12], l[13], 30); });
            line.geometry.dispose();
            line.geometry = new THREE.BufferGeometry().setFromPoints(pts);
            line.visible = true;
          } else line.visible = false;
        }
      }
      // frames/targets visibility toggles
      e.root.traverse((o) => {
        if (o.userData.isFrame) o.visible = this.showFrames;
        if (o.userData.isTarget) o.visible = this.showTargets;
      });
      // selection / collision highlight
      const sel = selectedIds.has(item.id) || this.collisionIds.has(item.id);
      if (sel && !e.selectionBox) {
        const box = new THREE.BoxHelper(e.root, this.collisionIds.has(item.id) ? 0xff4d4d : 0xffd43b);
        box.matrixAutoUpdate = true;
        e.selectionBox = box;
        this.scene.add(box);
      } else if (!sel && e.selectionBox) {
        this.scene.remove(e.selectionBox);
        e.selectionBox = undefined;
      }
      if (e.selectionBox) { e.selectionBox.setFromObject(e.root); (e.selectionBox.material as THREE.LineBasicMaterial).color.setHex(this.collisionIds.has(item.id) ? 0xff4d4d : 0xffd43b); }
    }
    if (this.gizmoTarget && !this.transform.dragging) {
      const m = mat4ToThree(this.gizmoTarget.poseAbs());
      m.decompose(this.gizmoProxy.position, this.gizmoProxy.quaternion, this.gizmoProxy.scale);
    }
  }

  /** Render the scene from an item's frame (camera looks along +Z, X right, Y down) into a canvas element. */
  renderFromItem(item: Item, target: HTMLCanvasElement, fov = 60, near = 10, far = 50000, depth = false): void {
    const cam = new THREE.PerspectiveCamera(fov, target.width / target.height, near, far);
    const m = mat4ToThree(item.poseAbs());
    // three cameras look along -Z: rotate 180° about X so the item's +Z becomes the viewing direction with Y down
    const flip = new THREE.Matrix4().makeRotationX(Math.PI);
    cam.matrixAutoUpdate = false;
    cam.matrix.copy(m.multiply(flip));
    cam.matrixWorld.copy(cam.matrix);
    cam.matrixWorldInverse.copy(cam.matrixWorld).invert();
    cam.updateProjectionMatrix();
    const rt = new THREE.WebGLRenderTarget(target.width, target.height);
    const helperVisible = this.transform.getHelper().visible;
    this.transform.getHelper().visible = false;
    const prevOverride = this.scene.overrideMaterial;
    if (depth) this.scene.overrideMaterial = new THREE.MeshDepthMaterial({ depthPacking: THREE.BasicDepthPacking });
    this.renderer.setRenderTarget(rt);
    this.renderer.render(this.scene, cam);
    this.scene.overrideMaterial = prevOverride;
    const buf = new Uint8Array(target.width * target.height * 4);
    this.renderer.readRenderTargetPixels(rt, 0, 0, target.width, target.height, buf);
    this.renderer.setRenderTarget(null);
    this.transform.getHelper().visible = helperVisible;
    rt.dispose();
    const ctx = target.getContext('2d');
    if (!ctx) return;
    const img = ctx.createImageData(target.width, target.height);
    // flip vertically
    for (let y = 0; y < target.height; y++) img.data.set(buf.subarray((target.height - 1 - y) * target.width * 4, (target.height - y) * target.width * 4), y * target.width * 4);
    ctx.putImageData(img, 0, 0);
  }

  /** Camera pose of the 3D view as a 4x4 (RoboDK ViewPose / setViewPose). */
  getViewPose(): Mat4 {
    this.camera.updateMatrixWorld();
    return fromArray(this.camera.matrixWorld.elements);
  }
  setViewPose(m: Mat4): void {
    const t = mat4ToThree(m);
    t.decompose(this.camera.position, this.camera.quaternion, this.camera.scale);
    const dir = new THREE.Vector3(0, 0, -1).applyQuaternion(this.camera.quaternion);
    this.controls.target.copy(this.camera.position.clone().add(dir.multiplyScalar(3000)));
    this.dirty = true;
  }

  /** Animate a joint sequence on a robot (RoboDK ShowSequence). */
  showSequence(robot: Robot, rows: number[][], fps = 30): void {
    let i = 0;
    const n = robot.dof;
    const off = this.onAnimate(() => {
      if (i >= rows.length) { off(); return; }
      const row = rows[i++];
      robot.setJoints(row.length > n + 1 ? row.slice(2, 2 + n) : row.slice(0, n));
    });
    void fps;
  }

  screenshot(): string {
    this.renderer.render(this.scene, this.camera);
    return this.renderer.domElement.toDataURL('image/png');
  }

  dispose(): void {
    this.renderer.setAnimationLoop(null);
    this.renderer.dispose();
  }
}

function hashCells(c: Uint8Array): number {
  let h = 0;
  for (let i = 0; i < c.length; i += 7) h = (h * 31 + c[i]) | 0;
  return h;
}

export { Program };
