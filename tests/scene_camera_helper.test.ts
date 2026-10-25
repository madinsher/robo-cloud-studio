/**
 * V7 finding: camera / depth / lidar frustum gizmos were drawn doubly transformed (three's CameraHelper uses
 * camera.matrixWorld as its own matrix, and it was added under the item root): the UTP-G-01 rgbd camera frustum hung at
 * (-21600, -10820, 11780) mm instead of at the camera (-10800, -11300, 480). The helper now sits exactly on the camera.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { SceneRenderer } from '../src/scene/renderer';
import { Station, Frame, Camera } from '../src/core/items/item';
import { transl, mul, rotz, DEG, getPos } from '../src/core/math/pose';
import { mat4ToThree } from '../src/scene/three-geometry';

describe('3D view: camera frustum gizmo', () => {
  it('is drawn at the camera, not at twice the camera transform', () => {
    const st = new Station('s');
    const robot = st.addChild(new Frame('UTP-G-01'));
    robot.setPose(mul(transl(-10800, -12500, 80), rotz(30 * DEG)));
    const cam = robot.addChild(new Camera('rgbd_camera')) as Camera;
    cam.setPose(transl(1200, 0, 400));
    const fake: any = { station: st, assets: null, frameSize: 100, showReach: false };
    const root = new THREE.Group();
    root.matrixAutoUpdate = false;
    (SceneRenderer.prototype as any).buildItem.call(fake, cam, { item: cam, root, sig: '' });
    root.matrix.copy(mat4ToThree(cam.poseAbs()));
    const scene = new THREE.Scene(); scene.add(root);
    scene.updateMatrixWorld(true);
    const helper = root.children.find((o) => o instanceof THREE.CameraHelper)!;
    expect(helper).toBeTruthy();
    const at = new THREE.Vector3().setFromMatrixPosition(helper.matrixWorld);
    const want = getPos(cam.poseAbs());
    expect([at.x, at.y, at.z].map((v, i) => Math.abs(v - want[i]))).toEqual([expect.closeTo(0, 3), expect.closeTo(0, 3), expect.closeTo(0, 3)]);
  });
});

describe('3D view: Webots Background / Fog / DirectionalLight (V7 finding: never rendered)', () => {
  it('sky colour, linear fog to visibilityRange and the sun along the world DirectionalLight', () => {
    const st = new Station('w');
    st.settings.background = { skyColor: [0.4, 0.6, 0.8], luminosity: 1 } as any;
    st.settings.fog = { color: [0.9, 0.9, 0.9], visibilityRange: 50000 } as any;
    st.settings.lights = [{ type: 'DirectionalLight', direction: [0, 0.6, -0.8], intensity: 1.2, color: [1, 1, 1], ambientIntensity: 0, castShadows: true }] as any;
    const fake: any = { sun: new THREE.DirectionalLight(), hemi: new THREE.HemisphereLight(), scene: new THREE.Scene(), defaultBackground: 0x1e2229, station: st };
    (SceneRenderer.prototype as any).applyEnvironment.call(fake, st);
    expect((fake.scene.background as THREE.Color).getHexString()).toBe(new THREE.Color(0.4, 0.6, 0.8).getHexString());
    expect(fake.scene.fog.far).toBe(50000);
    const dir = fake.sun.position.clone().normalize();
    expect([dir.x, dir.y, dir.z].map((v: number) => +v.toFixed(3))).toEqual([-0, -0.6, 0.8].map((v) => +v.toFixed(3)));
    expect(fake.sun.intensity).toBeCloseTo(1.92, 6);
    // a station without Webots settings gets the studio defaults back
    (SceneRenderer.prototype as any).applyEnvironment.call(fake, new Station('plain'));
    expect((fake.scene.background as THREE.Color).getHex()).toBe(0x1e2229);
  });
});
