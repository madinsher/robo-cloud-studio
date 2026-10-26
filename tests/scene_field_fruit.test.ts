/**
 * V3 finding: fruit picked during a fleet harvest stayed drawn on the trees until a whole row side was finished
 * (process.ts notifies the CropRow, whose renderer signature is constant; only field.notify('rows') rebuilt the field's
 * fruit). The 3D view now refreshes the field's fruit instances in place when one of its rows changes.
 * SceneRenderer runs here without WebGL: its field build and change wiring are called on a stand-in object.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { SceneRenderer } from '../src/scene/renderer';
import { Station } from '../src/core/items/item';
import { FieldItem, cropParams } from '../src/agri/items';
import { generateOrchard, rectPolygon } from '../src/agri/orchard';

const P: any = SceneRenderer.prototype;

function stand(station: Station) {
  const r: any = {
    station, entries: new Map(), meshUsers: new Map(), structural: new Set<string>(), fieldsDirty: new Set<string>(), fieldRefreshAt: 0,
    offTree: [], scene: new THREE.Scene(), dirty: false, showReach: false,
    setGizmo() {}, rebuildAll() {}, removeEntry() {}, applyEnvironment() {},
  };
  for (const k of ['signature', 'buildField', 'writeFieldFruit', 'refreshFields']) r[k] = P[k];
  P.bindStation.call(r, station);
  return r;
}

describe('3D view: picked fruit disappear from the trees at once', () => {
  it('a row change refreshes the field fruit instances in place (no rebuild, count drops by the picked fruit)', () => {
    const st = new Station('orchard');
    const field = st.addChild(new FieldItem('Block'));
    field.polygon = rectPolygon(30000, 11500);
    field.crop = cropParams('apple', { rowHeading: 0, headland: 4000, ripeFraction: 0.5 });
    generateOrchard(field, 7);
    const r = stand(st);
    const e: any = { item: field, root: new THREE.Group(), sig: P.signature.call(r, field) };
    P.buildField.call(r, field, e);
    r.entries.set(field.id, e);
    const visible = () => field.rows().reduce((n, row) => n + row.plants.reduce((m, p) => m + p.fruit.filter((f) => f.ripe >= 0.5 && !f.picked).length, 0), 0);
    const n0 = visible();
    expect(n0).toBeGreaterThan(50);
    expect(e.fieldFruit.count).toBe(n0);
    const mesh = e.fieldFruit;
    // the harvest process picks 7 fruit of one row and notifies the row only (src/agri/process.ts)
    const row = field.rows()[0];
    let k = 0;
    for (const p of row.plants) for (const f of p.fruit) if (k < 7 && f.ripe >= 0.5 && !f.picked) { f.picked = true; k++; }
    row.notify('plants');
    expect(r.fieldsDirty.has(field.id)).toBe(true);
    P.refreshFields.call(r);
    expect(e.fieldFruit).toBe(mesh);                 // same mesh, updated in place
    expect(mesh.count).toBe(n0 - 7);
    expect(r.structural.has(field.id)).toBe(false);  // no full rebuild of trunks / canopies
    expect(e.sig).toBe(P.signature.call(r, field));  // a later structural pass does not rebuild either
    expect(row.plants.flatMap((p) => p.fruit.filter((f) => f.picked)).length).toBe(7);
  });
});
