import { describe, it, expect } from 'vitest';
import { llToLocal, localToLL, fieldsFromGeoJSON, fieldsToGeoJSON, polygonArea } from '../src/agri/geo';
import { FieldItem, CropRow, MissionItem, SensorItem, cropParams, CROP_PRESETS } from '../src/agri/items';
import { Component, ProcessSimulator, makeConveyor, makeFeeder, makeProcess, makeSink, makeBuffer } from '../src/vc/component';
import { Station, SceneObject, ItemType } from '../src/core/items/item';
import { transl, mul, rotz, DEG, getPos } from '../src/core/math/pose';

describe('geodesy helpers', () => {
  const origin = { lat: 45.0, lon: 7.5 };
  it('converts lat/lon to local mm and back', () => {
    const [x, y] = llToLocal(45.001, 7.501, origin);
    expect(y).toBeCloseTo(111.3e3, -3); // ~111 m north in mm
    expect(x).toBeCloseTo(78.7e3, -3); // ~78.7 m east at 45° latitude
    const [lat, lon] = localToLL(x, y, origin);
    expect(lat).toBeCloseTo(45.001, 9);
    expect(lon).toBeCloseTo(7.501, 9);
    expect(llToLocal(origin.lat, origin.lon, origin)).toEqual([0, 0]);
  });
  it('imports FeatureCollections, Features, bare geometries and MultiPolygons; exports back', () => {
    const square = [[7.5, 45.0], [7.501, 45.0], [7.501, 45.001], [7.5, 45.001], [7.5, 45.0]];
    const fc = {
      type: 'FeatureCollection',
      features: [
        { type: 'Feature', properties: { name: 'North block', crop: 'apple' }, geometry: { type: 'Polygon', coordinates: [square] } },
        { type: 'Feature', properties: { Name: 'Multi' }, geometry: { type: 'MultiPolygon', coordinates: [[square], [square.map(([lo, la]) => [lo + 0.01, la])]] } },
        { type: 'Feature', properties: {}, geometry: null },
        { type: 'Feature', geometry: { type: 'Point', coordinates: [7.5, 45] } },
      ],
    };
    const { fields, origin: o } = fieldsFromGeoJSON(fc);
    expect(fields.length).toBe(3);
    expect(fields.map((f) => f.name)).toEqual(['North block', 'Multi', 'Multi']);
    expect(fields[0].properties.crop).toBe('apple');
    // the closing vertex is dropped and the origin is the centroid of all vertices
    expect(fields[0].polygon.length).toBe(4);
    expect(o.lat).toBeGreaterThan(45.0); // centroid of all ring vertices (closing points included)
    expect(o.lat).toBeLessThan(45.001);
    expect(o.lon).toBeGreaterThan(7.5);
    const area = polygonArea(fields[0].polygon);
    expect(area / 1e6).toBeCloseTo(111.3 * 78.7, -3); // m²
    // explicit origin
    const withOrigin = fieldsFromGeoJSON({ type: 'Feature', properties: {}, geometry: { type: 'Polygon', coordinates: [square.slice(0, 4)] } }, origin);
    expect(withOrigin.origin).toBe(origin);
    expect(withOrigin.fields[0].name).toBe('Field 1');
    expect(withOrigin.fields[0].polygon.length).toBe(4);
    expect(withOrigin.fields[0].polygon[0]).toEqual([0, 0]);
    const bare = fieldsFromGeoJSON({ type: 'Polygon', coordinates: [square] }, origin);
    expect(bare.fields.length).toBe(1);
    expect(fieldsFromGeoJSON({ type: 'FeatureCollection', features: [] })).toEqual({ fields: [], origin: { lat: 0, lon: 0 } });
    expect(fieldsFromGeoJSON({ type: 'FeatureCollection', features: [] }, origin).origin).toBe(origin);
    const gj = fieldsToGeoJSON(withOrigin.fields, origin);
    expect(gj.type).toBe('FeatureCollection');
    expect(gj.features[0].properties.name).toBe('Field 1');
    const ring = gj.features[0].geometry.coordinates[0];
    expect(ring.length).toBe(5);
    expect(ring[0]).toEqual(ring[4]);
    expect(ring[2][0]).toBeCloseTo(7.501, 9);
    expect(ring[2][1]).toBeCloseTo(45.001, 9);
  });
  it('computes polygon areas independent of winding', () => {
    expect(polygonArea([[0, 0], [10, 0], [10, 5], [0, 5]])).toBe(50);
    expect(polygonArea([[0, 0], [0, 5], [10, 5], [10, 0]])).toBe(50);
    expect(polygonArea([[0, 0], [4, 0], [0, 3]])).toBe(6);
    expect(polygonArea([])).toBe(0);
  });
});

describe('agri items', () => {
  it('crop presets merge with defaults and overrides', () => {
    const apple = cropParams('apple');
    expect(apple.crop).toBe('apple');
    expect(apple.training).toBe('spindle');
    expect(apple.rowSpacing).toBe(3500);
    expect(apple.headland).toBe(6000); // default kept
    const tomato = cropParams('tomato', { rowHeading: 90, headland: 2000 });
    expect(tomato.training).toBe('greenhouse_gutter');
    expect(tomato.rowHeading).toBe(90);
    expect(tomato.headland).toBe(2000);
    for (const k of Object.keys(CROP_PRESETS)) expect(cropParams(k as any).plantSpacing).toBeGreaterThan(0);
  });
  it('rows: geometry helpers and ripe fruit counting', () => {
    const row = new CropRow('R1');
    row.start = [0, 0];
    row.end = [3000, 4000];
    expect(row.length()).toBe(5000);
    expect(row.direction()).toEqual([0.6, 0.8]);
    expect(row.pointAt(2500)).toEqual([1500, 2000]);
    // lateral offset to the left of the travel direction
    const [px, py] = row.pointAt(0, 1000);
    expect(px).toBeCloseTo(-800, 9);
    expect(py).toBeCloseTo(600, 9);
    const line = row.travelLine(-500);
    expect(line.start[0]).toBeCloseTo(400, 9);
    expect(line.end[1]).toBeCloseTo(4000 - 300, 9);
    row.plants = [
      { s: 0, height: 1, width: 1, health: 1, fruit: [{ p: [0, 0, 1], ripe: 0.9, picked: false, d: 70 }, { p: [0, 0, 1], ripe: 0.9, picked: true, d: 70 }, { p: [0, 0, 1], ripe: 0.2, picked: false, d: 70 }] },
      { s: 1, height: 1, width: 1, health: 1, fruit: [{ p: [0, 0, 1], ripe: 0.5, picked: false, d: 70 }] },
    ];
    expect(row.ripeFruitCount()).toBe(2);
    const zero = new CropRow();
    zero.end = [0, 0];
    expect(zero.direction()).toEqual([0, 0]);
    expect(zero.length()).toBe(0);
  });
  it('field, row, mission and sensor items serialize through the station', () => {
    const st = new Station('farm');
    const field = st.addChild(new FieldItem('Block A'));
    field.polygon = [[0, 0], [1e5, 0], [1e5, 5e4], [0, 5e4]];
    field.crop = cropParams('cherry', { rowHeading: 12 });
    field.geoOrigin = { lat: 44.5, lon: 8.1, alt: 250 };
    field.slopeDeg = 3;
    field.slopeHeading = 90;
    field.indoor = true;
    const row = field.addChild(new CropRow('Row 1'));
    row.start = [100, 200];
    row.end = [900, 200];
    row.index = 4;
    row.segmentId = 'seg-4';
    row.plants = [{ s: 10, height: 2500, width: 1200, health: 0.8, fruit: [{ p: [1, 2, 3], ripe: 0.7, picked: false, d: 26 }] }];
    field.addChild(new SceneObject('post'));
    expect(field.rows()).toEqual([row]);
    expect(field.color).toBe('#6b8e23');
    const mission = st.addChild(new MissionItem('Harvest A'));
    mission.missionType = 'spray';
    mission.fieldId = field.id;
    mission.fleetId = 'fleet-1';
    mission.rowIds = [row.id];
    mission.status = 'running';
    mission.settings = { workSpeed: 700 };
    mission.taskIds = ['t1', 't2'];
    mission.progress = 0.4;
    const sensor = st.addChild(new SensorItem('Lidar'));
    sensor.kind = 'lidar3d';
    sensor.range = 30000;
    sensor.fov = 270;
    sensor.rate = 20;
    const st2 = Station.deserialize(JSON.parse(JSON.stringify(st.serialize())));
    const f2 = st2.find('Block A') as FieldItem;
    expect(f2).toBeInstanceOf(FieldItem);
    expect(f2.type).toBe(ItemType.FIELD);
    expect(f2.polygon).toEqual(field.polygon);
    expect(f2.crop).toEqual(field.crop);
    expect(f2.geoOrigin).toEqual({ lat: 44.5, lon: 8.1, alt: 250 });
    expect([f2.slopeDeg, f2.slopeHeading, f2.indoor]).toEqual([3, 90, true]);
    const r2 = f2.rows()[0];
    expect(r2).toBeInstanceOf(CropRow);
    expect(r2.start).toEqual([100, 200]);
    expect(r2.end).toEqual([900, 200]);
    expect(r2.index).toBe(4);
    expect(r2.segmentId).toBe('seg-4');
    expect(r2.plants).toEqual(row.plants);
    expect(r2.ripeFruitCount()).toBe(1);
    const m2 = st2.find('Harvest A') as MissionItem;
    expect(m2).toBeInstanceOf(MissionItem);
    expect(m2.missionType).toBe('spray');
    expect(m2.fieldId).toBe(field.id);
    expect(m2.fleetId).toBe('fleet-1');
    expect(m2.rowIds).toEqual([row.id]);
    expect(m2.status).toBe('running');
    expect(m2.settings).toEqual({ workSpeed: 700 });
    expect(m2.taskIds).toEqual(['t1', 't2']);
    expect(m2.progress).toBe(0.4);
    const s2 = st2.find('Lidar') as SensorItem;
    expect(s2).toBeInstanceOf(SensorItem);
    expect([s2.kind, s2.range, s2.fov, s2.rate]).toEqual(['lidar3d', 30000, 270, 20]);
    // defaults when fields are missing from the serialized data
    const bare = new FieldItem();
    bare.deserializeExtra({} as any);
    expect(bare.polygon).toEqual([]);
    expect(bare.geoOrigin).toBeNull();
    expect(bare.indoor).toBe(false);
    const bareRow = new CropRow();
    bareRow.deserializeExtra({} as any);
    expect(bareRow.end).toEqual([10000, 0]);
    const bareMission = new MissionItem();
    bareMission.deserializeExtra({} as any, {} as any);
    expect(bareMission.missionType).toBe('harvest');
    expect(bareMission.settings).toEqual({});
    const bareSensor = new SensorItem();
    bareSensor.deserializeExtra({} as any);
    expect(bareSensor.kind).toBe('camera');
  });
});

describe('Visual Components style process flow', () => {
  function line() {
    const st = new Station('line');
    const feeder = st.addChild(makeFeeder('Feeder', 2, { name: 'Crate', geometry: { primitive: { kind: 'box', size: [300, 200, 200] }, origin: Array.from(transl(0, 0, 100)), color: '#d9a066' }, massKg: 5 }));
    const conv = st.addChild(makeConveyor('Conveyor', 2000, 500, 400));
    conv.setPose(transl(0, 0, 800));
    (conv.behaviour as any).stopSignal = 'stop_conv';
    (conv.behaviour as any).spacing = 400;
    const proc = st.addChild(makeProcess('Grader', 1, 1));
    proc.setPose(transl(2500, 0, 800));
    Object.assign(proc.behaviour, { busySignal: 'grader_busy', mtbf: 1e9, mttr: 5 });
    const buffer = st.addChild(makeBuffer('Pallet', 4, { cols: 2, rows: 2, layers: 2, pitch: [400, 300, 250] }));
    buffer.setPose(transl(3500, 0, 0));
    const transfer = st.addChild(new Component('Robot', { type: 'transfer', enabled: true, robotId: null, from: buffer.id, next: null, cycleTime: 0.5 }));
    const human = st.addChild(new Component('Worker', { type: 'human', enabled: true, walkSpeed: 1000, taskTime: 0.5, next: null }));
    const sink = st.addChild(makeSink('Truck'));
    const sensor = st.addChild(new Component('Eye', { type: 'sensor', enabled: true, signal: 'eye', range: 300 }));
    sensor.setPose(transl(2000, 0, 800)); // conveyor end
    const off = st.addChild(makeSink('Disabled'));
    off.behaviour.enabled = false;
    (feeder.behaviour as any).next = conv.id;
    (conv.behaviour as any).next = proc.id;
    (proc.behaviour as any).next = buffer.id;
    (buffer.behaviour as any).next = null; // buffers are emptied by the transfer robot
    (transfer.behaviour as any).next = human.id;
    (human.behaviour as any).next = sink.id;
    return { st, feeder, conv, proc, buffer, transfer, human, sink, sensor, off };
  }
  it('moves products feeder -> conveyor -> process -> buffer -> robot -> worker -> sink with signals and stats', () => {
    const { st, feeder, conv, proc, buffer, transfer, human, sink, sensor, off } = line();
    const sim = new ProcessSimulator(st);
    expect(sim.components().length).toBe(8);
    expect(sim.components()).not.toContain(off);
    const created: string[] = [];
    const consumed: string[] = [];
    const signals: Array<[string, boolean | number]> = [];
    sim.events.on('productCreated', (e) => created.push(e.product.name));
    sim.events.on('productConsumed', (e) => consumed.push(`${e.product.name}@${e.by.name}`));
    sim.events.on('signal', (e) => signals.push([e.name, e.value]));
    sim.step(0.1); // the feeder starts full (acc = interval) and creates immediately
    expect(created).toEqual(['Crate 1']);
    expect(st.find('Products', ItemType.FOLDER)).toBeTruthy();
    const crate = st.find('Crate 1') as SceneObject;
    expect(crate.mass).toBe(5);
    expect(crate.color).toBe('#d9a066');
    expect(conv.products).toEqual([crate.id]);
    expect(getPos(crate.poseAbs())).toEqual([50, 0, 800]); // created at 0 and advanced 500 mm/s x 0.1 s in the same step
    // stop signal halts the conveyor
    sim.setSignal('stop_conv', true);
    sim.step(1);
    expect(sim.products.get(crate.id)!.progress).toBe(50);
    expect(conv.stats.busyTime).toBeCloseTo(1.1, 9);
    sim.setSignal('stop_conv', false);
    for (let i = 0; i < 10; i++) sim.step(0.1);
    expect(sim.products.get(crate.id)!.progress).toBeCloseTo(550, 6);
    expect(getPos(crate.poseAbs())[0]).toBeCloseTo(550, 6);
    // a second crate is created after the interval; spacing keeps it behind the first
    expect(created.length).toBeGreaterThanOrEqual(2);
    // run until crates reach the sink
    for (let i = 0; i < 400 && (sink.behaviour as any).count < 2; i++) sim.step(0.1);
    expect((sink.behaviour as any).count).toBeGreaterThanOrEqual(2);
    expect(consumed[0]).toBe('Crate 1@Truck');
    expect(st.find('Crate 1')).toBeNull(); // consumed products are removed from the tree
    expect(signals.some(([n, v]) => n === 'grader_busy' && v === true)).toBe(true);
    expect(signals.some(([n, v]) => n === 'eye' && v === true)).toBe(true);
    expect(signals.some(([n, v]) => n === 'eye' && v === false)).toBe(true);
    expect(proc.stats.entered).toBeGreaterThanOrEqual(2);
    expect(proc.stats.exited).toBeGreaterThanOrEqual(2);
    expect(buffer.stats.entered).toBeGreaterThanOrEqual(2);
    expect(transfer.stats.entered).toBeGreaterThanOrEqual(2);
    expect(human.stats.entered).toBeGreaterThanOrEqual(2);
    expect(human.stats.busyTime).toBeGreaterThan(0);
    const stats = sim.statistics();
    const byName = Object.fromEntries(stats.map((s) => [s.name, s]));
    expect(byName.Truck.exited).toBe((sink.behaviour as any).count);
    expect(byName.Feeder.exited).toBe((feeder.behaviour as any).created);
    expect(byName.Conveyor.utilization).toBeGreaterThan(0);
    expect(byName.Conveyor.utilization).toBeLessThanOrEqual(1);
    expect(byName.Grader.type).toBe('process');
    expect(stats.every((s) => s.failures === 0)).toBe(true);
    expect(stats.find((s) => s.name === 'Disabled')).toBeUndefined();
    // reset clears products, stats and counters
    sim.reset();
    expect(sim.time).toBe(0);
    expect(sim.products.size).toBe(0);
    expect(st.children.filter((c) => c.name.startsWith('Crate')).length).toBe(0);
    expect((feeder.behaviour as any).created).toBe(0);
    expect((sink.behaviour as any).count).toBe(0);
    expect(conv.stats.busyTime).toBe(0);
    expect(sim.statistics().every((s) => s.entered === 0 && s.wip === 0)).toBe(true);
    void sensor;
  });
  it('buffers stack products on a grid and a limited feeder stops; process failures take the machine down', () => {
    const st = new Station();
    const feeder = st.addChild(makeFeeder('F', 0.5, { name: 'Box', geometry: { primitive: { kind: 'box', size: [100, 100, 100] }, color: '#fff' } }));
    (feeder.behaviour as any).limit = 3;
    const buffer = st.addChild(makeBuffer('B', 8, { cols: 2, rows: 1, layers: 4, pitch: [200, 0, 120] }));
    buffer.setPose(transl(1000, 0, 0));
    (feeder.behaviour as any).next = buffer.id;
    const sim = new ProcessSimulator(st);
    for (let i = 0; i < 40; i++) sim.step(0.1);
    expect((feeder.behaviour as any).created).toBe(3);
    expect(buffer.products.length).toBe(3);
    const pos = buffer.products.map((id) => getPos(st.findById(id)!.poseAbs()));
    expect(pos[0]).toEqual([1000, 0, 0]);
    expect(pos[1]).toEqual([1200, 0, 0]);
    expect(pos[2]).toEqual([1000, 0, 120]);
    // feeder blocked when the next component cannot accept
    const st2 = new Station();
    const f2 = st2.addChild(makeFeeder('F', 0.1, { name: 'P', geometry: { primitive: { kind: 'sphere', radius: 10 } } }));
    const p2 = st2.addChild(makeProcess('M', 100, 1));
    Object.assign(p2.behaviour, { mtbf: 0.001, mttr: 10 }); // fails almost immediately
    (f2.behaviour as any).next = p2.id;
    const sim2 = new ProcessSimulator(st2);
    for (let i = 0; i < 30; i++) sim2.step(0.1);
    expect(p2.stats.failures).toBeGreaterThanOrEqual(1);
    expect(p2.runtime.down).toBe(true);
    expect(p2.stats.downTime).toBeGreaterThan(0);
    expect(f2.stats.blockedTime).toBeGreaterThan(0);
    expect(sim2.statistics()[1].failures).toBe(p2.stats.failures);
    // a feeder with no downstream component never creates anything
    const st3 = new Station();
    const f3 = st3.addChild(makeFeeder('Lonely', 0.1, { name: 'P', geometry: { primitive: { kind: 'sphere', radius: 10 } } }));
    const sim3 = new ProcessSimulator(st3);
    sim3.step(1);
    expect((f3.behaviour as any).created).toBe(0);
    expect(f3.stats.blockedTime).toBe(1);
  });
  it('conveyor poses follow multi-segment paths with heading; components serialize', () => {
    const c = new Component('Bend', { type: 'conveyor', enabled: true, path: [[0, 0, 0], [1000, 0, 0], [1000, 500, 100]], speed: 200, spacing: 100, next: null });
    c.setPose(mul(transl(10, 20, 30), rotz(90 * DEG)));
    expect(c.conveyorLength()).toBeCloseTo(1000 + Math.hypot(500, 100), 9);
    const p0 = c.conveyorPoseAt(0);
    expect(getPos(p0).map((v) => +v.toFixed(6))).toEqual([10, 20, 30]);
    const p1 = c.conveyorPoseAt(500);
    expect(getPos(p1).map((v) => +v.toFixed(6))).toEqual([10, 520, 30]); // along +X of the rotated component = world +Y
    const p2 = c.conveyorPoseAt(1000 + Math.hypot(500, 100) / 2);
    expect(getPos(p2).map((v) => +v.toFixed(6))).toEqual([10 - 250, 1020, 80]);
    // heading of the second segment is 90° locally -> the product X axis points along world -X
    expect(p2[0]).toBeCloseTo(-1, 9);
    expect(getPos(c.conveyorPoseAt(99999)).map((v) => +v.toFixed(6))).toEqual([10 - 500, 1020, 130]); // clamped to the end
    const short = new Component('Dot', { type: 'conveyor', enabled: true, path: [[0, 0, 0]], speed: 1, spacing: 1, next: null });
    short.setPose(transl(1, 2, 3));
    expect(getPos(short.conveyorPoseAt(5))).toEqual([1, 2, 3]);
    const st = new Station();
    st.addChild(c);
    const conv = st.addChild(makeConveyor('Belt', 1500));
    expect(conv.geometry[0].primitive).toEqual({ kind: 'box', size: [1500, 400, 80] });
    const st2 = Station.deserialize(JSON.parse(JSON.stringify(st.serialize())));
    const c2 = st2.find('Bend') as Component;
    expect(c2).toBeInstanceOf(Component);
    expect(c2.behaviour).toEqual(c.behaviour);
    expect((st2.find('Belt') as Component).geometry).toEqual(conv.geometry);
    expect(c2.conveyorLength()).toBeCloseTo(c.conveyorLength(), 9);
    const bare = new Component('x', { type: 'sink', enabled: true, count: 0 });
    bare.deserializeExtra({ behaviour: { type: 'buffer', enabled: true, capacity: 2, next: null } } as any, {} as any);
    expect(bare.behaviour.type).toBe('buffer');
    expect(bare.geometry).toEqual([]);
  });
});
