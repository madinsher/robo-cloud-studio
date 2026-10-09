import { it, expect } from 'vitest';
import { Station, Camera } from '../src/core/items/item';
import { FieldItem, cropParams } from '../src/agri/items';
import { generateOrchard, rectPolygon } from '../src/agri/orchard';
import { detectFruit } from '../src/agri/vision';
import { mul, transl, rotz, roty, DEG } from '../src/core/math/pose';

it('camera detects fruit in front of it and none behind', () => {
  const st = new Station();
  const field = st.addChild(new FieldItem('F'));
  field.polygon = rectPolygon(20000, 10000);
  field.crop = cropParams('apple', { headland: 2000 });
  const rows = generateOrchard(field, 3);
  const row = rows[0];
  const [x, y] = row.pointAt(row.length() / 2, -1500);
  const cam = st.addChild(new Camera('Cam'));
  cam.fov = 70; cam.width = 640; cam.height = 480; cam.far = 8000;
  // camera at 1.5 m height looking +Y (towards the row): item +Z must point +Y => rotate -90 about X... use rotz/roty composition
  cam.setPose(mul(transl(x, y, 1500), rotz(90 * DEG), roty(90 * DEG)));
  const det = detectFruit(st, cam, { onlyRipe: false, occlusionPerMeter: 0 });
  expect(det.length).toBeGreaterThan(20);
  expect(det.every((d) => d.range > 0 && d.u >= 0 && d.u < 640)).toBe(true);
  cam.setPose(mul(transl(x, y, 1500), rotz(-90 * DEG), roty(90 * DEG)));
  const away = detectFruit(st, cam, { onlyRipe: false, occlusionPerMeter: 0, maxRange: 3000 });
  expect(away.length).toBeLessThan(det.length / 4);
});
