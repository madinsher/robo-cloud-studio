// @vitest-environment jsdom
/**
 * F1 (V1 finding): RoboDK Library .tool files keep their TCP. RoboDK 5.9.4 opens a .tool as a Tool whose TCP is the
 * table's [160..175] pose (tests/fixtures/robodk_gt/<tool>.tool_.json.gz poseTool); with a robot selected the tool is
 * mounted on its flange. The app passes the file name without its extension to the importer, which used to decide
 * "tool" from that name only, so every .tool became a plain object and the TCP sat on the flange.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { existsSync, readFileSync, readdirSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import path from 'node:path';
import { installDomStubs, makeApp } from './ui_harness';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { ItemType, Tool } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { toRows } from '../src/core/math/pose';

beforeAll(() => installDomStubs());
const LIB = 'C:/RoboDK/Library';
const GT = path.resolve(__dirname, 'fixtures/robodk_gt');
const TOOLS = existsSync(LIB) ? [...readdirSync(LIB).filter((f) => /\.tool$/i.test(f)), ...(existsSync(`${LIB}/Macros`) ? readdirSync(`${LIB}/Macros`).filter((f) => /\.tool$/i.test(f)).map((f) => `Macros/${f}`) : [])] : [];
const gtTcp = (file: string): number[] | null => {
  const p = path.join(GT, path.basename(file).replace(/ /g, '_') + '_.json.gz');
  if (!existsSync(p)) return null;
  const g = JSON.parse(gunzipSync(readFileSync(p)).toString().replace(/\bNaN\b/g, 'null'));
  return g.tools?.[0]?.poseTool ?? null;
};
/** Largest element difference between a pose and RoboDK's row-major 16 numbers. */
const diff = (m: Float64Array, rowMajor: number[]) => Math.max(...toRows(m).flat().map((v, i) => Math.abs(v - rowMajor[i])));
const fileOf = (p: string) => { const b = readFileSync(p); return new File([b], path.basename(p)); };

describe('RoboDK .tool files keep their TCP', () => {
  it('there are Library .tool files with ground truth', () => {
    if (!existsSync(LIB)) return;
    expect(TOOLS.filter((t) => gtTcp(t)).length).toBeGreaterThanOrEqual(9);
  });

  for (const f of TOOLS) {
    const want = gtTcp(f);
    it.skipIf(!want)(`${f}: imported as a Tool with RoboDK's TCP (name passed with or without the extension)`, async () => {
      for (const name of [path.basename(f), path.basename(f).replace(/\.tool$/i, '')]) {
        const b = readFileSync(`${LIB}/${f}`);
        const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer, name, new AssetStore());
        const tools = station.itemsOfType<Tool>(ItemType.TOOL);
        expect(tools.length, name).toBe(1);
        expect(station.itemsOfType(ItemType.OBJECT).length).toBe(0);
        expect(diff(tools[0].poseTool(), want!), name).toBeLessThan(1e-4);
      }
    }, 60000);
  }

  it.skipIf(!TOOLS.includes('Spindle.tool') || !existsSync(`${LIB}/UR10e.robot`))('File › Open UR10e.robot then Spindle.tool: mounted on the flange with TCP [557.2, 0, 176]; opened alone: a tool in the station', async () => {
    const app = makeApp();
    app.newStation?.();
    await app.openFiles([fileOf(`${LIB}/UR10e.robot`)]);
    const robot = app.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    expect(robot).toBeTruthy();
    app.setActiveRobot(robot);
    await app.openFiles([fileOf(`${LIB}/Spindle.tool`)]);
    const tool = robot.tools().find((t) => t.name === 'Spindle')!;
    expect(tool).toBeTruthy();
    expect(robot.activeTool()).toBe(tool);
    const tcp = tool.poseTool();
    expect([tcp[12], tcp[13], tcp[14]].map((v) => +v.toFixed(3))).toEqual([557.2, 0, 176]);
    expect(app.logs.some((l) => /no TCP in the file/.test(l.text))).toBe(false);

    const alone = makeApp();
    alone.newStation?.();
    await alone.openFiles([fileOf(`${LIB}/Spindle.tool`)]);
    const t2 = alone.station.itemsOfType<Tool>(ItemType.TOOL);
    expect(t2.map((t) => t.name)).toEqual(['Spindle']);
    expect(alone.station.itemsOfType(ItemType.OBJECT).length).toBe(0);
    expect(diff(t2[0].poseTool(), gtTcp('Spindle.tool')!)).toBeLessThan(1e-4);
  }, 120000);
});
