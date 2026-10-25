/**
 * "Robot programs with IO handshake" for a UR robot: the Polyscope .urp is a gzip file. The generator keeps it as
 * base64 (GenFile.encoding) and the export archive must contain its bytes, not the base64 text.
 */
import { describe, it, expect } from 'vitest';
import { unzipSync, gunzipSync, strFromU8 } from 'fflate';
import { demos } from '../src/demos';
import { ItemType } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { vendorPrograms } from '../src/ctl/codegen/vendor';
import { genFileBytes } from '../src/ctl/codegen/common';
import { exportArchive } from '../src/ui/control_ui';

const NET = `petri Weld both
place start tokens=1
place pL
place dL
transition tL
arc start -> tL -> dL
action tL program="Weld left"`;

describe('vendor programs: binary files', () => {
  it('the UR .urp in the export zip is a real gzip file whose content is the Polyscope program', () => {
    const st = demos.find((d) => d.id === 'welding')!.build();
    const r = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const files = vendorPrograms(st, 'petri', NET, 'Weld both', { post: 'Universal_Robots_URP' });
    const urp = files.find((f) => f.name.endsWith('.urp'))!;
    expect(urp, files.map((f) => f.name).join(', ')).toBeDefined();
    expect(urp.encoding).toBe('base64');
    expect(Array.from(genFileBytes(urp).slice(0, 2))).toEqual([0x1f, 0x8b]);
    const zip = unzipSync(exportArchive(files));
    const bytes = zip[urp.name];
    expect(Array.from(bytes.slice(0, 2))).toEqual([0x1f, 0x8b]); // gzip magic, not 'H4' (base64 text)
    expect(strFromU8(gunzipSync(bytes))).toMatch(/<URProgram/);
    // text files are unchanged
    const csv = files.find((f) => f.name.endsWith('_robot_io.csv'))!;
    expect(strFromU8(zip[csv.name])).toBe(csv.content);
    void r;
  });
});
