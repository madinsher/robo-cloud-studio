/**
 * Export a whole station as a RoboDK API Python script. Running it inside RoboDK rebuilds the
 * station (frames, robots from RoboDK's library by name, tools, targets, programs, objects with
 * geometry saved next to the script) so it can be saved as a native .rdk project.
 */
import { Station, Item, ItemType, Frame, Tool, Target, SceneObject } from '../../core/items/item';
import { Robot } from '../../core/items/robot';
import { Program } from '../../core/items/program';
import { toRows } from '../../core/math/pose';
import { compileForPost } from '../../posts/base';
import { ROBODK_PYTHON } from '../../posts/robodk_python';

const mat = (m: Float64Array) => `Mat(${JSON.stringify(toRows(m).map((r) => r.map((v) => +v.toFixed(6))))})`;
const py = (s: string) => JSON.stringify(s);

export function stationToRoboDKScript(station: Station): string {
  const L: string[] = [
    `# Station "${station.name}" exported by VerticalBot Studio as a RoboDK API script.`,
    `# Run in RoboDK: Tools > Run Script (or: python this_file.py with RoboDK open), then File > Save Station (.rdk).`,
    `from robodk.robolink import *`,
    `from robodk.robomath import *`,
    `import os`,
    `RDK = Robolink()`,
    `RDK.Render(False)`,
    `station = RDK.AddStation(${py(station.name)})`,
    `items = {}`,
    `def lib_robot(name, parent):`,
    `    # Try to load the robot from the local RoboDK library by (partial) name`,
    `    lib = RDK.getParam('PATH_LIBRARY')`,
    `    cands = []`,
    `    for root, dirs, files in os.walk(lib):`,
    `        for f in files:`,
    `            if f.lower().endswith('.robot') and all(t.lower() in f.lower() for t in name.split()):`,
    `                cands.append(os.path.join(root, f))`,
    `    if cands:`,
    `        r = RDK.AddFile(cands[0], parent)`,
    `        if r.Valid():`,
    `            return r`,
    `    print('Robot not found in library: ' + name + ' (add it manually)')`,
    `    return None`,
    ``,
  ];
  const varOf = new Map<string, string>();
  let n = 0;
  const v = (it: Item) => {
    if (!varOf.has(it.id)) varOf.set(it.id, `it${++n}`);
    return varOf.get(it.id)!;
  };
  const parentExpr = (it: Item) => (it.parent && it.parent.type !== ItemType.STATION && varOf.has(it.parent.id) ? varOf.get(it.parent.id)! : 'station');

  const emit = (it: Item) => {
    for (const c of it.children) {
      const pv = parentExpr(c);
      if (c instanceof Robot) {
        L.push(`${v(c)} = lib_robot(${py(c.model || c.name)}, ${pv})`);
        L.push(`if ${v(c)} is None:`, `    ${v(c)} = RDK.AddFrame(${py(c.name + ' (robot placeholder)')}, ${pv})`);
        L.push(`${v(c)}.setName(${py(c.name)})`, `${v(c)}.setPose(${mat(c.pose())})`);
        L.push(`try:`, `    ${v(c)}.setJoints(${JSON.stringify(c.joints().map((x) => +x.toFixed(4)))})`, `except Exception:`, `    pass`);
      } else if (c instanceof Tool) {
        L.push(`${v(c)} = ${pv}.AddTool(${mat(c.pose())}, ${py(c.name)}) if ${pv}.Type() == ITEM_TYPE_ROBOT else RDK.AddFrame(${py(c.name)}, ${pv})`);
      } else if (c instanceof Target) {
        L.push(`${v(c)} = RDK.AddTarget(${py(c.name)}, ${pv})`);
        L.push(`${v(c)}.setPose(${mat(c.pose())})`);
        if (c.joints) L.push(`${v(c)}.setJoints(${JSON.stringify(c.joints.map((x) => +x.toFixed(4)))})`);
        L.push(c.isJointTarget ? `${v(c)}.setAsJointTarget()` : `${v(c)}.setAsCartesianTarget()`);
      } else if (c instanceof Frame) {
        L.push(`${v(c)} = RDK.AddFrame(${py(c.name)}, ${pv})`, `${v(c)}.setPose(${mat(c.pose())})`);
      } else if (c instanceof SceneObject) {
        const meshFile = `${c.name.replace(/[^A-Za-z0-9_]/g, '_')}.stl`;
        L.push(`${v(c)} = RDK.AddFile(os.path.join(os.path.dirname(__file__), ${py(meshFile)}), ${pv}) if os.path.exists(os.path.join(os.path.dirname(__file__), ${py(meshFile)})) else RDK.AddFrame(${py(c.name)}, ${pv})`);
        L.push(`${v(c)}.setName(${py(c.name)})`, `${v(c)}.setPose(${mat(c.pose())})`);
      } else if (c instanceof Program) {
        // programs (and their instructions) are emitted after everything else
        continue;
      } else if (c.type === ItemType.FOLDER) {
        L.push(`${v(c)} = RDK.AddFolder(${py(c.name)}, ${pv})`);
      } else {
        L.push(`${v(c)} = RDK.AddFrame(${py(c.name)}, ${pv})`, `${v(c)}.setPose(${mat(c.pose())})`);
      }
      if (!c.visible) L.push(`${varOf.has(c.id) ? v(c) : 'None'} and ${v(c)}.setVisible(False)`);
      emit(c);
    }
  };
  emit(station);

  for (const prog of station.itemsOfType<Program>(ItemType.PROGRAM)) {
    const compiled = compileForPost(station, prog);
    const script = ROBODK_PYTHON.generate(compiled)[0].content;
    L.push(``, `# ---- Program ${prog.name} ----`);
    // strip the header of the sub-script (imports/RDK) to embed
    L.push(...script.split('\n').filter((l) => !/^from robodk|^RDK = Robolink|^#/.test(l)));
  }
  L.push(``, `RDK.Render(True)`, `print("Station ${station.name} rebuilt in RoboDK — use File > Save Station to write the .rdk file")`);
  return L.join('\n') + '\n';
}
