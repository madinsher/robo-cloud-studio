import { describe, it, expect } from 'vitest';
import { analyse } from '../src/ctl/analysis';
import { TEMPLATES, parseDes, parseBt, parseSmv, detectKind, parsePddlDoc, parsePetri } from '../src/ctl/dsl';
import { COURSE_EXAMPLES, YOUBOT_BT_DSL, YOUBOT_DES_DSL } from '../src/ctl/examples_dsl';
import { ControlRuntime, simulatedBindings, stationBindings } from '../src/ctl/runtime';
import { DES, parallel, supcon, supervisorTable } from '../src/ctl/des';
import { youbotPlant, youbotSpecs, YOUBOT_UC, YOUBOT_UO } from '../src/ctl/examples';
import { robustness, parseSTL } from '../src/ctl/vv';
import { CONTROL_KINDS, addControlModel, controlModels, ControlModelItem } from '../src/ctl/model';
import { Station } from '../src/core/items/item';
import { MobileRobot, ZoneItem } from '../src/mobile/items';

describe('Control DSL and analysis', () => {
  it('every template parses, is detected and analyses without error', () => {
    for (const { kind } of CONTROL_KINDS) {
      const src = TEMPLATES[kind];
      expect(detectKind(src), kind).toBe(kind);
      const r = analyse(kind, src);
      expect(r.error, `${kind}: ${r.error}`).toBeUndefined();
      expect(r.sections.length).toBeGreaterThan(0);
      expect(r.markdown).toContain('##');
    }
  });
  it('every course example analyses; verdicts match the course', () => {
    const byId = Object.fromEntries(COURSE_EXAMPLES.map((e) => [e.id, analyse(e.kind, e.source)]));
    for (const [id, r] of Object.entries(byId)) expect(r.error, `${id}: ${r.error}`).toBeUndefined();
    expect(byId['A-des'].metrics['supervisor states']).toBe(324); expect(byId['A-des'].metrics['plant states']).toBe(648); expect(byId['A-des'].ok).toBe(true);
    expect(byId['A-des-e4'].metrics['realizable']).toBe(false); expect(byId['A-des-e4'].ok).toBe(false);
    expect(byId['A-gr1'].metrics['realizable']).toBe(true); expect(byId['A-gr1'].metrics['game states']).toBe(160);
    expect(byId['A-pddl'].metrics['plan found']).toBe(true); expect(byId['A-pddl'].metrics['HTN plan found']).toBe(true);
    expect(byId['B-petri'].metrics['deadlocks']).toBe(1); expect(byId['B-petri'].metrics['monitors added']).toBe(1); expect(byId['B-petri'].ok).toBe(false);
    expect(byId['B-s3pr'].metrics['bad siphons']).toBeGreaterThan(0);
    expect(byId['B-smv'].ok).toBe(false); expect(byId['B-smv-fair'].ok).toBe(true);
    expect(byId['B-jobshop'].metrics['best makespan']).toBe(54); expect(byId['B-jobshop'].metrics['lower bound']).toBe(44);
    expect(byId['B-mrta'].metrics['optimal sum']).toBe(81); expect(byId['B-mapf'].metrics['CBS found']).toBe(true); expect(byId['B-mapf'].metrics['priority failed']).toBe(1);
    expect(byId['B-reliability'].metrics['S_p (m)']).toBe('1.54'); expect(byId['B-reliability'].metrics['PL']).toBe('d');
    expect(byId['A-fta'].metrics['min order']).toBe(4); expect(byId['A-realtime'].metrics['RM schedulable']).toBe(true); expect(byId['A-realtime'].ok).toBe(false); // latency budget fails at 0.15 m/s
    expect(byId['B-perf'].metrics['bottleneck']).toBe('S'); expect(byId['B-stn'].metrics['dynamically controllable']).toBe(true);
    expect(byId['A-bt'].ok).toBe(true); expect(byId['A-mdp'].sections[0].table!.rows.find((r) => r[0] === 's0')![1]).toBe('12.943');
  });
  it('DSL parsers round-trip the course DES and report syntax errors with line numbers', () => {
    const doc = parseDes(YOUBOT_DES_DSL);
    expect(doc.plant.length).toBe(5); expect(doc.specs.length).toBe(4); expect(doc.uncontrollable).toEqual(YOUBOT_UC); expect(doc.checks.length).toBe(2);
    const g = parallel(...doc.plant.map((a) => DES.fromSpec(a))); expect(g.X.size).toBe(648);
    expect(() => parseDes('automaton X\n  A -a-> B\n  garbage line')).toThrow(/line 3/);
    expect(() => parseBt('bt x\naction a\naction b')).toThrow(/one root/);
    const bt = parseBt(YOUBOT_BT_DSL); expect(bt.root.children!.length).toBe(3); expect(bt.monitors.length).toBe(2); expect(bt.outcomes.halt).toEqual(['success', 'running']);
    const smv = parseSmv(TEMPLATES.smv); expect(smv.vars.length).toBe(3); expect(smv.next!['owner'].length).toBe(5); expect(smv.specs!.length).toBe(3);
    const pd = parsePddlDoc(TEMPLATES.pddl); expect(pd.domain).toContain('(:action move'); expect(pd.problem).toContain('(:goal');
    const pn = parsePetri(TEMPLATES.petri); expect(pn.spec.arcs.length).toBe(20); expect(pn.checks.length).toBe(1);
  });
  it('control model items live in the station tree and survive serialisation', () => {
    const st = new Station('s');
    const m = addControlModel(st, 'des', 'plant', TEMPLATES.des);
    expect(controlModels(st).length).toBe(1);
    const st2 = Station.deserialize(st.serialize());
    const m2 = controlModels(st2)[0]; expect(m2).toBeInstanceOf(ControlModelItem); expect(m2.kind).toBe('des'); expect(m2.source).toBe(TEMPLATES.des); expect(m2.id).toBe(m.id);
  });
});

describe('Mission runtime', () => {
  const supervisorOf = () => { const g = parallel(...youbotPlant(true)); const S = youbotSpecs(); const r = supcon(g, [S.E1, S.E2, S.E3, S.E5], YOUBOT_UC); return supervisorTable(r.supervisor, YOUBOT_UC, YOUBOT_UO, 5); };
  it('runs the course mission in the simulated world under the synthesised supervisor: all objects delivered, no denials, monitors hold', () => {
    const bt = parseBt(YOUBOT_BT_DSL);
    const world = simulatedBindings({ targets: 1 }, undefined, { speed: 4 }); // the course plant models one object per mission
    const rt = new ControlRuntime({ bt, supervisor: supervisorOf(), bindings: world });
    rt.run(400, 0.1, (bb) => bb.reported === true);
    const s = rt.summary();
    expect(world.state.targets).toBe(0); expect(rt.bb.reported).toBe(true);
    expect(s.denied).toBe(0); expect(s.supervisorErrors).toBe(0); expect(s.violations).toBe(0);
    expect(s.plantState).toMatch(/^H,S,O/); // back home, arm stowed, gripper open
    expect(rt.trace.t.length).toBe(rt.ticks);
    expect(robustness(parseSTL('G (held -> F[0,60] !held)'), rt.trace)).toBeGreaterThan(0);
  });
  it('supervisor denies a move with the arm extended; monitor catches a drop; e-stop pre-empts and halts', () => {
    const bt = parseBt(`bt unsafe
sequence root memory
  action goto zone=table event=b_go_table arrive=b_arrive timeout=60
  action reach event=a_reach done=a_reached timeout=10
  action goto zone=bin event=b_go_box arrive=b_arrive timeout=60
monitor G(held -> (held U placed))`);
    const world = simulatedBindings({}, undefined, { speed: 4 });
    const rt = new ControlRuntime({ bt, supervisor: supervisorOf(), bindings: world });
    rt.run(60, 0.1, () => rt.status !== 'running');
    expect(rt.status).toBe('failure'); expect(rt.denied).toBe(1); expect(rt.log.some((l) => /denied b_go_box/.test(l))).toBe(true);
    // drop monitor
    const bt2 = parseBt(`bt drop
sequence root memory
  action set held=true
  action wait seconds=0.5
  action set held=false
monitor G(held -> (held U placed))`);
    const rt2 = new ControlRuntime({ bt: bt2, bindings: simulatedBindings() });
    rt2.run(3, 0.1); expect(rt2.violations).toBe(1); expect(rt2.verdicts()[0].verdict).toBe('⊥');
    // e-stop
    const bt3 = parseBt(YOUBOT_BT_DSL); const w3 = simulatedBindings({ targets: 1 }, undefined, { speed: 2 });
    const rt3 = new ControlRuntime({ bt: bt3, bindings: w3 });
    rt3.run(2); expect(w3.state.moving).toBe(true);
    w3.state.estop = true; rt3.tick(0.1); rt3.tick(0.1);
    expect(w3.state.moving).toBe(false); expect(rt3.status).toBe('success');
  });
  it('drives a real station mobile robot between zones', () => {
    const st = new Station('cell'); const r = st.addChild(new MobileRobot('AMR')); r.setPose2D(0, 0, 0);
    const mk = (name: string, x: number, y: number) => { const z = st.addChild(new ZoneItem(name)); z.polygon = [[x - 1000, y - 1000], [x + 1000, y - 1000], [x + 1000, y + 1000], [x - 1000, y + 1000]]; return z; };
    mk('home', 0, 0); mk('table', 8000, 0); mk('bin', 8000, 6000);
    const bt = parseBt(`bt hop
sequence root memory
  action goto zone=table timeout=60
  action grasp timeout=10
  action goto zone=bin timeout=60
  action place timeout=10
  action goto zone=home timeout=60
  action report`);
    const w = stationBindings({ robot: r, station: st, durations: { grasp: 1, place: 1 } });
    const rt = new ControlRuntime({ bt, bindings: { ...w, step: (dt) => w.stepRobot(dt) } });
    rt.bb.targets = 1;
    rt.run(120, 0.1, (bb) => bb.reported === true);
    expect(rt.bb.reported).toBe(true); expect(Math.hypot(r.state.x, r.state.y)).toBeLessThan(1500); expect(r.params.holding).toBe(false);
  });
});
