/** Help › Demo scenarios: load a scenario station into the studio or run the whole suite and show the report. */
import type { App } from '../app';
import { h, clear, dialog, toast, downloadText } from './dom';
import { ALL_SCENARIOS, Scenario, ScenarioResult, scenarioReportMarkdown } from '../scenarios';
import { t } from './i18n';

export async function scenariosDialog(app: App, initial: 'all' | 'navigation' | 'vision' | 'control' | 'group' = 'all'): Promise<void> {
  let group: 'all' | 'navigation' | 'vision' | 'control' | 'group' = initial;
  const list = h('div', { class: 'nav-recs' });
  const results = new Map<string, ScenarioResult>();
  const status = h('div', { class: 'hint' });
  const render = () => {
    clear(list);
    for (const s of ALL_SCENARIOS.filter((x) => group === 'all' || x.group === group)) {
      const r = results.get(s.id);
      list.appendChild(h('div', { class: 'nav-rec' },
        h('div', { class: 'nav-rec-head' }, h('b', null, `${s.group === 'navigation' ? '🧭' : s.group === 'vision' ? '👁' : s.group === 'group' ? '🤖' : '🎛'} ${s.title}`), r ? h('span', { class: `badge ${r.pass ? 'ok' : 'warn'}` }, r.pass ? 'pass' : 'FAIL') : h('span', { class: 'badge' }, s.method)),
        h('div', { class: 'hint' }, s.description),
        h('div', { class: 'hint' }, `${t('How to')}: ${s.howTo}`),
        r ? h('div', { class: 'hint' }, r.metrics.filter((m) => m.ok !== undefined).map((m) => `${m.name} ${m.value}${m.unit ? ' ' + m.unit : ''} ${m.ok ? '✓' : '✗'}`).join(' · ')) : null,
        h('div', { class: 'btn-row' },
          h('button', { class: 'btn small primary', onClick: () => load(s) }, t('Load into the studio')),
          h('button', { class: 'btn small', onClick: () => runOne(s) }, t('Run headless')))));
    }
  };
  const load = (s: Scenario) => {
    const { station, focus } = s.build();
    app.setStation(station);
    const item = focus ? app.station.findById(focus.id) : null;
    if (item) app.select(item);
    (app as any).bottom?.show?.(s.group === 'navigation' ? 'nav' : s.group === 'vision' ? 'vision' : 'control');
    if (s.group === 'group') (app as any).controlPanel?.render?.();
    if (s.group === 'navigation') app.startWorld();
    toast(`${s.title} loaded`, 'ok');
    app.log(`Scenario ${s.id}: ${s.howTo}`);
    document.querySelector<HTMLElement>('.dialog .dialog-title .btn-icon')?.click();
  };
  const runOne = async (s: Scenario) => {
    status.textContent = `${t('Running')} ${s.title}…`;
    const r = await s.run();
    results.set(s.id, r);
    app.log(`${r.pass ? 'PASS' : 'FAIL'} ${r.title}: ${r.metrics.map((m) => `${m.name}=${m.value}${m.unit ?? ''}`).join(', ')}`, r.pass ? 'info' : 'warn');
    status.textContent = `${r.pass ? '✓' : '✗'} ${r.title} (${r.durationMs} ms)`;
    render();
  };
  const runAll = async () => {
    for (const s of ALL_SCENARIOS.filter((x) => group === 'all' || x.group === group)) await runOne(s);
    const all = [...results.values()];
    status.textContent = `${all.filter((r) => r.pass).length}/${all.length} ${t('scenarios pass')} — ${t('report saved to the Log; Download report for Markdown')}`;
  };
  const body = h('div', null,
    h('div', { class: 'btn-row' },
      Object.assign(h('select', { onChange: (e: Event) => { group = (e.target as HTMLSelectElement).value as any; render(); } }, h('option', { value: 'all' }, t('All scenarios')), h('option', { value: 'navigation' }, t('Navigation & localization')), h('option', { value: 'vision' }, t('Machine vision')), h('option', { value: 'control' }, t('Control design')), h('option', { value: 'group' }, t('Group control'))), { value: initial }),
      h('button', { class: 'btn small', onClick: runAll }, t('Run all (headless)')),
      h('button', { class: 'btn small', onClick: () => { if (!results.size) return toast('Run the scenarios first', 'warn'); downloadText('scenario-results.md', scenarioReportMarkdown([...results.values()]), 'text/markdown'); } }, t('Download report (.md)'))),
    status, list);
  render();
  await dialog('Demo scenarios — navigation, machine vision, control design, group control', [], { width: 900, okLabel: 'Close', body });
}
