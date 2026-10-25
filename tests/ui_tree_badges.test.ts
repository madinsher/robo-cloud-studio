// @vitest-environment jsdom
/**
 * V3 finding: the station tree showed every fleet robot as "idle 100%" while the Fleet tab listed them as working (the
 * robot status / battery change without item events, so the tree never repainted its badges). While the world runs the
 * visible rows are now repainted twice a second.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { installDomStubs, makeApp, tickMs } from './ui_harness';
import { TreePanel } from '../src/ui/tree';
import { MobileRobot } from '../src/mobile/items';
import { ItemType } from '../src/core/items/item';

beforeAll(() => installDomStubs());

describe('station tree badges follow the fleet', () => {
  it('a robot that starts working shows "working" in the tree while the world runs', async () => {
    const app = makeApp('orchard');
    const tree = new TreePanel(app); document.body.append(tree.el);
    await tickMs(60);
    const robot = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0];
    expect(robot).toBeTruthy();
    const badge = () => [...tree.el.querySelectorAll<HTMLElement>('.tree-row')].find((r) => r.querySelector('.tree-name')!.childNodes[0].textContent === robot.name)?.querySelector('.badge')?.textContent ?? '';
    expect(badge()).toMatch(/^idle/);
    robot.state.status = 'working';
    (app as any).worldRunning = true;
    await tickMs(650);
    expect(badge()).toMatch(/^working/);
    (app as any).worldRunning = false;
  });
});
