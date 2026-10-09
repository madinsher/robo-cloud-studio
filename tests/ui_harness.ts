/**
 * Shared jsdom harness for the UI tests: DOM stubs for what jsdom lacks (canvas 2D context, object URLs,
 * anchor clicks, pointer capture), a renderer stub for `App` (mount() needs WebGL) and small helpers to
 * drive menus, context menus and dialogs through real DOM events.
 */
import * as THREE from 'three';
import { App } from '../src/app';
import { demos } from '../src/demos';
import { identity } from '../src/core/math/pose';

export interface Download { name: string; href: string }
export const downloads: Download[] = [];
export const rendererCalls: string[] = [];

function ctx2d(canvas: HTMLCanvasElement): CanvasRenderingContext2D {
  const target: Record<string | symbol, unknown> = {
    canvas,
    measureText: (s: string) => ({ width: s.length * 6 }),
    getImageData: (_x: number, _y: number, w: number, h: number) => ({ data: new Uint8ClampedArray(w * h * 4), width: w, height: h }),
    createImageData: (w: number, h: number) => ({ data: new Uint8ClampedArray(w * h * 4), width: w, height: h }),
  };
  return new Proxy(target, {
    get: (t, k) => (k in t ? t[k] : typeof k === 'string' ? () => undefined : undefined),
    set: (t, k, v) => { t[k] = v; return true; },
  }) as unknown as CanvasRenderingContext2D;
}

let installed = false;
/** Install the jsdom polyfills once per test file. */
export function installDomStubs(): void {
  if (installed) return;
  installed = true;
  HTMLCanvasElement.prototype.getContext = function (this: HTMLCanvasElement) { return ctx2d(this); } as any;
  HTMLCanvasElement.prototype.toDataURL = () => 'data:image/png;base64,AAAA';
  URL.createObjectURL = () => `blob:test/${Math.random().toString(36).slice(2)}`;
  URL.revokeObjectURL = () => {};
  HTMLAnchorElement.prototype.click = function (this: HTMLAnchorElement) { downloads.push({ name: this.download, href: this.href }); };
  if (!(Element.prototype as any).setPointerCapture) (Element.prototype as any).setPointerCapture = () => {};
  if (!(Element.prototype as any).releasePointerCapture) (Element.prototype as any).releasePointerCapture = () => {};
  if (!document.elementFromPoint) (document as any).elementFromPoint = () => null;
  if (typeof globalThis.requestAnimationFrame !== 'function') (globalThis as any).requestAnimationFrame = (fn: FrameRequestCallback) => setTimeout(() => fn(performance.now()), 16);
  window.open = (() => null) as any;
  (HTMLElement.prototype as any).scrollIntoView = () => {};
}

/** Replace the WebGL renderer with a recording stub (every unknown method is a no-op). */
export function stubRenderer(app: App, overrides: Record<string, unknown> = {}): void {
  const scene = new THREE.Scene();
  const base: Record<string | symbol, unknown> = {
    showFrames: true, showTargets: true, showReach: false, showTrajectory: true,
    scene,
    transform: { getHelper: () => ({ visible: true }) },
    renderer: { domElement: document.createElement('canvas') },
    screenshot: () => 'data:image/png;base64,AAAA',
    getViewPose: () => identity(),
    pick: () => null,
    onAnimate: () => () => {},
    ...overrides,
  };
  app.renderer = new Proxy(base, {
    get: (t, k) => { if (k in t) return t[k]; return (..._a: unknown[]) => { rendererCalls.push(String(k)); return undefined; }; },
    set: (t, k, v) => { t[k] = v; return true; },
  }) as any;
}

/** A fresh App with the renderer stub and (optionally) a demo station. */
export function makeApp(demoId?: string): App {
  installDomStubs();
  const app = new App();
  stubRenderer(app);
  if (demoId) app.setStation(demos.find((d) => d.id === demoId)!.build());
  return app;
}

export const tickMs = (ms = 0) => new Promise<void>((r) => setTimeout(r, ms));
export const flush = async (n = 3) => { for (let i = 0; i < n; i++) await tickMs(0); };
export async function waitFor<T>(fn: () => T | null | undefined | false, ms = 3000, label = ''): Promise<T> {
  const t0 = Date.now();
  for (;;) {
    const v = fn();
    if (v) return v as T;
    if (Date.now() - t0 > ms) throw new Error(`waitFor: timeout ${label || fn.toString().slice(0, 120)}`);
    await tickMs(5);
  }
}

// ---- toasts ---------------------------------------------------------------------------------
export const toasts = (): string[] => [...document.querySelectorAll('#toasts .toast')].map((e) => e.textContent ?? '');
export const lastToast = (): string => toasts().at(-1) ?? '';
export const clearToasts = (): void => { document.getElementById('toasts')?.remove(); };

// ---- menus / context menus -------------------------------------------------------------------
export const ctxItems = (root: ParentNode = document): HTMLElement[] => [...root.querySelectorAll<HTMLElement>('.ctx-menu .ctx-item')];
export function ctxItem(label: string, root: ParentNode = document): HTMLElement {
  const it = ctxItems(root).find((e) => (e.textContent ?? '').replace(/^✓ /, '').startsWith(label));
  if (!it) throw new Error(`context menu item "${label}" not found among: ${ctxItems(root).map((e) => e.textContent).join(' | ')}`);
  return it;
}
export const closeMenus = (): void => document.querySelectorAll('.ctx-menu').forEach((m) => m.remove());
/** Click the top-level menu button with this label and return the opened menu. */
export function openMenu(label: string): HTMLElement {
  closeMenus();
  const btn = [...document.querySelectorAll<HTMLButtonElement>('.menu-btn')].find((b) => b.textContent === label);
  if (!btn) throw new Error(`menu "${label}" not found`);
  btn.click();
  return document.querySelector<HTMLElement>('.ctx-menu')!;
}
/** Open a top-level menu and click one of its items. */
export function clickMenu(menu: string, item: string): void {
  openMenu(menu);
  ctxItem(item).click();
  closeMenus();
}

// ---- dialogs ---------------------------------------------------------------------------------
export const dialogs = (): HTMLElement[] => [...document.querySelectorAll<HTMLElement>('.overlay')];
export const currentDialog = (): HTMLElement => { const d = dialogs().at(-1); if (!d) throw new Error('no dialog open'); return d; };
export const dialogTitle = (d: HTMLElement = currentDialog()): string => d.querySelector('.dialog-title')?.childNodes[0]?.textContent ?? '';
export const dialogFields = (d: HTMLElement = currentDialog()): HTMLElement[] => [...d.querySelectorAll<HTMLElement>('.dialog-form label.field')];
export function fieldInput(labelText: string, d: HTMLElement = currentDialog()): HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement {
  const f = dialogFields(d).find((l) => l.querySelector('.field-label')?.textContent === labelText) ?? dialogFields(d).find((l) => (l.querySelector('.field-label')?.textContent ?? '').startsWith(labelText));
  if (!f) throw new Error(`dialog field "${labelText}" not found among: ${dialogFields(d).map((l) => l.querySelector('.field-label')?.textContent).join(' | ')}`);
  return f.querySelector('input,select,textarea') as HTMLInputElement;
}
/** Set a dialog field and fire its change/input event. */
export function setField(labelText: string, value: string | number | boolean, d: HTMLElement = currentDialog()): void {
  const inp = fieldInput(labelText, d);
  if (inp instanceof HTMLInputElement && inp.type === 'checkbox') inp.checked = !!value;
  else inp.value = String(value);
  inp.dispatchEvent(new Event(inp instanceof HTMLInputElement && inp.type === 'range' ? 'input' : 'change', { bubbles: true }));
}
export const dialogButton = (text: string, d: HTMLElement = currentDialog()): HTMLButtonElement => {
  const b = [...d.querySelectorAll<HTMLButtonElement>('button')].find((x) => x.textContent === text);
  if (!b) throw new Error(`dialog button "${text}" not found among: ${[...d.querySelectorAll('button')].map((x) => x.textContent).join(' | ')}`);
  return b;
};
export const clickOk = (d: HTMLElement = currentDialog()): void => { d.querySelector<HTMLButtonElement>('.dialog-actions .btn.primary')!.click(); };
export const clickCancel = (d: HTMLElement = currentDialog()): void => { d.querySelector<HTMLButtonElement>('.dialog-actions .btn:not(.primary)')!.click(); };
export const closeAllDialogs = (): void => dialogs().forEach((d) => d.remove());

// ---- files -----------------------------------------------------------------------------------
/** Resolve a pending pickFiles() (hidden file input appended to body) with these files. */
export function resolvePickFiles(files: File[]): void {
  const inputs = [...document.querySelectorAll<HTMLInputElement>('body > input[type=file]')];
  const input = inputs.at(-1);
  if (!input) throw new Error('no pending file input');
  Object.defineProperty(input, 'files', { value: files, configurable: true });
  input.dispatchEvent(new Event('change'));
}
export const textFile = (name: string, text: string): File => new File([text], name);

// ---- events ----------------------------------------------------------------------------------
export const mouse = (type: string, init: MouseEventInit = {}) => new MouseEvent(type, { bubbles: true, cancelable: true, ...init });
export const pointer = (type: string, init: PointerEventInit = {}) => new PointerEvent(type, { bubbles: true, cancelable: true, pointerId: 1, ...init });
export const key = (k: string, init: KeyboardEventInit = {}) => new KeyboardEvent('keydown', { bubbles: true, cancelable: true, key: k, ...init });
export function change(el: Element, value?: string | boolean): void {
  const inp = el as HTMLInputElement;
  if (value !== undefined) { if (inp.type === 'checkbox') inp.checked = value === true || value === 'true'; else inp.value = String(value); }
  el.dispatchEvent(new Event('change', { bubbles: true }));
}
export function input(el: Element, value: string): void { (el as HTMLInputElement).value = value; el.dispatchEvent(new Event('input', { bubbles: true })); }
export const buttonByText = (root: ParentNode, text: string): HTMLButtonElement => {
  const all = [...root.querySelectorAll<HTMLButtonElement>('button')];
  const b = all.find((x) => (x.textContent ?? '').trim() === text) ?? all.find((x) => (x.textContent ?? '').trim().startsWith(text));
  if (!b) throw new Error(`button "${text}" not found among: ${[...root.querySelectorAll('button')].map((x) => x.textContent?.trim()).join(' | ')}`);
  return b;
};
