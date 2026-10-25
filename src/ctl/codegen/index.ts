/** Code generators from control models (see the modules of this folder). */
import type { Station } from '../../core/items/item';
import type { ControlKind } from '../model';

export interface GenFile { name: string; content: string; mime?: string }
export interface ExportEntry { id: string; label: string; files(): GenFile[] }
export function controlExports(kind: ControlKind, source: string, name: string, station?: Station): ExportEntry[] { void kind; void source; void name; void station; return []; }
