/// <reference types="vite/client" />
declare module '*.py?raw' { const src: string; export default src; }
declare module 'occt-import-js';
declare module 'occt-import-js/dist/occt-import-js.wasm?url' { const url: string; export default url; }
