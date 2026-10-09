import { defineConfig } from 'vite';
import { fileURLToPath, URL } from 'node:url';

/** Bundles the studio server (Node) into dist-server/index.js. */
export default defineConfig({
  resolve: { alias: { '@': fileURLToPath(new URL('./src', import.meta.url)) } },
  build: {
    ssr: 'server/index.ts',
    outDir: 'dist-server',
    target: 'node20',
    sourcemap: false,
    emptyOutDir: true,
    rollupOptions: { external: ['ws'], output: { entryFileNames: 'index.js', format: 'es' } },
  },
  ssr: { noExternal: true, external: ['ws'] },
} as any);
