import { defineConfig } from 'vite';
import { fileURLToPath, URL } from 'node:url';

/** Browser end-to-end tests: `npm run test:e2e` (needs `npm run build` first; a preview server is started by the global setup). */
export default defineConfig({
  resolve: { alias: { '@': fileURLToPath(new URL('./src', import.meta.url)) } },
  test: {
    environment: 'node',
    include: ['tests/e2e/**/*.test.ts'],
    globalSetup: ['tests/e2e/setup.ts'],
    testTimeout: 300000,
    hookTimeout: 120000,
    fileParallelism: false,
    reporters: ['default', ['json', { outputFile: 'test-results/e2e-results.json' }]],
  },
} as any);
