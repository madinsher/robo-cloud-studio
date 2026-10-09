import { defineConfig } from 'vite';
import { fileURLToPath, URL } from 'node:url';

export default defineConfig({
  base: './',
  resolve: { alias: { '@': fileURLToPath(new URL('./src', import.meta.url)) } },
  server: { port: 5173, host: true },
  build: { target: 'es2022', sourcemap: true, chunkSizeWarningLimit: 2000 },
  test: {
    environment: 'node',
    include: ['tests/**/*.test.ts'],
    exclude: ['tests/e2e/**', 'node_modules/**'],
    testTimeout: 20000,
    coverage: { provider: 'v8', include: ['src/**/*.ts'], exclude: ['src/main.ts', 'src/vite-env.d.ts'], reporter: ['text-summary', 'json-summary', 'html'], reportsDirectory: 'coverage', reportOnFailure: true },
  },
} as any);
