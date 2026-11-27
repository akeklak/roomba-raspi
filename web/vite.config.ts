import react from '@vitejs/plugin-react';
import { defineConfig } from 'vitest/config';

export default defineConfig({
  plugins: [react()],
  server: { host: true, port: 8080 },
  preview: { host: true, port: 8080 },
  test: { environment: 'node' },
});
