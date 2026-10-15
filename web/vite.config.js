import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react(), { name: 'relay', configureServer: async () => { await import('./relay.js'); } }],
});
