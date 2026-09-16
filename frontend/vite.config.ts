import path from 'node:path'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: {
    alias: {
      '@': path.resolve(import.meta.dirname, './src'),
    },
  },
  // 前端默认按同源请求后端(见 src/api.ts)。dev server 在 5173、后端在 8000,
  // 靠这个 proxy 把三类路径转过去, 这样 dev 和"后端 host 前端"的部署形态用的是
  // 同一套相对路径, 不需要 .env.local 也不会有 CORS。
  //   /api  REST      /map  地图静态资源(pgm/png/点云)     /ws  WebSocket
  server: {
    proxy: {
      '/api': { target: 'http://localhost:8000', changeOrigin: true },
      '/map': { target: 'http://localhost:8000', changeOrigin: true },
      '/ws': { target: 'ws://localhost:8000', ws: true },
    },
  },
})
