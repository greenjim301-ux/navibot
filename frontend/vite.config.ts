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
  //
  // **键必须带 ^ 和结尾的斜杠。** vite 的 proxy 键默认是**前缀匹配**, 写 '/map' 会
  // 把 /maps 和 /mapping/xxx 这两个前端路由一起吃掉 —— 它们都以 /map 开头。后果:
  // 直接输地址或刷新这两个页面时, vite 把导航请求代理给 localhost:8000, 而后端常
  // 常不在本机(见 .env.local 的 VITE_BACKEND_HTTP 指向板子), 于是整页 502 Bad
  // Gateway, 而 /routes 这类不撞前缀的页面一直是好的。
  // 键以 ^ 开头时 vite 按正则匹配, '^/map/' 只命中真正的资源路径 /map/<图>/<文件>,
  // 不再命中 /maps 和 /mapping/xxx。
  server: {
    proxy: {
      '^/api/': { target: 'http://localhost:8000', changeOrigin: true },
      '^/map/': { target: 'http://localhost:8000', changeOrigin: true },
      '^/ws/': { target: 'ws://localhost:8000', ws: true },
    },
  },
})
