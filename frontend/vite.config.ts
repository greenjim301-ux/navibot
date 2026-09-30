import path from 'node:path'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

// 开发时后端在哪: 默认本机 8000, 指到板子/别的机器就用这个环境变量覆盖。
// WS 的地址由它推导(http -> ws, https -> wss), 免得两个变量写得不一致。
// 注意这是 dev server 的 proxy 目标, **不会进构建产物**, 所以不需要 .env.local。
const DEV_BACKEND = process.env.NAVIBOT_DEV_BACKEND || 'http://localhost:8000'
const DEV_BACKEND_WS = DEV_BACKEND.replace(/^http/, 'ws')

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
  // 后端不在本机时(比如真机跑在板子上)**不要**用 .env.local 去指: 那是编译期内联的,
  // 会把调试用的绝对地址一起打进 `npm run build` 的产物里 —— 踩过一次, 部署到板子上
  // 的前端还在请求开发机的 192.168.123.200:8000, 页面能开但接口全超时。指别处只用
  // 上面那个环境变量, 只影响 dev server 的 proxy, 产物保持同源相对路径:
  //   NAVIBOT_DEV_BACKEND=http://10.42.0.1:8000 npm run dev     # 或者 npm run dev:board
  //
  // **键必须带 ^ 和结尾的斜杠。** vite 的 proxy 键默认是**前缀匹配**, 写 '/map' 会
  // 把 /maps 和 /mapping/xxx 这两个前端路由一起吃掉 —— 它们都以 /map 开头。后果:
  // 直接输地址或刷新这两个页面时, vite 把导航请求代理给后端, 而后端不在本机时
  // 整页 502 Bad Gateway, 而 /routes 这类不撞前缀的页面一直是好的。
  // 键以 ^ 开头时 vite 按正则匹配, '^/map/' 只命中真正的资源路径 /map/<图>/<文件>,
  // 不再命中 /maps 和 /mapping/xxx。
  server: {
    proxy: {
      '^/api/': { target: DEV_BACKEND, changeOrigin: true },
      '^/map/': { target: DEV_BACKEND, changeOrigin: true },
      '^/ws/': { target: DEV_BACKEND_WS, ws: true },
    },
  },
})
