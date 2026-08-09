#!/usr/bin/env bash
# 在 ros_host (ROS Noetic) 环境里启动后端服务。
# 需要先有 roscore 在跑 (另开一个终端: `mamba run -n ros_host roscore`)。
set -euo pipefail
cd "$(dirname "$0")"
# PYTHONUNBUFFERED: mamba run 会缓冲子进程的 stdout/stderr, 不设这个的话后端日志
# (下发的导航点坐标、Δ 的告警等) 要攒够一个缓冲区才吐出来, 看起来像"什么都没打印"。
exec mamba run -n ros_host -e PYTHONUNBUFFERED=1 uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
