#!/usr/bin/env bash
# 在 ros_host (ROS Noetic) 环境里启动后端服务。
# 需要先有 roscore 在跑 (另开一个终端: `mamba run -n ros_host roscore`)。
set -euo pipefail
cd "$(dirname "$0")"
exec mamba run -n ros_host uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
