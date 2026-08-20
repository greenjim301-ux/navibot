#!/usr/bin/env bash
# 启动后端服务。需要先有 roscore 在跑, 且当前 shell 已经装好了依赖 (fastapi /
# uvicorn / rospy 等) 并 source 过 ROS 环境。
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONUNBUFFERED=1
exec uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
