# navibot

给四足机器狗（Unitree Go2）做的导航控制台：在网页上看点云地图、标导航点、下发路线、看它实际走到哪。**支持上下楼梯。**

底层的局部规划和避障由 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)（`navi_mode=2`）负责，本项目只做它上面那一层：地图预处理、路线管理、导航点下发、状态可视化。

---

## 这个项目最核心的一件事：不要假设"地面只有一个高度"

最早的版本用一个全局 `floor_z` 表示地面高度。这在单层房间里能用，**碰到带楼梯的地图会彻底算错**，而且错得很安静：

| | 改之前 | 改之后 |
|---|---|---|
| 检测出的地面 `floor_z` | 1.804（其实是一楼天花板） | **−0.557** |
| 层高 | **0.263 m** | 2.624 m |
| 俯视图 | 一根墙都没有，只有几块灰板 | 完整平面图，墙是清晰的细线 |

原因是"z 直方图上下半各取一峰"这个办法在两层楼里，"下半的最高峰"取到的是一楼天花板。障碍带于是扫在天花板上。

现在改成 **逐格高程面**（`map_pipeline/elevation.py`）：每个 xy 栅格估一个可站立高度，楼梯就是高程连续爬升的一条窄带，不需要引入"楼层"这个抽象。

---

## 三个部分

```
map_pipeline/     离线地图预处理: 点云 -> 高程面 / 占据栅格 / 俯视图 / 3D 预览点云
backend/          FastAPI + rospy: 地图管理、路线存储、A* 参考路线、导航点下发、状态推送
frontend/         React + Vite + Three.js + Konva: 地图管理、俯视图选点、3D 预览、导航控制台
```

### 数据流

```
data/maps.db (SQLite)                地图表: 名称 -> 存储路径, 网页"导入地图"按钮写入
        │
        ▼
<存储路径>/3d_map/dense_cloud_map.pcd    HandBot-S1 建的稠密点云
<存储路径>/3d_map/keyframe_info_3d.txt   建图轨迹 —— 高程提取的种子, 缺了它带楼梯的图就没法处理
        │
        │  map_pipeline/generate_map_assets.py
        ▼
web_assets/map/<name>/
    elevation.npy          逐格可站立高度 (float32, NaN=不可站立)
    occupancy.npy          占据栅格 (0不可站立 / 1可通行 / 2障碍)
    topview.png            俯视图 (两色平面图 + 楼梯按抬升着色)
    topview_standable.png  可站立区提示层
    topview_safety.png     安全边距提示层
    overlaps.json          自检: 同一格存在多个可站立高度 (非空 = 单值表示已不够用)
    pointcloud.bin         降采样点云 (自定义 PCW1 格式), 供前端 3D 预览
    topview_meta.json      坐标元数据 + 高程统计
```

地图的原始数据（`dense_cloud_map.pcd` / `keyframe_info_3d.txt`）不归 navibot 管，留在用户自己的存储路径下；"导入地图"只是往 `data/maps.db` 里记一笔名字和路径，不拷贝文件。删除地图只删数据库这一行和 `web_assets/map/<name>/` 下自己生成的预处理产物，不会碰存储路径下的原始文件。

---

## 跑起来

前置：`roscore`；后端需要 `fastapi` / `uvicorn` / `numpy`；离线预处理另需 `open3d` / `scipy` / `pillow`。

> **Python 版本**：后端必须兼容 **Python 3.8** —— 机器（lubancat）上跑的是 ROS Noetic
> 自带的 3.8，`rospy` 也是为它编译的。开发机用的是 mamba `ros_host` 里的 3.12，
> `dict[str, int]`、`X | None`、`asyncio.to_thread` 这些写法在本地一路绿灯，推到机器上
> 才在 import 阶段炸掉，本地永远测不出来。提交前跑一次：
>
> ```bash
> python3 tools/check_py38.py
> ```

```bash
# 1. 准备一份地图数据, 目录里要有 3d_map/ 子目录 (带楼梯的图 keyframe_info_3d.txt 必须有)
#   /path/to/myroom/3d_map/dense_cloud_map.pcd
#   /path/to/myroom/3d_map/keyframe_info_3d.txt
# 然后在网页"地图管理"里点"导入地图", 名称填 myroom, 存储路径填 /path/to/myroom
# (只是往 data/maps.db 记一笔账, 不拷贝文件)

# 2. 预处理 (也可以在网页的"地图管理"里点按钮触发)
mamba run -n ros_host python map_pipeline/generate_map_assets.py \
    --input /path/to/myroom/3d_map/dense_cloud_map.pcd \
    --outdir web_assets/map/myroom

# 3. 后端 (需要 roscore 已在跑)
./backend/run.sh

# 4. 前端
cd frontend && npm install && npm run dev
```

没有实机时，用模拟器验证整条链路（它刻意复刻了真 planner 的到达判据、跳点规则、frozen 行为）：

```bash
mamba run -n ros_host python backend/mock_planner.py --start <X> <Y> <Z> <YAW>
```

> `--start` 的 Z 是 **odom 系机体高度**（地面高程 + 传感器离地高度），不是离地高度本身。默认 `(0,0,0,0)` 在多数地图里都在墙里。

---

## 与 SCAN-Planner 的接口（navi_mode=2）

契约的每一条都对着 `scan_replan_fsm.cpp` 核过，细节写在 `backend/app/config.py` 的模块注释里。

| 话题 | 类型 | 方向 |
|---|---|---|
| `/preset_waypoints` | `nav_msgs/Path` | backend → planner，一条 Path = 一整轮任务 |
| `/planning/go2_execution_frozen` | `std_msgs/Bool` | backend → planner，冻结/解冻轨迹执行 |
| `/hand_lio/odom_vehicle` | `nav_msgs/Odometry` | → backend，位姿 + `covariance[0]` 定位质量 |

**三条必须照抄 planner 行为的地方**，抄错任何一条都会静默错位：

1. **到达判定是 3D 距离 < 0.5 m**，而且 planner **不发布任何到达/完成话题**。后端只能订阅 odom 用同一套判据自己推进度。
2. **新一轮开始时跳过距当前位置 0.5 m 以内的点**（`planNextWaypoint`），后端下发后立刻做同样的跳过，否则第一个点会一直显示成"没到过"。
3. **`/preset_waypoints` 不 latch 且队列为 1**，没订阅者时发出去被静默丢弃。所以下发前等订阅者连上，等不到就返回 HTTP 503。发布端也刻意**不 latch** —— latch 会让 planner 一重启就自己跑上一轮路线。

**路线只发一次。** 重发不是冗余而是有害：planner 收到新 Path 会 `current_wp_ = 0` 整轮重置，跑到一半重发会让狗掉头回起点。

---

## 导航点的 z 是怎么来的

```
下发 z = 该点地面高程 + Δ + z_offset
```

- **地面高程**：查 `elevation.npy`；点击位置不在认证格上时就近找（1 m 内）
- **Δ**：`当前 odom.z − 狗脚下的地面高程`，**运行时实测**
- **z_offset**：用户微调（SCAN-Planner 的 README 明确写了爬不上楼梯就抬 keypoint 的 z）

**路线里不存绝对 z。** odom 的 z 基准取决于 hand-lio 的 `lidar_t_body` 外参，存了绝对值就会在某天标定之后集体失效，而且失效得很安静 —— 偏 0.3 m 不报错，只是让那个 0.5 m 的到达判据变脆。

**Δ 也不能用预处理时存的那个值**（`delta_sensor_m`）。那是从建图轨迹量的，而运行时 odom 还要经过 `imu_T_lidar` 和 `lidar_T_body` 两次外参变换 —— 两者不是同一个基准。现场量就自洽：Δ 和 planner 的到达判据用的是同一个 odom，所以无论外参怎么改都对得上。

---

## 一些被数据打回来才定下的规则

高程提取（`map_pipeline/elevation.py`）的每条规则都对应一个具体的失败模式，改之前先看清楚它在挡什么：

| 试过的做法 | 被什么打回来 |
|---|---|
| 落脚面取最高的 | 平地棘轮式漂移，中位高程从 −0.56 爬到 +0.14 |
| 改成取支撑最强的 | 大邻域下仍漂：自己脚下没扫到的格子会挑中半米外的家具 |
| **取离邻格最近的** | 自稳，采纳 |
| 不做净空判据 | 顺着墙面每格爬一档爬上墙 |
| 不限高度带 | 顺楼梯爬到 3.74 m 再踩上一楼天花板摊开全图（楼上没进去过，天花板上方"没有点"被当成了"空的"） |
| 台阶不限位置 | 踩着柜子、床、桌面跨上去。几何上无法和楼梯区分，只能靠"楼梯狗走过、家具没走过" |
| 只限单格高差 | 0.06 m/格沿几十格攒起来照样爬一米，需要一条全局的"背书"约束 |
| 生长做在格上 | 天花板阵面先到先得，把真正的地面挤没了。改到 (格, 高程) 状态空间 |

同一个主题反复出现：**未观测 ≠ 空**。"上方没有点"既可能是空的，也可能是从没扫到过。

预处理还会做一次**自检**：如果某些格子存在多个可站立高度（折返楼梯、夹层），写进 `overlaps.json` 并告警，表示当前的单值表示已经不够用了。宁可举手也不要默默画出一张看着正常、实际把上下楼梯压在一起的地图。

---

## 已知缺口

这几条是如实标注的，不要当成已解决：

- **"冻结执行"不是硬急停。** navi_mode=2 没有外部急停接口，内部的 `EMERGENCY_STOP` 只由它自己的碰撞检测触发。冻结只让 planner 不再推进轨迹时间，**不断电、不立即制动**。真正的硬急停必须在 `unitree_bridge` 那一层做。
- **取消不会清空 planner 的任务队列**，只是冻结。要真正换任务只能下发新路线（新 Path 整轮替换）。
- **进度是推断出来的**，不是 planner 报的。planner 卡住时我们看不出区别，只有 60 s 超时兜底。
- **Δ 只用单次采样。** 下发那一刻狗若正好站在楼梯踏面上，台阶量化误差（实测楼梯段 IQR 0.090 vs 平地 0.031）会落到 Δ 上并施加到整条路线。改成滑动窗口取中位数可解。
- **可站立区只覆盖走过的地方。** 想要更大的可用区域，让狗多走两圈比调参数可靠。
- **前端未经真人浏览器验证。** 类型检查和构建通过，但布局/配色/交互没有实际看过。

---

## 相关仓库

- [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) — 局部规划器，本项目对接 `navi_mode=2`
- `hand-lio` — 把 HandBot-S1 的 `/latest_imu_odom` 和原始点云桥接成 map 系点云与机体位姿
