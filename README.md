# navibot

给四足机器狗（Unitree Go2）做的导航控制台：在网页上看点云地图、标导航点、下发路线、看它实际走到哪。**导航点 z 靠建图轨迹自动算，楼梯目前要靠用户手动加 `z_offset` 抬高，还没有自动分层判断**（见下文"导航点的 z 是怎么来的"）。

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

后来改成 **逐格高程面**（`map_pipeline/elevation.py`）：每个 xy 栅格估一个可站立高度，楼梯就是高程连续爬升的一条窄带，不需要引入"楼层"这个抽象。这条路子精度高，但代价是对整张点云做三维直方图统计，内存/耗时跟**地图物理跨度**成正比——实测一张 3430m×1161m×56m 的图算下来要 ~1.8TB，完全不可行，调分辨率也救不回来（会粗到连楼梯踏步都分不清）。

现在（导航点 z 的获取，先不管楼梯）改成 **就近查建图轨迹**：建图轨迹的 z 就是机器狗当时站在那个 (x, y) 时的身体高度——"狗站过的地方一定能站"，直接拿轨迹当地面高度用，不用碰点云。开销只跟轨迹点数（通常几千到几万个关键帧）成正比，跟地图物理跨度/点云大小完全无关，天然兼容任意大小的地图。代价是不再处理楼梯/多层重叠这类需要"逐格高程面"才能表达的复杂情况——`map_pipeline/elevation.py` 里那套区域生长算法目前没有代码路径在用，是留着的设计参考，见下面"一些被数据打回来才定下的规则"。

---

## 三个部分

```
map_pipeline/     离线地图预处理: 点云 -> 高程面 / 占据栅格 / 俯视图 / 3D 预览点云
backend/          FastAPI + rospy: 地图管理、路线存储、A* 参考路线、导航点下发、状态推送
frontend/         React + Vite + Three.js + Konva: 地图管理、俯视图选点、3D 预览、导航控制台
```

### 数据流

```
map-data-dir/<name>/                 地图数据根目录 (默认 /home/lisi/Documents/map-data),
                                      地图列表直接扫这个目录, 不落数据库
        │
        ▼
<map-data-dir>/<name>/3d_map/dense_cloud_map.pcd    HandBot-S1 建的稠密点云
<map-data-dir>/<name>/3d_map/keyframe_info_3d.txt   建图轨迹 —— 后端运行时直接查它算导航点 z (见下文), 不
                                                     经过预处理, 缺了它这份地图不满足目录结构, 不会出现在列表里
<map-data-dir>/<name>/2d_map/map_2d.pgm(+.yaml)     handbot slam 自带的原始 2D 占据栅格图。只用来判定目录
                                                     结构完整、给下面流水线当只读的沿用来源——localization.service
                                                     等第三方组件直接认这个固定路径, 流水线绝不写回它
        │
        │  map_pipeline/generate_map_assets.py (只读 2d_map/map_2d.pgm, 不修改)
        ▼
web_assets/map/<name>/
    map_2d.pgm(+.yaml)      流水线重新生成的 2D 占据栅格图, 全局规划 (global_planner.py)
                            实际读的是这一份, 不是 map-data-dir 下那份原始图
    topview.png             上面这份 2D 占据栅格图转成的展示用 PNG（灰度逐像素一致；
                            额外把建图轨迹周围 0.25m 内**本来就是 free** 的格子染成
                            淡蓝灰，纯展示，见下面「topview.png 上的建图轨迹带」）
    pointcloud.bin          降采样点云 (自定义 PCW1 格式), 供前端 3D 预览
    pointcloud_meta.json    点数/降采样体素/分片清单等
    topview_meta.json       坐标元数据(世界边界 + 2D 栅格图的分辨率/像素尺寸);
                            这个文件是否存在就是 MapStatus.READY 的判定依据
```

地图目录名就是地图名，四个源文件齐了才会出现在地图列表里（少任何一个都视为不存在）。删除地图会把 `map-data-dir/<name>/` 下的原始数据和 `web_assets/map/<name>/` 下自己生成的预处理产物一起删掉，不可恢复。

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
# 1. 把地图数据放进 map-data-dir (默认 /home/lisi/Documents/map-data, 可用
#    NAVIBOT_MAP_DATA_DIR 覆盖), 目录名就是地图名, 要有下面 4 个文件才会出现
#    在网页"地图管理"的列表里:
#   map-data-dir/myroom/3d_map/dense_cloud_map.pcd
#   map-data-dir/myroom/3d_map/keyframe_info_3d.txt
#   map-data-dir/myroom/2d_map/map_2d.pgm
#   map-data-dir/myroom/2d_map/map_2d.yaml

# 2. 预处理 (也可以在网页的"地图管理"里点按钮触发)
mamba run -n ros_host python map_pipeline/generate_map_assets.py \
    --input /path/to/myroom/3d_map/dense_cloud_map.pcd \
    --outdir web_assets/map/myroom

# 3. 后端 (需要 roscore 已在跑)
./backend/run.sh

# 4. 前端 (开发模式, vite dev server 在 5173)
cd frontend && npm install && npm run dev
```

开发模式下 vite 有 proxy，把 `/api` `/map` `/ws` 转到 `localhost:8000`（见
`frontend/vite.config.ts`），所以前端代码里全用**相对路径**、不需要 `.env.local`，
也没有 CORS。要连别的机器（比如前端跑本地、后端跑板子）才需要 `.env.local` 里的
`VITE_BACKEND_HTTP` / `VITE_BACKEND_WS` 覆盖。

## 部署：后端自己 host 前端 + systemd

板子上不用再跑 nginx 或 vite —— 后端直接把 `frontend/dist` 挂在 `/` 上，前后端同源：

```bash
cd frontend && npm ci && npm run build      # 产出 frontend/dist
sudo cp deploy/navibot.service /etc/systemd/system/
sudo cp deploy/navibot.env     /etc/default/navibot   # 按机器改路径/用户
sudo systemctl daemon-reload && sudo systemctl enable --now navibot
journalctl -u navibot -f
```

| | |
|---|---|
| `deploy/navibot.service` | unit 文件。`User` / `WorkingDirectory` 按"仓库在 `/home/cat/navibot`、用户是 `cat`"写的，换机器要改 |
| `deploy/navibot.env` | 装到 `/etc/default/navibot`，端口和各种 `NAVIBOT_*` 路径都在这儿调，不用动 unit |

`ExecStart` 显式起一个 shell 先 `source /opt/ros/noetic/setup.bash` 再 `exec uvicorn`
（systemd 不会 source bash 脚本）。用 `exec` 是为了让 uvicorn 直接接管 PID，
`systemctl stop` 的信号才打得到它身上，不会把进程留下。

路由规则（`main.py` 末尾的 `_SpaStaticFiles`）：`/api` `/map` `/ws` `/assets` 开头的
按真实文件/接口处理，**找不到就如实 404**；其余路径一律回落到 `index.html`，让前端
的 React Router 接管（`/maps`、`/mapping/xxx` 这些路径在磁盘上没有对应文件，直接输
地址或刷新页面时必须回落）。`assets/` 不回落是因为缺一个 js chunk 时回落成 HTML，
浏览器只会报个 MIME 类型错误，把"这个 chunk 没构建出来"这个真实原因盖掉了。

没有 `frontend/dist` 时不挂载，只提供接口，日志里会说明——只跑后端做开发不受影响。

> **这套部署没有在真实板子上跑过**（开发机上没有 board 访问权限）。`systemd-analyze verify`
> 通过、`ExecStart` 那行命令在开发机上验过能起来，但第一次上机大概率还要调路径和
> `After=` 依赖。

没有实机时，用模拟器验证整条链路（它刻意复刻了真 planner 的到达判据、跳点规则、急停悬停确认行为）：

```bash
mamba run -n ros_host python backend/mock_planner.py --start <X> <Y> <Z> <YAW>
```

> `--start` 的 Z 是 **odom 系机体高度**（地面高程 + 传感器离地高度），不是离地高度本身。默认 `(0,0,0,0)` 在多数地图里都在墙里。

---

## 服务状态管理

系统管理页有一张"服务状态"卡片，管 7 个固定的 systemd 单元（id/显示名/unit 名见
`backend/app/config.py` 的 `SYSTEMD_SERVICES`，不接受任意 unit 名）：

| id | 显示名 | systemd 单元 | 来源 |
|---|---|---|---|
| `lidar` | 激光雷达 | `mid360.service` | — |
| `camera` | 相机 | `camera.service` | — |
| `localization` | 导航定位 | `localization.service` | — |
| `hand_lio` | 实时里程计 | `hand_lio.service` | navi-planner-bringup |
| `planner` | 路线规划 | `navi_planner.service` | navi-planner-bringup |
| `deep_bridge` | 运动控制 · 云深处 | `deep_bridge.service` | navi-planner-bringup |
| `unitree_bridge` | 运动控制 · 宇树 | `unitree_bridge.service` | navi-planner-bringup |

后四个 unit 名以 `navi-planner-bringup/systemd/` 下的实际文件为准。
`deep_bridge` / `unitree_bridge` 是两种底盘各自的 `cmd_vel` 桥接（云深处 Lynx M20
的 UDP/JSON 协议 vs 宇树 Go2 SDK），用哪个取决于装在哪台狗上——**这里不做互斥**，
见下面「服务之间没有关系」。

对应接口：`GET /api/services`（3s 轮询查状态）、`POST /api/services/{id}/start`、
`POST /api/services/{id}/stop`（`service_manager.py`）。查状态用 `systemctl show`，
不需要特权；启动/停止需要特权，默认用 `sudo -n systemctl ...`（`-n` 非交互，没配
免密的话直接报错而不是卡住等密码），可用 `NAVIBOT_SYSTEMCTL_SUDO_CMD` 覆盖。

**这意味着部署到机器上时要单独配一条 sudoers 规则**，只放行跑后端的用户对上表
这些单元（外加建图模式的 4 个）执行 `start`/`stop`（仓库里没有现成的 unit 文件/sudoers 配置，这步要在
机器上手动做），例如：

```
# /etc/sudoers.d/navibot-services
<backend-user> ALL=(root) NOPASSWD: /bin/systemctl start mid360.service, \
    /bin/systemctl stop mid360.service, \
    /bin/systemctl start camera.service, /bin/systemctl stop camera.service, \
    /bin/systemctl start localization.service, /bin/systemctl stop localization.service, \
    /bin/systemctl start hand_lio.service, /bin/systemctl stop hand_lio.service, \
    /bin/systemctl start navi_planner.service, /bin/systemctl stop navi_planner.service, \
    /bin/systemctl start deep_bridge.service, /bin/systemctl stop deep_bridge.service, \
    /bin/systemctl start unitree_bridge.service, /bin/systemctl stop unitree_bridge.service, \
    /bin/systemctl start cloud_mapping_small.service, /bin/systemctl stop cloud_mapping_small.service, \
    /bin/systemctl start cloud_mapping_large.service, /bin/systemctl stop cloud_mapping_large.service, \
    /bin/systemctl start color_mapping_small.service, /bin/systemctl stop color_mapping_small.service, \
    /bin/systemctl start color_mapping_large.service, /bin/systemctl stop color_mapping_large.service
```

没配这条规则时，点"启动"/"停止"会在页面上收到 500 和 `sudo` 的报错文本（比如
`sudo: a password is required`），不是静默失败。

### 服务之间没有关系，每个服务单独管理

**启动一个服务就只启动它自己，不自动带依赖；停止就只停它自己，不检查"还有没有
别的服务在用它"。** 这是有意为之，也跟部署侧一致——`navi-planner-bringup/systemd/`
下那几个 unit 文件彼此没有任何 `Requires=`/`After=`，`navi_planner.service` 的
`Description` 里直接写着 "hand-lio, unitree_bridge started separately"。

以前这里有一整套应用层的依赖解析，现在**整套删掉了**，别再加回来：

| 删掉的东西 | 原来干什么的 |
|---|---|
| `config.SERVICE_DEPENDENCIES` | 声明 `localization`→`mid360`、`navi_planner`→`localization` |
| `config.SERVICE_COSTART` | 启动「导航定位」时顺带带起 `hand_lio.service` |
| `config.MAPPING_MODE_DEPENDENCIES` | 开始建图时自动先启 `mid360` / `camera` |
| `service_manager.start_with_dependencies` | 递归确认/启动依赖 |
| `service_manager.find_blocking_dependents` | 停止前查有没有服务还依赖它，有就拒绝 |
| `service_manager.ServiceDependencyError` | 上面那条拒绝对应的 400 |

随之而来的三个变化：

1. **`hand_lio.service` 现在是独立条目**（上表 `hand_lio`），自己启停。以前它没有
   自己的条目，只能跟着「导航定位」被带起来；关系拆掉之后不给它独立入口的话就
   完全没法管了。
2. **开始建图不再自动启雷达/相机**，要用户自己先在系统管理页打开。没打开建图
   服务照样能起来，只是收不到数据、建图页上看不到点云。
3. **停止底层服务后端不再拦**。唯一的防呆是前端停止确认弹窗里那句提示（会说明
   停的是哪个 unit、用到它的功能会不可用、而且不会自动重启）。

**这些都没有在真实机器上验证过**，见「已知缺口」。

---

## 参数配置页

侧边栏「参数配置」(`/params`)，按服务分别改它的 ROS 参数 yaml。schema 在
`backend/app/config.py` 的 `SERVICE_PARAM_SCHEMAS`，目前接了两个服务。

### `deep_bridge`

| 键 | 类型 | 说明 |
|---|---|---|
| `use_dtls` | bool | 是否启用 DTLS 加密。指南说本体默认开，但实测这台是关的，开了反而握不上手 |
| `usage_mode` | enum | `0` 常规模式（归一化轴指令）/ `1` 导航模式（真实轴指令） |
| `max_vx` | float | 最大前后速度 [m/s]，0 ~ 2.0 |
| `max_vy` | float | 最大左右速度 [m/s]，0 ~ 1.0 |
| `max_vyaw` | float | 最大偏航角速度 [rad/s]，0 ~ 2.0 |
| `gait_on_start` | enum | `4097` 标准-基础 / `4099` 标准-楼梯 / `12290` 敏捷-平地 / `12291` 敏捷-楼梯 |

float 的上下界取自厂商《各个步态的有效速度范围》表的区间上界。

### `hand_lio`

| 键 | 类型 | 说明 |
|---|---|---|
| `blind` | float | 盲区半径 [m]，0 ~ 2.0。靠近雷达中心这个距离以内的点直接丢弃 |
| `lidar_R_body` | mat3 | 雷达→机体的旋转，行优先 3x3。保存时校验它确实是旋转矩阵 |
| `lidar_t_body` | vec3 | 配套的平移 [x, y, z]，单位 m |
| `enable_virtual_obstacles` | bool | 是否把 navibot 发的人工禁行区采样点云混进输出点云 |

`lidar_R_body` / `lidar_t_body` 是手持设备绑在狗背上的**机械安装关系，装好后必须
自己标定**——默认还是占位的单位阵 + 零向量，那样 `/hand_lio/odom_vehicle` 给出的是
雷达的位姿而不是机体中心的。雷达斜着装（比如前倾 15°）时倾角就体现在 `lidar_R_body`
里，见 `lidar_tilt_impact.md` 第 3 节。**这两个外参 `tools/calibrate_cmd_vel.py` 能
顺带标出来**（要带 `--spin-turns`），怎么标、有什么原理上的天花板见「/cmd_vel 标定 ›
顺带能标 hand_lio 的两个外参」。页面上矩阵是 3×3 九宫格输入框，下面实时显示
等效的 `roll / pitch / yaw`（只读，9 个数字直接看是看不出转了多少度的）。

`blind` 的硬件本身只有 0.1~0.2m，多出来的是支架/外壳自遮挡；同款硬件的
Elevator-LIO 实测用的是 0.8。关掉 `enable_virtual_obstacles` 之后，地图上圈的禁行区
只在全局规划里生效，局部规划器不知道它们的存在。

### `planner`（SCAN-Planner）

**这个不是 yaml，是 roslaunch XML**（`SCAN-Planner/.../launch/advanced_param.xml`），
schema 里 `format: "roslaunch"`。键就是 `name` 属性的完整值。

| 键 | 类型 | 说明 |
|---|---|---|
| `max_vel` | float | 最大速度 [m/s]，0 ~ 2.0 |
| `max_acc` | float | 最大加速度 [m/s²]，0 ~ 5.0 |
| `grid_map/double_cylinder_radius` | float | 碰撞半径 [m]，0 ~ 1.0 |
| `grid_map/double_cylinder_offset` | float | 碰撞圆柱前后偏移 [m]，0 ~ 1.0 |
| `closed_loop_controller/max_vy` | float | 闭环控制器侧向限速 [m/s]，0 ~ 1.0 |
| `closed_loop_controller/max_vyaw` | float | 闭环控制器偏航限速 [rad/s]，0 ~ 2.0 |

几条值得记住的联动：

- **`max_vel` 在 launch 里被引用三处** —— `manager/max_vel`、`optimization/max_vel`、
  以及 `closed_loop_controller/max_vx`（闭环控制器的前向限速跟着它走），改一个影响三处。
- **碰撞模型是前后两个圆柱**（`grid_map.h` 的 `getInflateOccupancy`：圆心在
  `pos ± offset·heading`，半径 `radius`），所以机身包络长约 `2×(offset+radius)`、
  宽约 `2×radius` —— 默认 0.10/0.20 对应 0.6m×0.4m。`radius` 同时也是
  `rebuildInflationOffsets` 用的障碍物膨胀半径。
- **`closed_loop_controller/max_vy` 默认 0.35 跟底盘步态死区打架**（带
  `warn_on_change` 标记）：云深处 0x1001 标准-基础步态的 Y 轴有效区间是
  `[-1.0,-0.35]∪[0.35,1.0]`，0.35 恰好压在下界上，侧向指令几乎全落进死区被吃掉。
  要让狗真的会横移就得调到 0.35 以上。
- `closed_loop_controller/max_vyaw` 跟 `deep_bridge` 的 `max_vyaw` 是**两道独立的闸**
  （控制器自己的限速 vs 下发给底盘前的安全限速），取两者更小的才是实际生效值。

接口：`GET /api/services/{id}/params`（schema + 当前值）、
`PUT /api/services/{id}/params`（只传要改的键）、
`POST /api/services/{id}/restart`。`GET /api/services` 的每条多了个
`configurable` 字段，前端据此决定哪些服务能点进去。

### 写回是按行原地替换，不是 yaml 重新序列化

`deep_bridge.yaml` 里那些注释（协议出处、厂商的步态速度范围表、"实测这台本体加密
是关掉的"这类坑）信息量比配置本身还大，用 PyYAML 读出来再 dump 回去会**全部冲掉**。
所以 `service_params.py` 只做**精确替换 value 那一段**，行内注释/缩进/空行/其它
所有内容原样不动。支持两种写法，因为实际的 yaml 两种都有：

```yaml
# (a) 一行式
enable_virtual_obstacles: true   # 行内注释

# (b) 跨行式 —— hand_lio.yaml 的 blind / virtual_obstacle_* 都是这种
blind:
  0.35 # 盲区半径 [m]，靠近雷达中心的点直接丢弃
  # 后面还能接着写好几行纯注释
```

(b) 只认第一个"缩进且有实际内容"的行当值；中间的空行和纯注释行跳过；一旦遇到
不缩进的行就判定这个键没有标量值（说明它是个 map/list 或者空值），不乱猜。

列表型的键（`vec3` / `mat3`）另走一套：定位 `key: [`，把 `[` 到 `]` 之间的内容整段
取出来按逗号切，方括号可以跨行。写回时整段重新渲染 —— 3×3 矩阵按原样铺成三行、
续行缩进对齐到 `[`，所以**值没变的时候写回去跟原文逐字节相同**（有回归测试盯着这条）：

```yaml
# prettier-ignore
lidar_R_body: [1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0]
lidar_t_body: [0.0, 0.0, 0.0]
```

一个键占多行意味着替换前后行数可能变，所以写的时候是**从文件末尾往前应用**的，
不然改完前面的会把后面的行号顶偏。

`mat3` 如果 schema 里标了 `rotation`，保存前还会校验它确实是旋转矩阵
（`R·Rᵀ` 偏离单位阵 ≤ 1e-3 且 `det ≈ +1`）——随手填 9 个数很容易填出一个不正交的
矩阵，那样算出来的位姿是歪的而且**不会有任何报错**。反射矩阵（`det = -1`）也一并挡掉。

roslaunch XML 同一套路子，改的是同一个标签里的 `value=` 或 `default=`：

```xml
<arg   name="max_vel"                         default="0.75"/>
<param name="grid_map/double_cylinder_radius" value="0.20" />
```

`name` 是**精确匹配**的，所以 `max_vel` 不会误伤同一个文件里的
`<param name="manager/max_vel" value="$(arg max_vel)"/>` —— 那是引用不是定义。
值是 `$(arg ...)` / `$(eval ...)` 这类替换表达式时，读出来当"读不出当前值"，
写则**直接拒绝**：那个位置存的是一条引用，拿字面量盖掉会把 launch 里原本的联动
关系悄悄拆掉。

实测：`deep_bridge.yaml` 90 行改 6 个键 diff 只有 6 行、`hand_lio.yaml` 84 行改
2 个键 diff 只有 2 行（改跨行矩阵 + 平移则是 4 行，原值写回**逐字节相同**）、
`advanced_param.xml` 113 行改 6 个键 diff 只有 6 行，改完
`yaml.safe_load` / `ElementTree.parse` 都仍能正常解析，三处 `$(arg max_vel)` 引用
一个都没被误伤。

代价是这个解析器**很窄**——只处理顶层键的标量值，不处理嵌套结构。schema 里声明的
键在文件里找不到时直接报错而不是追加一行（追加多半是缩进/命名空间写错了，静默追加
只会得到一个永远不生效的配置）。写的时候先全文找齐所有行号、全部校验通过才动文件，
然后写临时文件 + `os.replace` 原子替换（并保留原文件权限位），不会留下改了一半的配置。

### 配置文件路径必须按环境覆盖

**这套要在多台板子上跑，每台的工作空间路径都可能不一样**，所以路径是环境变量，
每个服务各有各的（schema 里的 `env_var` 字段，接口会一起返回，页面据此提示用户该设
哪个，不写死）：

```
NAVIBOT_DEEP_BRIDGE_CONFIG=/home/cat/ros1_ws/src/deep_bridge/config/deep_bridge.yaml
NAVIBOT_HAND_LIO_CONFIG=/home/cat/ros1_ws/src/hand-lio/config/hand_lio.yaml
NAVIBOT_NAVI_PLANNER_LAUNCH=/home/cat/ros1_ws/src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml
```

默认值是开发机上的路径。**路径没配对不算接口失败**——`GET` 照样返回 200，schema
照给，`file_error` 里带上原因，页面把"文件在哪、为什么读不到"显示出来。这个错在
多板子环境里是常态不是意外，返回 500 只会让人不知道该去改什么。

### 改完必须重启服务

参数 yaml 是 roslaunch **启动时一次性加载**的，服务跑着的时候改文件不会生效。所以
保存成功后会弹窗问"现在重启服务吗"，页面上也常驻一条提示。

重启在后端做成 **stop + start 两步**，不是 `systemctl restart`——部署时给的 sudoers
规则只放行了 `start`/`stop`（见「服务状态管理」），用 `restart` 会因为不在允许列表
里被 `sudo` 拒掉，还得让每台板子都去改 sudoers。

### 一个没覆盖的耦合

`gait_on_start` 带 `warn_on_change` 标记，改它时页面会额外弹一条警示：
**换步态就必须同步改 yaml 里的 `full_scale_vx/vy/vyaw`**（满量程是按步态给的），
而那三个键不在这个页面里，要手工改配置文件。只在 `usage_mode=0`（常规模式）时
才用得到满量程，导航模式不受影响。

---

## 建图（新建地图）

地图管理页的"新建地图"是一次性的建图会话：选模式 → 填地图名 → 后端启动对应
systemd 服务 → 跳到建图页实时看点云 → 取消（丢弃）或保存。全局同时只有一个
建图会话，逻辑在 `backend/app/mapping_manager.py`（状态机模式仿
`route_manager.py`：`idle → running → (saving) → done/error`）。

4 个互斥的建图模式（`backend/app/config.py` 的 `MAPPING_MODES`）：

| id | 显示名 | systemd 单元 |
|---|---|---|
| `cloud_small` | 点云建图 · 室内小尺度 | `cloud_mapping_small.service`（面积 < 5000 ㎡） |
| `cloud_large` | 点云建图 · 室外大尺度 | `cloud_mapping_large.service`（面积 ≥ 5000 ㎡） |
| `color_small` | 彩色点云建图 · 室内小尺度 | `color_mapping_small.service`（面积 < 5000 ㎡） |
| `color_large` | 彩色点云建图 · 室外大尺度 | `color_mapping_large.service`（面积 ≥ 5000 ㎡） |

接口：`GET /api/mapping/modes`、`GET /api/mapping/status`、
`POST /api/mapping/start`（body `{mode_id, map_name}`）、`POST /api/mapping/cancel`、
`POST /api/mapping/save`；`/ws/mapping` 推 `mapping_status`/`mapping_pose`/
`mapping_surround_cloud`/`mapping_surf_cloud` 四种消息（新连接补发最新一份快照，
理由跟 `/ws/nav` 给 `surf_cloud` 补发一样）。启停服务复用「服务状态管理」那节
的 `sudo -n systemctl` 机制（`service_manager.py` 里的 `systemctl_status`/
`systemctl_action` 被两边共用），sudoers 规则要把上表 4 个单元也加进去（见上面
的示例）。

建图页看的三个数据源：

- 位姿：`/tf`（`MAPPING_TF_MAP_FRAME`→`MAPPING_TF_BODY_FRAME`，默认
  `map`→`livox_frame`）——建图模式下 SCAN-Planner 不跑，没有
  `/hand_lio/odom_vehicle`，只能查 TF。帧名最初是从
  `HandBot-S1-view/ros1.rviz` 里保存的 TF 树反推的（`latest_lidar`），后来
  对着实际跑起来的 `cloud_mapping_small.service` 直接 `rostopic echo /tf`
  核对过，发现纯点云（无相机）建图模式下 `/tf` 只广播 `map -> livox_frame`
  这一条，没有 `latest_lidar`——推测 `latest_lidar` 是彩色建图模式才会挂出来
  的额外相机锚点帧，rviz 那份配置大概率是彩色建图/带相机场景下截的。彩色
  建图模式（`color_mapping_small`/`color_mapping_large`）下这个默认值有没有
  问题还没现场核对过（见「已知缺口」）。
- `/surround_map_cloud`：建图模式下是"当前位姿附近的局部地图点云"，随关键帧
  更新（见 `hand-lio/hand-topic.csv`），是建图页真正在看的主体内容。
- `/surf_cloud_in_map`（`SURF_CLOUD_TOPIC`）：建图页当前这一帧扫描位置的高亮
  提示，跟导航页/地图预览页"实时点云"勾选框订阅的是**同一个话题**——以前这
  两个页面故意分开订阅两个不同话题（地图预览页用未降采样的
  `/hand_lio/clouds_lidar`，展示细节更好），现在统一合并成这一个话题/常量。
  两边各自还是独立的 `rospy.Subscriber`（开关生命周期不一样：地图预览页是
  勾选框，建图页是整页一次性开关，见 `ros_bridge.set_mapping_enabled` 的
  说明），只是不再各自配一份话题名（见 `config.py` 里 `SURF_CLOUD_TOPIC`
  的说明）。

保存（`POST /api/mapping/save`）立即返回 `saving`，真正的工作在后台线程里跑：

1. 执行 `config.SAVE_MAP_SCRIPT`（默认 `/home/cat/start_save_map.bash`，**只在
   板子上有，这个开发机上不存在，没法本地验证**）
2. 成功后把它的产出目录 `config.SAVE_MAP_DIR`（复用已有的
   `HANDBOT_SLAM_MAP_DIR` 推出父目录，即 `.../save_map/` 整个目录，不只是
   `3d_map` 子目录）`mv` 到 `map-data-dir/<name>/` 下
3. 之后这张图会以 `not_processed` 状态自然出现在地图列表里，跟手动把地图数据
   放进 `MAP_DATA_DIR` 是同一条路径——用已有的"预处理"按钮走完剩下的流程

第 2 步**假定** `start_save_map.bash` 产出的目录结构（`3d_map/{dense_cloud_map.pcd,
keyframe_info_3d.txt}` + `2d_map/{map_2d.pgm,map_2d.yaml}`）跟
`map_registry._is_valid_map_dir` 要求的完全一致——这个假设没有拿到脚本本身核对
过，是从 `HANDBOT_SLAM_MAP_DIR` 已有注释"留着给后面'新建地图'功能用"和文件名
常量正好对得上推断的。保存失败（脚本非 0 退出/超时/目标目录冲突）时状态转
`error`，**故意不停服务、不清理任何东西**——用户能看错误重试保存，不会因为
这一步失败就把建图进度也搭进去；只有保存成功才会停止建图服务。

### 激活地图 与 localization.service 共用的固定路径

`config.SAVE_MAP_DIR`（`/home/cat/handbot_slam/catkin_ws_grslam/save_map`）不只是
建图保存时的临时输出目录，`localization.service` 启动时也**只会读这一个固定
路径**，不接受传参指定用哪张地图——所以"激活哪张地图"实际上是"这个路径当前
指向哪张地图"。因此 `MapRegistry.activate_map`（`map_registry.py`）现在会：

1. 先查 `localization.service` 的状态（复用 `service_manager.systemctl_status`），
   不是 `inactive`/`failed` 就拒绝激活，报错提示"需要先在系统管理页停止该服务"——
   它可能正打开着 `save_map/` 下的文件，这时候把路径指向别的地图是不安全的。
2. 把 `config.SAVE_MAP_DIR` 建成一个指向 `map-data-dir/<name>/` 的**软链接**
   （`map_registry.clear_localization_link`）：已经是软链接就摘掉重建（不删任何
   地图数据）；已经是真实目录/文件（比如建图保存失败留下的残留）就**直接删掉，
   打一条 warning 日志**，不拒绝、不需要人工确认——建图服务自己
   `start_save_map.bash` 保存时本来就会整个覆写这个路径（见
   `mapping_manager._run_save`），这里跟它保持同一个尺度。真正的删除动作走
   `map_registry._rm_rf`（`sudo -n rm -rf`，见下面的特权说明），不是 Python
   自己 `unlink`/`shutil.rmtree`——建图服务的 systemd 单元是 root 起的，这个
   路径下产出的目录/文件是 root 所有，`shutil.rmtree` 递归删 root 建的子目录
   很容易半路 `PermissionError`。

`deactivate_map`/`delete_map`（删除的正好是激活地图时）也会顺带摘掉这个软链接，
让"没有激活地图"这个状态和磁盘上 `localization.service` 实际会读到的内容保持
一致。`mapping_manager.start`（见上面「建图」一节）复用的是同一个
`clear_localization_link`——开始新建图前会先清空可能残留的旧激活软链接/残留
目录，不然建图服务会把数据写进当前激活地图的目录里，或者跟残留目录混在一起；
同时会调 `MapRegistry.clear_active()` 把 `map_registry` 里"当前激活地图"的记账
也清掉——`clear_localization_link` 只管磁盘上那个软链接，不知道 `navibot`
自己在内存/`_active_map.json` 里记了哪张图是激活的，这两处状态本来是分开维护
的，不特意同步一次的话，`GET /api/maps` 会在整个建图会话期间一直显示一张其实
已经不再激活的旧地图（因为它的软链接已经被上面这行摘掉了）。

`mapping_manager.start` 清空前会先记一笔"开始建图前激活的是哪张图"
（`MappingManager._prev_active_map`），建图会话结束时尽力把它恢复回去
（`_restore_previous_active_map`，内部就是再调一次 `MapRegistry.activate_map`）：

- **取消（放弃）建图**、**保存成功**（状态转 `done`）都会恢复。
- **保存失败**（状态转 `error`）**不会**恢复——`_run_save` 的 `error` 分支故意
  不清理 `config.SAVE_MAP_DIR`（留着现场给用户重试），这时候恢复会撞上"目标
  已存在"；等用户放弃、真正调用取消时再恢复。
- 启动服务本身失败（`systemctl_action` 抛错，典型是 sudoers 没配好）
  也会恢复——这种情况下建图会话根本没有真正开始过。
- 恢复动作失败（比如那张图这期间被删了、`localization.service` 这期间被手动
  启动了）只记日志，不会让取消/保存这个动作本身也跟着报错——用户可以去地图
  预览页手动重新激活。

**部署时需要额外配一条 `rm` 的 sudoers 规则**（跟「服务状态管理」那节的
`systemctl` 规则是分开的两条），默认命令是 `sudo -n rm -rf`（`NAVIBOT_RM_SUDO_CMD`
可覆盖），只放行对 `config.SAVE_MAP_DIR` 这一个固定路径执行，例如：

```
# /etc/sudoers.d/navibot-mapping
<backend-user> ALL=(root) NOPASSWD: /bin/rm -rf /home/cat/handbot_slam/catkin_ws_grslam/save_map
```

没配这条规则时，开始建图（如果这个路径上有残留的软链接/真实目录）或激活地图
会在页面上收到 500 和 `rm`/`sudo` 的报错文本，不是静默失败或删不干净还继续跑。

**这一整块（激活时的状态检查、软链接创建/摘除、`rm` 特权命令）都没有在真实
机器上验证过**，见下面「已知缺口」。

---

## 巡检路线

在某张地图的 2D 栅格图上摆一串导航点、存下来、以后可以改。**执行链路还没做** ——
见下面那条大字。

存储跟地图列表一样不落 SQLite：一条路线一个 json，平铺在 `data/routes/` 下
（`NAVIBOT_ROUTE_DATA_DIR` 可覆盖），实现见 `backend/app/route_store.py`。文件名
就是路线 id（uuid4 的 32 位十六进制），目录里出现的坏文件/名字不对的文件会被跳过
并打一条 warning，不会让列表接口整个失败。

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/routes` | 全部路线，按更新时间倒序，**带完整的 points** |
| POST | `/api/routes` | 新建一条**空**路线（没有导航点），点在编辑页摆 |
| GET | `/api/routes/{id}` | 单条 |
| PUT | `/api/routes/{id}` | 整条替换（不是字段级 patch） |
| DELETE | `/api/routes/{id}` | 删除 |

注意跟 **`/api/route`（单数）** 区分：那个是"把一串途经点立刻下发给机器狗"
（navi_mode=2，`route_manager.py`），不落盘、不带名字；`/api/routes`（复数）是存储型
CRUD，完全不碰 ROS。两者现在**没有任何连接**。

几条定死的规则：

1. **`map_name` 创建后不可修改。** 导航点存的是那张图坐标系下的**世界坐标**（跟
   `Waypoint` 一样是 x/y/yaw/z_offset，不是像素也不是百分比），换一张图会让所有点
   静默指到错误的位置。`UpdateRouteRequest` 里干脆没有这个字段，请求里塞了也会被忽略。
2. **只有预处理完成（READY）且有 2D 栅格图的地图能用来建路线**，否则创建时 400——
   路线就是在 `topview.png` 上摆点，没有 `topview2d` 的话编辑页根本画不出底图。
3. **更新时不再校验关联地图还在不在。** 地图被删了之后这条路线的点确实没了参照，但
   那种时候用户最需要的恰恰是还能进去把它改名/删掉，而不是连打开都打不开。列表页会
   给这类路线标一个"地图已删除"，编辑页会说明为什么画不出图。
4. **导航点的 `id` 由后端兜底。** 前端新加的点带的是临时 id，空的/重复的都会被后端
   换成自己生成的（`_normalize_points`）——重复的 key 在 React 里会让选中态和输入框
   串到别的行上。

> **`mode`（巡检方式）、`RoutePoint.action`/`stay`（到达动作/停留）、`schedule`
> （定时计划）这四样只是存下来的用户配置，现在没有任何代码会读它们。** 后端把它们
> 当自由文本原样存取，不维护合法值列表——因为执行链路还没做，定一份枚举也无从验证
> 对不对。候选项写在前端的 `frontend/src/data/routeOptions.ts` 里。真做执行的时候
> 这些必须换成后端（以及 SCAN-Planner）认识的枚举，不能沿用现在这份中文字符串。
>
> 同样的道理，前端**不会**拿 `schedule` 去推算"下次执行时间"之类的展示——没有调度器
> 在跑，那种推算是凭空编造。列表页那一列显示的是用户配置了什么，不是承诺会发生什么。

## 地图编辑（人工修正误判区域）

在 2D 栅格图上圈多边形，补救 `detect_structure` 的误判：

- **可通行区**（绿）：把被误判成障碍的地方改回能走
- **禁行区**（红）：把被误判成可通行的地方改成不能走

入口在地图预览页右侧「显示面板 → 地图编辑」，**只在 2D 栅格图上可用**（多边形存的是世界坐标，
3D 点云里没有对应的作图平面，切到 3D 时整节置灰并丢弃没画完的草稿）。左键加顶点、右键撤销，
至少 3 点才能闭合。区域列表可以逐个停用/删除。

> 这个入口被 `073d5e8` 连同 `MapDetailPanel` 一起回退过一次，`7486409` 之后又加了回来——这次
> 直接写在 `MapPreviewPage` 自己的内联「显示面板」里，不再依赖那个外部面板组件。

| 方法 | 路径 |
|---|---|
| GET | `/api/maps/{name}/edits` |
| POST | `/api/maps/{name}/edits` |
| PATCH | `/api/maps/{name}/edits/{region_id}`（只改 `enabled`/`note`） |
| DELETE | `/api/maps/{name}/edits/{region_id}` |

### 三条定死的设计决定

1. **不烘进 `map_2d.pgm`，存矢量。** `map_registry.start_preprocess` 每次都把 `web_assets/map/<name>/` 整个重生成，烘进去的编辑会被无声抹掉。存在 `data/map_edits/<name>.json`（一张图一个文件，`/data/` 已被 `.gitignore` 排除）。
2. **多边形存世界坐标，不存像素。** 2D 图分辨率按地图跨度自动选（0.05~1.0m/格），点云一变就可能跨档，像素坐标会整体错位。
3. **只有一个应用点。** `global_planner.plan_path` 里 `prob` 算出来之后立刻叠加（`_apply_edit_regions`）——下游的 `blocked` / 膨胀 / `cost_weight` / 贴墙惩罚全部从 `prob` 派生，改一处四处正确。可通行区顺带清掉"未知"的 5 倍代价惩罚，也压过 `--block-unscanned`。

其它规则：**重叠时禁行优先**（与添加顺序无关，偏安全方向）；自相交多边形不拒绝，栅格化用**偶奇规则**；起终点落在人工禁行区里时报的是"在人工标注的禁行区内"，跟"离障碍物太近"分开（否则用户会去地图上找一堵不存在的墙）；删除地图时对应的编辑文件一起删（同名重建的新图坐标系可能完全不同）。

> **只影响全局规划，管不住局部避障。** SCAN-Planner 有它自己的实时 3D 栅格图，我们注入不进去。禁行区的效果是"全局路线不会规划到那里"，不是"狗不会走到那里"。**可通行区更要小心**：它只能修正离线建图的误判，修不了实时传感器看到的东西——如果 planner 的实时图仍然认为那里有障碍，狗到跟前还是过不去，表现为局部规划反复失败。

## 与 SCAN-Planner 的接口（navi_mode=2）

契约的每一条都对着 `scan_replan_fsm.cpp` 核过，细节写在 `backend/app/config.py` 的模块注释里。

| 话题 | 类型 | 方向 |
|---|---|---|
| `/preset_waypoints` | `nav_msgs/Path` | backend → planner，一条 Path = 一整轮任务 |
| `/planning/emergency_stop` | `std_msgs/Empty` | backend → planner，急停并作废当前任务 |
| `/planning/finished` | `scan_planner/PlanFinished` | planner → backend，整轮任务结束一次（`REACHED` 或 `EMERGENCY_STOP`） |
| `/hand_lio/odom_vehicle` | `nav_msgs/Odometry` | → backend，位姿 + `covariance[0]` 定位质量 |

> **只用 `navi_mode=2`。** 曾经还有一条 `navi_mode=3`（`/initial_path`，REFERENCE_PATH）的下发
> 链路（`POST /api/maps/{name}/plan_path` 传 `publish=true` 走它），**整套已经删掉了** —— 实测
> mode 3 对全局路线的贴合度不稳定，狗不一定真的顺着线走，早就改成拿规划出来的拐点当 mode 2 的
> 途经点走 `submitRoute` 下发，前端两处调用长期都传 `publish=false`，是彻底的死代码。
> 一并删掉的还有：`PlanPathRequest.publish`、`PlanPathResponse.published/publish_error`、
> `NavStatus.reference_path_active`、`RosBridge.publish_initial_path`、
> `RouteManager.mark_reference_path_dispatched`、`INITIAL_PATH_TOPIC` /
> `INITIAL_PATH_SUB_WAIT_S`（环境变量 `NAVIBOT_INITIAL_PATH_TOPIC` /
> `NAVIBOT_INITIAL_PATH_WAIT_S` 随之失效）。要找回来看 git 历史。
>
> `plan_path` 现在**只负责算，不负责发**，响应体只剩 `points`。

**几条必须照抄 planner 行为的地方**，抄错任何一条都会静默错位：

1. **途中点的到达判定是 3D 距离 < `waypoint_arrival_radius`（0.3 m）**，靠订阅 odom 自己推进度；最后一个点没有这条提前退出，精度更高的确认来自下面第 4 条的 `/planning/finished`。
2. **新一轮开始时跳过距当前位置 < `kDegenerateDist`（0.05 m）的点**（`planNextWaypoint`），后端下发后立刻做同样的跳过，否则第一个点会一直显示成"没到过"。
3. **`/preset_waypoints` 不 latch 且队列为 1**，没订阅者时发出去被静默丢弃。所以下发前等订阅者连上，等不到就返回 HTTP 503。发布端也刻意**不 latch** —— latch 会让 planner 一重启就自己跑上一轮路线。
4. **`/planning/finished` 整轮只发一次**（到达终点 `REACHED`，或急停悬停确认完退出 `EMERGENCY_STOP`），比距离判据精确得多，收到就直接确认——`REACHED`/`EMERGENCY_STOP` 都可能来自 planner 自己触发的 fail-safe，不一定是用户主动停止。急停期间（`exec_state_==EMERGENCY_STOP`）planner 会忽略新收到的 `/preset_waypoints`，必须等这条消息之后再重发路线。

**路线只发一次。** 重发不是冗余而是有害：planner 收到新 Path 会 `current_wp_ = 0` 整轮重置，跑到一半重发会让狗掉头回起点。

---

## 导航点的 z 是怎么来的

```
下发 z = 该点附近建图轨迹的高度 + Δ + z_offset
```

- **该点附近建图轨迹的高度**：直接查 `keyframe_info_3d.txt`（不经过任何预处理），取点击位置 1m 半径内轨迹点 z 的中位数；半径内没有轨迹经过就返回"未知"——建图轨迹开销只跟轨迹点数（几千到几万个关键帧）成正比，跟地图物理跨度/点云大小完全无关，天然兼容任意大小的地图（详见 `backend/app/path_planner.py`）
- **Δ**：`当前 odom.z − 狗当前位置附近的建图轨迹高度`，**运行时实测**
- **z_offset**：用户微调（SCAN-Planner 的 README 明确写了爬不上楼梯就抬 keypoint 的 z）

**路线里不存绝对 z。** odom 的 z 基准取决于 hand-lio 的 `lidar_t_body` 外参，存了绝对值就会在某天标定之后集体失效，而且失效得很安静 —— 偏 0.3 m 不报错，只是让那个 0.3 m 的到达判据变脆。

**不需要单独估"建图设备的传感器离地高度"这个常数。** 以前（点云版高程面）需要专门从点云里量一个 `delta_sensor_m`；现在这个常数会在"目标点轨迹高度 + Δ"这个式子里跟"建图轨迹 z 基准和运行时 odom z 基准的差异"一起自动抵消——两者都是加在轨迹高度上的固定偏移，做减法（算 Δ）再加回去（算目标点 z）就消掉了，不需要分别估计，也不需要碰点云，推导见 `path_planner.py` 模块注释。

**先不处理楼梯/多层重叠。** 每个 (x, y) 只取轨迹附近的中位数高度，不区分同一位置可能存在的多层可站立面——这是有意简化，复杂的分层逻辑等寻路重做时再上，见下一节里 `map_pipeline/elevation.py` 那套点云方案作为以后的参考。

---

## 一些被数据打回来才定下的规则（现在没有代码路径在用，留作以后处理楼梯/多层重叠的参考）

`map_pipeline/elevation.py` 这套点云逐格高程面提取，因为内存/耗时跟地图物理跨度成正比（见前面"不要假设地面只有一个高度"一节末尾），现在没有任何脚本调用它，导航点 z 改成查建图轨迹（见上一节）。但它对"怎么从带噪声的点云里抠出可站立面"这个问题踩过的坑仍然有参考价值，尤其是以后要处理楼梯/多层重叠时——每条规则都对应一个具体的失败模式，改之前先看清楚它在挡什么：

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

这套方案还带了一次**自检**：如果某些格子存在多个可站立高度（折返楼梯、夹层），写进 `overlaps.json` 并告警，表示当前的单值表示已经不够用了——宁可举手也不要默默画出一张看着正常、实际把上下楼梯压在一起的地图。这也是以后重新实现分层逻辑时要留意的点。

---

## 狗走过的整条轨迹，规划器必须能从头走到尾

`elevation.clear_trajectory` 沿建图轨迹清出一条 free 带（"狗走过这里，这里必定能走"），
`global_planner` 规划前又按 `GLOBAL_PLANNER_INFLATION_RADIUS_M` 把障碍膨胀回来。两边说
的是同一个 0.25m，但只要换算方式或者判据差一点，这条带子就会被吃穿。同一个地方栽过
两次，两次都是"图上看着通、A\* 说不通"，两次都是事后才补上检查。

### 一、换算差一格（`save_map_large_1`）

`clear_trajectory` 是 `int(round(0.25/0.1))` = **2px 方核**，规划器是
`ceil(0.25/0.1)` = **3px 圆核**。0.1m/格的图上带子被清成 5px 宽，正中间离障碍恰好 3px，
全被膨胀吃回去。表现是 `save_map_large_1` 上两个点规划不出路线，而它们在未膨胀的图上
完全连通（free 区是一整块）——最宽的通道净空 0.300m，要求 > 0.300m，**差整整一个像素**，
米数看着一样、日志里什么都看不出来。0.05m/格的图侥幸没事（`round` 和 `ceil` 都得 5）。

现在两边都是 `ceil` + 圆盘（用 `EDT <= R` 表达，跟 `_dilate_bool` 覆盖的格子集合逐格
相同，有对拍）。这给出一条硬保证：**清出来的格子经过膨胀之后一定还在**——任何 occupied
格子离轨迹格的距离都 > R（不然它早被清成 free 了），而膨胀只吃距离 <= R 的。

修之前 3 张 0.1m/格的图分别掉 4 / 36 / 98 个轨迹格，修之后全部 100%。

### 二、带子只剩一格宽（`save_map_small_1`）

上面那条只保证"**每个轨迹格自己**活下来"，没保证带子**宽于一格**。障碍正好落在轨迹两侧
`R+1` px 时，轨迹格离它 `R+1 > R` 活下来，而轨迹格旁边那一格离它只有 `R`，被膨胀吃掉——
于是只剩中心线一格宽。中心线要是斜着走的，就是一串只靠**对角**相连的格子，正好撞上
`_astar` 那条"两个正交邻格都是障碍就不许斜穿"（不然现实里会蹭墙角）。

表现：`save_map_small_1` 上 `(-0.07, -0.02)` → `(-121.71, -172.78)` 规划不出路线。两个点
都**在轨迹上**，free 区按普通 8 邻接算也完全连通（同一个 57906 格的连通块），但按规划器
的规则从起点只够得着 12772 格，整条路最少要斜穿 **8 个夹缝**，5 处全在轨迹上（离轨迹
0.00m，净空 0.32~0.36m）。逐格检查全绿，整条路照样不通。

现在 `clear_trajectory` 清 **`R+1`**：轨迹格的 4 个正交邻格离任何障碍都 > R（否则早被清
掉了），所以也必定存活——带子至少 3 格宽，相邻轨迹格之间不可能只剩对角相触。代价是在
轨迹周围多认一格（0.1m/格的图上 0.1m）是空的，而那是狗实际走过的地方；受影响的格子
上界是全图的 0.07%~1.7%。实测该图轨迹上够不着的格子 2795 → 0，可走格只多了 1024 个。

### 检查

想用眼睛看这条带子：地图预览页的「显示面板 → 地图 → 建图轨迹」开关会把它画出来（琥珀色），
2D 栅格图和 3D 点云都画，数据来自 `GET /api/maps/{name}/trajectory`。没有 `keyframe_info_3d.txt`
的旧地图返回空列表（不是 404），面板上给一句"这张地图没有建图轨迹数据"。

`tools/check_trajectory_clearance.py --all` 验的是**连通性**，不是逐格存活——第二条就是
逐格检查抓不到的。它照抄 `_astar` 的邻接规则，从轨迹第一个格子出发看能不能走遍整条轨迹。
改动任何一边之后跑一遍。**注意这不是改代码就生效的**，得重新预处理一遍地图才会体现在
`map_2d.pgm` 上。

---

## 全局规划优先走建图轨迹

狗当初从哪儿走过来的，那条线就是最可信的"这儿能走"：地面平整、宽度够、没有沟。而且它是
**独立于感知**的证据——`detect_structure` 的机体高度带和 planner 的实时 ESDF 都看不见负障碍，
"贴着走过的路走"能绕开它们看不见的东西。

三个开关（`config.py`）：

| | 默认 | 作用 |
|---|---|---|
| `GLOBAL_PLANNER_OFF_TRAJECTORY_MULTIPLIER` | 2.0 | 轨迹**压过的那些格子**代价 1.0，其余 ×2.0。设 1.0 关掉 |
| `GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION` | **0（已关）** | 轨迹格（+1 圈）**不被膨胀吃掉**。见下面「沟沿」一节 |

**免罚的只有轨迹真正压过的格子，不带半径（exact match）。** 一开始做成"轨迹 0.5m 以内"，
那等于把整条 1m 宽的带子都算成"走过"，A\* 在带子里走哪条线都一样便宜、该贴的地方不贴——
而沟正好在带子边上。注意这只影响**代价**，不限制能走的范围：该能走的地方照样能走，只是走出
轨迹要多付 `off_mult` 倍。

**"偏好"是靠罚别处实现的，不是给轨迹折扣。** `_astar` 的 `_octile` 启发式按权重恒为 1 估，
出现 <1 的格子会让它高估真实代价，A\* 就不保证最优了。

`OFF_MULTIPLIER` 的含义很直白：**绕着走过的路走不超过这个倍数时才绕**。合成图上验过（12×12m
全空地，轨迹走一个 U 形，直线 9m / U 形 27m = 3 倍）：

```
off=2.0  ->  走直线 9.0m，离轨迹最远 4.5m     (3 倍 > 2.0，不值得绕)
off=4.0  ->  跟着 U 走 24.9m，离轨迹最远 0.6m  (值得绕)
```

现有几张图的可走空间本来就只是一条窄带，没有抄近路的余地，所以这几个参数在它们身上看不出
差别——它对**有开阔地的图**才有意义。

### `--map2d-trajectory-clear-radius` 的取值是被量化死的

清掉的格数是 `ceil(r / res) + 1`，而这个数必须 **>= 规划器膨胀半径 R + 1**（理由见
`clear_trajectory` 的 docstring：清 R 只保证每个轨迹格自己活下来，没保证带子宽于一格；
只剩中心线一格宽、又斜着走的话，就是一串只靠对角相连的格子，正好撞上 `_astar` 那条
"两个正交邻格都是障碍就不许斜穿"）。

在 0.1m/px 的图上 `R = ceil(0.25/0.1) = 3px`，所以：

| `clear-radius` | 实际清 | 结果 |
|---|---|---|
| 0.10 / 0.15 / **0.20** | 2~3px | ❌ 不够 |
| **0.21 ~ 0.30** | 4px | ✅ 产出**完全一样** |

**也就是说 0.25 已经是这个分辨率下的最小可用值，往下没有空间。** 实测把它调到 0.20：
`save_map_small_1` 上 3086 个轨迹格里 **2795 个规划器够不着**（`被膨胀吃掉 0 个` 但走不到），
轨迹上随机 12 对起终点只成功 4 对 —— 正好复现 `clear_trajectory` docstring 里记载过的那个
已修 bug，连数字都一样。`house`（0.05m/px）和 `save_map_large_1` 没坏，所以**这个坑只在
0.1m/px 的图上踩得到**，单看一张图会误判成"能调"。

### 去掉了 `--map2d-known-radius`

原来它把"离轨迹超过 radius 的 free 格子"降级成 205（未知），让全局规划器加 5 倍代价、
优先走验证过的地方。去掉的理由是**它没在区分任何东西**：默认 radius 只有 0.25m，于是
几乎所有可走格子都成了未知 —— `save_map_small_1` 上 `free=14930` vs `unknown=226396`，
94% 是未知。统一乘 5 等于没乘。

两个副作用，去掉之后才正常：

- `GLOBAL_PLANNER_OFF_TRAJECTORY_MULTIPLIER`（2.0）**一直是失效的** ——
  `cost_weight = max(cost_weight, off_mult)`，94% 的格子已经是 5.0，`max(5.0, 2.0)` 恒为 5.0。
- `_astar` 的 `_octile` 启发式按权重恒为 1 估，满图 5.0 会让它极度低估真实代价、白扩一大堆节点。

实测去掉之后**路线形状没变**（`save_map_small_1` 路线到最近轨迹点：中位 0.050m / 最大
0.147m，跟去掉之前逐位相同）——因为现有几张图的可走空间本来就是一条窄带，没有抄近路的
余地。`free` 从 14930 涨到 241326，2D 图不再是一片灰。

> `GLOBAL_PLANNER_UNKNOWN_COST_MULTIPLIER` 和 `_cost_weight` 里的未知分支**没有删**：
> 没有建图轨迹的旧图走 `bootstrap_map2d_from_slam`，直接沿用 SLAM 自己那张 `map_2d.pgm`，
> 那里面是有 205 的。只是**这条流水线生成的图现在不会再产生 205**。

### 为什么沟沿的安全余量不能靠关掉这两个开关拿回来

实测反馈：**建图的时候人也会贴着沟沿走**，于是"优先走轨迹"会把狗也带到沟边上。直觉上的
补救是把两个"拆安全余量"的开关关掉——`GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION`（规划器侧）
和 `--map2d-trajectory-clear-radius`（预处理侧，轨迹周围强制标 free）。实测了 2×2：

| `clear_trajectory` | `BEATS_INFLATION` | `save_map_small_1` | `house` |
|---|---|---|---|
| 0.25 | on（原默认） | 12/12 | 12/12 |
| 0.25 | **off** | **12/12** | **12/12** |
| off | on | 1/12 | 8/12 |
| off | off | **0/12** | **0/12** |

（在轨迹上随机取 12 对起终点规划，统计成功数。）

两条结论：

1. **`clear_trajectory` 是承重墙，关不得。** 关掉之后 `save_map_small_1` 上 3086 个轨迹格里
   **1462 个（47%）被膨胀吃掉**，`tools/check_trajectory_clearance.py` 直接不通过，连"狗走过的
   地方"都会被判成"起点离障碍物太近"。
2. **`BEATS_INFLATION` 关掉是免费的，但也基本不产生效果。** 因为
   `clear_trajectory` 已经在预处理阶段把轨迹旁边的障碍抹掉了——实测这张图上轨迹落在膨胀带里的
   点是 **0/1795**，也就是 `BEATS_INFLATION` 根本没东西可顶。默认改成 0 只是去掉一个"必要时会
   压过安全余量"的机制，不是解决方案。

**所以这条路走不通，原因是问题根本不在膨胀上**：`detect_structure` 的判据是"局部地面之上
0.10~0.55m 有没有实体"，沟是负障碍，那个带子里什么都没有 → 判成 free。**沟压根不在地图里**，
膨胀再大也膨胀不出一个不存在的障碍。真正的方向是：

- **把沟标进地图**（`map_pipeline` 侧，`build_elevation` 的区域生长边界 / 相邻格高程突降 /
  地面回波缺失三个信号）——唯一能自动化的根治方向，但 `build_elevation` 从没在这条流水线上
  跑过，要先验它的输出质量；
- **手工沿沟沿画禁行区**（今天就能用，全局 + 局部都生效，见「地图编辑」一节）；
- **让雷达看得见地面**（雷达前倾，见 `lidar_tilt_impact.md` 第 2.1 节：水平安装时机体高度带在
  狗周围 3.18m 内全在盲锥里）——这是让**局部**规划器也能反应的唯一办法。

### 轨迹压过膨胀，但压不过明确的障碍

`free |= dilate(轨迹, 1) & ~blocked`：只顶掉膨胀出来的安全余量（狗的身子实实在在从那儿过去了，
那个余量在那里被实地证伪过），**顶不掉** `detect_structure` 判出的障碍，也顶不掉人工圈的禁行区
——人工画的禁行区是"这里现在不许走"（门关了、地塌了），必须压过"历史上走过"这个事实。

多放一圈是为了这条带子至少 3 格宽：只剩一格且斜着走时会撞上 `_astar` 的"不许斜穿夹缝"，
图上看着通、A\* 说不通（见上面那一节）。

**直接的好处**：`gen_corridor_walls.py --half-width 0.3` 现在能用了。实测 `house` 上按 0.30m
生成走廊墙：

```
轨迹压过膨胀=关  ->  失败: 起点离障碍物太近(膨胀半径 0.25m)
轨迹压过膨胀=开  ->  成功, 26 点 / 15.4m
```

> 但 `gen_corridor_walls.py` 仍然会提醒你余量为负：豁免只保住轨迹那 3 格宽的带子，狗一偏出去
> 就贴到自己立的墙上。要真正的余量还是得 `half_width ≥ 膨胀半径 + 0.15m`。

> **立了走廊墙之后，起终点必须落在走廊里。** 墙的位置经过两次栅格化（工具的 0.05m 距离场 +
> 规划图自己的分辨率），有效位置比标称值模糊 ±1.5 格。实测 `save_map_large_1`（0.1m/格、墙在
> 0.40m）：一个离轨迹 0.345m 的起点，它所在**格子的中心**在 0.377m 处，被判进了禁行区，报
> "起点在人工标注的禁行区内"。挑点时留出 `half_width − 2×分辨率` 的余量。

---

## plan_path 区分"路线预览"和"真实导航"

`POST /api/maps/{name}/plan_path` 的请求体带一个 `purpose` 字段
（`"preview"` / `"navigate"`，默认 `preview`）。**它只影响后端日志详略，不影响规划
结果**——两种情况拿到的是同一份路径。

地图预览页两处调用分别传：

| 调用方 | purpose | 后端打什么 |
|---|---|---|
| 「路线预览」`handleFinishStartGoalPick` | `preview` | 逐点打 `#N x= y= z=` + **每段长度** + 汇总 |
| 「开始导航」`handleStartNav` | `navigate` | 只打一行汇总，注明明细见随后的下发日志 |

`navigate` 之所以不打明细，是因为它紧接着就会调 `submit_route`，那边的
`_log_dispatch` 会把同一串点再打一遍，而且信息更全（还带机器狗当前位姿、每个点的
z 是怎么算出来的、planner 会不会跳过某个点）。两边都打就是同一串点刷两遍。

`_log_dispatch` 现在也逐点打**段长**（离上一个途经点多远，第一个点算离机器狗多远），
跟 `plan_path` 预览日志同一个口径（3D 距离），两边能直接对着看。末尾各有一行
`全长 X, 段长 最小/中位/最大`。

> **航点间距就是 navi_mode=2 的有效前瞻**，也是"平地一冲一冲 / 上下楼梯左右摆动"的
> 直接成因（见上一节），所以这个数比原来只打的"距狗多远"更值得逐点列出来。

顺带把 `plan_path` 里两条**逐点**打的警告（"这张图没有建图轨迹数据"、"算不出位姿
标定 Δ"）改成整条路线各打一次——它们讲的是整条路线共同的前提，每个点重复一遍没有
新信息，一条 300 多个点的路线原来会刷 300 多行 WARNING。

---

## /cmd_vel 标定

`tools/calibrate_cmd_vel.py` —— 量"我发了多少速度"到"机器狗实际走了多少"的传递关系，
x / y / yaw 三个轴分别标。**必须在板子上跑。**

```bash
systemctl stop navi_planner        # 必须！否则两个发布者抢 /cmd_vel
python3 tools/calibrate_cmd_vel.py --axis yaw               # 建议先只跑 yaw
python3 tools/calibrate_cmd_vel.py --axis x --axis y --axis yaw --verify 0.33,0.62
```

标的是**整条链路**（`/cmd_vel` → `deep_bridge` 夹限/满量程换算 → 本体 → 实际运动），
不拆开——中间至少四处可能藏比例，而闭环控制器本来就是拿 `/hand_lio/odom_vehicle`
闭环的，让这两端一致才是目标。真值就用那个 odom。

产出：每轴的**比例 / 死区 / 正反不对称 / 上升时间**，一个 3×3 的**轴间耦合**矩阵，
外加顺带标出来的 **`lidar_t_body` 杆臂**和 **`lidar_R_body` 安装旋转**（见下文）。

### 这套脚本跑过吗

板子上还没有。但**在一个假狗上端到端跑过**：`roscore` + 一个订阅 `/cmd_vel`、按
"死区 + 比例 + 轴间耦合 + 0.22m 杆臂 + 15° 安装倾角 + 4° 安装偏转 + 0.03m/s 横向漂移"
积分并以 50Hz 发 `/hand_lio/odom_vehicle` 的节点，然后拿**真正的脚本**去标它，看能不能
把埋进去的真值还原出来。这一趟抓到两个光看代码看不出来的问题：

1. `--axis yaw` 排在第一位时，转圈那几段 odom 会被 60 秒缓冲挤掉，三个外参估计一起
   **静默消失**（见「几个关键设计」里那条）。
2. 杆臂被横向漂移抬高了 38%，而圆拟合残差毫无异常（见下面 `lidar_t_body` 那节）。
3. 转圈命令原本写死成"最大幅值 × 0.7"，但 yaw 死区会吃掉一大半——实测命令 0.84 只
   转出 0.35 rad/s，而超时是按命令值算的，**余量只剩 4%**。死区再大一点就转不满圈，
   两个圆拟合一起静默退化。改成从前面的 yaw 阶跃里挑"实测最接近 0.5 rad/s"的幅值，
   超时也按实测转速算。
4. 报的杆臂向量在**雷达 yaw 系**而不是机体系，差一个 γ：不转的话报 (0.213, 0.050)，
   转完 (0.217, 0.033)，真值 (0.220, 0.030)。长度不受影响，方向受。
5. 只跑一段转圈、拿直线段的漂移去扣杆臂，误差 7~14cm——因为**转圈和平移是两种步态**，
   蹭的方向和大小都不一样。改成跑两段不同转速联立，误差 0.2cm（见 `lidar_t_body`
   那节）。
6. `window()` 改二分之后多了个"缓冲区必须有序"的前提，而这条链路上时间戳**真会回退**
   （见「前置检查」第 5 条）。

假狗的脚本不在仓库里（一次性的验证工具），但上面这些数字都是它跑出来的。

### 输出长这样

（下面是**真的跑出来的**：上面说的那只假狗 + 真的 `roscore`，命令是
`--axis yaw --axis x --axis y --repeats 1 --hold 3 --verify 'x=0.62' --spin-turns 2`。
埋进去的真值是：死区
0.187/0.189（x）、0.352/0.359（y）、0.471/0.468（yaw），比例 0.91/0.87、0.82/0.85、
0.95/0.93，x→yaw 耦合 0.068，杆臂 (0.22, 0.03)，安装 roll 1.5° / pitch 15° / γ 4°，
外加每步往左蹭 0.03 m/s。）

```
[x]  单位 m/s   厂商下界 0.20   有效 run 12 次
  正向: 比例 0.8987   死区 0.1887   拟合 R²=1.0000  (6 点)
  反向: 比例 0.8647   死区 0.1869   拟合 R²=1.0000  (6 点)
  正反不对称: -3.8%
  上升时间(到稳态 90%)中位: 0.65s

[y]  单位 m/s   厂商下界 0.35   有效 run 21 次
  正向: 比例 0.8154   死区 0.3147   拟合 R²=1.0000  (7 点)
  反向: 比例 0.8358   死区 0.3909   拟合 R²=0.9996  (6 点)
  正反不对称: +2.5%
  上升时间(到稳态 90%)中位: 0.45s

[yaw]  单位 rad/s   厂商下界 0.50   有效 run 18 次
  正向: 比例 0.9487   死区 0.4689   拟合 R²=1.0000  (7 点)
  反向: 比例 0.9287   死区 0.4702   拟合 R²=1.0000  (7 点)
  正反不对称: -2.1%
  上升时间(到稳态 90%)中位: 0.65s

轴间耦合(每 1 单位**实测**主轴运动, 伴随各轴多少; 对角线恒为 1):
        ->x      ->y      ->yaw
  x      1.0000   0.0788   0.0685
  y     -0.0659   1.0000  -0.0000
  yaw   -0.0300   0.2254   1.0000

顺带估的杆臂 lidar_t_body(来自 yaw 原地旋转):
  转速 +0.36 rad/s: 圆半径 0.186 m, 残差 0.001 m, 6945 样本
  转速 +0.69 rad/s: 圆半径 0.199 m, 残差 0.001 m, 3627 样本
  两段联立解: 水平分量 [+0.220, +0.028], 长 0.222 m
  转圈时的横向漂移 [+0.0309, -0.0130] m/s
  (直线段量到的是 [-0.0000, +0.0317] —— 差得多就说明转圈和平移是两种步态)
  ← **不依赖任何关于步态的假设**, 用这个值。

雷达安装倾角(来自原地转圈的姿态圆拟合):
  倾角 15.07°   倾斜方向 178.2°(odom 系)   地面坡度 0.00°
  注意: 这个倾角里**含狗自己的站姿倾角**, odom 原理上分不开 ——
        要拆开得在机体上放水平仪, 或者拿量角器量支架。

雷达安装偏转 γ(绕 z, 来自 x/y 直线段的行进方向):
  由 x 轴估: +4.50°  (标准误 0.41°, 拟合残差 1.00°, 8 点)
  由 y 轴估: +3.77°  (标准误 0.03°, 拟合残差 0.07°, 8 点)
  同时分离出的底盘横向漂移: wx=-0.0000 m/s, wy=+0.0317 m/s
  x 和 y 两组差 0.74° (容差 1.53°): 一致

hand_lio.yaml 的 lidar_R_body 建议值(行优先):
    [ 0.963149, -0.069319,  0.259883]
    [ 0.074294,  0.997193, -0.009354]
    [-0.258505,  0.028317,  0.965595]
  等价 rpy: 1.68°, 14.98°, 4.41°   正交性误差 1.11e-16
  写进配置前先确认上面两条前提: 站姿倾角、底盘漂移。

验证集(不参与拟合的幅值, 看预测准不准):
  x   cmd=+0.620  预测 +0.3876  实测 +0.3879  误差 +0.0003 (0.1%)
  x   cmd=-0.620  预测 -0.3745  实测 -0.3743  误差 +0.0001 (-0.0%)
```

真值 vs 估出来的：倾角 15.07°／15.07°，γ 4.00°／4.39°，杆臂 (0.220, 0.030)m／
(0.220, 0.028)m，转圈漂移 (0.030, −0.015)／(0.031, −0.013)，平移漂移 (0, 0.030)／
(0.000, 0.032)，死区和比例见上面逐行对照。

**除了 y 轴的死区**（真值 0.352/0.359，估成 0.315/0.391）：那是平移时的横向漂移把 y
轴正反两边的表观死区各推了一边——正向 `−b/k = d − w/k`、反向 `d + w/k`，代进去是
0.313 / 0.397，实测 0.315 / 0.391（示例里 0.3147 / 0.3909），量级和方向都对得上。不是拟合坏了，而是**漂移会污染
哪些量**的一个具体例子：它进得了 y 的死区，进不了 x 和 yaw 的。

怎么读：

- **比例 0.898 / 死区 0.189** = 命令 0.189 m/s 以内狗不动，超过之后实际速度 ≈
  `0.898 × (命令 − 0.189)`。命令 0.5 只能得到 0.28 —— 差了快一半，这就是要标它的原因。
  （埋的真值是 0.91 / 0.187。）
- **死区 0.189 vs 厂商下界 0.20** —— 对得上就说明厂商那张表可信，`deep_bridge` 里那条
  "下界数据可靠性存疑"的悬案可以结了；对不上就以实测为准。
- **耦合 x→yaw = 0.069** = 每前进 1m 偏 0.069 rad ≈ 3.9°（埋的真值就是 0.068）。这个
  数直接关系到"狗会不会自己往路边靠"。
- **耦合 yaw→y = 0.225 整个是假象。** 这只狗的 yaw 通道压根没接 y 耦合，这个数是
  杆臂（原地转时雷达在画圆，主项 ≈ 0.22）+ 转圈步态的横向漂移 + 圆弧被直线拟合成弦
  的几何偏差三样凑出来的。**看耦合矩阵时非对角线要先扣掉杆臂那一份**，这就是为什么
  杆臂和耦合要在同一份报告里给。
- **验证集误差 1% 量级**才说明这个线性模型站得住；要是验证点误差比拟合残差大一个
  量级，说明模型不对（可能有饱和或者迟滞），别硬套。

### 几个关键设计

- **不对位置做差分求速度。** 把窗口内的世界系位置转到窗口起始时刻的机体系，再对
  `x_body(t)` / `y_body(t)` 各做最小二乘直线拟合——一次拟合同时拿到主轴速度和横向
  耦合，而且 R² 顺带就是质量门：拟合不好说明那段根本不是匀速（还在加速、或者打滑），
  整段丢掉而不是硬算一个没意义的均值。yaw 先 `unwrap` 再拟合。
- **扫描幅值必须跨过死区，而且下界以上要留足点。** 默认值压在厂商《各步态有效速度
  范围》表 0x1001 那一行的下界两侧（X 0.2 / Y 0.35 / Yaw 0.5）。**换步态这三个数就
  不对了**，用 `--vendor-lower` 覆盖。实测死区可能比厂商下界还高一点，那些落进死区的
  幅值会因为 R² 太低被剔掉，所以下界以上每个方向要留 4~5 个点——点数少于 4 个时报告
  会直接标红提醒。
- **拟合比例时只用下界以上的点。** 死区里的行为本来就不是线性的（厂商表只给了下界，
  下界以内本体怎么处理没有说明，`deep_bridge` 也没做处理），混进去只会把比例带歪。
  小幅值的点原样保留在报告里。
- **验证集**（`--verify`）：这些幅值照样测但不参与拟合，最后报预测误差。没有这一步
  就只是拟合、不算验证。**按轴给**：`--verify 'x=0.33,0.62 y=0.42'`。写成不带轴名的
  `--verify 0.33,0.62` 是"所有轴都用这组"——三个轴量纲完全不同，那多半不是你要的
  （第一版只有这个写法，会把 x 的验证幅值悄悄塞进 yaw 的扫描表、还从 yaw 的拟合里
  排除掉）。`--amplitudes` 同理只接受单轴，给多个轴直接报错而不是静默忽略。
- **正负交替**跑，让狗来回走把位移抵消掉——x 轴 60 次阶跃只需要 4~5m 直线空间。
- **`OdomBuffer` 按 200Hz 设计。** hand-lio 的 odom 是 200Hz，`keep_sec` 默认 60 秒就是
  **12000 个样本**，所以容器和取数方式都不能随便写：用 `deque` + 锁（`list.pop(0)` 是
  O(N) memmove，200Hz 下约 19MB/s，而且会让另一线程正在遍历的下标错位、悄悄漏样本），
  `window()` 里锁只握"拷一份"那一下、二分定位和切片都在锁外做（`estimate_rise_time`
  一个 run 就要调二十几次，全量扫描会压住接收线程）。`keep_sec` 本身从
  `--ramp-duration` / `--hold` 推出来，不写死——写死 60 的话把 `--ramp-duration` 调到
  90 就会静默截断。
- **要留的 odom 段必须当场取走存成数组，不能存时间区间。** `OdomBuffer` 只保留最近
  60 秒，而整轮要跑 20 多分钟。`estimate_lever_arm` 从一开始就存的是 `(t_start,
  t_end)`、等报告阶段才回头 `window()` 取——**取出来是空的**，函数静默返回 `None`，
  报告里那一节直接不打印。默认轴序 `x, y, yaw` 恰好把 yaw 排在最后才侥幸有数据，而且
  也只剩最后 60 秒那一点；`--axis yaw --axis x --axis y` 就完全没有。转圈那几段改成
  `run_spin` 里当场 `append(win)`，跟缓冲长度彻底解耦。
- **轴间耦合按实测主轴速度归一，不是按命令幅值。** 有死区时 `v/u = k(1−d/u)` 会随
  幅值变，按命令归一平均出来是个没意义的数。除以实测主轴速度之后对角线恒为 1，
  非对角可以直接读成"每 1 单位实际主轴运动伴随多少侧向/偏航"。

### 原始数据先落盘，再做分析

采集要 23 分钟，而后面每个估计器都可能因为某组数据的形状抛异常。所以**先把
`cal.runs` 写进 `--out`，打一行"原始数据已存 …, 开始分析"，然后才做分析并追加**；
每个估计器各自 `try`，失败就在 `report["errors"]` 里记一条并在报告里显眼打出来，不
连累别的估计器，更不连累那两百多次阶跃的原始记录。

（早先的写法是把 `analyse` / `estimate_*` 全内联在 `report = {...}` 的字面量里、
`json.dump` 排在后面——任何一个异常，23 分钟的采集一起没。）

### 前置检查（不过就直接退出）

整套跑下来 **23 分钟左右**（`34 个幅值 × 2 方向 × 3 次 = 204 次阶跃`，每次
`--hold 5.0 + --rest 1.5 = 6.5s`，再加两段转圈 ~60s；`--ramp` 再多 1 分钟）。时间几乎全在
阶跃次数上，而幅值表之所以有 34 个，是为了让死区以上每个方向留够 4~5 个拟合点。

所以 **所有能提前发现"这轮注定白跑"的条件都挪到开跑前**：

1. **`/cmd_vel` 上不能有别的发布者。** `closed_loop_controller` 以 100Hz 发同一个话题，
   两个发布者交织 = 数据全废。而且它"收到过 bspline 之后就不再依赖规划器，会继续按
   最后那条轨迹发"（`advanced_param.xml` 的注释），所以得整组停：`systemctl stop
   navi_planner`，别只 kill 规划器节点。脚本靠 master 的 `getSystemState()` 查，
   查到了就报出是哪个节点。
2. **odom 在来、而且定位没失败**（`covariance[0] < 0.99`）。
3. **时钟对得上。** 样本时间取 `msg.header.stamp`（ROS 时间／发布端时钟），而切窗口用
   的是 `time.time()`（本机墙钟）。两者不可比——`use_sim_time` 开着，或者手持设备跟
   板子没对时——所有窗口都会**静默取空**，表现出来是"样本太少"，排查方向完全跑偏。
   开跑前把 `rospy.Time.now()` / `time.time()` / 最新 odom `stamp` 三个钟对一遍。
4. **odom 的静止噪声得低于判静止的阈值。** `wait_stationary` 判的是"0.7s 窗口内离均值
   的**最大**径向偏差"，是个极值统计，对噪声和样本数都敏感：hand-lio 的 odom 是
   **200Hz**，0.7s 就是 140 个样本，高斯噪声下最大径向偏差约 3.1σ，也就是默认的
   `--still-xy 0.02` 要求静止噪声 σ ≲ **6.4mm**。定位系统超过这个数完全可能，而一旦
   超了，每一步都等不到静止 → 每步 `run_step` 都返回 `None` → 跑满二十多分钟一条数据都
   没有。所以先静止测 3 秒，**拿 `wait_stationary` 那个统计量本身**去量（不用正态假设
   去推），超了就报错并给出建议的 `--still-xy` / `--still-yaw`。
   顺带一提，这也是唯一会让总时长失控的东西：`wait_stationary` 每次最多等 8 秒，204
   步全超时就是 **+27 分钟**，正好撞上 `--max-runtime` 默认的 60 分钟上限（撞上会中止
   并保存已采到的数据，不是丢掉）。

5. **odom 时间戳必须单调。** `window()` 是二分定位的，缓冲区一旦乱序，切出来的区间
   会**静默错**。而这不是纸面担忧：`/hand_lio/odom_vehicle` 的 stamp 直接继承自
   `/latest_imu_odom`（`HandLioNode.cpp:532`），hand-lio 自己就专门为此写了防护
   （`:119`，"timestamp went backwards, clearing pose buffer"，播包重播那类情况）。
   脚本照抄它的做法：发现回退就清空缓冲重新累积、限流告警、计数，结尾报出来。被清掉
   那一段覆盖的阶跃会取不到样本，表现为该次被跳过——有据可查，不会悄悄给个错答案。

   > 判据是**严格**回退（`t < 上一个 t`），不是 `<=`。**并列的时间戳对二分无害**：
   > 非递减序列二分照样正确，闭区间两端也都取得到（验过：并列 3 条全取到）。拿 `<=`
   > 判的话，一次无害的并列就会清空整个缓冲、白跳一次阶跃。hand-lio 那边用 `<=` 是
   > 对的——它那个缓冲是做位姿插值的，零长度区间确实有问题；这里的需求不一样。

另外结尾会打**跳过率**：单次失败只有一行 `logwarn`，滚上去就看不见了；跳过超过 30%
会显著告警并提示别把拟合当真。

### 顺带能标 hand_lio 的两个外参

`hand_lio.yaml` 的 `lidar_R_body` / `lidar_t_body` 现在还是占位的单位阵 + 零向量（那份
yaml 自己注明"装好后必须自己标定"）。这套动作正好把两个都覆盖了，报告里会直接给出
可以抄进配置的值。前提是**带上 `--spin-turns`（默认 2 圈）**——没有整圈覆盖，下面两个
圆拟合的条件数都极差。

**`lidar_t_body`（杆臂）** —— 原地纯 yaw 旋转时，如果雷达不在机体中心，odom 的 xy 会
画一个圆，半径就是杆臂长度。**但"纯"字是个陷阱：**狗迈腿时往一边蹭的那点横向速度
`w` 也在跟着转。用复数写清楚（ψ = ωt）：

```
ṗ_body  = e^{iψ}·w           =>  p_body  = C + e^{iψ}·w/(iω)
p_lidar = p_body + e^{iψ}·ℓ  =>  p_lidar = C + e^{iψ}·(ℓ − i·w/ω)
```

**量到的半径是 `|ℓ − i·w/ω|`，不是 `|ℓ|`**——漂移给杆臂硬加了一项 `w/ω`，方向还转了
90°。而且 `(p_lidar − C)·e^{−iψ}` 是个常向量，按 ψ 解出来就拿到带方向的 `ℓ − i·w/ω`。
剩下的问题是怎么把 `w` 剥掉：

**跑两段不同转速，联立解。** `ℓ` 是常量而 `w/ω` 随 ω 变：

```
vec_x(ω) = ℓx + wy/ω
vec_y(ω) = ℓy − wx/ω
```

两个不同的 ω 就把 `(ℓx, wy)` 和 `(ℓy, wx)` 各自解出来了——**不需要任何关于步态的
假设**，还顺带给出转圈时的 `w` 本身。所以脚本默认跑两段：取实测**最慢**（下限
0.3 rad/s，兜住时间）和**最快**的两个 yaw 幅值，因为方程里进的是 `1/ω`，两个转速离得
越开条件数越好。多花约 40 秒。

**为什么不能只跑一段、拿直线段量到的 `w` 去扣**：那要假设"转圈和平移蹭得一样多"，而
四足狗原地旋转和平移根本是两种步态。假狗上把这件事埋进去实测（真 ℓ=(0.220, 0.030)，
转圈步态 w=(0.030, −0.015)，平移步态 w=(0, 0.030)）：

| 做法 | 估出来的 ℓ | 误差 |
|---|---|---|
| 单段 ω=0.36，拿直线段的 w 扣 | (0.094, −0.041) | **14.4 cm** |
| 单段 ω=0.69，拿直线段的 w 扣 | (0.153, −0.000) | **7.3 cm** |
| 两段联立 | (0.220, 0.029) | **0.2 cm** |

注意两段单独量到的圆半径是 0.186 和 0.199——**彼此对不上，也都不等于真值 0.222，而
两段的圆拟合残差都只有 0.001m**。这又是一次"残差好看但答案错"，也正是为什么要报每段
的半径而不是只报一个平均值：两段对不上本身就是漂移存在的信号。

报告里还会把**转圈时的 w**和**直线段的 w**并排打出来（这次实测 `(+0.031, −0.013)` vs
`(+0.000, +0.032)`），差得多就直接说明两种步态确实不一样，不用猜。

> 反过来也成立：**yaw 标定时 x/y 方向的"耦合"有一部分是杆臂造成的假象**，不是真的
> 平移耦合，看耦合矩阵时要记得扣掉。

**`lidar_R_body`（安装旋转）** —— 拆成"倾角"和"绕 z 的偏转 γ"两半分别估：

- **倾角**：世界系是重力对齐的（见 `lidar_tilt_impact.md`），所以雷达 z 轴在世界系里
  的水平投影满足 `(zx, zy) = c + Rz(ψ)·δ`——**地面坡度 `c` 固定在世界系不随 yaw 转，
  安装倾角 `δ` 固定在机体系随 yaw 转**，转一圈做一次线性最小二乘就把两者分开了，圆心
  是坡度、半径是倾角。**这就是为什么不能用 roll/pitch 欧拉角**：欧拉角里这两样搅在
  一起，根本分不开。
- **偏转 γ**：机体沿自己的 +x 走时，odom（报的是雷达系）看到的行进方向会偏 γ。但这个
  夹角里混着**底盘自己的横向漂移**——狗迈腿时往一边蹭。模型是
  `夹角 ≈ γ + (w·n̂⊥)/v`，第二项**随命令方向翻号而 γ 不翻**，所以每个轴拿正反两组做
  一次两参数最小二乘 `[1, s/v]`，截距就是 γ、斜率就是漂移，干净分开。x 和 y 两组再
  互相对一次，作为"漂移是不是恒定向量那个形状"的第二道关。

  > 这一处返工过一次：最早那版只按轴取平均、拿**组内散布**当一致性容差，合成数据一验，
  > 5% 的横向漂移照样被判成"一致"就写进矩阵了——因为漂移的信号恰恰**就在**组内那个
  > 散布里（它随命令方向翻号）。

拼矩阵时还有两处坐标系修正，都是合成数据验出来的（不补的话 15° 倾角下矩阵差 1.8°）：
倾角拟合用的相位是**雷达**的 yaw 而不是机体的，差一个 γ，填进矩阵前要转回来；而 γ 的
观测量是在"去掉 yaw 的雷达系"里量的，倾角大时跟 γ 本身差零点几度，脚本拿构造出的矩阵
正向预测一遍观测量、按差值迭代几轮修回去。

**验证方式**：`build_lidar_R_body` 没法用实测数据验（真值未知），所以是拿合成数据做的
往返——构造一个已知的 `lidar_R_body`，按它生成转圈的姿态和 x/y 段的速度，再看估计器能
不能还原。在 roll 2° / pitch 15° / γ 7° / 坡度 0.8° / 横向漂移 15% 这组参数下，还原出
的矩阵与真值的等价残余转角 **0.15°**。

#### 这个估计有个原理上的天花板

**安装倾角和狗自己的站姿倾角分不开。** 两者都固定在机体系、都随 yaw 转，而 odom 只
看得见雷达，没有第二个观测量。要拆开只能靠外部参照——机体上放个水平仪，或者拿量角器
量支架——那是一次性机械测量，不是标定能解决的。同理，**x 和 y 同向偏一样多的漂移**跟
安装偏转本来就等价，任何标定都分不开（脚本的一致性检查只排除"差模"漂移）。

所以报告里那个矩阵是**建议值**，写进 `hand_lio.yaml` 前先确认这两条前提。

### 这份数据回答不了的事

标定标的是"**cmd_vel 对 odom** 一致"，这对闭环是对的。但如果 **odom 自己有尺度误差**，
标完之后狗的实际位移仍然是错的，而且从这份数据里完全看不出来。花十分钟用卷尺量 2~3
次（固定速度走固定时间，量实际位移跟 odom 报的比），就能把"cmd_vel 的问题"和"odom 的
问题"分开。报告末尾会打这条提示。

### 结果往哪里落

- **`usage_mode=0` 时，量出来的比例直接就是 `full_scale_v*` 该填的值**——标定本质上
  就是"量真实满量程"。这三个参数在「参数配置」页里能直接改。
- **死区补偿**如果要做，该做在 `deep_bridge`（贴着底盘那层）。但 `deep_bridge.yaml`
  的注释提醒过它会跟控制器整定打架（`closed_loop_controller/max_vy` 默认 0.35 恰好
  等于 0x1001 步态的 Y 轴下界），**有了数据再决定，别先入为主**。
- **不要去动控制器增益。** 那是另一个问题，混在一起两头都调不明白。

---

## topview.png 上的建图轨迹带

前端 2D 视图的底图 `topview.png` 会把**建图轨迹周围 0.25m 内的 free 格子**染成淡蓝灰
（`TRAJECTORY_BAND_RGB = (214, 228, 243)`），让人一眼看出"狗当初从哪儿走过"。
`--topview-trajectory-band` 控制半径，传 0 关掉。

**这是 topview.png 唯一的修饰，而且有两条硬约束：**

1. **只影响 `topview.png`，`map_2d.pgm` 一个字节都不改。** 规划器读的是 pgm。
2. **只染本来就是 free 的格子**（代码里 `band &= gray >= 250`）。这一条是为了不跟
   「展示图不许跟规划器说两套话」那条原则冲突——上面那条原则禁的是"把不能走的地方画成
   能走的"，而这里只给已经能走的地方上色，图上"哪里挡路"的信息一个像素没变。
   实测六张图：染色像素占 free 的 2.2%~33%，**盖到 occupied/unknown 的像素数全部为 0**。

半径用的是几何意义上的 `EDT <= r/res`，**不是** `elevation.clear_trajectory` 那个
`ceil(r/res)+1`——多出来那一格是为了让规划器在带子里能斜着走（见该函数 docstring），
是规划的需要，展示图不该跟着虚胖。所以这条带子会比 `clear_trajectory` 实际清出来的
略窄一点，是有意的。

颜色选择的约束是用户给的：**跟 free 分得开，但不能太突出**——它只是背景信息，不该比
真正的障碍还抢眼。淡蓝灰跟 free 的白（254）、unknown 的灰（205）、occupied 的黑都不撞色。
要调只改 `TRAJECTORY_BAND_RGB` 一个常量。

> 跟前端「显示面板 → 地图 → 建图轨迹」那个开关是两回事：那个画的是**中心线**（琥珀色，
> 数据来自 `/api/maps/{name}/trajectory`，3D/2D 都能开），这里画的是**带宽**，而且是烘进
> 底图的。两者可以同时看。

---

## 途经点间距的上下限都是 SCAN-Planner 的参数逼出来的

`navi_mode=2`(`/preset_waypoints`)下,planner **一次只规划到下一个航点**
(`scan_replan_fsm.cpp:254` `planNextWaypoint`):`planGlobalTraj(start, v0, 0, 航点, 0, 0)`,
两个点 → `one_segment_traj_gen`,一条五次曲线。所谓"全局参考"从来不是我们下发的那条折线,
而是"当前位置 → 下一个航点"这一段曲线。

而且 `getLocalTarget()` 沿这条曲线走 `planning_horizon_`(5.0m)找局部目标,曲线全长就是
航点间距——间距 < 5m 时走不满,`local_target_pt_` 直接就是航点本身。**航点间距才是
mode 2 的有效前瞻,那个 5m 在窄路上从来没生效过。**

| | 值 | 来自 |
|---|---|---|
| 折线简化下限 `GLOBAL_PLANNER_MIN_WAYPOINT_SPACING_M` | 0.25m | `reboundReplan` 的 0.2m 死区(`planner_manager.cpp:94`)|
| 航点间距 `GLOBAL_PLANNER_WAYPOINT_SPACING_M` | 0.8m | 实机手感,见下面"短段才是真问题" |

鼓包实测(复刻 `one_segment_traj_gen`,`max_vel`=0.75,`T=2L/max_vel`,狗以夹角 α 进入):

| 段长 | α=15° | 30° | 45° |
|---|---|---|---|
| 3.70m | 0.38 | **0.73** | 1.03 |
| 1.50m | 0.15 | 0.30 | 0.42 |
| 1.00m | 0.10 | 0.20 | 0.28 |

`save_map_small_1` 那条 270m 的路线走廊净空中位只有 0.54m(10 分位 0.41m)。加上限之前
最长段 3.70m,按实际转角估算鼓包最大 0.47m、有 5 段超过 0.3m——**参考曲线本身跑到可通行区
外面**,B 样条优化器拿着一条穿墙的初值去优化。加上限之后(330 → 433 点)最大鼓包 0.23m,
超 0.3m 的段 0 个。

**窄路两边是沟的场景要特别小心**:沟是负障碍,`detect_structure` 的机体高度带判据(局部
地面之上 0.10~0.55m 有没有实体)和 planner 的实时 ESDF 都不一定看得见它,没有把曲线拉回来
的梯度。间距上限只能保证"贴着 A* 那条线走",A* 那条线本身避没避开沟,取决于建图时沟有
没有被标成障碍——可以开地图预览页的「建图轨迹」图层对着 2D 栅格图确认。

插点是在弦上等分的:每段弦都被 `_line_free` 验证过无碰撞,等分之后同一直线上相邻段转角是 0,
鼓包归零。比"把 A* 原路径的点塞回去"好——后者会把 8 连通网格的锯齿重新引进来,转角变大反而更鼓。

### 短段才是真问题:改成沿折线等距重采样

上面整节讲的都是"段太长会鼓出去"。但**轮足狗上实机跑下来,出问题的是段太短**:每到一个航点
都要减速进 `waypoint_arrival_radius_`(0.3m)再重规划下一段,航点一密就平地一冲一冲、上下
楼梯左右摆动(用户实测)。而原来的实现只管上限不管下限,短段到处都是:

| | 点数 | 段长中位 | 段长最小 | <0.2m 的段 |
|---|---|---|---|---|
| `save_map_stairs` 爬楼梯(水平 3.49m / 22°) | 10 | 0.30m | 0.25m | 0 |
| `save_map_small_1` 那条 270m | 450 | 0.59m | **0.10m** | 6 |

0.10m **比 `reboundReplan` 的 0.2m `TOO_CLOSE_TO_GOAL` 硬线还短** —— 那些段 planner 根本不
生成轨迹,还要给 `continuous_failures_count_` 加一。`_enforce_min_spacing` 名义上是兜底安全网,
但它遇到"并段会穿墙 / 爬升超限"就放弃并留下密点,所以下限其实兜不住。楼梯上则是
`max_climb=0.20` 把段长钉成一级台阶一个点。

现在改成 **沿剪枝后的折线等距重采样**(`_resample_polyline`)。关键是把两件事分开:

- 折线的**几何形状** —— 由剪枝 / 最小间距 / 爬升判据决定,`max_climb` 可以继续管得很严,
  让折线紧贴楼梯走;
- **在这条形状上怎么撒航点** —— 完全由 `GLOBAL_PLANNER_WAYPOINT_SPACING_M` 说了算。

于是平地和楼梯是同一套逻辑,不需要为楼梯单独放宽什么。份数用 `round` 不用 `ceil`(`ceil` 会
系统性地把实际间距压到目标值以下,而这次要治的就是"太短")。

**拐角怎么办**:等距撒出来的弦可能跨过折线拐点、把角切掉,而 `_line_free` 只验过原折线的
每一段。切了会穿墙的拐点,做法是把**最近的那个样本吸附到拐点上**,而不是插一个新点——实测
那条 270m 路线上这种拐点有 34 个,离最近样本最远 0.383m(半个步长),吸附后相邻段长落在
0.8±0.38 之间,一个短段都不会造出来;插点则会插出 0.01m 这种。吸附不下的极端情况才退回插点,
再由收尾那道"短段就丢掉不是拐点的那一端"把它清掉。

结果:

| | 改动前 | 现在 |
|---|---|---|
| 楼梯(3.49m / 22°) | 10 点,中位 0.30m | **5 点,全部 0.87m** |
| `save_map_small_1` 270m | 450 点,中位 0.59m,最小 0.10m | 334 点,中位 0.80m,最小 0.32m |
| `save_map_large_1` | 最小 0.10m,2 段 <0.2m | 最小 0.76m,**0 段 <0.2m** |
| 所有测过的图 | 共 8 段短于 planner 的 0.2m 硬线 | **0 段** |

0.8 这个值是纸面推算(楼梯上每段爬升 `0.8*tan(22°)=0.36m`,约两级台阶一个航点),
**手感要在真机上调**:`NAVIBOT_GLOBAL_PLANNER_WAYPOINT_SPACING_M`,还嫌密就往大调,
拐弯开始切角了就往小调。设成 0 关掉重采样,退回"剪枝出来多少点就发多少点"。

一个副作用:插出来的点各自查自己位置的 `ground_elevation`,z 剖面更贴真实地面,于是**原来被
粗采样掩盖的爬升会冒出来**(实测这条路线超 `max_climb` 的段 1 → 2 个)。是原来在撒谎,不是
插点把路弄陡了。

---

## 虚拟障碍点云（让局部规划器也看见禁行区）

地图编辑只影响**全局**规划。SCAN-Planner 的局部避障用它自己的实时 3D 栅格图（`grid_map`），
我们圈的禁行区它一无所知。最要命的是**沟**：沟是负障碍，雷达打到沟底那就是"地面，只是矮一点"，
`grid_map` 里没有任何占据体素；局部规划器既没有代价也没有梯度（何况它的 z 梯度还被
`grad_3D.row(2).setZero()` 清零了），狗可以直接"飞"过沟口。

办法：把禁行区采样成世界系的点，**混进喂给 `grid_map` 的那条点云里**，让它变成正障碍。

```
navibot 后端 ──latched──> /navibot/virtual_obstacles (PointCloud2, world 系)
                                      │
                          hand-lio 订阅并缓存，每帧发布前
                          append 进 /hand_lio/clouds_lidar
                                      │
                          SCAN-Planner grid_map (cloud_is_world=true)
```

**为什么不能另发一路到 `/hand_lio/clouds_lidar`**：`grid_map` 的 `cloudCallback` 每次从
`proj_points_cnt = 0` 开始**覆盖** `md_.proj_points_`（不累加），而 `updateOccupancyCallback`
是定时器、每次只消费最近那一帧。多发一路会让真实雷达帧被顶掉——安全倒退。必须在**同一条
消息**里带上，所以混入要在 hand-lio 里做。顺带的好处是 stamp 和 `/grid_map/sensor_pose`
天然对齐，不会有中继节点带来的一帧错位。

### 这一侧（navibot）已经做完的

- 只取 `enabled` 且 `kind == blocked` 的区域，**只发当前激活地图的**（机器狗的位姿在激活地图的
  坐标系里，发别的图等于凭空造墙）。没有激活地图就发空的。
- **只画多边形的外壳（一圈墙），不填实。** 雷达看到的永远是表面不是体积，一圈墙正是真有一堵
  墙时雷达会看到的样子；而且便宜——1m×5m 的沟填实要 2000 格，外壳只有一圈。
- z 用**途经点高度**（`ground_elevation + Δ`，跟 `plan_path` 发出去的 z 是同一个量）上下
  各 0.30m。局部轨迹的高度完全由我们下发的航点决定，墙套在这个高度才挡得住。
- 采样步长默认 0.025m = `grid_map/resolution`(0.05) 的一半。理由见下面的投票规则。
- 区域增删改、切换激活地图、后端启动都会重发；话题 latched，hand-lio 什么时候起来都能拿到
  当前这一份。空点云也发——空表示"现在没有虚拟障碍"，订阅方据此清掉上一批。

- 每个点带**朝外法向**（`normal_x/y/z`，PCL 约定的字段名），给 hand-lio 做背面剔除用。
  法向由外壳格子的 8 邻域里"在多边形外"的那些邻居的偏移量求和归一化得到——只用现成的掩膜，
  不引入第二套点在多边形内外的判据。实测方块和 L 形（凹多边形）都 100% 朝外。

### hand-lio 那一侧（已完成）

`enable_virtual_obstacles`（默认开）订阅上面那个 latched 话题并缓存，每帧发布前追加：

1. **量程剔除**：超过 `max_ray_length`(5m) 的不注入。
2. **背面剔除**：`dot(法向, 传感器方向) > 0` 才留。navibot 发的是整圈闭合的墙（它不知道机器人
   在哪），全注入的话射向**远侧**墙面的光束会穿过**近侧**墙面的体素给它记 miss，自己把自己的墙
   投票投掉——真实的墙不会这样，雷达看不见墙背面。
   > 先试过按 (方位角, 俯仰角) 分桶做深度缓冲，**不行**：桶要比墙面采样的角间距粗才挡得住，而
   > 那个角间距随距离变——2m 处 0.025m 的采样张 0.72°，比雷达自己的角分辨率(≈0.4°)还粗，光束
   > 直接从近侧点之间漏过去。合成数据上实测远侧墙 192 个点一个没剔掉。背面剔除没有这个尺度
   > 问题，也不需要跨仓库对齐常数。
3. **按距离加密**：每个可见点复制 `density_gain / r²` 份（默认 gain=12，即 1m 处 12 份），
   理由见下面的投票规则。复制点不加抖动——`setCacheOccupancy` 是按点计数的，同一坐标重复 N 次
   就是 N 个 hit。

合成数据验证（雷达在原点，x=2.0 和 x=2.5 两面墙）：远侧墙注入 **0** 点，近侧 123 个位置 ×
**3** 份 = 369（`12/2² = 3` ✓）；把雷达挪到 x=5，剔除关系正确翻转。

### 自动生成走廊墙：`tools/gen_corridor_walls.py`

按"建图轨迹 ±`half_width` 之外不许走"批量生成禁行区，省得一段一段手画：

```
schroot -c focal -- python3 tools/gen_corridor_walls.py house --half-width 0.40 --dry-run
schroot -c focal -- python3 tools/gen_corridor_walls.py house --half-width 0.40 --replace
```

生成的区域跟手画的完全一样（存进 `data/map_edits/<name>.json`），全局规划会绕开、局部避障也能
看见、在面板里能逐个停用/删除。`note` 带 `auto:corridor-wall` 前缀，`--replace` 只重做这些，
手画的不动。

**两端是敞开的。** 等值线是整条走廊的闭合边界，首尾各扣一个半圆盖；照着它铺墙会把走廊两头
也堵死，狗从起点出不去。默认切掉（`--close-ends` 恢复旧行为），两条判据并用：

1. **几何端盖**：一个边界点属于端盖，当且仅当**离它最近的轨迹采样点正好是首/尾那一个**，**且**
   它在端点的外侧（沿端点切向投影为正）。第二条不能少——只看"最近点是端点"会把紧挨端点的两侧
   墙也算进去，墙会从端点往回缺一截。
2. **末梢长度，自动算**（`_terminal_stub_lens`，可用 `--open-end-len` 手动压住）。
   **光靠第 1 条不够**——狗在终点附近拐一下（走过头再退回来）时，走廊最外面那个圆头对应的最近
   轨迹点是个**中间**采样点，第 1 条认不出来，那一端照样封死。实测 `save_map_large_1`：轨迹末端
   有个钩子（走到 `(94.35,0.67)` 又拐回 `(93.86,-0.18)`），圆头 `(94.79,0.87)` 对应的最近轨迹点
   **距末端还有 1.70m 弧长**。

   规则：**从端点往回走，只要路还在端点附近打转（`3×half_width` 的球里），这一段就算"末梢"**；
   末梢长度再加一个 `half_width`（等值线是从路往外偏 `half_width` 的，包住一段长 S 的末梢，弧长
   上要覆盖到 `S + half_width`）。球半径跟着走廊自己的尺度走，不另外拍数。首尾**各算各的**。

   实测（`half_width=0.40`）：`save_map_large_1` 1.59 / **1.73m**（末端那个圆头正好要 1.70），
   `save_map_small_1` 4.57 / 1.79m，`house` 1.71 / 4.54m，`save_map_stairs` 1.66 / 2.68m。
   **这个长度每张图都不一样，所以不该让人猜**——用户按 1.0m 试过，圆头照样封着。

   > 走过两条弯路，记下来免得再走：
   > - 按**到端点的直线距离**切：端点常被埋在钩子里，它周围那圈等值线是走廊的**两侧**而不是
   >   末端的圆头，口子切在了侧墙上，末端照样封着。
   > - 按**"能不能走出去"自动撑**：太松。从侧墙那个口子就能逃出去，撑到 0.5m 就判定"通了"，
   >   圆头还封着。连通性是必要条件不是充分条件。

   生成完仍然在**规划器真正用的那张栅格图**上复验一次"两端能不能走到走廊外面"（`_end_is_open`，
   BFS 邻接规则跟 `_astar` 一致），不通会明说。

合成图验证（12×12m 全空地，轨迹 `(2,6)→(10,6)` 直线，`half_width=0.45`，从走廊里的 `(6,6)` 出发）：

| 目标 | 两端敞开（默认） | `--close-ends` |
|---|---|---|
| 末端外 1.5m `(11.5,6)` | 成功 7 点 / 5.5m | **失败：找不到可行路径** |
| 侧面 2m `(6,8.5)` | 成功 12 点 / 9.5m（绕到端口出去） | **失败** |

开口宽度就是走廊宽（实测 `save_map_large_1` 0.93m、`save_map_small_1` 0.91/0.87m，≈2×half_width）。

> 轨迹首尾踩回自己身上时**本来就没有端盖**（端点的圆盘埋在走廊并集里面），工具会报
> "0 条等值线被切开了端盖"。实测 `house`（首尾离轨迹其余部分 0.03/0.02m）和 `save_map_stairs`
> （0.05/0.06m）都是这种；`save_map_large_1` 起点露在外面 1.86m、`save_map_small_1` 终点 1.21m，
> 这两张才真有端盖。

走廊边界取的是**距离场等值线**（"到轨迹的距离 == half_width"），不是逐点法向偏移。逐点偏移在
轨迹折返、反复走同一片地方时会大面积失效——house 上 1354 个偏移点只活下来 296 个，墙碎成 80 段
几厘米的小茬子。等值线天然就是"所有轨迹点的 half_width 圆盘的并集"的边界，折返/自交/绕圈都自动
处理好。

> **`half_width` 不能真取 0.3。** 墙立在 `half_width` 处，而规划器规划前会把障碍按
> `GLOBAL_PLANNER_INFLATION_RADIUS_M`（0.25m，按分辨率向上取整）膨胀回来——走廊净宽只剩
> `half_width` 减膨胀半径，再算上两次栅格化的取整就没了。实测 4 张图在 0.3m 下**全部被自己立的
> 墙封死**：
>
> | 地图 | 分辨率 | 膨胀 | 0.30m | 0.35m | 0.40m | 0.45m |
> |---|---|---|---|---|---|---|
> | `house` | 0.05 | 0.25m | −0.050 | +0.000 | **+0.054** | +0.054 |
> | `save_map_stairs` | 0.05 | 0.25m | −0.026 | | | |
> | `save_map_small_1` | 0.10 | 0.30m | −0.100 | −0.076 | −0.017 | **+0.061** |
> | `save_map_large_1` | 0.10 | 0.30m | −0.100 | | | |
>
> （"最窄处余量"，负数=封死）经验值 **`half_width ≥ 膨胀半径 + 0.15m`**。脚本每次都按规划器的
> 真实判据复验并给建议值，封死时退出码 2。

### 关键风险：注入密度要压过真实光束

`grid_map.cpp:664` 每个更新周期对每个体素投一次票：

```cpp
log_odds = count_hit >= count_hit_and_miss - count_hit ? prob_hit_log_ : prob_miss_log_;
```

展开就是 **`n_注入 >= n_真实光束穿过`**。虚拟墙所在的位置现实里是空的，每帧都有真实光束穿过去
打到后面的地面，每条贡献一个 miss。

粗估（Mid-360 约 2 万点/帧、FOV 360°×59° ≈ 6.2 sr）：3m 处一个 0.05m 体素约被 1 条光束穿过，
1m 处约 8 条。步长取 0.025 时一个体素里有 2×2×2 = 8 个注入点，够用。**这个估算是按包络算的，
没在真机上量过**——上机第一件事就是验它。

---

## 已知缺口

这几条是如实标注的，不要当成已解决：

- **"冻结执行"不是硬急停。** navi_mode=2 没有外部急停接口，内部的 `EMERGENCY_STOP` 只由它自己的碰撞检测触发。冻结只让 planner 不再推进轨迹时间，**不断电、不立即制动**。真正的硬急停必须在 `unitree_bridge` 那一层做。
- **取消不会清空 planner 的任务队列**，只是冻结。要真正换任务只能下发新路线（新 Path 整轮替换）。
- **进度是推断出来的**，不是 planner 报的。planner 卡住时我们看不出区别，只有 60 s 超时兜底。
- **Δ 只用单次采样。** 下发那一刻狗若正好站在楼梯踏面上，台阶量化误差（实测楼梯段 IQR 0.090 vs 平地 0.031）会落到 Δ 上并施加到整条路线。改成滑动窗口取中位数可解。
- **可站立区只覆盖走过的地方。** 想要更大的可用区域，让狗多走两圈比调参数可靠。
- **全局规划的"离墙远一点"偏好（`GLOBAL_PLANNER_WALL_CLEARANCE_M`/`_MULTIPLIER`，默认 0.3m / 3.0）没有拿真机验证过。** 只是软惩罚，不是硬挡——空间不够（比如窄门）时路径照样会贴着硬膨胀边界走；默认值是凭经验估的，偏保守可能路径还是贴得比预期近，偏激进则可能为了离墙远一点绕不必要的远路，需要在实机地图上试几条路线再调。
- **`detect_structure` 的机体区间（`--map2d-body-clearance` / `--map2d-body-height`，默认 0.10 / 0.55m）没有拿真机验证过。** 判据是"局部地面 + 这段高度里有没有点"，也就是狗的身子实际会扫过的那层体积。低于 0.10m 的当地面回波和它迈得过去的小坎，高于 0.55m 的当它能从下面走过去（桌面、挂墙置物架、天花板横梁）。这两个数是机器人的物理尺寸，理应比之前那些经验阈值稳，但**"狗到底能不能从 0.55m 的东西下面过去"没有实机试过**——偏保守（调大 `--map2d-body-height`）会把桌面也标成硬挡，偏激进则可能让全局路线穿过它其实过不去的地方。实测把上界放到 0.80 时 house 的召回 60.7%→66.9%，走廊误报 8.61%→15.82%。
- **`detect_structure` 在某些图上是椒盐噪点级的误判。** `save_map_large_1`：blocked 81.2%、free 只有 2.0%，3808 个障碍连通块里 3404 个面积 ≤5 格。后果是整张图的可走空间基本只剩轨迹清出来的那条带，全局规划全程贴着膨胀边界走（实测一条 58.6m 的路线吐了 99 个途经点——剪枝的 line-of-sight 一步都跨不出去）。实测把 ≤10 格（0.1 ㎡）的孤立障碍块去掉就能恢复连通，但还没做，也没验证会不会顺手抹掉真的细柱子。跟上一条是同一个判据的两种失败方式。
- **`detect_structure` 分不掉建图时扫到的人。** 人的躯干正好落在机体区间里。射线投射、SLAM 自己的自由空间图、时间持久性这三类方法都实测排除了——**家具对激光是多孔的**（椅子桌子大半是空隙，射线常年穿过），任何基于"射线穿没穿过"的判据都会把家具连同人一起铲掉。详见 `tools/probe_detect_structure.py`。目前只能靠建图时别让人进场景。
- **传感器离地高度（`estimate_sensor_height`）不是机器人常数。** 实测室内三张图一致（0.529~0.545），但室外的 `large` 是 0.476——多半是草地/植被的回波抬高了"地面"。所以每张图各自量，不写死；量不出来时退回 0.55 并打印警告，那种情况下整张图的障碍判定会系统性偏移。
- **巡检路线只有增删改查，没有执行。** 存下来的路线**发不出去** —— 没有"把这条路线
  下发给机器狗"的接口，`schedule` 也没有任何调度器在读（见"巡检路线"一节）。下发方式
  本身已经不用选了（地图预览页那条现成的链路：`plan_path` 算拐点 → `submitRoute` 走
  navi_mode=2），剩下的问题是 `action`/`stay` 这两个字段该由谁来实现——planner 侧不
  认识它们。
- **路线数据在 `data/routes/` 下，被 `.gitignore` 排除。** 这是用户自己创作的数据，
  跟 `web_assets/`（能从源头重新生成）不一样，换机器/重装要自己带走，没有任何导入
  导出功能。
- **地图编辑的多边形栅格化经过脚本验证，但画多边形那套交互没在真实浏览器里点过。** 栅格化（矩形/三角形/凹多边形/自相交/图外裁剪/世界坐标闭环）和"编辑真的改变规划结果"都验过（含一次完整的增→规划失败→停用→规划成功→删除的接口往返），但左键加点/右键撤销的手势只过了类型检查和构建。
- **前端未经真人浏览器验证。** 类型检查和构建通过，但布局/配色/交互没有实际看过。
  巡检路线的两个页面（列表/编辑）也一样——后端接口用 curl 逐条过了（见下面），但
  在图上点选导航点这套交互没有在真实浏览器里点过。
- **服务状态管理没有在真实机器上验证过。** `backend/app/service_manager.py` 调
  `systemctl`/`sudo -n systemctl`，开发机没有 systemd/这几个单元，本地跑不了；
  见"服务状态管理"一节的 sudoers 规则也还没有在机器上实际配过——权限没配对时
  的报错文本是否真的可读、`sudo -n` 在目标机器上的确切失败提示，都还没实机验证。
- **`localization.service`/`hand_lio.service` 的伴生启停顺序是用户口述的，没有
  实机核对过。** `config.SERVICE_COSTART` 假定先启动 `localization.service`
  再启动 `hand_lio.service`（停止顺序相反），如果实际顺序反了或者两者之间还有
  别的初始化时序要求，现象会是「导航定位」点了启动、`localization.service`
  正常起来了，但 `hand_lio.service` 起不来或状态不对——两个服务各自的
  `active_state` 仍然可以在 `systemctl status` 里查到，只是系统管理页目前只
  展示 `localization.service` 这一个的状态，`hand_lio.service` 没有单独的卡片。
- **建图页的保存流程还没在真实机器上跑过。** `start_save_map.bash` 产出的目录
  结构是否真的跟 `map_registry` 期望的一致是推断，不是核对过的事实——见
  「建图」一节。点云/机器狗位置这部分已经实机验证过（见下一条），保存这一步
  没有 board 上的完整建图-保存流程可跑，仍然是假设，不符的话现象会是"保存
  失败"，不是别的隐蔽 bug。
- **`/tf` 的 `map -> livox_frame` 帧名只在纯点云建图模式下核对过。** 对着实际
  跑起来的 `cloud_mapping_small.service` 直接连上机器人的 ROS master 读
  `/tf` 确认过（见 `MAPPING_TF_BODY_FRAME` 的说明），彩色建图模式
  （`color_mapping_small`/`color_mapping_large`）下 `/tf` 会不会额外发布/换成
  别的帧名（比如 `HandBot-S1-view/ros1.rviz` 里截到的 `latest_lidar`）还没有
  实机核对过，不符的话现象是"建图页（仅彩色建图模式下）看不到机器狗位置"，
  点云本身不受影响。
- **激活地图时创建软链接、检查 localization.service 状态，都没有实机验证过。**
  `map_registry.py` 的 `activate_map`/`clear_localization_link` 假定
  `localization.service` 只在启动时读一次 `config.SAVE_MAP_DIR`（之后不管软
  链接怎么变都不受影响），以及跑后端的用户对 `/home/cat/handbot_slam/...`
  这条路径的**父目录**有创建软链接的权限（这条本身不需要特权，`sudo -n rm
  -rf` 管的是删这个路径本身，不是它的父目录）——这两条都是推断，没有拿真实的
  localization 相关代码/权限配置核对过。`RM_SUDO_CMD` 对应的 sudoers 规则
  （见上面「建图」一节）也还没在机器上实际配过、验证过报错文本是否可读。见
  「激活地图 与 localization.service 共用的固定路径」一节。
- **服务之间的关系整套去掉之后，起停顺序完全交给用户，没有任何防呆。** 比如
  在「导航定位」还在跑的时候停掉「激光雷达」，后端不会拦，定位会静默失去数据
  源；反过来只启动「路线规划」而没启动「实时里程计」，planner 因为
  `!have_odom_` 会把收到的途经点直接丢掉（只打一条 ROS_WARN）。这是用户明确
  要求的行为（每个服务单独管理），但**实机上这些组合都没试过**——见「服务之间
  没有关系，每个服务单独管理」一节。
- **参数配置页没有在真实机器上验证过。** 读/写/校验/重启端点都是在开发机上对着
  `deep_bridge.yaml` / `hand_lio.yaml` / `advanced_param.xml` 的副本跑通的，但**没有在
  板子上验证过后端用户对那些文件有没有写权限**（`advanced_param.xml` 还在 SCAN-Planner
  的源码树里，改它等于在那个 git 仓库里留下本地改动） —— 写不动的话是 500 + "权限?" 的提示，不是静默失败。重启走的 stop + start
  也没在真机上试过。
- **`deep_bridge` / `unitree_bridge` 两个新服务只核对过 unit 文件，没跑过。**
  unit 名和 `ExecStart` 取自 `navi-planner-bringup/systemd/`，但这两条对应的
  机器（云深处 Lynx M20 / 宇树 Go2）都没有实际启停验证过，sudoers 规则也还
  没在机器上配过。

---

## 相关仓库

- [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) — 局部规划器，本项目对接 `navi_mode=2`
- `hand-lio` — 把 HandBot-S1 的 `/latest_imu_odom` 和原始点云桥接成 map 系点云与机体位姿
