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
    topview.png             上面这份 2D 占据栅格图转成的展示用 PNG（逐像素一致，不改色）
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

# 4. 前端
cd frontend && npm install && npm run dev
```

没有实机时，用模拟器验证整条链路（它刻意复刻了真 planner 的到达判据、跳点规则、急停悬停确认行为）：

```bash
mamba run -n ros_host python backend/mock_planner.py --start <X> <Y> <Z> <YAW>
```

> `--start` 的 Z 是 **odom 系机体高度**（地面高程 + 传感器离地高度），不是离地高度本身。默认 `(0,0,0,0)` 在多数地图里都在墙里。

---

## 服务状态管理

系统管理页有一张"服务状态"卡片，管 4 个固定的 systemd 单元（id/显示名/unit 名见
`backend/app/config.py` 的 `SYSTEMD_SERVICES`，不接受任意 unit 名）：

| id | 显示名 | systemd 单元 |
|---|---|---|
| `lidar` | 激光雷达 | `mid360.service` |
| `camera` | 相机 | `camera.service` |
| `localization` | 导航定位 | `localization.service` |
| `planner` | 路线规划 | `navi_planner.service` |

对应接口：`GET /api/services`（3s 轮询查状态）、`POST /api/services/{id}/start`、
`POST /api/services/{id}/stop`（`service_manager.py`）。查状态用 `systemctl show`，
不需要特权；启动/停止需要特权，默认用 `sudo -n systemctl ...`（`-n` 非交互，没配
免密的话直接报错而不是卡住等密码），可用 `NAVIBOT_SYSTEMCTL_SUDO_CMD` 覆盖。

**这意味着部署到机器上时要单独配一条 sudoers 规则**，只放行跑后端的用户对这
4 个单元执行 `start`/`stop`（仓库里没有现成的 unit 文件/sudoers 配置，这步要在
机器上手动做），例如：

```
# /etc/sudoers.d/navibot-services
<backend-user> ALL=(root) NOPASSWD: /bin/systemctl start mid360.service, \
    /bin/systemctl stop mid360.service, \
    /bin/systemctl start camera.service, /bin/systemctl stop camera.service, \
    /bin/systemctl start localization.service, /bin/systemctl stop localization.service, \
    /bin/systemctl start hand_lio.service, /bin/systemctl stop hand_lio.service, \
    /bin/systemctl start navi_planner.service, /bin/systemctl stop navi_planner.service, \
    /bin/systemctl start cloud_mapping_small.service, /bin/systemctl stop cloud_mapping_small.service, \
    /bin/systemctl start cloud_mapping_large.service, /bin/systemctl stop cloud_mapping_large.service, \
    /bin/systemctl start color_mapping_small.service, /bin/systemctl stop color_mapping_small.service, \
    /bin/systemctl start color_mapping_large.service, /bin/systemctl stop color_mapping_large.service
```

没配这条规则时，点"启动"/"停止"会在页面上收到 500 和 `sudo` 的报错文本（比如
`sudo: a password is required`），不是静默失败。

### 服务依赖关系

`backend/app/config.py` 的 `SERVICE_DEPENDENCIES` + `MAPPING_MODE_DEPENDENCIES`
声明了这几个 systemd 单元之间的依赖（只列直接依赖，间接依赖靠
`service_manager.py` 里的传递闭包算法推出来）：

- `localization.service` 依赖 `mid360.service`
- `navi_planner.service` 依赖 `localization.service`（因此间接依赖 `mid360.service`）
- 建图服务（`cloud_mapping_*.service`）依赖 `mid360.service`
- 彩色点云建图服务（`color_mapping_*.service`）额外依赖 `camera.service`

另外 `SERVICE_COSTART` 声明了一条方向相反的"伴生"关系：启动 `localization.
service` 后紧接着也要启动 `hand_lio.service`（导航定位用它输出的实时里程计），
顺序固定先 `localization.service` 后 `hand_lio.service`（用户口述的顺序要求，
没有拿到 `hand_lio` 的实际配置核对过反过来会怎样）；停止 `localization.
service` 时顺序相反，先停 `hand_lio.service` 再停 `localization.service`。
`hand_lio.service` 没有自己的 `SYSTEMD_SERVICES` 条目（系统管理页看不到它单独
的开关/状态），只能跟着「导航定位」这张卡片一起启/停。

这些 unit 文件本身没有声明 `Requires=`/`After=`（板子上是各自独立配置的脚本，
不假设它们互相知道对方存在），依赖关系是在应用层做的：

- **启动一个服务，会自动启动它依赖的服务**（没在跑才启动，见
  `service_manager.start_with_dependencies`）——比如在系统管理页点"启动"
  「路线规划」，会先确认「导航定位」和「激光雷达」都已经在跑，没跑就顺带启动；
  「新建地图」选彩色建图模式同理，会先确认「激光雷达」和「相机」。
- **停止一个服务，如果有其它正在运行的服务（直接或间接）依赖它，会拒绝**
  （`service_manager.find_blocking_dependents`），报错里列出是哪些服务，
  提示用户先停那些——比如「导航定位」还在跑的时候不能停「激光雷达」。

**这一整套依赖解析没有在真实机器上验证过**，见「已知缺口」。

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
- 启动服务本身失败（`start_with_dependencies` 抛错，典型是 sudoers 没配好）
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

> **现在只有后端接口，没有前端入口。** 原来的入口在地图预览页右侧「显示面板 → 地图编辑」，
> 那个面板（`components/map-detail/MapDetailPanel.tsx`）随 `22341b0` 一起回退掉了，画多边形
> 的交互也一并去掉。下面这套接口、`map_edit_store.py` 和 `global_planner` 里的叠加逻辑都还在，
> 直接打接口存进去的区域照样生效——要重新做入口的话，注意多边形只能画在 2D 栅格图上（3D 点云
> 里没有对应的作图平面）。

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
| 下限 `GLOBAL_PLANNER_MIN_WAYPOINT_SPACING_M` | 0.25m | `reboundReplan` 的 0.2m 死区(`planner_manager.cpp:94`)|
| 上限 `GLOBAL_PLANNER_MAX_WAYPOINT_SPACING_M` | 1.0m | 五次曲线的横向鼓包 ∝ 段长 |

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

一个副作用:插出来的点各自查自己位置的 `ground_elevation`,z 剖面更贴真实地面,于是**原来被
粗采样掩盖的爬升会冒出来**(实测这条路线超 `max_climb` 的段 1 → 2 个)。是原来在撒谎,不是
插点把路弄陡了。

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
  下发给机器狗"的接口，`schedule` 也没有任何调度器在读（见"巡检路线"一节）。接执行
  时要决定的第一件事是：走 navi_mode=2（`/api/route`，逐点判到达）还是先过一遍
  `global_planner.plan_path`，以及 `action`/`stay` 这两个字段该由谁来实现——planner
  侧不认识它们。
- **路线数据在 `data/routes/` 下，被 `.gitignore` 排除。** 这是用户自己创作的数据，
  跟 `web_assets/`（能从源头重新生成）不一样，换机器/重装要自己带走，没有任何导入
  导出功能。
- **地图编辑没有前端入口。** 栅格化（矩形/三角形/凹多边形/自相交/图外裁剪/世界坐标闭环）和"编辑真的改变规划结果"都用脚本验过，接口也能用，但画多边形那套交互随显示面板一起回退掉了——现在只能直接打 `/api/maps/{name}/edits`。
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
- **服务依赖关系（`SERVICE_DEPENDENCIES`/`MAPPING_MODE_DEPENDENCIES`）是按
  用户口述的依赖列出来的，没有拿板子上的实际配置核对过。** 如果实际依赖关系
  跟这两张表不一致（比如还有表里没列的依赖，或者某条依赖其实反了），后果分
  两种：该自动启动的没启动（现象是"点了启动，界面显示成功，但服务其实因为
  缺依赖起不来"）、或者该拦住的停止操作没拦住（现象是"停了一个服务，另一个
  正在依赖它的服务跟着挂了却没有任何提示"）——见「服务依赖关系」一节。

---

## 相关仓库

- [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) — 局部规划器，本项目对接 `navi_mode=2`
- `hand-lio` — 把 HandBot-S1 的 `/latest_imu_odom` 和原始点云桥接成 map 系点云与机体位姿
