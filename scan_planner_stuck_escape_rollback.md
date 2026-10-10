# SCAN-Planner 卡住脱困（stuck escape）：实机测试与回退记录

记录日期：2026-10-10（北京时间）。本文保存在 navibot 仓库，以下 Git 操作均针对 **SCAN-Planner 仓库**，不要在 navibot 仓库执行。

本机路径：`/home/lisi/Documents/work/ros1/src/SCAN-Planner`；ROS 工作空间：`/home/lisi/Documents/work/ros1`。板子上请替换成实际路径（板子工作空间为 `/home/cat/ros1_ws`，SCAN-Planner 的具体位置以板子为准）。

## 1. 版本关系与改动

四个提交是连续的父子提交。记录时 SCAN-Planner HEAD 为 `307cd2c`（在本范围之后，见下文），工作区干净；四个提交均只在仿真中验证，尚未在 M20S 上测试。

```text
d67b5ae8b6d380b964f7ff447aad64a90c79840d  脱困改动前的基线
  └─ 255d30903427da6a5c3201376753a612eaef3b92
       └─ f342833e3f882bec6764207d4ccddbfca27ba869
            └─ b5d976c3f0269f6d5545bd732e811f5ec9c0240a
                 └─ d53990c7796fca85034aec5dcc2ce510511b87ad
                      └─ 307cd2cbab5e822999de45b0455fb2e97861c51e  关闭跳点（不属于本次脱困改动）
```

| 提交 | 时间（北京时间） | 改动与实机关注点 |
| --- | --- | --- |
| `255d30903427da6a5c3201376753a612eaef3b92` | 2026-10-10 10:56:03 | `feat(planner): stuck escape`。有目标且处于 GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ 时，10 s 内平移 < 0.15 m 且转向 < 0.5 rad，并且机身到原始障碍的间距 < `double_cylinder_radius + escape_margin`（默认 0.2 + 0.1 m），FSM 进入新状态 `ESCAPE`：在 1 m 内找最近的、间距 ≥ 阈值 + 到达容差、膨胀地图空闲且已观测、直线路径不比起点更靠近障碍的点，发 `/planning/escape_goal`。`closed_loop_controller` 保持航向，以 0.3 m/s 在机体系平移（可后退、可横移），到达 0.05 m 或 5 s 超时后停止；FSM 回到 GEN_NEW_TRAJ，重新开始 10 s 计时。新 bspline 或 `/planning/stop` 会结束平移。只在 `controller_mode:=closed_loop` 时启用。关注：是否误触发（正常转身、慢速通过窄处）；0.3 m/s 能否让 M20S 实际走动；后退/横移方向是否正确；未知格子过多导致"no reachable free spot"。 |
| `f342833e3f882bec6764207d4ccddbfca27ba869` | 2026-10-10 11:16:04 | `fix(controller): ignore escape goals issued before the last stop`。控制器记录收到 `/planning/stop` 的时刻，丢弃时间戳不晚于它的 escape_goal，防止"用户急停先到、脱困目标后到"时狗重新开始平移。依赖 FSM 与控制器在同一时钟上。关注：日志 `ignore escape goal issued ... before the last stop` 是否在正常流程里误出现（若出现，多半是两节点时钟不一致）。 |
| `b5d976c3f0269f6d5545bd732e811f5ec9c0240a` | 2026-10-10 11:30:00 | `feat(planner): watch the escape path while moving`。ESCAPE 期间 `checkCollisionCallback` 以 20 Hz 用实时地图复查剩余路径：逐点比较实时与搜索时快照的机身间距（封顶在所需间距），任一点下降超过 `fsm/escape_abort_eps`（0.08 m）即进入 EMERGENCY_STOP（`ESCAPE_SAFETY`），fail_safe 后回到 GEN_NEW_TRAJ。同时修复：脱困结束时把局部轨迹重新停放在当前位置，否则原有安全检查会立即报一次多余的 "Suddenly discovered obstacles" 急停。关注：有人或物体进入后退区域时能否及时停下；墙边体素闪烁是否导致误停。 |
| `d53990c7796fca85034aec5dcc2ce510511b87ad` | 2026-10-10 12:20:48 | `fix(planner): stop the escape when the robot drifts off its line`。补上偏离检查：当前位置在实时地图、实际航向下的间距，与原脱困直线上同一进度（投影）处规划时的间距比较，低于后者 0.08 m 以上同样 `ESCAPE_SAFETY`。用于侧滑、航向漂移、定位跳变时贴向一面没变的墙的情况。关注：腿式机器人后退时的正常侧摆是否导致误停（日志 `Off the escape line`）。 |

主要代码位于 `src/planner/plan_manage/`：`scan_replan_fsm.cpp`、`include/plan_manage/scan_replan_fsm.h`、`closed_loop_controller.cpp`、新增 `include/plan_manage/escape_search.h`，以及 `launch/run.launch`、`launch/advanced_param.xml`、`CMakeLists.txt`。四个提交还包括单元测试 `tests/escape_search_test.cpp`、闭环仿真 `tests/escape_sim.py`（`--intruder`、`--drift` 两种模式）和 `tests/README.md` 中"卡住脱困"一节。

**`307cd2c`（关闭跳点，`fsm/waypoint_skip_fail_count=100000`）不属于本次脱困改动**。下面任一回退方案都不会撤销它，跳点保持关闭；如需恢复跳点，另行 `git revert 307cd2c`。

`255d309` 还把 `run.launch` 中 `controller_mode` 参数的声明移到了 `advanced_param.xml` include 之前（声明内容和默认值不变），撤销后恢复原来的位置。

## 2. 先用开关关闭脱困（不改代码、不重新编译）

适用：需要快速对照"没有脱困"的旧行为。先取消导航、确认机器人停稳。

`run.launch` 没有可从命令行传入的脱困开关（`escape_enable` 由 `controller_mode` 推导），所以需要改一行 launch 文件。把 `src/planner/plan_manage/launch/run.launch` 中

```xml
    <arg name="escape_enable" value="$(eval arg('controller_mode') == 'closed_loop')" />
```

改为

```xml
    <arg name="escape_enable" value="false" />
```

然后**用原来完整的启动参数**重启规划器。launch 文件在节点启动时读取；catkin devel 空间直接使用源码目录下的 launch 文件，不需要重新编译（若板子使用 install 空间，则需要重新安装）。确认：

```bash
rosparam get /scan_planner_node/fsm/escape_enable   # 应为 false
```

关闭后 FSM 不再检测卡住、不再进入 ESCAPE；控制器里的脱困代码仍在，但不会收到 escape_goal。恢复时改回原值并重启。这个开关用于行为对照；若怀疑代码本身有问题，执行下面的 Git 回退。

只想调整脱困行为、不想关掉时，常用参数（同样需要重启）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `fsm/stuck_timeout` | 10.0 s | 卡住判定时长 |
| `fsm/escape_search_radius` | 1.0 m | 搜索半径 |
| `fsm/escape_margin` | 0.1 m | 在 `double_cylinder_radius` 之外要求的间距 |
| `fsm/escape_allow_unknown` | false | 是否允许目标落在未观测格子 |
| `fsm/escape_abort_eps` | 0.08 m | 路径监控 / 偏离检查的触发裕量 |
| `closed_loop_controller/escape_speed` | 0.3 m/s | 平移速度；调慢时 `escape_timeout` 要大于 `escape_search_radius / escape_speed` |
| launch arg `escape_timeout` | 5.0 s | FSM 与控制器共用 |
| launch arg `escape_tolerance` | 0.05 m | 到达距离，FSM 与控制器共用 |

## 3. 撤销提交（保留历史）

回退前先停稳并停止规划器，保存本轮日志、bag、路线及参数。在板子对应仓库检查：

```bash
cd /home/lisi/Documents/work/ros1/src/SCAN-Planner
git status --short
git log -6 --oneline
git rev-parse HEAD
# 为回退前版本建立备份分支；重复操作时换一个未使用的名字
git branch backup/scan-planner-before-escape-rollback-20261010
```

有本地改动时先单独保存，工作区干净后再回退。以下方案按问题选择一个执行，**不要依次执行多套命令**；撤销多个提交时必须**从新到旧**。

以下四个方案已于 2026-10-10 在 HEAD=`307cd2c` 的临时副本上实际执行过，均无冲突；方案 C 的结果另外编译通过，方案 A、B 的结果与之前编译过的提交代码完全相同。

### 方案 A：只撤销偏离检查（`d53990c`）

适用：后退时腿部正常侧摆被误判为偏离，频繁出现 `Off the escape line ... Emergency stop!`。

```bash
git revert --no-edit d53990c7796fca85034aec5dcc2ce510511b87ad
```

结果：`src/` 与 `include/` 代码与 `b5d976c` 相同。保留闯入检测（逐点比较），失去对侧滑贴墙的保护。也可以先尝试调大 `fsm/escape_abort_eps`（例如 0.12），不必回退。

### 方案 B：撤销路径监控（`d53990c` + `b5d976c`）

适用：路径监控本身频繁误停（`New obstacle on the escape path` 在没人时出现），而调大 `escape_abort_eps` 也无效。

```bash
git revert --no-edit d53990c7796fca85034aec5dcc2ce510511b87ad
git revert --no-edit b5d976c3f0269f6d5545bd732e811f5ec9c0240a
```

结果：代码与 `f342833` 相同。**注意两点**：脱困平移期间不再检查障碍（有人进入后退区域仍会继续走，最远 1 m）；同时撤销了"脱困结束时重新停放局部轨迹"的修复，每次脱困结束后会多出一次 "Suddenly discovered obstacles" 急停。不建议在有人的环境中长期这样运行，更稳妥的做法是用第 2 节的开关关掉脱困。

### 方案 C：只撤销"丢弃过期脱困目标"（`f342833`）

适用：正常流程中出现 `ignore escape goal issued ... before the last stop`，导致脱困目标被丢弃、狗不动（通常是 FSM 与控制器不在同一时钟上）。

```bash
git revert --no-edit f342833e3f882bec6764207d4ccddbfca27ba869
```

结果：其他三个提交保留，可以单独撤销。会重新出现"用户急停先到、过期的脱困目标后到，狗重新开始平移"的竞态（概率很低，需要急停恰好落在脱困开始的几毫秒内）。

### 方案 D：撤销全部脱困改动

```bash
git revert --no-edit d53990c7796fca85034aec5dcc2ce510511b87ad
git revert --no-edit b5d976c3f0269f6d5545bd732e811f5ec9c0240a
git revert --no-edit f342833e3f882bec6764207d4ccddbfca27ba869
git revert --no-edit 255d30903427da6a5c3201376753a612eaef3b92
```

如果之前已执行方案 A 或 B，只撤销其中尚未撤销的提交，顺序仍然从新到旧。

在原 HEAD 为 `307cd2c` 且没有其他改动的情况下，方案 D 的结果与基线 `d67b5ae` 相比应只差 `307cd2c` 的跳点参数（`advanced_param.xml` 中 `waypoint_skip_fail_count` 及其注释）：

```bash
git diff --stat d67b5ae8b6d380b964f7ff447aad64a90c79840d HEAD
# 预期只有：src/planner/plan_manage/launch/advanced_param.xml | 6 +++++-
```

方案 A、B 可以这样检查代码（无输出表示相同）：

```bash
# 方案 A
git diff --exit-code b5d976c3f0269f6d5545bd732e811f5ec9c0240a HEAD -- src/planner/plan_manage/src src/planner/plan_manage/include
# 方案 B
git diff --exit-code f342833e3f882bec6764207d4ccddbfca27ba869 HEAD -- src/planner/plan_manage/src src/planner/plan_manage/include
```

若仓库在 `307cd2c` 之后已有新提交，revert 会保留后续提交，可能产生冲突；出现冲突时可用 `git revert --abort` 取消当前这一次撤销，确认后续代码依赖后再处理。此时不能要求代码与旧提交完全一致，差异要逐项核对。

## 4. 编译、重启与确认

Git 回退只改变源码，**不会替换正在运行的进程或已有二进制**。在实际运行机器的工作空间中，用原来的 ROS 环境与构建方式重新编译，成功后再启动：

```bash
cd /home/lisi/Documents/work/ros1
source /opt/ros/noetic/setup.bash
catkin_make --pkg scan_planner -j4
source devel/setup.bash
rospack find scan_planner
```

本机使用 focal schroot，在工作空间根目录的对应命令为：

```bash
schroot -c focal -- bash -c 'source /opt/ros/noetic/setup.bash && catkin_make --pkg scan_planner -j4'
```

方案 D 之后，`advanced_param.xml` 不再声明 `escape_enable` / `escape_timeout` / `escape_tolerance` 参数，若上层 launch 或启动命令显式传了这些参数需删除。确认 `rospack find scan_planner` 指向本次编译的仓库，并确认：

```bash
rosparam get /scan_planner_node/fsm/escape_enable   # 方案 D 后应报参数不存在
rostopic info /planning/escape_goal                 # 方案 D 后应无发布者/订阅者
```

需要恢复被撤销的提交时，找到本次生成的 revert 提交，按与回退相反的顺序逐个 `git revert <撤销提交的SHA>`，再编译重启。

## 5. 实机测试留证

日志关键字（规划器 `<ROS_WS>/run/scan_planner.log` 与控制器输出）：

| 关键字 | 含义 |
| --- | --- |
| `[escape] Stuck for ... Moving ... (body frame dx=... dy=...)` | 判定卡住，开始脱困；dx < 0 为后退，dy 为横移 |
| `[STUCK]: from ... to ESCAPE` | 进入 ESCAPE 状态 |
| `[closed_loop_controller] escape to [...]` / `escape done` / `escape timed out` | 控制器开始 / 完成 / 超时 |
| `[escape] Reached escape target` / `Not at escape target after` | FSM 判定到达 / 超时，回到 GEN_NEW_TRAJ |
| `[escape] No progress ... not blocked by an obstacle` | 卡住但离障碍不近，不脱困 |
| `[escape] ... no reachable free spot` | 离障碍近但找不到可去的点（检查 `escape_allow_unknown`） |
| `[escape] New obstacle on the escape path` | 路径上出现新障碍，`ESCAPE_SAFETY` 急停 |
| `[escape] Off the escape line and closer to obstacles` | 偏离路线贴近障碍，`ESCAPE_SAFETY` 急停 |
| `ignore escape goal issued ... before the last stop` | 丢弃了过期的脱困目标 |

每轮记录以下信息：

- SCAN-Planner / navibot 的实际运行 commit、构建版本、完整启动参数（含 `controller_mode`）。
- 卡住时的场景（墙、窄通道、台阶边）、狗的朝向与目标方向。
- 脱困方向与距离、实际是否移动、是否误触发、急停是否及时。
- navibot 的 `tools/record_nav_bag.sh` 录制结果、规划器日志。重启前复制规划器日志，避免启动脚本覆盖。脱困期间控制器 CSV（`~/scan_planner_track_logs`）不记录数据，以 WARN 日志为准。

既有仿真结果（`tests/README.md`，运动学理想模型，非实机）：墙前 0.25 m 起步，后退 0.20 m 后间距 0.18 → 0.33 m，不重复触发；`--intruder` 约 0.11–0.16 s 急停，狗只多走约 1 cm；`--drift` 在间距 0.14 m（规划 0.23 m）时急停，未碰墙；正常转身约 5 s 未误触发（仿真 `stuck_timeout` 为 4 s）。单元测试 44 项通过。

| 测试日期/机器 | 实际 commit / 开关 | 场景 | 问题与日志位置 | 回退方案 / 结果 |
| --- | --- | --- | --- | --- |
| 待填写 | | | | |
