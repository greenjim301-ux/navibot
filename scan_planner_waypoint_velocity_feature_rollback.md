# SCAN-Planner 途中航点连续速度：实机测试与回退记录

记录日期：2026-10-09（北京时间）。本文保存在 navibot 仓库，以下 Git 操作均针对 **SCAN-Planner 仓库**，不要在 navibot 仓库执行。

本机路径：`/home/lisi/Documents/work/ros1/src/SCAN-Planner`；ROS 工作空间：`/home/lisi/Documents/work/ros1`。板子上请替换成实际路径。

## 1. 版本关系与改动

这两个提交是连续的父子提交，记录时 SCAN-Planner HEAD 为 `1e0f178`，工作区干净。

```text
bc9371b3563567f674b77634413d7ee1933e4f98  两次改动前的基线
  └─ 41fcd41926c9da8567f2be93a96dfd0e62cdca5e
       └─ 1e0f178559232e4875e74ad864f0b1b91f28a598
```

| 提交 | 时间（北京时间） | 改动与实机关注点 |
| --- | --- | --- |
| `41fcd41926c9da8567f2be93a96dfd0e62cdca5e` | 2026-10-08 10:49:01 | `feat(navigation): carry velocity through intermediate waypoints`。mode 2 途中航点采用非零终端速度，默认通过速度上限 0.5 m/s；按转角、段长和加速度限速，最终点、120°及以上转角等仍零速。重规划与局部目标保留终端速度；非共线边界速度添加参考约束点；障碍替代目标改为零速。闭环控制器允许非零速轨迹等待下一段最多 0.15 s，超时停车。关注接段停顿、转弯偏移、终点停车及接段超时。 |
| `1e0f178559232e4875e74ad864f0b1b91f28a598` | 2026-10-08 11:30:16 | `feat(navigation): keep cruise speed through straight mode-2 waypoints`。默认通过速度改为 0，表示跟随 `manager/max_vel`；增加按真实起止速度分配局部初始多项式时长，减少直线段中掉速；增加坡度阈值 0.15，超过阈值的入/出段保持航点零速。关注直线速度提高后的跟踪、制动、楼梯与坡道停点。 |

主要代码位于 `src/planner/plan_manage/`：`scan_replan_fsm.cpp`、`closed_loop_controller.cpp`、`planner_manager.cpp`、`waypoint_velocity.h` 及 `launch/run.launch`、`launch/advanced_param.xml`。两个提交也包含单元测试、仿真脚本及测试说明。

基线 `bc9371b` 已包含实机栅格地图射线原点切换到 `/hand_lio/odom_sensor` 的修复；撤销这两个提交不会撤销该修复。

## 2. 先用功能开关回到旧行为

适用：需要快速对照“途中点零速”的旧行为，不修改代码、不重新编译。先取消导航、确认机器人停稳，再重启规划器。**重启时保留原来的地图、定位话题、控制器等全部启动参数**，仅增加或替换：

```text
waypoint_continuous:=false
```

若使用仓库启停脚本，调用方式为：

```bash
cd /home/lisi/Documents/work/ros1/src/SCAN-Planner
# 下列命令需补齐本次实机原有的其他 roslaunch 参数后执行
./tools/scan_planner.sh restart navi_mode:=2 waypoint_continuous:=false
```

`run.launch` 会同步关闭三项：

| ROS 参数 | 关闭后的值 |
| --- | --- |
| `/scan_planner_node/fsm/waypoint_continuous` | `false` |
| `/scan_planner_node/manager/boundary_aware_time` | `false`（第二个提交新增） |
| `/closed_loop_controller/continuous_handoff` | `false` |

```bash
rosparam get /scan_planner_node/fsm/waypoint_continuous
rosparam get /scan_planner_node/manager/boundary_aware_time
rosparam get /closed_loop_controller/continuous_handoff
```

这些开关在节点初始化时读取，仅运行 `rosparam set` 不足以切换，必须重启。关闭功能用于行为对照；若怀疑代码本身有问题，执行下面的 Git 回退。

恢复连续模式：用完整原启动参数重启，设置 `waypoint_continuous:=true`。在 `1e0f178` 上，`waypoint_pass_speed:=0` 跟随 `manager/max_vel`；设置为 `0.5` 只能限制通过速度，不能完整恢复 `41fcd41` 的实现。

## 3. 撤销提交（保留历史）

回退前先停稳并停止规划器，保存本轮日志、bag、路线及参数。在板子对应仓库检查：

```bash
cd /home/lisi/Documents/work/ros1/src/SCAN-Planner
git status --short
git log -5 --oneline
git rev-parse HEAD
# 为回退前版本建立备份分支；重复操作时换一个未使用的名字
git branch backup/scan-planner-before-rollback-20261009
```

有本地改动时先单独保存，工作区干净后再回退。以下两个方案按问题选择一个执行，**不要依次执行两套命令**。

### 方案 A：只撤销第二个提交

保留第一版带速度接段，撤销巡航速度、边界时长及坡度判断的改动：

```bash
git revert --no-edit 1e0f178559232e4875e74ad864f0b1b91f28a598
```

默认通过速度恢复为 0.5 m/s；第二版新增的楼梯/坡度停点判断也会被撤销。实机启动参数不要继续显式传 `waypoint_pass_speed:=0`：第一版中它表示零通过速度，需省略该参数或设为 `0.5`。

### 方案 B：撤销两个提交

按从新到旧的顺序撤销，回到这两次改动之前的实现：

```bash
git revert --no-edit 1e0f178559232e4875e74ad864f0b1b91f28a598
git revert --no-edit 41fcd41926c9da8567f2be93a96dfd0e62cdca5e
```

如果之前已执行方案 A，这里只需撤销 `41fcd41`。如果仓库后续已有新提交，revert 会保留后续提交，可能产生冲突；出现冲突时可用 `git revert --abort` 取消当前这一次撤销，确认后续代码依赖后再处理。

在原 HEAD 正好为 `1e0f178` 且没有其他改动的情况下，方案 A 的代码树应与 `41fcd41` 相同，方案 B 应与 `bc9371b` 相同。按所选方案检查（无输出表示相同）：

```bash
# 方案 A
git diff --exit-code 41fcd41926c9da8567f2be93a96dfd0e62cdca5e HEAD
# 方案 B
git diff --exit-code bc9371b3563567f674b77634413d7ee1933e4f98 HEAD
```

若有后续提交，差异应逐项核对，不能要求整棵代码树与旧基线一致。

## 4. 编译、重启与确认

Git 回退只改变源码，**不会替换正在运行的进程或已有二进制**。在实际运行机器的工作空间中，用原来的 ROS 环境与构建方式重新编译，成功后再启动：

```bash
cd /home/lisi/Documents/work/ros1
source /opt/ros/noetic/setup.bash
catkin_make --pkg scan_planner -j4
source devel/setup.bash
rospack find scan_planner
```

本机现有测试说明使用 focal schroot，在工作空间根目录的对应命令为：

```bash
schroot -c focal -- bash -c 'source /opt/ros/noetic/setup.bash && catkin_make --pkg scan_planner -j4'
```

重新启动时使用原实机完整配置。方案 B 的旧 `run.launch` 不再声明 `waypoint_continuous` / `waypoint_pass_speed`，需从启动命令或上层 launch 中删除这两个新增参数，否则可能报 unused args。

确认 `rospack find scan_planner` 指向本次编译的仓库，运行进程来自本次工作空间；检查启动日志及实机直线、转弯、最终点停车表现。需要恢复原两个提交时，找到本次生成的 revert 提交，按与回退相反的顺序逐个 `git revert <撤销提交的SHA>`，再编译重启。

## 5. 实机测试留证

每轮记录以下信息，保持路线和速度配置一致做对照：

- SCAN-Planner / navibot 的实际运行 commit、构建版本、完整启动参数。
- 导航点坐标与点距、`manager/max_vel`、通过速度及连续模式开关。
- 途中点最低速度、接段停顿、转弯偏移、楼梯/坡道停点、最终点误差与停车情况。
- `trajectory handoff timed out` 等告警及发生时刻。
- navibot 的 `tools/record_nav_bag.sh` 录制结果、控制器 `~/scan_planner_track_logs`、规划器 `<ROS_WS>/run/scan_planner.log`。重启前复制规划器日志，避免启动脚本覆盖。

已有提交说明和 `src/planner/plan_manage/tests/README.md` 记录的理想仿真：1.5 m 点距、max_vel 0.9 m/s，途中点最低指令速度约为旧行为 0.77、第一版（pass 0.5）0.46、第二版 0.88–0.90 m/s。这些是既有本地仿真记录，本文未重新运行，也不代表实机结果；既有测试说明称当时尚未部署到板子、尚未完成 M20S 实机验证。

| 测试日期/机器 | 实际 commit / 开关 | 路线 / 速度 | 问题与日志位置 | 回退方案 / 结果 |
| --- | --- | --- | --- | --- |
| 待填写 | | | | |
