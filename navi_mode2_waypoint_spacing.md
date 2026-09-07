# 全局规划途经点间距 — 对着 SCAN-Planner 源码核对的结论

状态: **调研笔记, 还没动代码。** 起因是"global_planner 规划出来的路径点有的太靠近"。

对照的源码: `/home/lisi/Documents/work/ros1/src/SCAN-Planner`
(下面 `fsm.cpp` = `src/planner/plan_manage/src/scan_replan_fsm.cpp`,
`pm.cpp` = `src/planner/plan_manage/src/planner_manager.cpp`,
`param.xml` = `src/planner/plan_manage/launch/advanced_param.xml`)

本项目侧涉及的是 `backend/app/global_planner.py` 的 `_prune_path`(:309)。

---

## 0. 先更正两条之前的错误认识

这两条之前写在 `config.py` 的注释/对话里, 按源码核对是不对的:

**(a) mode 2 不做跨途经点的 min-snap。**

`presetWaypointsCallback` 只是把整串点存进 `active_waypoints_`, 然后
`planNextWaypoint()` 每次只规划 **odom → 当前这一个途经点**
(`fsm.cpp:281` 调 `planGlobalTraj`)。`planGlobalTraj`(`pm.cpp:411`)里:

- 只有当这一段距离 > `dist_thresh = 4.0`(`pm.cpp:423`)才**均匀**插中间点;
- 插完点数 >= 3 才走 `minSnapTraj`(`pm.cpp:463`), 否则走
  `one_segment_traj_gen`(`pm.cpp:465`)。

也就是说用户下发的途经点**从来不会一起进同一个 min-snap**, 段与段之间是
独立规划的。跨途经点的多点 min-snap(`planGlobalTrajWaypoints`, `pm.cpp:329`)
只有 navi_mode=3 走。

→ 推论: memory 里那条 "navi_mode=2 异常 goal 疑似间距悬殊致 min-snap 过冲"
的猜想按这个实现站不住。那条 bug 的排查记在 memory 里, 不在本文范围内。

**(b) "太近"的真正阈值是 0.2m, 不是 0.3m。**

三个常量是三件不同的事, 之前混用了:

| 常量 | 值 | 位置 | 语义 |
|---|---|---|---|
| `reboundReplan` 的距离门槛 | **0.2m** | `pm.cpp:94` | `(start_pt - local_target_pt).norm() < 0.2` → 返回 `TOO_CLOSE_TO_GOAL`, **不生成任何轨迹** |
| `waypoint_arrival_radius_` | 0.3m | `fsm.cpp:35`, `param.xml:41` | EXEC_TRAJ 里 `(end_pt_ - odom_pos_).norm() <` 它就**提前**切下一个点(`fsm.cpp:709`), 不等轨迹跑完 |
| `kDegenerateDist` | 0.05m | `fsm.cpp:263` | 跟 **odom 当前位置**比(不是跟前一个途经点比), 而且是 `while` 循环, 一口气吞掉一整簇重合点 |

本项目 `config.py` 里的 `REACH_EPS_M=0.3` / `DEGENERATE_DIST_M=0.05` 对齐的是
后两条, 都没有对应到 0.2 这条真正的"规划不出来"的门槛。

---

## 1. 核心发现: 途经点间距 == 局部规划器的前瞻距离

这是"点太密"最实质的代价, 也是整份笔记的重点。

`getLocalTarget()`(`fsm.cpp:1055`)的逻辑:

```
local_target_pt_ = end_pt_;                       // fsm.cpp:1080  默认就是当前途经点
for (t = t_proj; t < duration; t += t_step) {
    total_dist += ...;
    if (total_dist >= planning_horizon_) {        // fsm.cpp:1086
        local_target_pt_ = pos_t; break;          // 只有全局轨迹够长才往前推
    }
}
```

`planning_horizon_` = **5.0m**(`param.xml:18`)。

而 mode 2 的全局轨迹只到下一个途经点 —— 所以 **`local_target_pt_` 永远不会
超过下一个途经点, 前瞻被途经点间距硬截断**:

- 间距 5m+ → 前瞻满格 5m, planner 按自己的能力跑;
- 间距 1m → B 样条优化器只能看 1m, 避障和平滑都在这个窗口里做;
- 间距 0.5m → 只能看 0.5m;
- 间距 < 0.2m → 连轨迹都不生成, 直接 `TOO_CLOSE_TO_GOAL` → 切下一个点
  (`fsm.cpp:643-655`), 白跑一轮 100Hz 的 FSM(`exec_timer_` 周期 0.01s,
  `fsm.cpp:49`)。

**结论: 途经点不是"越贴合规划路径越好", 而是越稀疏越好。** 密集途经点等于
主动把 SCAN-Planner 的 5m 前瞻废掉。

## 2. 稀疏化安全吗: 碰不到, 但软余量会丢

**碰撞层面是安全的。** `_prune_path` 的 `_line_free` 已经保证保留点之间的直连
无障碍(判据是按机身半径的**硬膨胀**), 而 planner 在两个保留点之间走的
`one_segment_traj_gen` 几何上基本就是直线 —— 走的正是我们检查过的那条线。
稀疏化的安全边界就是 LOS 检查本身, 不需要额外的"密度保险"。

**但 0.3m 软余量会丢。** `_line_free` 不含 `e1f919d` 加的那条 0.3m 贴墙惩罚带,
所以一条"合法"的捷径完全可以贴着硬膨胀边界走 —— 碰不到, 但余量归零。详见
第 5 节。

**而且局部规划器补不回来。** 本来以为 SCAN-Planner 的 B 样条优化器自己会保持
离墙距离, 那样 A* 这层的贴墙偏好就是冗余的、丢了无所谓。**核对下来不是这样**:

- `optimization/dist0 = 0.2`(`param.xml:95`)确实是个 0.2m 净空目标,
  `calcDistanceCostRebound`(`bspline_optimizer.cpp:396`)里
  `dist_err = clearance - dist`, 靠近了就罚;
- **但这个罚项需要 `base_point`/`direction` 对, 而这些对只在轨迹真的进入膨胀
  障碍时才生成**: `initControlPoints`(`bspline_optimizer.cpp:57`)明写
  "Segment the initial trajectory **according to obstacles**", 只有
  `getInflateOccupancy` 为真的区段才切出 `in_id`/`out_id`;
  `check_collision_and_rebound`(`:718`)同样只在 `occ` 为真时补新的对。

所以它是个**碰撞逃逸**项, 不是通用的"保持距离"项: 撞进膨胀区它把你推出来
(以 0.2m 为目标余量), 但一条**擦着膨胀边界、没进去**的轨迹它一个字都不说。
再加上 `lambda_fitness = 1.0`(`param.xml:94`)会把 B 样条往初始那条多项式
(= 我们两点之间的直线)上拉。

→ **两个稀疏途经点之间, 狗走的基本就是直线, 离墙多远完全由这条直线决定。**

现在多出来的密集点不是几何上需要的, 是 `_prune_path` 贪心循环卡住的产物
(见第 3 节)—— 那部分可以放心删, 跟上面这个取舍无关。

---

## 3. 本项目侧: 密集点是怎么产生的

`_prune_path`(`global_planner.py:309`)的贪心 LOS 循环, 内层一旦 `break`
(被 `_line_free` 挡住, 或代价比较不过), 就只能 `anchor += 1`, 输出两个
**相邻栅格**的点 —— 5cm 分辨率下就是 0.05m / 对角 0.0707m。

注意这个间距恰好卡在最坏位置: 大于 `kDegenerateDist`(0.05, 而且那条是跟
odom 比的)所以不会被跳过, 又远小于 0.2 所以生成不了轨迹。而且每撞一次
`TOO_CLOSE_TO_GOAL` 都会 `continuous_failures_count_++`(`pm.cpp:97`),
这个计数器**只有 `reboundReplan` 完整成功才清零**(`pm.cpp:312`)—— 又一条
"别产生密集点"的理由。

两个诱因:

1. **代价容差是纯乘性的。** `_PRUNE_COST_TOLERANCE = 1.02`
   (`global_planner.py:306`)乘在 `raw_cost` 上, 段越短、`raw_cost` 越小,
   能容忍的绝对差就越小 → 短段特别容易被判"更贵"而 break。这正是产生
   密集点的场景。
2. **`break` 语义过强。** 代价检查不通过就 `break`, 但代价场沿路径不是单调的,
   某个 j 变贵不代表更远的 j 也贵。

---

## 4. 待改方案(还没实现)

按优先级:

**A. 代价容差加绝对松弛项** — 把
`line_cost <= raw_cost * 1.02`
改成
`line_cost <= raw_cost * 1.02 + abs_slack`(abs_slack 折合几十厘米的代价量级)。
消掉尺度偏差, 这是根因。也是让长直线段能一路拉直的关键一环。

**B. 修 `break` 语义** — 代价检查不通过时 `continue` 继续往后扫, 只记录
"最后一个两项检查都通过的 j"; `_line_free` 的 break 保留(那个近似单调)。

**C. 输出后压簇, 但只当兜底安全网** — 遍历剪枝结果, 把间距小于 `d_min` 的
一簇点压成一个代表点, 代表点取簇内**转角最大**的那个(真正的拐点), 压完用
`_line_free` 复核压缩后那一段仍然可走。

  - **`d_min` 取 0.25~0.3m, 不要取米级。** 依据是 `pm.cpp:94` 那条 0.2m 硬线
    (低于它 `reboundReplan` 直接返回 `TOO_CLOSE_TO_GOAL`, 根本不生成轨迹),
    留一点余量即可。**上界不需要** —— 间距不该由这个参数决定, 而该由 A/B
    修好之后的代价门槛和几何决定。理由见第 5 节。
    (早先版本这里写的是"米级 2~3m", **是错的**, 那等于绕过代价门槛砍掉
    `e1f919d` 的贴墙偏好。)
  - 直线走廊上无论多长都只留首尾两点 —— 但这是 A/B 的功劳, 不是压簇的。
  - **终点必须原样保留** —— 它是唯一有 `/planning/finished` 精确到达判定的点
    (中途点只有 0.3m 提前切换, 见第 0(b) 节)。
  - 起点如果离狗当前位置 < 0.05m 可以直接丢(planner 侧 `fsm.cpp:265` 反正
    也会跳过)。

**D. 明确不做: 补最大间距 / 长段插点。** 之前想过为了让间距均匀而在长直线段
上插点, 按第 0(a) 节这是错的 —— mode 2 没有跨途经点的 min-snap, 均匀间距
没有收益; 而插点会白白截短前瞻, 是净损失。长段超过 4m 时 planner 自己会在
`planGlobalTraj` 里均匀插点(`pm.cpp:423-441`), 不需要我们代劳。

---

## 5. 跟 e1f919d(贴墙偏好)的关系

`e1f919d` 加的 `_wall_clearance_weight` 是在硬膨胀之外再叠一条 0.3m 的软惩罚带
(`GLOBAL_PLANNER_WALL_CLEARANCE_M`, 倍率 3.0), 让 A* 在有余量时偏向远离墙。
**这个偏好只存在于代价场里**, A* 走完就体现为路径的形状; 而 planner 只拿到
保留下来的那几个点, 两点之间走直线。所以:

> A* 的路线形状, 只能通过"保留哪些点"这一个通道传给 planner。

`_prune_path` 里唯一阻止捷径吃掉软余量的东西, 就是 `_line_cost` 那道代价门槛
(`_line_free` 只管硬膨胀)。**米级 `d_min` 压簇会绕过这道门槛 —— 这就是冲突。**

### 但冲突比想的窄

想清楚 A* 在软代价场里实际产出什么形状:

- **长直走廊、墙在一侧**: 软带让整条路径**平移** 0.3m, 但它还是**一条直线**。
  直线上任意两点的连线代价跟原路径一样 → 代价门槛通过 → 照样剪成首尾两点。
  **偏移量完整保留, 点数也没增加。**(跟当时那个合成地图测试 8→4 点、bulge
  保住了 是一致的。)
- **拐角、门洞、绕柱子**: 这里才会出现真正的弧, 弦的代价明显更高 → 门槛拒绝
  → 保留点。但这些地方**本来就该留点**(几何上是真拐点)。

也就是说, 软带主要表现为"整条路径的横向偏移", 而不是"需要很多点才能描述的
高曲率弧"。**偏移靠端点位置传达, 不靠点密度。**

### 结论: A/B 跟 e1f919d 不冲突, 米级 C 才冲突

- **A + B 修的是"门槛因为数值尺度问题误判"这个 bug**, 修完门槛判得更准:
  直线段该拉直的拉直、拐角该留的留。密集点是这个 bug 的产物, **不是**贴墙
  偏好的必要代价。两者完全不冲突。
- **前瞻长度问题靠 A + B 天然解决**: 直线走廊剪成首尾两点, 间距就是走廊长度,
  远超 5m 的 `planning_horizon`, 前瞻直接拉满。**不需要为了前瞻牺牲贴墙偏好。**
- 米级压簇是在用钝刀砍掉门槛刚判对的东西, 所以 C 降级成 0.25~0.3m 的兜底。

### 万一之后两边都要

如果实测下来拐角附近还是点太密、前瞻被压得难受, **不要加大 `d_min`**, 换传达
方式: 保留点照样少留, 但把每个保留点**主动往远离墙的方向推**(在小半径内找
局部净空最大的位置)。直线由端点决定, 锚点离墙整条直线就离墙 —— 用**锚点
位置**而不是**点密度**承载这个偏好, 跟米级间距完全兼容。(未实现, 也未验证。)

---

## 6. 未决 / 待验证

- 以上全部是**读源码**得出的, 没有实机验证(没有 board 访问权限)。
- **`d_min` 那条已经不是未决项了** —— 见第 4 节 C 和第 5 节, 它有硬依据
  (0.2m 那条线)且不该当成自由参数调。
- **真正还需要实机定的只剩 A 里的 `abs_slack`。** 可以给个起点: 软带倍率 3.0、
  带宽 0.3m, 所以一段捷径贴墙走 1m 相对于绕开的额外代价量级约是
  `1m × (3-1) = 2`(以格代价计); `abs_slack` 取这个量级的十分之一(折合
  几十厘米的绕行)应该能既滤掉数值噪声、又不放过真正贴墙的捷径。**未验证。**
- 第 5 节最后那个"把保留点推离墙"的方案没实现也没验证, 只是备选。
- `param.xml` 里的 `planning_horizon=5.0` / `max_vel=0.75` / `max_acc=2.0`
  是仓库默认值, 实机上跑的 launch 是否覆盖过没有核对。
