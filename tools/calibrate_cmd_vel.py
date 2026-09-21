#!/usr/bin/env python3
"""标定 /cmd_vel: 量"我发了多少速度"到"机器狗实际走了多少"的传递关系。

x / y / yaw 三个轴分别标, 每轴给出**比例、死区、正反不对称、上升时间**, 外加一个
3x3 的**轴间耦合**矩阵(命令纯 x 时冒出多少 y 和 yaw)。

标定的是**整条链路**, 不拆开:

    closed_loop_controller ──/cmd_vel──> deep_bridge ──UDP──> 本体 ──> 实际运动
                                              │
                            max_v* 夹限 → ToRatio(v/full_scale_v*) [usage_mode=0]
                                         或 直接下发 m/s           [usage_mode=1]

中间至少四处可能藏比例(夹限、满量程换算、本体自己的解释、步态死区), 逐个拆既不
现实也没必要 —— 闭环控制器本来就是拿 /hand_lio/odom_vehicle 闭环的, 让这两端一致
才是目标。真值就用那个 odom。

**注意这标的是"cmd_vel 对 odom"**: 如果 odom 自己有尺度误差, 标完之后狗的实际位移
仍然是错的, 而且从这份数据里完全看不出来。独立核对一次的办法在最后的报告里会提示
(卷尺量 2~3 次), 十分钟的事, 能把"cmd_vel 的问题"和"odom 的问题"分开。

用法(**必须在板子上跑**, 理由见 preflight):

    systemctl stop navi_planner          # 必须! 否则两个发布者抢 /cmd_vel
    python3 tools/calibrate_cmd_vel.py --axis yaw            # 先只跑 yaw 最稳妥
    python3 tools/calibrate_cmd_vel.py --axis x --axis y --axis yaw --out cal.json

yaw 轴顺带能估出 **lidar_t_body 杆臂** —— 原地旋转时如果雷达不在机体中心, odom 的
位置会画一个圆, 半径就是杆臂长度。那个外参现在还是占位零向量, 白捡的。
"""
import argparse
import json
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# 跟 backend/app/config.py 的 POSE_COV_BAD、hand_lio.yaml 的 pose_cov_reject_thresh
# 是同一套约定: covariance[0] >= 0.99 表示这一帧定位失败, x/y/yaw 不可信。
# 这里独立定义一份而不是 import backend —— tools 是离线脚本, 不依赖 backend 包。
POSE_COV_BAD = 0.99

# 每个轴的默认扫描幅值。**必须跨过死区**, 所以下界附近点得密。
# vendor_lower 取自 deep_bridge.yaml 里厂商《各个步态的有效速度范围》表的
# 0x1001 标准-基础那一行(X [0.2,2.0] / Y [0.35,1.0] / Yaw [0.5,2.0]) ——
# **换步态这三个数就不对了**, 用 --vendor-lower 覆盖。它只用来决定"拟合比例时从
# 哪个幅值往上取", 不影响采样。
AXES = {
    "x":   {"unit": "m/s",   "vendor_lower": 0.20,
            "amps": [0.05, 0.10, 0.15, 0.18, 0.20, 0.22, 0.30, 0.40, 0.55, 0.75]},
    "y":   {"unit": "m/s",   "vendor_lower": 0.35,
            "amps": [0.10, 0.20, 0.30, 0.33, 0.35, 0.38, 0.45, 0.60]},
    "yaw": {"unit": "rad/s", "vendor_lower": 0.50,
            "amps": [0.10, 0.25, 0.40, 0.48, 0.50, 0.53, 0.70, 1.00]},
}


def make_twist(axis: str, value: float) -> Twist:
    """只给一个轴赋值, 另外两个严格是 0 —— 轴间耦合是要测的量, 不能自己先掺进去。"""
    cmd = Twist()
    if axis == "x":
        cmd.linear.x = value
    elif axis == "y":
        cmd.linear.y = value
    elif axis == "yaw":
        cmd.angular.z = value
    else:
        raise ValueError("未知轴: %r" % axis)
    return cmd


class OdomBuffer:
    """订阅 odom, 存最近一段时间的位姿。

    只存, 不在回调里做任何计算 —— 回调跑在 rospy 的接收线程上, 算东西会拖慢接收。
    """

    def __init__(self, topic: str, keep_sec: float = 60.0):
        self.keep_sec = keep_sec
        self._samples: List[Tuple[float, float, float, float, float]] = []
        self._twist_seen = False
        self._sub = rospy.Subscriber(topic, Odometry, self._cb,
                                     queue_size=200, tcp_nodelay=True)

    def _cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        t = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        cov = msg.pose.covariance[0]
        self._samples.append((t, p.x, p.y, yaw, cov))
        tw = msg.twist.twist
        if abs(tw.linear.x) + abs(tw.linear.y) + abs(tw.angular.z) > 1e-9:
            self._twist_seen = True
        cutoff = t - self.keep_sec
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.pop(0)

    @property
    def twist_seen(self) -> bool:
        """odom 里的 twist 字段有没有被填过。没填的话报告里注明"只能靠位置拟合"。"""
        return self._twist_seen

    def window(self, t0: float, t1: float) -> np.ndarray:
        """取 [t0, t1] 区间的样本, 返回 (N, 5) 的 t/x/y/yaw/cov。"""
        rows = [s for s in self._samples if t0 <= s[0] <= t1]
        return np.array(rows, dtype=float).reshape(-1, 5)

    def latest(self) -> Optional[Tuple[float, float, float, float, float]]:
        return self._samples[-1] if self._samples else None


def fit_line(t: np.ndarray, v: np.ndarray) -> Tuple[float, float]:
    """最小二乘拟合 v = k*t + b, 返回 (斜率, R²)。

    **不对位置做差分** —— 差分会把 odom 的位置噪声放大成速度噪声。拟合一条直线
    既压噪声, 又顺带给出 R²: 拟合不好说明这一段根本不是匀速(还在加速、或者打滑),
    调用方据此把整段丢掉, 而不是硬算出一个没意义的平均值。
    """
    if len(t) < 3:
        return float("nan"), 0.0
    k, b = np.polyfit(t, v, 1)
    resid = v - (k * t + b)
    ss_tot = float(((v - v.mean()) ** 2).sum())
    r2 = 1.0 - float((resid ** 2).sum()) / ss_tot if ss_tot > 1e-12 else 1.0
    return float(k), r2


def measure_window(win: np.ndarray) -> Optional[Dict[str, float]]:
    """从一段 odom 样本里解出机体系的 (vx, vy, vyaw) 和各自的拟合质量。

    先把世界系位置转到**窗口起始时刻的机体系**, 再对 x_body(t) / y_body(t) 各拟合
    一条直线 —— 一次拟合同时拿到主轴速度**和横向耦合**, 不用另外算。
    yaw 要先 unwrap, 否则跨 ±π 时斜率会炸。
    """
    if len(win) < 5:
        return None
    if float(win[:, 4].max()) >= POSE_COV_BAD:
        return None  # 窗口里有定位失败帧, 整段不要

    t = win[:, 0] - win[0, 0]
    yaw0 = win[0, 3]
    dx, dy = win[:, 1] - win[0, 1], win[:, 2] - win[0, 2]
    c, s = math.cos(yaw0), math.sin(yaw0)
    x_b = c * dx + s * dy
    y_b = -s * dx + c * dy
    yaw_u = np.unwrap(win[:, 3])

    vx, r2x = fit_line(t, x_b)
    vy, r2y = fit_line(t, y_b)
    vyaw, r2yaw = fit_line(t, yaw_u)
    return {"vx": vx, "vy": vy, "vyaw": vyaw,
            "r2_x": r2x, "r2_y": r2y, "r2_yaw": r2yaw,
            "n": int(len(win)), "duration": float(t[-1])}


class Calibrator:
    def __init__(self, args):
        self.args = args
        self.pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=10)
        self.odom = OdomBuffer(args.odom_topic)
        self.runs: List[Dict] = []
        self.started_at = time.time()

    # ---- 安全 ----
    def publish_zero(self, seconds: float) -> None:
        """连续发零。**必须发够时间**, 不能只发一帧 —— deep_bridge 的看门狗
        (cmd_timeout_sec 默认 0.5s)是靠"停发"兜底的, 但停发之前最后那一帧要是
        非零, 在看门狗超时之前狗还会按它走。发够 1s 以上, 确保零指令真的到了。"""
        rate = rospy.Rate(self.args.rate)
        end = time.time() + seconds
        zero = Twist()
        while time.time() < end and not rospy.is_shutdown():
            self.pub.publish(zero)
            rate.sleep()

    def check_budget(self) -> None:
        if time.time() - self.started_at > self.args.max_runtime:
            raise RuntimeError("超过 --max-runtime %.0fs, 主动中止" % self.args.max_runtime)

    def wait_stationary(self, timeout: float = 8.0) -> bool:
        """等狗真的停稳。上一轮的余速没退干净就开下一轮, 测出来的稳态是偏的。"""
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            win = self.odom.window(time.time() - 0.7, time.time() + 1.0)
            if len(win) >= 5:
                span_xy = float(np.hypot(win[:, 1] - win[:, 1].mean(),
                                         win[:, 2] - win[:, 2].mean()).max())
                span_yaw = float(np.ptp(np.unwrap(win[:, 3])))
                if span_xy < self.args.still_xy and span_yaw < self.args.still_yaw:
                    return True
            self.publish_zero(0.2)
        return False

    # ---- 单次 run ----
    def run_step(self, axis: str, value: float) -> Optional[Dict]:
        """一次阶跃: 静止确认 → 阶跃保持 → 取稳态窗口 → 回零。"""
        self.check_budget()
        if not self.wait_stationary():
            rospy.logwarn("等不到静止, 跳过 %s=%+.3f", axis, value)
            return None

        rate = rospy.Rate(self.args.rate)
        cmd = make_twist(axis, value)
        t_step = time.time()
        while time.time() - t_step < self.args.hold and not rospy.is_shutdown():
            self.pub.publish(cmd)
            rate.sleep()
        self.publish_zero(self.args.rest)

        # 丢掉前 settle 秒的暂态, 只用后半段算稳态
        win = self.odom.window(t_step + self.args.settle, t_step + self.args.hold)
        meas = measure_window(win)
        if meas is None:
            rospy.logwarn("%s=%+.3f 的窗口取不到有效样本(样本太少或定位失败)", axis, value)
            return None

        # 上升时间: 从阶跃开始到速度达到稳态 90% 所用的时间。用整段(含暂态)的
        # 滑窗拟合估, 精度有限但够判断 --settle 给够了没有。
        rise = self.estimate_rise_time(axis, t_step, meas)
        # 记下窗口的绝对时间: 杆臂估计要按轴回头取这几段 odom, 不能拿整个缓冲区
        # (那里面混着 x/y 的直线段, 拟合圆会得到一个没意义的巨大半径)。
        run = {"axis": axis, "cmd": value, "rise_time": rise,
               "t_start": t_step + self.args.settle, "t_end": t_step + self.args.hold}
        run.update(meas)
        # 主轴 / 两个耦合轴分别记下来, 后面分析不用再判断是哪个轴
        run["main"] = {"x": meas["vx"], "y": meas["vy"], "yaw": meas["vyaw"]}[axis]
        run["r2_main"] = {"x": meas["r2_x"], "y": meas["r2_y"], "yaw": meas["r2_yaw"]}[axis]
        self.runs.append(run)
        flag = "" if run["r2_main"] >= self.args.min_r2 else "  ← R² 偏低, 分析时会剔除"
        print("    %-3s cmd=%+.3f -> 实测 %+.4f %s (R²=%.3f, n=%d)%s"
              % (axis, value, run["main"], AXES[axis]["unit"], run["r2_main"], meas["n"], flag))
        return run

    def estimate_rise_time(self, axis: str, t_step: float, steady: Dict[str, float]) -> float:
        """粗估上升时间: 在暂态段上滑窗拟合速度, 找第一次到稳态 90% 的时刻。"""
        target = abs(steady[{"x": "vx", "y": "vy", "yaw": "vyaw"}[axis]]) * 0.9
        if target < 1e-6:
            return float("nan")
        step = 0.1
        w = 0.3
        t = 0.0
        while t + w <= self.args.hold:
            win = self.odom.window(t_step + t, t_step + t + w)
            m = measure_window(win)
            if m is not None:
                v = abs(m[{"x": "vx", "y": "vy", "yaw": "vyaw"}[axis]])
                if v >= target:
                    return round(t + w / 2, 3)
            t += step
        return float("nan")

    # ---- 慢斜坡(给人看一张图, 不参与拟合) ----
    def run_ramp(self, axis: str) -> Dict:
        """很慢地从 0 升到最大幅值, 记原始曲线。阶跃给精确数值, 斜坡给一张能一眼
        看出死区形状的图, 两者互为佐证。"""
        self.check_budget()
        self.wait_stationary()
        amps = AXES[axis]["amps"]
        vmax = max(amps)
        rate = rospy.Rate(self.args.rate)
        t0 = time.time()
        cmds: List[Tuple[float, float]] = []
        while True:
            el = time.time() - t0
            if el > self.args.ramp_duration or rospy.is_shutdown():
                break
            u = vmax * el / self.args.ramp_duration
            self.pub.publish(make_twist(axis, u))
            cmds.append((round(el, 3), round(u, 4)))
            rate.sleep()
        self.publish_zero(self.args.rest)
        win = self.odom.window(t0, t0 + self.args.ramp_duration)
        return {"axis": axis, "duration": self.args.ramp_duration,
                "commands": cmds,
                "odom": [[round(r[0] - t0, 3), round(r[1], 4), round(r[2], 4),
                          round(r[3], 5)] for r in win.tolist()]}


def preflight(args) -> Dict:
    """开跑之前必须过的几关。任何一关不过就直接退出 —— 带着错误的前提跑完整套,
    数据全是废的, 而且要几十分钟才能发现。"""
    info: Dict[str, object] = {}
    cmd_topic = rospy.resolve_name(args.cmd_topic)

    # ① 有没有别人也在发 /cmd_vel。closed_loop_controller 以 100Hz 发同一个话题,
    #    两个发布者交织 = 数据全废。而且它"收到过 bspline 之后就不再依赖规划器,
    #    会继续按最后那条轨迹发"(advanced_param.xml 的注释), 所以得整组停掉:
    #    systemctl stop navi_planner, 别只 kill 规划器节点。
    others: List[str] = []
    try:
        # getSystemState() -> (code, msg, [publishers, subscribers, services]),
        # publishers 是 [[topic, [node, ...]], ...]
        publishers = rospy.get_master().getSystemState()[2][0]
        for topic, pubs in publishers:
            if topic == cmd_topic:
                others = [n for n in pubs if n != rospy.get_name()]
    except Exception as e:  # noqa: BLE001 - 查不到就跳过这项检查, 不该因此不让跑
        rospy.logwarn("查 %s 的发布者失败(%s), 跳过这项检查", cmd_topic, e)
    info["cmd_topic"] = cmd_topic
    info["other_publishers"] = others
    if others and not args.force:
        raise SystemExit(
            "还有别的节点在发 %s: %s\n"
            "  最常见的是 SCAN-Planner 的 closed_loop_controller —— 先\n"
            "    systemctl stop navi_planner\n"
            "  再跑。确实想带着它一起跑就加 --force(数据会被污染, 不建议)。"
            % (cmd_topic, ", ".join(others))
        )

    # ② odom 有没有在来, 新不新鲜, 定位好不好
    print("  等 %s ..." % args.odom_topic)
    deadline = time.time() + 10.0
    buf_ready = False
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message(args.odom_topic, Odometry, timeout=2.0)
        except rospy.ROSException:
            continue
        cov = msg.pose.covariance[0]
        info["odom_cov"] = float(cov)
        if cov >= POSE_COV_BAD:
            raise SystemExit(
                "odom 的 covariance[0]=%.3f >= %.2f, 定位处于失败状态, 位姿不可信 "
                "—— 先把定位搞好再标定。" % (cov, POSE_COV_BAD))
        buf_ready = True
        break
    if not buf_ready:
        raise SystemExit("10s 内没收到 %s, 检查 hand_lio 在不在跑。" % args.odom_topic)
    return info


def analyse(runs: List[Dict], args) -> Dict:
    """按轴 + 正负分组拟合。

    模型: 幅值大于 vendor_lower 的那些点上拟合 v = k*u + b, 死区 d = -b/k。
    为什么不把小幅值也放进去一起拟: 死区里的行为本来就不是线性的(厂商表只给了
    "区间下界", 下界以内本体怎么处理**没有说明**, deep_bridge 也没做处理),
    混进去只会把比例也带歪。小幅值的点原样保留在报告里, 让人自己看死区长什么样。
    """
    out: Dict[str, Dict] = {}
    for axis in AXES:
        axis_runs = [r for r in runs if r["axis"] == axis and r["r2_main"] >= args.min_r2]
        if not axis_runs:
            continue
        lower = args.vendor_lower.get(axis, AXES[axis]["vendor_lower"])
        entry: Dict[str, object] = {"unit": AXES[axis]["unit"], "vendor_lower": lower,
                                    "n_runs": len(axis_runs)}
        for sign, name in ((1, "pos"), (-1, "neg")):
            pts = [r for r in axis_runs
                   if math.copysign(1, r["cmd"]) == sign and abs(r["cmd"]) >= lower]
            if len(pts) < 2:
                entry[name] = None
                continue
            u = np.array([r["cmd"] for r in pts])
            v = np.array([r["main"] for r in pts])
            k, b = np.polyfit(u, v, 1)
            resid = v - (k * u + b)
            ss = float(((v - v.mean()) ** 2).sum())
            entry[name] = {
                "scale": float(k),
                "intercept": float(b),
                # 直线跟 v=0 的交点 = 有效死区。跟厂商表的下界对不对得上, 是这次
                # 标定最想回答的问题之一(deep_bridge.yaml 里那条悬案)。
                # 拟合直线跟 v=0 的交点 —— 正向分支为正、反向分支为负, 合起来就是
                # "命令落在 [deadzone_neg, deadzone_pos] 之间狗不动"这个区间。
                "deadzone": float(-b / k) if abs(k) > 1e-6 else float("nan"),
                "fit_r2": 1.0 - float((resid ** 2).sum()) / ss if ss > 1e-12 else 1.0,
                "n": len(pts),
            }
        # 正反不对称
        p, n = entry.get("pos"), entry.get("neg")
        if isinstance(p, dict) and isinstance(n, dict) and abs(p["scale"]) > 1e-6:
            entry["asymmetry"] = float((n["scale"] - p["scale"]) / p["scale"])
        # 上升时间
        rises = [r["rise_time"] for r in axis_runs if r["rise_time"] == r["rise_time"]]
        entry["rise_time_median"] = float(np.median(rises)) if rises else None
        out[axis] = entry

    # 轴间耦合: 命令某个轴时, 另外两个轴冒出来多少(按命令幅值归一)。腿式底盘上
    # "命令纯 x 却持续偏航"是很常见的, 而那正好是"狗自己往路边靠"的直接成因。
    coupling: Dict[str, Dict[str, float]] = {}
    for axis in AXES:
        big = [r for r in runs
               if r["axis"] == axis and r["r2_main"] >= args.min_r2
               and abs(r["cmd"]) >= args.vendor_lower.get(axis, AXES[axis]["vendor_lower"])]
        if not big:
            continue
        coupling[axis] = {}
        for out_axis, key in (("x", "vx"), ("y", "vy"), ("yaw", "vyaw")):
            ratios = [r[key] / r["cmd"] for r in big if abs(r["cmd"]) > 1e-6]
            coupling[axis][out_axis] = float(np.mean(ratios)) if ratios else float("nan")
    return {"per_axis": out, "coupling": coupling}


def estimate_lever_arm(runs: List[Dict], odom: OdomBuffer, args) -> Optional[Dict]:
    """从 yaw 原地旋转的 odom 轨迹估 lidar 相对机体中心的杆臂。

    原地纯 yaw 旋转时, 如果雷达不在机体中心, odom 的 xy 会画一个圆, **半径就是
    杆臂长度**。`hand_lio.yaml` 的 lidar_t_body 现在还是占位零向量(那份 yaml 自己
    注明"装好后必须自己标定"), 这里是白捡的。

    注意反过来也成立: **yaw 标定时 x/y 方向的"耦合"有一部分是杆臂造成的假象**,
    不是真的平移耦合 —— 分析耦合矩阵时要记得这一条。
    """
    yaw_runs = [r for r in runs if r["axis"] == "yaw" and r["r2_main"] >= args.min_r2]
    if not yaw_runs:
        return None
    # 用记录下来的整段 odom 拟合圆: (x-cx)² + (y-cy)² = R² 线性化成
    # 2x·cx + 2y·cy + (R²-cx²-cy²) = x²+y², 直接最小二乘。
    chunks = [odom.window(r["t_start"], r["t_end"]) for r in yaw_runs]
    chunks = [c for c in chunks if len(c) >= 10]
    if not chunks:
        return None
    win = np.vstack(chunks)
    if len(win) < 30:
        return None
    x, y = win[:, 1], win[:, 2]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    sol, *_ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)
    cx, cy, c = sol
    r2 = c + cx ** 2 + cy ** 2
    if r2 <= 0:
        return None
    radius = float(math.sqrt(r2))
    resid = np.abs(np.hypot(x - cx, y - cy) - radius)
    return {
        "radius_m": radius,
        "center": [float(cx), float(cy)],
        "fit_residual_m": float(resid.mean()),
        "note": ("圆拟合残差远大于半径时这个估计不可信(说明这段不是纯原地旋转, "
                 "或者杆臂本来就接近 0)。半径 ≈ |lidar_t_body| 的水平分量。"),
    }


def print_report(report: Dict) -> None:
    print("\n" + "=" * 72)
    print("标定结果")
    print("=" * 72)
    for axis, e in report["analysis"]["per_axis"].items():
        print("\n[%s]  单位 %s   厂商下界 %.2f   有效 run %d 次"
              % (axis, e["unit"], e["vendor_lower"], e["n_runs"]))
        for name, label in (("pos", "正向"), ("neg", "反向")):
            f = e.get(name)
            if not isinstance(f, dict):
                print("  %s: 有效点不足, 没拟合" % label)
                continue
            # 报告里显示绝对值: JSON 里存的 deadzone 是拟合直线跟 v=0 的交点,
            # 反向分支上它天然是负数(命令 -0.17 以内不动), 打成 "-0.17" 容易被
            # 误读成"负的死区"。
            print("  %s: 比例 %.4f   死区 %.4f   拟合 R²=%.4f  (%d 点)"
                  % (label, f["scale"], abs(f["deadzone"]), f["fit_r2"], f["n"]))
        if e.get("asymmetry") is not None:
            print("  正反不对称: %+.1f%%" % (100 * e["asymmetry"]))
        if e.get("rise_time_median") is not None:
            print("  上升时间(到稳态 90%%)中位: %.2fs" % e["rise_time_median"])
    cp = report["analysis"].get("coupling") or {}
    if cp:
        print("\n轴间耦合(命令 1 单位某轴, 实测各轴冒出多少):")
        print("        ->x      ->y      ->yaw")
        for axis in ("x", "y", "yaw"):
            if axis in cp:
                row = cp[axis]
                print("  %-4s %8.4f %8.4f %8.4f"
                      % (axis, row.get("x", float("nan")), row.get("y", float("nan")),
                         row.get("yaw", float("nan"))))
    la = report.get("lever_arm")
    if la:
        print("\n顺带估的杆臂(lidar 相对机体中心, 来自 yaw 原地旋转):")
        print("  半径 %.3f m   圆拟合残差 %.3f m" % (la["radius_m"], la["fit_residual_m"]))
    v = report.get("verification")
    if v:
        print("\n验证集(不参与拟合的幅值, 看预测准不准):")
        for row in v:
            print("  %-3s cmd=%+.3f  预测 %+.4f  实测 %+.4f  误差 %+.4f (%.1f%%)"
                  % (row["axis"], row["cmd"], row["predicted"], row["measured"],
                     row["error"], 100 * row["error_frac"]))
    print("\n" + "-" * 72)
    print("提醒: 这份结果只对下面这组配置有效, 换步态/换使用模式都要重标 ——")
    print("  " + json.dumps(report["config_hint"], ensure_ascii=False))
    print("还有一件事这份数据回答不了: **odom 自己准不准**。标定标的是 cmd_vel 对")
    print("odom 一致; odom 要是有尺度误差, 狗的实际位移仍然是错的。花十分钟用卷尺")
    print("量 2~3 次(固定速度走固定时间, 量实际位移跟 odom 报的比), 就能把这两件")
    print("事分开。")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="标定 /cmd_vel 到实际运动的传递关系(x/y/yaw 分别标)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--axis", action="append", choices=list(AXES),
                    help="要标的轴, 可重复。不给就三个轴都标 —— 但**建议先只跑 "
                         "--axis yaw**: 原地转不占场地、风险最小, 先验证整条流程")
    ap.add_argument("--amplitudes", help="覆盖默认扫描幅值, 逗号分隔(只在标单轴时有意义)")
    ap.add_argument("--verify", help="验证集幅值, 逗号分隔。这些点照样测但**不参与拟合**, "
                                     "用来检查拟合的预测准不准 —— 没有这一步就只是拟合、不算验证")
    ap.add_argument("--repeats", type=int, default=3,
                    help="每个幅值每个方向重复几次。至少 3 次, 否则分不清 5%% 的真实"
                         "误差和噪声")
    ap.add_argument("--hold", type=float, default=5.0, help="每次阶跃保持多久 [s]")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="丢掉阶跃开头这么久的暂态 [s]。跑完看报告里的上升时间, "
                         "比它小就得调大")
    ap.add_argument("--rest", type=float, default=1.5, help="每次之间发零休息多久 [s]")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="发布频率 [Hz]。必须远高于 deep_bridge 的 cmd_timeout_sec "
                         "(默认 0.5s)的倒数")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/hand_lio/odom_vehicle")
    ap.add_argument("--min-r2", type=float, default=0.98,
                    help="稳态窗口的直线拟合 R² 门槛, 低于它的 run 不参与分析"
                         "(说明那段不是匀速: 还在加速, 或者打滑)")
    ap.add_argument("--still-xy", type=float, default=0.02, help="判静止的位置抖动上限 [m]")
    ap.add_argument("--still-yaw", type=float, default=0.02, help="判静止的朝向抖动上限 [rad]")
    ap.add_argument("--max-runtime", type=float, default=3600.0,
                    help="总时长硬上限 [s], 超了自动停并保存已有数据")
    ap.add_argument("--ramp", action="store_true",
                    help="每轴额外跑一次很慢的斜坡, 原始曲线存进报告 —— 阶跃给精确"
                         "数值, 斜坡给一张能一眼看出死区形状的图")
    ap.add_argument("--ramp-duration", type=float, default=20.0, help="斜坡时长 [s]")
    ap.add_argument("--vendor-lower", default="",
                    help="覆盖厂商速度区间下界, 形如 x=0.2,y=0.35,yaw=0.5。**换步态就"
                         "得改** —— 默认值是 0x1001 标准-基础那一行")
    ap.add_argument("--force", action="store_true",
                    help="即使 /cmd_vel 上还有别的发布者也照跑(数据会被污染, 不建议)")
    ap.add_argument("--out", default="cmd_vel_calibration.json", help="报告输出路径")
    args = ap.parse_args()

    args.axis = args.axis or list(AXES)
    args.vendor_lower = dict(
        (k, float(v)) for k, v in
        (kv.split("=", 1) for kv in args.vendor_lower.split(",") if kv.strip())
    )
    verify_amps = [float(a) for a in args.verify.split(",")] if args.verify else []

    rospy.init_node("calibrate_cmd_vel", anonymous=True, disable_signals=True)
    print("== /cmd_vel 标定 ==")
    print("  轴: %s   每幅值每方向 %d 次   hold=%.1fs settle=%.1fs"
          % (", ".join(args.axis), args.repeats, args.hold, args.settle))
    info = preflight(args)
    print("  前置检查通过(%s 上没有别的发布者, odom cov=%.3f)"
          % (info["cmd_topic"], info.get("odom_cov", float("nan"))))

    cal = Calibrator(args)
    ramps: List[Dict] = []
    interrupted = False
    try:
        for axis in args.axis:
            amps = ([float(a) for a in args.amplitudes.split(",")]
                    if args.amplitudes and len(args.axis) == 1 else AXES[axis]["amps"])
            amps = sorted(set(amps) | set(verify_amps))
            print("\n[%s] 扫描 %d 个幅值 x 2 方向 x %d 次 = %d 次阶跃"
                  % (axis, len(amps), args.repeats, len(amps) * 2 * args.repeats))
            for rep in range(args.repeats):
                for amp in amps:
                    # 正负交替: 让狗来回走, 把位移抵消掉, 省场地
                    for sign in (1, -1):
                        cal.run_step(axis, sign * amp)
            if args.ramp:
                print("  [%s] 慢斜坡 %.0fs ..." % (axis, args.ramp_duration))
                ramps.append(cal.run_ramp(axis))
    except (KeyboardInterrupt, RuntimeError) as e:
        interrupted = True
        print("\n中断: %s —— 已采到的数据照样会分析并保存" % e)
    finally:
        # 不管怎么退出都要把零发够, 见 publish_zero 的说明
        print("\n发零刹停 ...")
        cal.publish_zero(max(1.5, 3.0 / max(args.rate, 1.0) + 1.5))

    fit_runs = [r for r in cal.runs if abs(r["cmd"]) not in set(verify_amps)]
    analysis = analyse(fit_runs, args)

    # 验证集: 用拟合结果预测, 跟实测比
    verification = []
    for r in cal.runs:
        if abs(r["cmd"]) not in set(verify_amps) or r["r2_main"] < args.min_r2:
            continue
        e = analysis["per_axis"].get(r["axis"]) or {}
        f = e.get("pos" if r["cmd"] > 0 else "neg")
        if not isinstance(f, dict):
            continue
        pred = f["scale"] * r["cmd"] + f["intercept"]
        err = r["main"] - pred
        verification.append({
            "axis": r["axis"], "cmd": r["cmd"], "predicted": pred, "measured": r["main"],
            "error": err, "error_frac": err / r["main"] if abs(r["main"]) > 1e-6 else float("nan"),
        })

    report = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "interrupted": interrupted,
        "args": {k: v for k, v in vars(args).items()},
        "preflight": info,
        "odom_twist_filled": cal.odom.twist_seen,
        "runs": cal.runs,
        "analysis": analysis,
        "verification": verification,
        "lever_arm": estimate_lever_arm(cal.runs, cal.odom, args),
        "ramps": ramps,
        # 这几项脚本自己读不到(它们在 deep_bridge / launch 的参数服务器里, 而且
        # 标定时那些节点未必都在跑), 所以只放提示, 让人手工核对填进报告。
        "config_hint": {
            "usage_mode": "手工填: deep_bridge 的 usage_mode",
            "gait_on_start": "手工填: deep_bridge 的 gait_on_start(死区按步态给, 换步态必须重标)",
            "max_v": "手工填: deep_bridge 的 max_vx/max_vy/max_vyaw",
            "full_scale_v": "手工填: deep_bridge 的 full_scale_vx/vy/vyaw(usage_mode=0 时才用到)",
            "bridge": "手工填: deep_bridge 还是 unitree_bridge",
        },
    }
    with open(args.out, "w") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    print_report(report)
    print("\n完整报告(含每次 run 的原始记录): %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
