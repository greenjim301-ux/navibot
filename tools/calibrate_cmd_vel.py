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

yaw 轴顺带能把 hand_lio 的两个外参也标了(它们现在还是占位的单位阵 + 零向量):

  * **lidar_t_body 杆臂** —— 原地旋转时雷达不在机体中心的话, odom 的位置画一个圆,
    半径就是杆臂长度。
  * **lidar_R_body 安装旋转** —— 转圈时雷达 z 轴在世界系里画的那个圆给出安装倾角,
    x/y 直线段的行进方向偏差给出绕 z 的偏转 γ, 两者拼成完整的 3x3。**倾角里含狗
    自己的站姿, odom 原理上分不开**, 见 estimate_mount_tilt 的说明。

所以即使只想标外参, 也得带上 --spin-turns(默认 2 圈)。
"""
import argparse
import bisect
import collections
import json
import math
import sys
import threading
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
#
# 下界**以上**要留足点(每个方向至少 4~5 个): 实测死区可能比厂商下界还高一点,
# 那些落在死区里的幅值会因为 R² 太低被剔掉, 剩下的点不够就拟合不出可信的比例
# —— 第一版 y 轴只给了 0.45/0.60 两个点, 死区估出来就偏了 0.013。
AXES = {
    "x":   {"unit": "m/s",   "vendor_lower": 0.20,
            "amps": [0.05, 0.10, 0.15, 0.18, 0.20, 0.22, 0.26, 0.32, 0.40, 0.50, 0.62, 0.75]},
    "y":   {"unit": "m/s",   "vendor_lower": 0.35,
            "amps": [0.10, 0.20, 0.30, 0.33, 0.35, 0.38, 0.42, 0.48, 0.55, 0.65, 0.80]},
    "yaw": {"unit": "rad/s", "vendor_lower": 0.50,
            "amps": [0.10, 0.25, 0.40, 0.48, 0.50, 0.53, 0.60, 0.70, 0.85, 1.00, 1.20]},
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
        # deque + 锁, 不是 list。回调跑在 rospy 的接收线程上, 主线程在 window()
        # 里遍历同一个容器: list.pop(0) 会让遍历中的下标错位、悄悄漏掉样本, 而且
        # pop(0) 是 O(N) 的 memmove —— 恰好违背了本类"回调里不做计算"的初衷。
        # deque.popleft 是 O(1), 锁只保护"取快照"这一下。
        self._samples = collections.deque()  # type: collections.deque
        self._lock = threading.Lock()
        self._twist_seen = False
        self._backwards = 0
        self._sub = rospy.Subscriber(topic, Odometry, self._cb,
                                     queue_size=200, tcp_nodelay=True)

    def _cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        yaw = tft.euler_from_quaternion(quat)[2]
        # **雷达 z 轴在世界系里的指向**(只留水平两个分量)。安装倾角要靠它估,
        # 不能用 roll/pitch 欧拉角 —— 欧拉角里地面坡度和安装倾角搅在一起分不开,
        # 换成世界系 z 轴之后: 坡度是个固定偏移(不随 yaw 转), 安装倾角是个随 yaw
        # 转的向量, 一次圆拟合就分开了。见 estimate_mount_tilt。
        m = tft.quaternion_matrix(quat)
        t = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        cov = msg.pose.covariance[0]
        tw = msg.twist.twist
        if abs(tw.linear.x) + abs(tw.linear.y) + abs(tw.angular.z) > 1e-9:
            self._twist_seen = True
        cutoff = t - self.keep_sec
        with self._lock:
            # **时间戳必须单调**: window() 是二分定位的, 缓冲区一旦乱序, 切出来的
            # 区间会静默错。而这条链路上回退是真会发生的 —— odom_vehicle 的 stamp
            # 直接继承自 /latest_imu_odom(HandLioNode.cpp:532), hand_lio 自己就为
            # 此写了防护(:119, "timestamp went backwards, clearing pose buffer",
            # 播包重播那类情况)。这里照抄它的做法: 清空重新累积, 并计数, 结尾报出来
            # —— 被清掉那一段的窗口会取不到样本, 表现为该次阶跃被跳过, 有据可查。
            if self._samples and t <= self._samples[-1][0]:
                self._backwards += 1
                rospy.logwarn_throttle(
                    1.0, "odom 时间戳回退(%.3f -> %.3f), 清空缓冲重新累积",
                    self._samples[-1][0], t)
                self._samples.clear()
            self._samples.append((t, p.x, p.y, yaw, cov, m[0][2], m[1][2]))
            while self._samples and self._samples[0][0] < cutoff:
                self._samples.popleft()

    @property
    def backwards_count(self) -> int:
        """时间戳回退了几次。见 _cb。"""
        return self._backwards

    @property
    def twist_seen(self) -> bool:
        """odom 里的 twist 字段有没有被填过。没填的话报告里注明"只能靠位置拟合"。"""
        return self._twist_seen

    def window(self, t0: float, t1: float) -> np.ndarray:
        """取 [t0, t1] 区间的样本, 返回 (N, 7): t/x/y/yaw/cov/zx/zy。

        前 5 列的含义不能动 —— measure_window 按下标取。

        **只保留最近 keep_sec 秒。** 想留住一段数据就当场取走存成数组, 别存时间
        区间等到最后再来取 —— 那正是 estimate_lever_arm 之前踩的坑, 见 run_spin。
        """
        # hand-lio 的 odom 是 **200Hz**, keep_sec=60 就是 12000 个样本。老写法在锁
        # 里线性扫全量, 而 estimate_rise_time 一个 run 就要调二十几次 —— 接收线程
        # 会被压住。改成: 锁里只做一次 C 级拷贝, 二分定位和切片都在锁外做。
        with self._lock:
            rows = list(self._samples)
        lo = bisect.bisect_left(rows, (t0,))
        hi = bisect.bisect_right(rows, (t1, float("inf")))
        return np.array(rows[lo:hi], dtype=float).reshape(-1, 7)

    def latest(self) -> Optional[Tuple[float, ...]]:
        with self._lock:
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
    if ss_tot <= 1e-12:
        # 完全没动。R² 没定义, 但**不能当成 1.0** —— 那会把"狗根本没动"判成
        # "完美匀速", 让一个落在死区里的大幅值点带着 v=0 混进线性拟合。实测有
        # 噪声时走不到这个分支(R² 自然会掉到很低被剔掉), 但理想数据/回放会。
        return float(k), 0.0
    return float(k), 1.0 - float((resid ** 2).sum()) / ss_tot


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
        # keep_sec 必须盖得住最长的一次回看: run_ramp 结束后要取整段斜坡, 默认 20s,
        # 但它是命令行可调的 —— 写死 60 的话 --ramp-duration 一调大就静默截断。
        self.odom = OdomBuffer(args.odom_topic,
                               keep_sec=max(60.0, args.ramp_duration + 15.0,
                                            args.hold + 15.0))
        self.runs: List[Dict] = []
        # 原地转圈那几段的时间区间, 给 estimate_mount_tilt / estimate_lever_arm 用
        # **存数组本身, 不存时间区间。** OdomBuffer 只留 60s, 而这几段要等整轮跑
        # 完(20 多分钟后)才用得上 —— 存区间的话回头 window() 取出来是空的,
        # 三个估计器一起静默返回 None, 报告里那几节直接不打印, 人还以为没这功能。
        # estimate_lever_arm 从 0651bef 起就一直踩这个坑: 只有默认轴序(yaw 恰好
        # 最后)才侥幸有数据, 而且也只剩最后 60s 那一点。
        self.spin_windows: List[np.ndarray] = []
        # 跳过统计。一次 run 失败只打一行 logwarn, 跑满二十多分钟才发现一条数据都没
        # 采到就太晚了 —— 结尾按这个给汇总和告警。
        self.attempted = 0
        self.skipped = {"stationary": 0, "window": 0}
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

    def measure_noise_floor(self, seconds: float = 3.0) -> Dict[str, float]:
        """静止测几秒, 看 odom 的抖动会不会把 --still-xy/--still-yaw 卡死。

        wait_stationary 判的是"窗口内离均值的**最大**径向偏差", 极值统计对噪声
        和样本数都敏感 —— 阈值要是压在噪声底以下, 每一步都等不到静止, 整轮下来
        一条数据都采不到, 而且只留一堆 logwarn。所以开跑前先量一遍, 拿的就是
        wait_stationary 那个统计量本身, 不做正态假设去推。
        """
        print("  静止测 %.0fs 噪声底(狗现在别动)..." % seconds)
        self.publish_zero(seconds)
        t1 = time.time()
        win = self.odom.window(t1 - seconds + 0.3, t1)
        if len(win) < 20:
            raise SystemExit("静止段只收到 %d 个 odom 样本, 检查 odom 频率。" % len(win))
        # 按 wait_stationary 用的窗长切片, 取各片统计量的最大值
        span = 0.7
        xy, yw = [], []
        t = win[0, 0]
        while t + span <= win[-1, 0]:
            sub = win[(win[:, 0] >= t) & (win[:, 0] <= t + span)]
            if len(sub) >= 5:
                xy.append(float(np.hypot(sub[:, 1] - sub[:, 1].mean(),
                                         sub[:, 2] - sub[:, 2].mean()).max()))
                yw.append(float(np.ptp(np.unwrap(sub[:, 3]))))
            t += 0.1
        if not xy:
            raise SystemExit("静止段切不出 0.7s 的子窗口, 检查 odom 频率。")
        out = {"noise_xy": max(xy), "noise_yaw": max(yw),
               "rate_hz": float(len(win)) / (win[-1, 0] - win[0, 0]),
               "n": int(len(win))}
        print("    odom %.0f Hz; 0.7s 窗口内抖动上界 xy %.4f m / yaw %.4f rad"
              % (out["rate_hz"], out["noise_xy"], out["noise_yaw"]))
        bad = []
        if out["noise_xy"] >= self.args.still_xy:
            bad.append("--still-xy %.3f (建议 %.3f)"
                       % (self.args.still_xy, 1.5 * out["noise_xy"]))
        if out["noise_yaw"] >= self.args.still_yaw:
            bad.append("--still-yaw %.3f (建议 %.3f)"
                       % (self.args.still_yaw, 1.5 * out["noise_yaw"]))
        if bad:
            raise SystemExit(
                "odom 静止噪声已经超过判静止的阈值: %s\n"
                "  照这个跑下去每一步都等不到静止, 整轮采不到数据。放宽阈值再跑,\n"
                "  或者先查 odom 为什么这么抖。" % "; ".join(bad))
        return out

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
        self.attempted += 1
        if not self.wait_stationary():
            self.skipped["stationary"] += 1
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
            self.skipped["window"] += 1
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

    # ---- 原地转圈: 给"安装倾角"和"杆臂"两个圆拟合提供全朝向覆盖 ----
    def run_spin(self, turns: float, rate_cmd: float,
                 expected_rate: Optional[float] = None) -> Optional[Dict]:
        """原地匀速转 turns 圈, 单独记一段窗口。

        为什么不复用 yaw 的阶跃数据: 一次阶跃只转 0.5rad/s x 3.5s ≈ 100°, 而且正负
        交替会转回原处 —— 朝向覆盖不到一整圈, 圆拟合的条件数极差, 半径和相位都不
        可信。转两整圈的一段连续数据, 两个圆拟合都稳。
        """
        self.check_budget()
        if not self.wait_stationary():
            return None
        rate = rospy.Rate(self.args.rate)
        cmd = make_twist("yaw", rate_cmd)
        t0 = time.time()
        # 超时兜底: 万一命令值落在死区里狗根本不转, 不能死等。
        # **按实测转速算, 不是按命令值。** yaw 有死区, 实际转速能只有命令的 40%,
        # 按命令值再乘个 2.5 的余量恰好只剩 4% —— 死区再大一点就转不满圈, 而转不满
        # 圈时两个圆拟合都会静默退化。expected_rate 由调用方从前面的 yaw 阶跃里取。
        ref = abs(expected_rate) if expected_rate else abs(rate_cmd)
        timeout = turns * 2 * math.pi / max(ref, 1e-3) * 2.5 + 10.0
        while not rospy.is_shutdown():
            self.pub.publish(cmd)
            rate.sleep()
            # 从 t0+settle 起算, 跟最后交给圆拟合的那段窗口对齐 —— 从 t0 起算的话
            # 前 settle 秒的转动也被计进去, --spin-turns 2 实际只留下 1.9 圈可用。
            win = self.odom.window(t0 + self.args.settle, time.time() + 1.0)
            if len(win) > 10 and abs(np.unwrap(win[:, 3])[-1] - win[0, 3]) >= turns * 2 * math.pi:
                break
            if time.time() - t0 > timeout:
                rospy.logwarn("转圈超时(命令 %.2f rad/s 可能落在死区里), 用已转到的部分",
                              rate_cmd)
                break
        t_end = time.time()
        self.publish_zero(self.args.rest)
        win = self.odom.window(t0 + self.args.settle, t_end)
        if len(win) < 50:
            return None
        turned = float(abs(np.unwrap(win[:, 3])[-1] - win[0, 3]))
        print("    原地转圈: 命令 %.2f rad/s, 实际转过 %.0f° (%.1f 圈), %d 个样本"
              % (rate_cmd, math.degrees(turned), turned / (2 * math.pi), len(win)))
        self.spin_windows.append(win)      # 当场存下来, 见 __init__ 的注释
        return {"cmd": rate_cmd, "turned_rad": turned, "n": int(len(win))}

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
    数据全是废的, 而且要几十分钟才能发现。

    还有一关需要 odom 缓冲区, 在这之后: Calibrator.measure_noise_floor。
    """
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

    # ② odom 有没有在来, 新不新鲜, 定位好不好(③ 时钟检查在这个循环里)
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
        # ③ 时钟。样本时间用的是 msg.header.stamp(ROS 时间/发布端时钟), 而切窗口
        #    用的是 time.time()(本机墙钟)。两者不可比的话所有窗口都静默取空, 表现
        #    出来是"样本太少", 排查方向完全跑偏。这里一次性把三个钟对上。
        ros_now, wall = rospy.Time.now().to_sec(), time.time()
        stamp = msg.header.stamp.to_sec() or ros_now
        info["clock_ros_minus_wall"] = float(ros_now - wall)
        info["clock_odom_lag"] = float(ros_now - stamp)
        if abs(ros_now - wall) > 1.0:
            raise SystemExit(
                "ROS 时间跟墙钟差 %.1fs —— 多半是 use_sim_time 开着。脚本用 "
                "time.time() 切窗口、用 header.stamp 存样本, 两者不可比就什么都测"
                "不到。" % (ros_now - wall))
        if abs(ros_now - stamp) > 1.0:
            raise SystemExit(
                "odom 的 header.stamp 比现在慢 %.1fs —— 要么消息是陈的, 要么手持"
                "设备跟板子的时钟没同步。先把时钟对上再标定。" % (ros_now - stamp))
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

    # 轴间耦合: 命令某个轴时, 另外两个轴冒出来多少。腿式底盘上"命令纯 x 却持续
    # 偏航"很常见, 而那正好是"狗自己往路边靠"的直接成因。
    #
    # **按实测的主轴速度归一, 不是按命令幅值。** 按命令归一的话, 有死区时
    # v/u = k(1 - d/u) 会随幅值变, 平均出来是个没有意义的数(第一版就是这么写的,
    # 对角线上直接出了 0.44 而真实比例是 0.92)。除以实测主轴速度之后, 对角线
    # 恒为 1, 非对角是"每 1 单位实际主轴运动伴随多少侧向/偏航", 可以直接读:
    # 比如 x->yaw = 0.035 就是"每前进 1m 偏 0.035 rad"。
    coupling: Dict[str, Dict[str, float]] = {}
    for axis in AXES:
        big = [r for r in runs
               if r["axis"] == axis and r["r2_main"] >= args.min_r2
               and abs(r["main"]) >= args.min_main_speed]
        if not big:
            continue
        coupling[axis] = {}
        for out_axis, key in (("x", "vx"), ("y", "vy"), ("yaw", "vyaw")):
            ratios = [r[key] / r["main"] for r in big]
            coupling[axis][out_axis] = float(np.mean(ratios)) if ratios else float("nan")
    return {"per_axis": out, "coupling": coupling}


def estimate_mount_tilt(windows: List[np.ndarray]) -> Optional[Dict]:
    """从原地转圈估**安装倾角**(lidar_R_body 的 roll/pitch 那部分)。

    世界系是重力对齐的(见 lidar_tilt_impact.md 第 1 节), 于是雷达 z 轴在世界系里
    的水平投影满足:

        (zx, zy) = c + Rz(ψ) · δ

      c = 地面坡度 —— **固定在世界系**, 不随 yaw 转
      δ = 安装倾角(+ 狗的站姿倾角)在机体系里的水平分量 —— 固定在机体系, 随 yaw 转

    所以对 (zx, zy, ψ) 做一次线性最小二乘就把两者分开了: 圆心是坡度, 半径 |δ| 是
    倾角, 相位是倾斜方向。**这就是为什么不能用 roll/pitch 欧拉角** —— 欧拉角里这
    两样是搅在一起的。

    分不开的那一条: **安装倾角和狗自己的站姿倾角**。两者都固定在机体系、都随 yaw
    转, odom 只看得见雷达, 原理上无法区分。要拆开只能靠外部参照(机体上放水平仪,
    或者拿量角器量支架)—— 那是一次性机械测量, 不是标定能解决的。

    ψ 用的是 odom 报的 yaw(即雷达的 yaw), 跟机体 yaw 差一个常数 γ。常数偏移只会
    把 δ 整体转 γ, **不影响倾角大小**, 只影响报出来的"倾斜方向"。下面 estimate_
    mount_yaw 估出 γ 之后, build_lidar_R_body 会把这一层补回去。
    """
    rows = [w for w in windows if len(w) >= 50]
    if not rows:
        return None
    win = np.vstack(rows)
    psi = np.unwrap(win[:, 3])
    span = float(psi.max() - psi.min())
    zx, zy = win[:, 5], win[:, 6]
    cp, sp = np.cos(psi), np.sin(psi)
    # [zx; zy] = [1 0 cos -sin; 0 1 sin cos] · [cx, cy, dx, dy]
    A = np.zeros((2 * len(psi), 4))
    A[0::2, 0] = 1.0
    A[0::2, 2] = cp
    A[0::2, 3] = -sp
    A[1::2, 1] = 1.0
    A[1::2, 2] = sp
    A[1::2, 3] = cp
    b = np.empty(2 * len(psi))
    b[0::2], b[1::2] = zx, zy
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, dx, dy = (float(v) for v in sol)
    resid = float(np.sqrt(((A @ sol - b) ** 2).mean()))
    tilt = math.hypot(dx, dy)
    return {
        "tilt_rad": tilt,
        "tilt_deg": math.degrees(math.asin(min(1.0, tilt))),
        # 机体系里倾斜指向哪边(还没扣 γ, 见 docstring)
        "tilt_dir_rad_in_odom_frame": math.atan2(dy, dx),
        "delta": [dx, dy],
        "floor_slope_deg": math.degrees(math.asin(min(1.0, math.hypot(cx, cy)))),
        "residual": resid,
        "yaw_span_deg": math.degrees(span),
        "n": int(len(win)),
        "trustworthy": bool(span >= 1.5 * math.pi and resid < 0.02),
        "note": ("yaw 覆盖不到 270° 或者残差 > 0.02 时这个估计不可信。**倾角里含狗"
                 "自己的站姿, odom 分不开** —— 要拆开得在机体上放水平仪/量支架。"),
    }


def estimate_mount_yaw(runs: List[Dict], args) -> Optional[Dict]:
    """从 x / y 直线段估**安装偏转 γ**(lidar_R_body 绕 z 的那部分)。

    雷达绕机体 z 轴装歪 γ 的话, 机体沿自己的 +x 走时, odom(报的是雷达系)看到的
    行进方向会偏 γ。measure_window 出来的 (vx, vy) 已经在这个系里, 直接取"实际行进
    方向"跟"名义方向"的夹角就行。

    麻烦在于这个夹角里还混着**底盘自己的横向漂移**。设机体系里有一个恒定的漂移
    速度 w(狗迈腿时往一边蹭), 命令方向单位向量 n̂、速度 v, 则

        夹角 ≈ γ + (w · n̂⊥) / v            n̂⊥ = Rz(90°)·n̂

    关键在第二项**随命令方向翻号**而 γ 不翻: 命令 +x 时是 +wy/v, 命令 -x 时是
    -wy/v。所以每个轴拿正反两组做一次两参数最小二乘 [1, s/v], 截距就是 γ, 斜率
    就是漂移 —— 两者干净分开, 不需要 x 和 y 都跑。
    (最早那版只按轴取平均、拿组内散布当容差, 合成数据一验: 5% 的横向漂移照样被
     判成"一致"就写进矩阵了 —— 因为漂移的信号恰恰**就在**组内那个散布里。)

    x 和 y 两组分别给出 γ 之后再对一次, 是第二道关: 上面的模型假设 w 恒定, 如果
    漂移其实随速度成比例、或者根本不是这个形状, 两组就对不上。
    """
    per_axis: Dict[str, Dict[str, float]] = {}
    for axis, nominal in (("x", 0.0), ("y", math.pi / 2)):
        ang, inv_v = [], []
        for r in runs:
            if r["axis"] != axis or r["r2_main"] < args.min_r2:
                continue
            v = abs(r["main"])
            if v < args.min_main_speed:
                continue
            sgn = math.copysign(1.0, r["cmd"])
            # 名义方向带上命令的正负, 这样反向那些 run 不用单独处理
            nx, ny = sgn * math.cos(nominal), sgn * math.sin(nominal)
            vx, vy = r["vx"], r["vy"]
            ang.append(math.atan2(nx * vy - ny * vx, nx * vx + ny * vy))
            inv_v.append(sgn / v)
        if len(ang) < 4 or len(set(np.sign(inv_v))) < 2:
            continue                      # 缺一个方向就分不开 γ 和漂移
        A = np.column_stack([np.ones(len(ang)), np.array(inv_v)])
        y = np.array(ang)
        sol, *_ = np.linalg.lstsq(A, y, rcond=None)
        resid = y - A.dot(sol)
        dof = max(1, len(ang) - 2)
        cov = (float((resid ** 2).sum()) / dof) * np.linalg.pinv(A.T.dot(A))
        per_axis[axis] = {
            "gamma_rad": float(sol[0]),
            "gamma_deg": math.degrees(float(sol[0])),
            # w·n̂⊥ [m/s]: x 组给的是 wy, y 组给的是 -wx
            "drift_mps": float(sol[1]),
            "stderr_deg": math.degrees(math.sqrt(max(0.0, cov[0][0]))),
            "resid_deg": math.degrees(float(np.sqrt((resid ** 2).mean()))),
            "n": len(ang),
        }
    if not per_axis:
        return None
    out: Dict[str, object] = {"per_axis": per_axis}
    # 漂移向量(机体系), 两个分量各来自一个轴
    drift = {}
    if "x" in per_axis:
        drift["wy"] = per_axis["x"]["drift_mps"]
    if "y" in per_axis:
        drift["wx"] = -per_axis["y"]["drift_mps"]
    out["chassis_drift_mps"] = drift
    if "x" in per_axis and "y" in per_axis:
        gx, gy = per_axis["x"]["gamma_rad"], per_axis["y"]["gamma_rad"]
        spread = math.degrees(abs(gx - gy))
        tol = 3.0 * math.hypot(per_axis["x"]["stderr_deg"],
                               per_axis["y"]["stderr_deg"]) + 0.3
        out["gamma_rad"] = float((gx + gy) / 2)
        out["gamma_deg"] = math.degrees(float((gx + gy) / 2))
        out["xy_spread_deg"] = spread
        out["consistent"] = bool(spread <= tol)
        out["note"] = ("x 和 y 两组差 %.2f° (容差 %.2f°): %s" % (
            spread, tol,
            "一致" if spread <= tol else
            "**对不上 —— 漂移不是恒定向量那个形状, γ 不可信, 别写进 lidar_R_body**"))
    else:
        only = list(per_axis)[0]
        out["gamma_rad"] = per_axis[only]["gamma_rad"]
        out["gamma_deg"] = per_axis[only]["gamma_deg"]
        out["consistent"] = True
        out["note"] = ("只有 %s 轴的数据。正反两个方向都跑了, γ 和漂移仍然分得开, "
                       "但少了 x/y 互相印证这道关 —— 想更稳就把另一个轴也跑上"
                       % only)
    return out


def _yaw_free_tilt(dx: float, dy: float) -> np.ndarray:
    """由 z 轴的水平分量还原"去掉 yaw"的倾斜矩阵 L = Ry(p)·Rx(r)。

    该矩阵第三列 = (sin p·cos r, -sin r, cos p·cos r), 所以 r 和 p 有闭式解。
    measure_window 把世界位移按 odom 报的 yaw 转回来, 剩下的就是这个 L —— 倾角
    大的时候, 它会让"水平面上看到的行进方向"偏离真正的 γ。
    """
    dz = math.sqrt(max(0.0, 1.0 - dx * dx - dy * dy))
    r = math.asin(max(-1.0, min(1.0, -dy)))
    p = math.atan2(dx, dz)
    return tft.euler_matrix(r, p, 0.0)[:3, :3]


def _compose_from_u_gamma(dx: float, dy: float, gamma: float) -> np.ndarray:
    """给定倾角拟合值 δ_est 和偏转 γ, 拼出 lidar_R_body。见 build_lidar_R_body。"""
    cg, sg = math.cos(gamma), math.sin(gamma)
    ux, uy = cg * dx + sg * dy, -sg * dx + cg * dy     # δ_body = Rz(-γ)·δ_est
    u = np.array([ux, uy, math.sqrt(max(0.0, 1.0 - ux * ux - uy * uy))])
    # ⊥u 的一组正交基
    seed = np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    a = np.cross(u, seed)
    a /= np.linalg.norm(a)
    b = np.cross(u, a)
    phi = math.atan2(b[0], a[0]) - gamma
    r1 = math.cos(phi) * a + math.sin(phi) * b
    r2 = -math.sin(phi) * a + math.cos(phi) * b
    R = np.vstack([r1, r2, u])
    # (a, b, u) 是右手正交基(b = u x a, 于是 a x b = u), r1/r2 是 (a, b) 在平面内
    # 转了 φ, 所以 r1 x r2 ≡ u, det ≡ +1 —— 不需要翻号分支。之前写了一个
    # "det<0 就翻第二行"的兜底, 2 万组随机输入无一触发, 而且真触发反倒会破坏
    # γ 约束(γ 是第一**列**的水平角, 依赖 r2[0])。留个断言挡住将来的改坏。
    assert np.linalg.det(R) > 0.9, "构造出的不是旋转矩阵, 正交基的手性搞反了"
    return R


def build_lidar_R_body(tilt: Optional[Dict], mount_yaw: Optional[Dict]) -> Optional[Dict]:
    """把倾角和偏转拼成可以直接填进 hand_lio.yaml 的 lidar_R_body(行优先 3x3)。

    约定跟那份 yaml 一致: p_lidar = lidar_R_body · p_body, 也就是 R 的**行**是雷达
    各轴在机体系里的表示, **列**是机体各轴在雷达系里的表示。于是:

      第 3 行 = 雷达 z 轴在机体系 = u = (δx, δy, sqrt(1-δx²-δy²))   <- 倾角给的
      第 1 列的水平角 = 机体 x 轴在雷达系里的方向 = γ                <- 偏转给的

    剩下的自由度(绕 u 转)由 γ 唯一确定: 取 ⊥u 的任意正交基 (a, b), 令
    r1 = cosφ·a + sinφ·b、r2 = -sinφ·a + cosφ·b, 则 (r1x, r2x) 是 (ax, bx) 转了
    -φ, 所以 φ = atan2(bx, ax) - γ 就是闭式解, 不用数值搜。

    两处必须补的坐标系修正(合成数据验证出来的, 不补的话 15° 倾角下矩阵差 1.8°):

    1. estimate_mount_tilt 的 δ 是拿 **odom(雷达)的 yaw** 当相位拟合出来的, 雷达
       yaw 比机体 yaw 少 γ, 所以拟合值是 δ_est = Rz(γ)·δ_body。填进 u 之前要先
       转回来: δ_body = Rz(-γ)·δ_est。
    2. estimate_mount_yaw 量到的不是 γ 本身。它量的是"水平面上看到的行进方向",
       而 measure_window 的参考系是**去掉 yaw 的雷达系** L, 于是实际观测量是
       atan2(horiz(L·R[:,0]))。倾角大的时候这跟 γ 差零点几度。这里不去反解析,
       直接拿构造出来的 R 正向预测一遍观测量, 按差值迭代几次修 γ —— 三四轮就
       收敛到 1e-9, 比推闭式解省事也不容易推错。
    """
    if tilt is None:
        return None
    dx, dy = tilt["delta"]
    if dx * dx + dy * dy >= 1.0:
        return None
    gamma_meas = 0.0
    gamma_known = False
    if mount_yaw and mount_yaw.get("consistent") and "gamma_rad" in mount_yaw:
        gamma_meas = float(mount_yaw["gamma_rad"])
        gamma_known = True

    L = _yaw_free_tilt(dx, dy)
    gamma = gamma_meas
    R = _compose_from_u_gamma(dx, dy, gamma)
    if gamma_known:
        for _ in range(8):        # 见 docstring 第 2 条
            v = L.dot(R[:, 0])
            gamma += gamma_meas - math.atan2(v[1], v[0])
            R = _compose_from_u_gamma(dx, dy, gamma)
    orth = float(np.abs(R.dot(R.T) - np.eye(3)).max())
    rpy = tft.euler_from_matrix(np.vstack([np.hstack([R, [[0], [0], [0]]]),
                                           [0, 0, 0, 1]]))
    return {
        "matrix": [[round(float(v), 6) for v in row] for row in R],
        "rpy_deg": [round(math.degrees(v), 3) for v in rpy],
        "gamma_used_deg": math.degrees(gamma),
        "gamma_known": gamma_known,
        "orthonormality_error": orth,
        "note": ("gamma_known=false 表示绕 z 的那一维没测出来(x/y 两组对不上, 或者"
                 "没跑 x/y), 矩阵里那一维按 0 填, 只有 roll/pitch 可信。"
                 "另外 roll/pitch 里含狗的站姿倾角, 见 estimate_mount_tilt。"),
    }


def estimate_lever_arm(windows: List[np.ndarray],
                       drift: Optional[Dict[str, float]] = None,
                       gamma: Optional[float] = None) -> Optional[Dict]:
    """从原地转圈估雷达相对机体中心的杆臂(hand_lio 的 lidar_t_body)。

    原地纯 yaw 旋转时雷达画一个圆, 半径就是杆臂长度 —— **但"纯"字是个陷阱**。
    狗迈腿时往一边蹭的那点横向速度 w 也在转, 用复数写清楚(ψ = ωt):

        ṗ_body = e^{iψ}·w          =>  p_body = C + e^{iψ}·w/(iω)
        p_lidar = p_body + e^{iψ}·ℓ =>  p_lidar = C + e^{iψ}·(ℓ − i·w/ω)

    所以**量到的半径是 |ℓ − i·w/ω|, 不是 |ℓ|**: 漂移给杆臂加了一项 w/ω, 方向还
    转了 90°。假狗端到端跑出来的例子: 真杆臂 0.222m、漂移 0.03m/s、实际转速
    0.351rad/s, 量到 0.307m —— 偏了 38%, 而且圆拟合残差只有 0.001m, 光看残差
    完全发现不了。

    (p_lidar − C)·e^{−iψ} 是个常向量, 等于 ℓ − i·w/ω, 按 ψ 解出来就拿到了带方向
    的它。剩下的问题是怎么把 w 这一项剥掉, 两条路:

    **首选: 跑两段不同转速, 联立解。** ℓ 是常量而 w/ω 随 ω 变, 所以

        vec_x(ω) = ℓx + wy/ω
        vec_y(ω) = ℓy − wx/ω

    两个不同的 ω 就能把 (ℓx, wy) 和 (ℓy, wx) 各自解出来 —— **不需要任何假设**,
    而且顺带给出转圈时的 w 本身。

    **退路: 拿 x/y 直线段量到的 w 去扣。** 只有一段转圈时只能这么办, 但要清楚它
    的前提: **转圈和平移是两种步态, 横向蹭的量没理由相同**。数值上(真 ℓ=(0.220,
    0.030), 转圈 w=(0.030,−0.015), 平移量到 w=(0.030, 0), ω=0.35):

        w 恰好相同   扣完 (0.2200, 0.0300)   差 0.0 cm
        w 不同       扣完 (0.1771, 0.0300)   差 4.3 cm
        完全不扣                              差 9.6 cm

    扣比不扣好一截, 但**不是"精确"**。所以默认会跑两段转速, 走联立那条路。
    """
    chunks = [w for w in windows if len(w) >= 50]
    if not chunks:
        return None
    segs = []
    for win in chunks:
        x, y = win[:, 1], win[:, 2]
        # 拟合圆: (x-cx)² + (y-cy)² = R² 线性化成 2x·cx + 2y·cy + (R²-cx²-cy²) = x²+y²。
        # 用**原地转圈**那几段而不是逐次 yaw 阶跃 —— 阶跃只转 100° 左右而且正负
        # 交替转回原处, 朝向覆盖不足, 圆拟合条件数极差。
        A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
        sol, *_ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)
        cx, cy, c = sol
        r2 = c + cx ** 2 + cy ** 2
        if r2 <= 0:
            continue
        radius = float(math.sqrt(r2))
        psi = np.unwrap(win[:, 3])
        cp, sp = np.cos(psi), np.sin(psi)
        segs.append({
            "radius_m": radius,
            "center": [float(cx), float(cy)],   # 每段场地位置不同, **不能平均**
            "fit_residual_m": float(np.abs(np.hypot(x - cx, y - cy) - radius).mean()),
            "vec": [float(np.mean(cp * (x - cx) + sp * (y - cy))),
                    float(np.mean(-sp * (x - cx) + cp * (y - cy)))],
            "spin_rate_rad_s": float(fit_line(win[:, 0] - win[0, 0], psi)[0]),
            "n": int(len(win)),
        })
    if not segs:
        return None

    def to_body(lx, ly):
        """雷达 yaw 系 -> 机体系(差一个 γ)。长度不变, 方向变。"""
        if gamma is None:
            return lx, ly
        cg, sg = math.cos(gamma), math.sin(gamma)
        return cg * lx + sg * ly, -sg * lx + cg * ly

    out = {"segments": segs, "n_spins": len(segs)}
    rates = [sg["spin_rate_rad_s"] for sg in segs]
    spread = (max(map(abs, rates)) / max(min(map(abs, rates)), 1e-6)) if len(segs) > 1 else 1.0

    if len(segs) > 1 and spread >= 1.3:
        # 联立: 每段两个方程, 未知 (ℓx, ℓy, wx, wy)。按样本数加权。
        A, b = [], []
        for sg in segs:
            om = sg["spin_rate_rad_s"]
            if abs(om) < 1e-3:
                continue
            wgt = math.sqrt(sg["n"])
            A.append([wgt, 0.0, 0.0, wgt / om]); b.append(wgt * sg["vec"][0])
            A.append([0.0, wgt, -wgt / om, 0.0]); b.append(wgt * sg["vec"][1])
        sol, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
        lx, ly = to_body(float(sol[0]), float(sol[1]))
        out.update({
            "method": "joint",
            "lidar_t_body_xy": [lx, ly],
            "radius_corrected_m": float(math.hypot(lx, ly)),
            "spin_drift_mps": [float(sol[2]), float(sol[3])],
            "rate_spread": spread,
            "note": ("两段不同转速联立解出的, **不依赖任何关于步态的假设**。"
                     "spin_drift_mps 是转圈时的横向漂移, 跟 x/y 直线段量到的"
                     "chassis_drift_mps 对比就知道两种步态蹭得一不一样。"),
        })
        return out

    # 只有一段(或者两段转速太接近): 退回用平移段量到的 w
    sg = max(segs, key=lambda d: d["n"])
    out["radius_m"] = sg["radius_m"]
    out["fit_residual_m"] = sg["fit_residual_m"]
    out["spin_rate_rad_s"] = sg["spin_rate_rad_s"]
    wx, wy = (drift or {}).get("wx"), (drift or {}).get("wy")
    if wx is not None and wy is not None and abs(sg["spin_rate_rad_s"]) > 0.05:
        om = sg["spin_rate_rad_s"]
        lx, ly = to_body(sg["vec"][0] - wy / om, sg["vec"][1] + wx / om)
        out.update({
            "method": "drift_subtracted",
            "lidar_t_body_xy": [lx, ly],
            "radius_corrected_m": float(math.hypot(lx, ly)),
            "drift_term_m": float(math.hypot(wx, wy) / abs(om)),
            "note": ("只有一段转速, 只能拿 x/y 直线段量到的漂移去扣。**前提是转圈和"
                     "平移两种步态蹭得一样多, 四足狗上没理由成立** —— 上面那组数里"
                     "这个前提不成立时还差 4.3cm。跑两段不同转速就能免掉这个前提。"),
        })
    else:
        out["method"] = "raw"
        out["note"] = ("既没有第二段转速、也没有 x/y 直线段的漂移, 这个半径是没扣过"
                       "漂移的, 可能明显偏大 —— 漂移会给它硬加一项 w/ω, 而且完全"
                       "不影响圆拟合残差。")
    return out


def print_report(report: Dict) -> None:
    print("\n" + "=" * 72)
    print("标定结果")
    print("=" * 72)
    sk = report.get("skipped") or {}
    n_try, n_skip = sk.get("attempted", 0), sk.get("stationary", 0) + sk.get("window", 0)
    if n_try and n_skip:
        # 单次失败只有一行 logwarn, 滚上去就看不见了。整轮的比例必须在报告顶上,
        # 不然"跑了二十多分钟其实大半白跑"这件事不会有人注意到。
        frac = float(n_skip) / n_try
        print("\n%s跳过 %d/%d 次阶跃 (%.0f%%): 等不到静止 %d, 窗口无效 %d"
              % ("!! " if frac > 0.3 else "", n_skip, n_try, 100 * frac,
                 sk.get("stationary", 0), sk.get("window", 0)))
        if frac > 0.3:
            print("   跳过这么多, 下面的拟合都别当真。等不到静止就放宽 --still-xy/")
            print("   --still-yaw(开跑时打的噪声底给了建议值), 窗口无效多半是定位")
            print("   在失败(cov >= %.2f)。" % POSE_COV_BAD)
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
            warn = "   ← 点数太少, 比例/死区都不可信, 把死区以上的幅值加密" if f["n"] < 4 else ""
            print("  %s: 比例 %.4f   死区 %.4f   拟合 R²=%.4f  (%d 点)%s"
                  % (label, f["scale"], abs(f["deadzone"]), f["fit_r2"], f["n"], warn))
        if e.get("asymmetry") is not None:
            print("  正反不对称: %+.1f%%" % (100 * e["asymmetry"]))
        if e.get("rise_time_median") is not None:
            print("  上升时间(到稳态 90%%)中位: %.2fs" % e["rise_time_median"])
    cp = report["analysis"].get("coupling") or {}
    if cp:
        print("\n轴间耦合(每 1 单位**实测**主轴运动, 伴随各轴多少; 对角线恒为 1):")
        print("        ->x      ->y      ->yaw")
        for axis in ("x", "y", "yaw"):
            if axis in cp:
                row = cp[axis]
                print("  %-4s %8.4f %8.4f %8.4f"
                      % (axis, row.get("x", float("nan")), row.get("y", float("nan")),
                         row.get("yaw", float("nan"))))
    la = report.get("lever_arm")
    if la:
        print("\n顺带估的杆臂 lidar_t_body(来自 yaw 原地旋转):")
        for sg in la["segments"]:
            print("  转速 %+.2f rad/s: 圆半径 %.3f m, 残差 %.3f m, %d 样本"
                  % (sg["spin_rate_rad_s"], sg["radius_m"], sg["fit_residual_m"],
                     sg["n"]))
        if la["method"] == "joint":
            print("  两段联立解: 水平分量 [%+.3f, %+.3f], 长 %.3f m"
                  % (la["lidar_t_body_xy"][0], la["lidar_t_body_xy"][1],
                     la["radius_corrected_m"]))
            print("  转圈时的横向漂移 [%+.4f, %+.4f] m/s"
                  % (la["spin_drift_mps"][0], la["spin_drift_mps"][1]))
            cd = (report.get("mount_yaw") or {}).get("chassis_drift_mps") or {}
            if "wx" in cd and "wy" in cd:
                print("  (直线段量到的是 [%+.4f, %+.4f] —— 差得多就说明转圈和平移"
                      "是两种步态)" % (cd["wx"], cd["wy"]))
            print("  ← **不依赖任何关于步态的假设**, 用这个值。")
        elif la["method"] == "drift_subtracted":
            print("  扣掉漂移那一项(%.3f m)之后: 水平分量 [%+.3f, %+.3f], 长 %.3f m"
                  % (la["drift_term_m"], la["lidar_t_body_xy"][0],
                     la["lidar_t_body_xy"][1], la["radius_corrected_m"]))
            print("  ← 只有一段转速, 拿直线段的漂移去扣。**前提是转圈和平移蹭得一样")
            print("    多, 四足狗上没理由成立** —— 不成立时还会差几厘米。")
        else:
            print("  ← 没扣漂移, 这个半径可能明显偏大(假狗实测: 0.222m 量成 0.307m),")
            print("    而且完全不影响圆拟合残差, 光看残差发现不了。")
    bw = report.get("odom_backwards")
    if bw:
        print("\n!! odom 时间戳回退了 %d 次, 每次都清空了缓冲重新累积。" % bw)
        print("   跨过那些时刻的阶跃会被跳过(计在上面的跳过数里)。播包重播/设备")
        print("   对时都会造成这个, 先把时间源理顺再重标。")
    tilt = report.get("mount_tilt")
    if not tilt:
        # 同样是"静默少一节"的毛病, 这里显式说一句为什么没有。
        print("\n没有安装倾角/杆臂的估计: 要么没跑 yaw 轴, 要么 --spin-turns 设了 0,")
        print("  要么转圈那段样本不足(<50)。这两个外参靠原地整圈覆盖才估得出来。")
    if tilt:
        ok = "" if tilt["trustworthy"] else "   ← 不可信(yaw 覆盖 %.0f°/残差 %.4f)" % (
            tilt["yaw_span_deg"], tilt["residual"])
        print("\n雷达安装倾角(来自原地转圈的姿态圆拟合):")
        print("  倾角 %.2f°   倾斜方向 %.1f°(odom 系)   地面坡度 %.2f°%s"
              % (tilt["tilt_deg"], math.degrees(tilt["tilt_dir_rad_in_odom_frame"]),
                 tilt["floor_slope_deg"], ok))
        print("  注意: 这个倾角里**含狗自己的站姿倾角**, odom 原理上分不开 ——")
        print("        要拆开得在机体上放水平仪, 或者拿量角器量支架。")
    my = report.get("mount_yaw")
    if my:
        print("\n雷达安装偏转 γ(绕 z, 来自 x/y 直线段的行进方向):")
        for axis in ("x", "y"):
            pa = my["per_axis"].get(axis)
            if pa:
                print("  由 %s 轴估: %+.2f°  (标准误 %.2f°, 拟合残差 %.2f°, %d 点)"
                      % (axis, pa["gamma_deg"], pa["stderr_deg"], pa["resid_deg"],
                         pa["n"]))
        d = my.get("chassis_drift_mps") or {}
        if d:
            print("  同时分离出的底盘横向漂移: %s"
                  % ", ".join("%s=%+.4f m/s" % kv for kv in sorted(d.items())))
        print("  " + str(my["note"]))
    sug = report.get("lidar_R_body_suggestion")
    if sug:
        print("\nhand_lio.yaml 的 lidar_R_body 建议值(行优先):")
        for row in sug["matrix"]:
            print("    [%9.6f, %9.6f, %9.6f]" % tuple(row))
        print("  等价 rpy: %.2f°, %.2f°, %.2f°   正交性误差 %.2e"
              % (sug["rpy_deg"][0], sug["rpy_deg"][1], sug["rpy_deg"][2],
                 sug["orthonormality_error"]))
        if not sug["gamma_known"]:
            print("  ← γ 没测出来, 绕 z 那一维按 0 填, 只有 roll/pitch 可信")
        print("  写进配置前先确认上面两条前提: 站姿倾角、底盘漂移。")

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
    ap.add_argument("--amplitudes", help="覆盖默认扫描幅值, 逗号分隔。**只能配合单轴使用**"
                                         "(多轴时三个轴量纲都不同), 给了多个轴会直接报错")
    ap.add_argument("--verify", help="验证集幅值: 这些点照样测但**不参与拟合**, 用来检查"
                                     "预测准不准 —— 没有这一步就只是拟合、不算验证。"
                                     "写 '0.33,0.62' 是**所有轴都用这组**(三个轴量纲不同, "
                                     "多半不是你要的); 按轴给写 'x=0.33,0.62 y=0.42'")
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
    ap.add_argument("--min-main-speed", type=float, default=0.08,
                    help="算轴间耦合时, 主轴实测速度低于这个值的 run 不参与 —— 耦合是"
                         "按实测主轴速度归一的, 主轴接近 0 时比值会炸")
    ap.add_argument("--still-xy", type=float, default=0.02, help="判静止的位置抖动上限 [m]")
    ap.add_argument("--still-yaw", type=float, default=0.02, help="判静止的朝向抖动上限 [rad]")
    ap.add_argument("--max-runtime", type=float, default=3600.0,
                    help="总时长硬上限 [s], 超了自动停并保存已有数据")
    ap.add_argument("--spin-turns", type=float, default=2.0,
                    help="标 yaw 轴时额外原地转几圈, 给'安装倾角'和'杆臂'两个圆拟合"
                         "提供全朝向覆盖。**少于 1 圈这两个估计都不可信**; 设 0 跳过")
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
    # --verify: 'a,b' = 所有轴通用; 'x=a,b y=c' = 按轴给。两种写法都归一成 per-axis,
    # 免得像第一版那样把 x 的验证幅值悄悄塞进 yaw 的扫描表、还从 yaw 的拟合里排除。
    verify_amps: Dict[str, List[float]] = dict((a, []) for a in AXES)
    for chunk in (args.verify or "").split():
        if "=" in chunk:
            ax, _, vals = chunk.partition("=")
            if ax not in AXES:
                raise SystemExit("--verify 里的轴名 %r 不认识, 只能是 %s"
                                 % (ax, "/".join(AXES)))
            verify_amps[ax] = [float(v) for v in vals.split(",") if v.strip()]
        else:
            for ax in AXES:
                verify_amps[ax] += [float(v) for v in chunk.split(",") if v.strip()]
    if args.amplitudes and len(args.axis) > 1:
        raise SystemExit("--amplitudes 只能配合单个 --axis 使用(三个轴量纲不同), "
                         "现在给了 %s。" % ", ".join(args.axis))

    def is_verify(run: Dict) -> bool:
        return abs(run["cmd"]) in set(verify_amps[run["axis"]])

    rospy.init_node("calibrate_cmd_vel", anonymous=True, disable_signals=True)
    print("== /cmd_vel 标定 ==")
    print("  轴: %s   每幅值每方向 %d 次   hold=%.1fs settle=%.1fs"
          % (", ".join(args.axis), args.repeats, args.hold, args.settle))
    info = preflight(args)
    print("  前置检查通过(%s 上没有别的发布者, odom cov=%.3f)"
          % (info["cmd_topic"], info.get("odom_cov", float("nan"))))

    cal = Calibrator(args)
    info["noise_floor"] = cal.measure_noise_floor()
    ramps: List[Dict] = []
    spins: List[Optional[Dict]] = []
    interrupted = False
    try:
        for axis in args.axis:
            amps = ([float(a) for a in args.amplitudes.split(",")]
                    if args.amplitudes else AXES[axis]["amps"])
            amps = sorted(set(amps) | set(verify_amps[axis]))
            print("\n[%s] 扫描 %d 个幅值 x 2 方向 x %d 次 = %d 次阶跃"
                  % (axis, len(amps), args.repeats, len(amps) * 2 * args.repeats))
            for rep in range(args.repeats):
                for amp in amps:
                    # 正负交替: 让狗来回走, 把位移抵消掉, 省场地
                    for sign in (1, -1):
                        cal.run_step(axis, sign * amp)
            if axis == "yaw" and args.spin_turns > 0:
                # 放在 yaw 阶跃**之后**, 就是为了能用实测结果挑命令值: 直接取
                # "实测转速最接近 0.5 rad/s 的那个幅值"。不用 0.7*最大幅值那种拍脑袋
                # 的写法 —— 死区吃掉多少完全取决于这台狗, 拍出来的值可能根本转不动,
                # 也可能快得打滑。挑不出来(yaw 全落死区)才退回去用幅值。
                good = [r for r in cal.runs
                        if r["axis"] == "yaw" and r["cmd"] > 0
                        and r["r2_main"] >= args.min_r2 and r["main"] > 0.15]
                # **两段不同转速**, 不是一段: ℓ 是常量而漂移项 w/ω 随 ω 变, 两个
                # 转速就能把杆臂和"转圈时的横向漂移"联立解出来, 免掉"转圈和平移
                # 蹭得一样多"那个在四足狗上没理由成立的假设。见 estimate_lever_arm。
                # 慢的那段用来把圆画大(信噪比好), 快的那段提供第二个 ω。
                # 两个转速离得越开, 联立的条件数越好(方程里进的是 1/ω)。所以不挑
                # 什么"目标转速", 直接取**能用的最慢**和**最快**: 下限 0.3 rad/s 是
                # 为了兜住时间(2 圈 @0.3 已经 42s), 上限就是狗能转到的最快。
                usable = [r for r in good if r["main"] >= 0.3]
                picks = []
                if usable:
                    slow = min(usable, key=lambda r: r["main"])
                    fast = max(usable, key=lambda r: r["main"])
                    picks = [(slow["cmd"], slow["main"])]
                    if fast["main"] / slow["main"] >= 1.3:
                        picks.append((fast["cmd"], fast["main"]))
                elif good:
                    pick = max(good, key=lambda r: r["main"])
                    picks = [(pick["cmd"], pick["main"])]
                if not picks:
                    picks = [(0.7 * max(AXES["yaw"]["amps"]), None)]
                    print("  [yaw] yaw 阶跃没一个转得动, 退回用 %.2f rad/s" % picks[0][0])
                elif len(picks) == 1:
                    print("  [yaw] 只挑得出一个转速(%.2f rad/s), 杆臂只能用直线段的"
                          "漂移去扣, 精度打折" % picks[0][1])
                for spin_cmd, spin_rate in picks:
                    print("  [yaw] 原地转 %.1f 圈: 命令 %.2f rad/s%s ..."
                          % (args.spin_turns, spin_cmd,
                             "" if spin_rate is None else
                             " (实测能转出 %.2f)" % spin_rate))
                    spins.append(cal.run_spin(args.spin_turns, spin_cmd,
                                              expected_rate=spin_rate))
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

    fit_runs = [r for r in cal.runs if not is_verify(r)]
    analysis = analyse(fit_runs, args)

    # 验证集: 用拟合结果预测, 跟实测比
    verification = []
    for r in cal.runs:
        if not is_verify(r) or r["r2_main"] < args.min_r2:
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

    tilt = estimate_mount_tilt(cal.spin_windows)
    mount_yaw = estimate_mount_yaw(fit_runs, args)

    report = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "interrupted": interrupted,
        "args": {k: v for k, v in vars(args).items()},
        "preflight": info,
        "odom_twist_filled": cal.odom.twist_seen,
        "odom_backwards": cal.odom.backwards_count,
        "skipped": dict(cal.skipped, attempted=cal.attempted),
        "runs": cal.runs,
        "analysis": analysis,
        "verification": verification,
        "lever_arm": estimate_lever_arm(
            cal.spin_windows, (mount_yaw or {}).get("chassis_drift_mps"),
            (mount_yaw or {}).get("gamma_rad") if (mount_yaw or {}).get("consistent")
            else None),
        "mount_tilt": tilt,
        "mount_yaw": mount_yaw,
        "lidar_R_body_suggestion": build_lidar_R_body(tilt, mount_yaw),
        "spins": [sp for sp in spins if sp],
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
