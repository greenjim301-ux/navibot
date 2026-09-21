#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发一次 /cmd_vel, 看 /hand_lio/odom_vehicle 动了多少。

一次只做一件事: 选一个轴、给一个速度、给一个时长, 下发, 然后报出 odom 三个轴各
变了多少。用来手动试探"命令 0.4 到底能走多快""这个幅值动不动得了"这类问题 ——
tools/calibrate_cmd_vel.py 那套要跑二十多分钟, 摸一个数不值当。

    systemctl stop navi_planner          # 必须! 否则两个发布者抢 /cmd_vel
    python3 tools/cmd_vel_step.py --axis x   --speed 0.4 --duration 3
    python3 tools/cmd_vel_step.py --axis yaw --speed 0.8 --duration 4

变化量按**起点机体系**的前进/侧移/转过给, 并且拆成三行:

    命令段     <- 命令造成的, 头条看这个
    停命令后   <- 零命令之后又滑了多少
    合计

为什么拆开: 零命令不是立刻停(deep_bridge 的看门狗 cmd_timeout_sec 默认 0.5s 靠停发
兜底, 本体自己还有惯性), 所以脚本会持续发零、等到判定停稳为止。**那是一段等待时长,
狗要是零命令下还在蠕动, 等得越久"合计"就越大** —— 把一个超时时长混进头条数字是不对
的。平均速度同理只用命令段算。

为什么用机体系而不是世界系: 狗本来就不是朝着世界 x 轴站的, 世界系的 Δx/Δy 读不出
"有没有按命令走"; 命令 x 而机体系侧移很大, 才说明有横向耦合或者打滑。世界系的三个数
也照样打出来(直接相减, 没有任何解释成分), 只是放在后面。
"""

import argparse
import math
import sys
import time

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# 跟 backend/app/config.py 的 POSE_COV_BAD、hand_lio.yaml 的 pose_cov_reject_thresh
# 是同一个约定: >= 0.99 表示定位失败, 位姿不可信。
POSE_COV_BAD = 0.99

AXES = {"x": "m/s", "y": "m/s", "yaw": "rad/s"}


class Odom(object):
    """只存最新一帧。这个脚本不做拟合, 用不着缓冲区。"""

    def __init__(self, topic):
        self.latest = None          # (t, x, y, yaw, cov)
        self.worst_cov = 0.0
        rospy.Subscriber(topic, Odometry, self._cb, queue_size=50, tcp_nodelay=True)

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        cov = msg.pose.covariance[0]
        self.worst_cov = max(self.worst_cov, cov)
        self.latest = (msg.header.stamp.to_sec() or rospy.Time.now().to_sec(),
                       p.x, p.y, yaw, cov)

    def wait(self, timeout=10.0):
        deadline = time.time() + timeout
        while self.latest is None and time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
        return self.latest


def make_twist(axis, value):
    t = Twist()
    if axis == "x":
        t.linear.x = value
    elif axis == "y":
        t.linear.y = value
    else:
        t.angular.z = value
    return t


def wrap(a):
    """把角度归到 [-pi, pi]。两个 yaw 直接相减会在 ±pi 处炸出 2pi 的假跳变。"""
    return math.atan2(math.sin(a), math.cos(a))


def delta(a, b):
    """b - a, 返回 (世界系 dx, dy, 机体系 前进/侧移, dyaw)。

    机体系那两个分量 = 把世界系位移按**起点朝向**转回来。
    """
    dx, dy = b[1] - a[1], b[2] - a[2]
    c, s = math.cos(a[3]), math.sin(a[3])
    return dx, dy, c * dx + s * dy, -s * dx + c * dy, wrap(b[3] - a[3])


def check_other_publishers(topic, force):
    """/cmd_vel 上不能有别的发布者。

    closed_loop_controller 以 100Hz 发同一个话题, 两个发布者交织 = 这次测量作废。
    而且它收到过 bspline 之后就不再依赖规划器、会继续按最后那条轨迹发, 所以要整组
    停: systemctl stop navi_planner, 别只 kill 规划器节点。
    """
    try:
        publishers = rospy.get_master().getSystemState()[2][0]
    except Exception as e:                                  # noqa: BLE001
        rospy.logwarn("查 %s 的发布者失败(%s), 跳过这项检查", topic, e)
        return []
    others = []
    for name, pubs in publishers:
        if name == topic:
            others = [n for n in pubs if n != rospy.get_name()]
    if others and not force:
        raise SystemExit(
            "还有别的节点在发 %s: %s\n"
            "  最常见的是 SCAN-Planner 的 closed_loop_controller —— 先\n"
            "    systemctl stop navi_planner\n"
            "  再跑。确实想带着它一起跑就加 --force(数据会被污染)。"
            % (topic, ", ".join(others)))
    return others


def wait_still(odom, pub, rate_hz, still_xy, still_yaw, timeout):
    """持续发零, 等到 0.5s 内几乎不动为止。返回等了多久。

    **必须持续发零而不是发一帧**: deep_bridge 的看门狗是靠"停发"兜底的, 但停发之前
    最后那一帧要是非零, 在看门狗超时(默认 0.5s)之前狗还会按它走。
    """
    rate = rospy.Rate(rate_hz)
    zero = Twist()
    t0 = time.time()
    ref = odom.latest
    ref_t = time.time()
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        pub.publish(zero)
        rate.sleep()
        now = odom.latest
        if now is None:
            continue
        if time.time() - ref_t >= 0.5:
            d = delta(ref, now)
            if math.hypot(d[0], d[1]) < still_xy and abs(d[4]) < still_yaw:
                return time.time() - t0
            ref, ref_t = now, time.time()
    return time.time() - t0


def main():
    ap = argparse.ArgumentParser(
        description="发一次 /cmd_vel, 报出 odom 三个轴的变化量",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--axis", required=True, choices=list(AXES), help="哪个轴")
    ap.add_argument("--speed", type=float, required=True,
                    help="速度, x/y 是 m/s, yaw 是 rad/s。可以是负的")
    ap.add_argument("--duration", type=float, required=True, help="持续多久 [s]")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="发布频率 [Hz]。必须远高于 deep_bridge 的 cmd_timeout_sec"
                         "(默认 0.5s)的倒数, 不然狗会一顿一顿")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/hand_lio/odom_vehicle")
    ap.add_argument("--settle-timeout", type=float, default=8.0,
                    help="回零之后最多等多久算停稳 [s]")
    ap.add_argument("--still-xy", type=float, default=0.01, help="判停稳的位移阈值 [m/0.5s]")
    ap.add_argument("--still-yaw", type=float, default=0.01, help="判停稳的转角阈值 [rad/0.5s]")
    ap.add_argument("--force", action="store_true",
                    help="即使 /cmd_vel 上还有别的发布者也照跑(数据会被污染)")
    args = ap.parse_args()

    rospy.init_node("cmd_vel_step", anonymous=True, disable_signals=True)
    unit = AXES[args.axis]
    print("== /cmd_vel 单次下发 ==")
    print("  轴 %s, 速度 %+.3f %s, 持续 %.1fs, 发布 %.0fHz"
          % (args.axis, args.speed, unit, args.duration, args.rate))

    others = check_other_publishers(rospy.resolve_name(args.cmd_topic), args.force)
    pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=10)
    odom = Odom(args.odom_topic)
    if odom.wait() is None:
        raise SystemExit("10s 内没收到 %s, 检查 hand_lio 在不在跑。" % args.odom_topic)
    if odom.latest[4] >= POSE_COV_BAD:
        raise SystemExit("odom 的 covariance[0]=%.3f >= %.2f, 定位处于失败状态, "
                         "位姿不可信 —— 先把定位搞好。" % (odom.latest[4], POSE_COV_BAD))
    # 等发布者和订阅者握上手, 不然开头几帧会丢
    time.sleep(0.5)
    if others:
        # 走到这里只能是 --force。**不能照打"没有别的发布者"** —— 那正是这行唯一
        # 会说假话的场合, 而且这次测量的数据本来就已经被污染了。
        print("  !! %s 上还有: %s —— --force 强行下发, 这次的变化量是两边命令的"
              "共同结果, 别当成标定数据" % (args.cmd_topic, ", ".join(others)))
        print("  odom cov=%.3f" % odom.latest[4])
    else:
        print("  前置检查通过(%s 上没有别的发布者, odom cov=%.3f)"
              % (args.cmd_topic, odom.latest[4]))

    start = odom.latest
    odom.worst_cov = start[4]
    cmd = make_twist(args.axis, args.speed)
    rate = rospy.Rate(args.rate)
    print("\n  下发中 ... %.1fs" % args.duration)
    t0 = time.time()
    while time.time() - t0 < args.duration and not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()
    cmd_elapsed = time.time() - t0
    at_cmd_end = odom.latest

    print("  回零, 等停稳 ...")
    settle = wait_still(odom, pub, args.rate, args.still_xy, args.still_yaw,
                        args.settle_timeout)
    end = odom.latest

    d_all = delta(start, end)
    d_cmd = delta(start, at_cmd_end)
    d_coast = delta(at_cmd_end, end)

    # 分三行给, 而且**命令段排第一**。合计值里含"等停稳"那段时间里的位移, 而那段
    # 时间是个等待时长 —— 狗要是零命令下还在蠕动, 等得越久合计越大。把一个超时时长
    # 混进头条数字是不对的, 所以头条给命令段, 滑行和合计另列。
    print("\nodom 变化量(起点机体系) —— 判断'有没有按命令走'看这组:")
    print("              前进(m)    侧移(m)   转过(°)      用时")
    print("  命令段     %+8.4f   %+8.4f   %+7.2f    %5.2fs   ← 命令造成的"
          % (d_cmd[2], d_cmd[3], math.degrees(d_cmd[4]), cmd_elapsed))
    print("  停命令后   %+8.4f   %+8.4f   %+7.2f    %5.2fs"
          % (d_coast[2], d_coast[3], math.degrees(d_coast[4]), settle))
    print("  合计       %+8.4f   %+8.4f   %+7.2f"
          % (d_all[2], d_all[3], math.degrees(d_all[4])))
    print("\n世界系(直接相减): Δx %+.4f m   Δy %+.4f m   Δyaw %+.2f°"
          % (d_all[0], d_all[1], math.degrees(d_all[4])))

    # 平均速度只能用**命令段**算: 合计里含滑行, 拿它除以命令时长会偏大
    main_moved = {"x": d_cmd[2], "y": d_cmd[3], "yaw": d_cmd[4]}[args.axis]
    avg = main_moved / cmd_elapsed if cmd_elapsed > 1e-6 else float("nan")
    print("\n主轴(%s)命令段平均速度 %+.4f %s, 命令是 %+.3f" % (args.axis, avg, unit, args.speed))
    if abs(args.speed) > 1e-6:
        print("  比值 %.3f  (1.0 = 说什么走什么; 明显小于 1 多半是死区吃掉了一截)"
              % (avg / args.speed))

    if settle >= args.settle_timeout - 0.05:
        print("\n!! 等了 %.1fs 还没判定停稳(--settle-timeout 上限)。要么狗零命令下"
              "仍在蠕动," % settle)
        print("   要么 --still-xy/--still-yaw 压在了 odom 噪声底以下。**这种情况下"
              "'停命令后'")
        print("   和'合计'两行都不可信** —— 那段时间的蠕动全算进去了; 看'命令段'那行。")
    if odom.worst_cov >= POSE_COV_BAD:
        print("\n!! 这段里 odom 的 covariance[0] 最高到过 %.3f(>= %.2f, 定位失败), "
              "上面的数都不可信。" % (odom.worst_cov, POSE_COV_BAD))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
