#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只发 /cmd_vel, 不订阅任何 odom。选一个轴、给一个速度、给一个时长, 下发, 回零。

跟 tools/cmd_vel_step.py 的唯一区别就是**不看 odom**, 所以:

  * hand_lio 没跑、定位挂了、板子上根本没有 /hand_lio/odom_vehicle —— 照样能用。
  * 代价是**全程没有任何反馈**: 狗到底动没动、走了多远、停没停, 这个脚本一概不知道。
    要量位移就拿卷尺, 或者改用 cmd_vel_step.py。

因为看不到"停稳了没有", 回零只能按固定时长发(--zero-duration), 不能像 cmd_vel_step
那样等到判定静止为止。**这个时长不能省也不能太短**: deep_bridge 的看门狗
(cmd_timeout_sec 默认 0.5s)是靠"停发"兜底的, 但停发之前最后那一帧要是非零, 在看门狗
超时之前狗还会按它走。所以必须持续发零发够时间, 而不是发一帧零就退出。

    systemctl stop navi_planner          # 必须! 否则两个发布者抢 /cmd_vel
    python3 tools/cmd_vel_send.py --axis x   --speed 0.4 --duration 3
    python3 tools/cmd_vel_send.py --axis yaw --speed 0.8 --duration 4

**狗会真的动起来, 而且这个脚本不会自己发现任何异常。** 跑之前确认周围没人没障碍,
手边留着急停。
"""

import argparse
import sys
import time

import rospy
from geometry_msgs.msg import Twist

AXES = {"x": "m/s", "y": "m/s", "yaw": "rad/s"}


def make_twist(axis, value):
    t = Twist()
    if axis == "x":
        t.linear.x = value
    elif axis == "y":
        t.linear.y = value
    else:
        t.angular.z = value
    return t


def check_other_publishers(topic, force):
    """/cmd_vel 上不能有别的发布者。

    closed_loop_controller 以 100Hz 发同一个话题, 两个发布者交织 = 狗的实际动作是
    两边命令的混合, 谁也说不清。而且它收到过 bspline 之后就不再依赖规划器、会继续按
    最后那条轨迹发, 所以要整组停: systemctl stop navi_planner, 别只 kill 规划器节点。

    (这个函数跟 cmd_vel_step.py 里的是同一份。**故意不抽成公共模块** —— 板子环境
     各不相同, 有的能 git pull 有的只能单个 scp 文件过去, 单文件能独立跑更重要。)
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
            "  再跑。确实想带着它一起跑就加 --force(狗的动作会是两边命令的混合)。"
            % (topic, ", ".join(others)))
    return others


def publish_for(pub, twist, seconds, rate_hz, label):
    """按 rate_hz 持续发同一条命令, 发够 seconds。返回实际用时。

    **必须持续发**, 不能发一帧就等: deep_bridge 的 cmd_timeout_sec 默认 0.5s, 发布
    频率低于 2Hz 狗就会一顿一顿地走。
    """
    rate = rospy.Rate(rate_hz)
    t0 = time.time()
    next_tick = 1.0
    while not rospy.is_shutdown():
        el = time.time() - t0
        if el >= seconds:
            break
        pub.publish(twist)
        if el >= next_tick:
            print("    %s %.0f/%.1fs" % (label, el, seconds))
            sys.stdout.flush()
            next_tick += 1.0
        rate.sleep()
    return time.time() - t0


def main():
    ap = argparse.ArgumentParser(
        description="只发 /cmd_vel, 不订阅 odom(开环, 没有任何反馈)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--axis", required=True, choices=list(AXES), help="哪个轴")
    ap.add_argument("--speed", type=float, required=True,
                    help="速度, x/y 是 m/s, yaw 是 rad/s。可以是负的")
    ap.add_argument("--duration", type=float, required=True, help="持续多久 [s]")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="发布频率 [Hz]。必须远高于 deep_bridge 的 cmd_timeout_sec"
                         "(默认 0.5s)的倒数, 不然狗会一顿一顿")
    ap.add_argument("--zero-duration", type=float, default=2.0,
                    help="结束后持续发零多久 [s]。没有 odom 就判不了停稳, 只能按固定"
                         "时长发。**别设太小**: 看门狗超时前最后一帧非零的话狗还会走")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--force", action="store_true",
                    help="即使 /cmd_vel 上还有别的发布者也照发")
    args = ap.parse_args()

    rospy.init_node("cmd_vel_send", anonymous=True, disable_signals=True)
    unit = AXES[args.axis]
    print("== /cmd_vel 下发(开环, 不看 odom) ==")
    print("  轴 %s, 速度 %+.3f %s, 持续 %.1fs, 发布 %.0fHz"
          % (args.axis, args.speed, unit, args.duration, args.rate))

    others = check_other_publishers(rospy.resolve_name(args.cmd_topic), args.force)
    pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=10)
    # 等订阅方跟发布者握上手, 不然开头几帧直接丢掉 —— 没有 odom 的话这件事完全看不见
    time.sleep(0.5)
    if others:
        # 走到这里只能是 --force。**不能照打"没有别的发布者"** —— 那正是这行唯一
        # 会说假话的场合, 而且是在狗即将动起来之前说的。
        print("  !! %s 上还有: %s —— --force 强行下发, 狗的动作是两边命令的混合"
              % (args.cmd_topic, ", ".join(others)))
    else:
        print("  %s 上没有别的发布者" % args.cmd_topic)

    print("\n  下发中 ...")
    sent = publish_for(pub, make_twist(args.axis, args.speed),
                       args.duration, args.rate, "命令")
    print("  回零 %.1fs ..." % args.zero_duration)
    zeroed = publish_for(pub, Twist(), args.zero_duration, args.rate, "零")

    print("\n发完了: %s %+.3f %s 持续 %.2fs, 之后发零 %.2fs"
          % (args.axis, args.speed, unit, sent, zeroed))
    print("**这个脚本不看 odom, 所以狗到底动没动、走了多远、停没停, 它都不知道。**")
    print("要数字就拿卷尺量, 或者改用 tools/cmd_vel_step.py(订阅 odom 报变化量)。")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
