#!/usr/bin/env bash
#
# record_nav_bag.sh —— 按"最小集"录一轮导航执行, 并把配套证据收成一份现场
#
# Usage:
#   tools/record_nav_bag.sh [options]
#
# Options:
#   --out DIR               输出根目录 (default: $HOME/bags)
#   --label NAME            会话名前缀, 只用 [A-Za-z0-9._-] (default: 无)
#   --duration SEC          录满 SEC 秒自动停 (default: 手动 Ctrl-C)
#   --extra T1[,T2,...]     额外话题, 逗号分隔, 可重复
#   --minimal               只录六个必需话题 (不要默认附加的诊断话题)
#   --stop-on-finish        收到 /planning/finished 就停 (无人值守跑一整轮用)
#   --no-logs               不抓 journal 和控制器 CSV
#   --journal-units "U ..." 要抓的 systemd unit, 空格分隔
#   --lz4                   用 lz4 压缩 bag (板子上未必编译了 lz4 支持, 默认关)
#   --strict                前置检查发现缺话题就退出 (default: 只警告并继续)
#   --dry-run               只做前置检查, 不录
#   -h, --help
#
# ============================ 录什么, 为什么是这些 ============================
#
# 默认十三个话题 (六个必需 + 七个诊断话题):
#   /cmd_vel                 闭环控制器的唯一输出 (closed_loop_controller.cpp:394)
#   /hand_lio/odom_vehicle   规划器和控制器共用的位姿源 (run.launch body_pose_topic)
#   /planning/bspline        参考轨迹 (scan_replan_fsm.cpp:58), 每次重规划一条
#   /preset_waypoints        navi_mode=2 的任务输入 (scan_replan_fsm.cpp:67)
#   /planning/stop           "急停"信号, 全仓库只在 callEmergencyStop 发
#   /planning/finished       整轮任务结束 (REACHED / EMERGENCY_STOP), 只发一次
#
# 另外默认加七条诊断话题:
#   /latest_imu_odom         grodom 的 200Hz 高频位姿, hand_lio 的输入。位姿上和
#                            /hand_lio/odom_vehicle 只差一个外参, 但只有它带滤波器自己
#                            估的速度 (odom_vehicle 的 twist 是全零), 排查"两次激光修正
#                            之间的前推为什么偏"要靠它
#   /lidar_pose              grodom 的扫描匹配位姿 (grodom_ros1 发, 实测 5~8Hz, 达不到 10Hz 的雷达帧率)
#   /hand_lio/odom_vehicle_stream  高频流直接合成的机体位姿。hand_lio 的 pose_source=fused 时
#                            odom_vehicle 换成了融合位姿, 靠它跟原来的位姿对比 (stream 时两者相同)
#   /hand_lio/odom_fused_shadow  融合里程计的旁路输出 (pose_source=fused 时不用再起旁路节点)
#   /hand_lio/pose_fusion_diag   位姿融合诊断
#   /deep_bridge_node/motion_status  底盘运动状态
#   /deep_bridge_node/motion_status_raw  底盘运控状态上报的 JSON 原文 (字段含义未定, 离线对比用)
# 用 --minimal 可以退回只录六个必需话题。
#
# 不录点云。这六条足以定位"参考轨迹 -> 命令 -> 响应"这条执行链路:
# 横向/纵向跟踪误差、命令饱和、cmd_vel 与 odom 的传动关系、odom 实际更新率、
# 每条轨迹的存活时间与被打断的频率。它定位不了的是"为什么这条轨迹长这样"
# (要 /grid_map/occupancy*)。底盘运动状态可结合 motion_status 和 deep_bridge journal 排查。
#
# 所以本脚本除了录 bag, 还做三件在板子上必须做的事:
#   1. 录之前核对话题和关键参数, 并明确告诉你"现在开始录了, 去点导航"。
#      /preset_waypoints 不 latch 且订阅队列 1, 录制起晚了这一轮的任务输入就
#      永久缺失 —— 这是最小集里最容易白录的一条。
#   2. 录完把配套证据收进同一个目录: 控制器的逐拍 CSV (比 bag 里的 /cmd_vel
#      信息更多: exec_time / x_des / y_des / cov / odom_age / ff_*)和各服务的
#      journal。
#   3. 录完给结论: 每个话题收到多少条、实际频率是多少、这轮 bag 能不能用。
#      /preset_waypoints=0 或 /planning/bspline=0 的 bag 是废的, 要重录。
#
# ================================ 用法示例 ==================================
#
#   # 板子上, 先在网页上把服务都起好, 但**先别点导航**
#   tools/record_nav_bag.sh --label corridor1
#   ... 看到 "现在开始录了" 之后, 再去网页点导航 ...
#   ... 这一轮走完, Ctrl-C ...
#
#   # 无人值守跑一轮: 收到 /planning/finished 自动停
#   tools/record_nav_bag.sh --label corridor1 --stop-on-finish --duration 1200
#
#   # 想同时看栅格(比点云小, 但仍是 5Hz 整片重发, 体积可能和点云同量级)
#   tools/record_nav_bag.sh --extra /grid_map/occupancy,/rosout
#
#   # 只录六个必需话题 (不录默认诊断话题)
#   tools/record_nav_bag.sh --label corridor1 --minimal
#
#   # 只体检不录
#   tools/record_nav_bag.sh --dry-run
#
# 注意: 录制本身会给板子加负载, 建议写到有空间的盘上, 别写满根分区。
# 脚本会先查一次剩余空间。
#
# ============================================================================

set -uo pipefail
# 故意不用 set -e: 收尾靠"信号 + kill -0 轮询", sleep 被 SIGINT 打断会返回 130,
# -e 会在做善后(journal / CSV / 结论)之前把脚本杀掉, 而善后正是这个脚本的价值。

# ------------------------------------------------------------------ 默认值 ---
OUT_DIR=${HOME:-/tmp}/bags
LABEL=""
DURATION=""
STOP_ON_FINISH=0
NO_LOGS=0
USE_LZ4=0
STRICT=0
DRY_RUN=0
JOURNAL_UNITS="deep_bridge navi_planner hand_lio localization"
MIN_FREE_MB=500
EXTRA_TOPICS=()
MINIMAL=0

REQUIRED_TOPICS=(
    /cmd_vel
    /hand_lio/odom_vehicle
    /planning/bspline
    /preset_waypoints
    /planning/stop
    /planning/finished
)

# 默认附加话题: 不进"必需"判定 (缺了只 warn, --strict 也不会因此退出),
# 但默认就录, 方便排查位姿融合和底盘运动状态。
DEFAULT_EXTRA_TOPICS=(
    /latest_imu_odom
    /lidar_pose
    /hand_lio/odom_vehicle_stream
    /hand_lio/odom_fused_shadow
    /hand_lio/pose_fusion_diag
    /deep_bridge_node/motion_status
    /deep_bridge_node/motion_status_raw
)

# ------------------------------------------------------------------- 输出 ---
say()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
ok()   { printf '[ OK ] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*"; }
die()  { fail "$*"; exit 1; }

usage() { sed -n '2,/^# =====*$/p' "$0" | sed 's/^# \{0,1\}//' | sed '/^=\{10,\}$/d'; }

# ------------------------------------------------------------------ 参数解析 ---
while [ $# -gt 0 ]; do
    case "$1" in
        --out)            OUT_DIR=${2:?--out 需要目录}; shift 2 ;;
        --out=*)          OUT_DIR=${1#*=}; shift ;;
        --label)          LABEL=${2:?--label 需要名字}; shift 2 ;;
        --label=*)        LABEL=${1#*=}; shift ;;
        --duration)       DURATION=${2:?--duration 需要秒数}; shift 2 ;;
        --duration=*)     DURATION=${1#*=}; shift ;;
        --extra)          IFS=',' read -r -a _extra <<< "${2:?--extra 需要话题}"; EXTRA_TOPICS+=("${_extra[@]}"); shift 2 ;;
        --extra=*)        IFS=',' read -r -a _extra <<< "${1#*=}"; EXTRA_TOPICS+=("${_extra[@]}"); shift ;;
        --minimal)        MINIMAL=1; shift ;;
        --stop-on-finish) STOP_ON_FINISH=1; shift ;;
        --no-logs)        NO_LOGS=1; shift ;;
        --journal-units)  JOURNAL_UNITS=${2:?--journal-units 需要 unit 名}; shift 2 ;;
        --journal-units=*) JOURNAL_UNITS=${1#*=}; shift ;;
        --lz4)            USE_LZ4=1; shift ;;
        --strict)         STRICT=1; shift ;;
        --dry-run)        DRY_RUN=1; shift ;;
        -h|--help)        usage; exit 0 ;;
        *)                die "未知参数: $1 (--help 看用法)" ;;
    esac
done

if [ -n "$DURATION" ] && ! [[ "$DURATION" =~ ^[0-9]+$ ]]; then
    die "--duration 要是正整数秒数, 收到 '$DURATION'"
fi
if [ -n "$LABEL" ] && ! [[ "$LABEL" =~ ^[A-Za-z0-9._-]+$ ]]; then
    die "--label 只允许 [A-Za-z0-9._-], 收到 '$LABEL'"
fi

ALL_TOPICS=("${REQUIRED_TOPICS[@]}")
OPT_TOPICS=()
[ "$MINIMAL" -eq 0 ] && OPT_TOPICS+=("${DEFAULT_EXTRA_TOPICS[@]}")
[ ${#OPT_TOPICS[@]} -gt 0 ] && ALL_TOPICS+=("${OPT_TOPICS[@]}")
[ ${#EXTRA_TOPICS[@]} -gt 0 ] && ALL_TOPICS+=("${EXTRA_TOPICS[@]}")

TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/record_nav_bag.XXXXXX")
cleanup_tmp() { rm -rf "$TMP_DIR"; }
trap cleanup_tmp EXIT

# --------------------------------------------------------------- ROS 环境 ---
# 板子上 rosbag 一般直接在 PATH 里; 开发机(ROS 在 mamba 环境里)和路径不同的机器
# 走下面这条兜底, 跟 deep_bridge.sh / hand_lio.sh 的做法保持一致。
if ! command -v rosbag >/dev/null 2>&1; then
    if command -v mamba >/dev/null 2>&1; then
        set +u   # mamba/conda 的 hook 会碰未定义变量, 与本脚本的 set -u 冲突
        eval "$(mamba shell hook --shell bash)" 2>/dev/null || true
        mamba activate ros_host 2>/dev/null || true
        set -u
    fi
fi
if ! command -v rosbag >/dev/null 2>&1; then
    for _ws in "${ROS_WS:-}" "$HOME/ros1_ws" /home/cat/ros1_ws "$HOME/catkin_ws"; do
        if [ -n "$_ws" ] && [ -f "$_ws/devel/setup.bash" ]; then
            # ROS 的 setup.bash 里会引用 $ROS_DISTRO 之类的变量, 在 set -u 下会直接
            # 报 "ROS_DISTRO: unbound variable" 而整脚本退出 —— 只有"当前 shell 里
            # 没有 rosbag"(新开的 ssh / cron / 干净环境)才会走到这里, 交互式 shell
            # 因为 .bashrc 已经 source 过而不会踩到, 所以这个坑很隐蔽。
            set +u
            # shellcheck disable=SC1091
            . "$_ws/devel/setup.bash"
            set -u
            break
        fi
    done
fi

command -v rosbag   >/dev/null 2>&1 || die "找不到 rosbag。先 source 工作空间的 devel/setup.bash, 或用 ROS_WS=<ws> 指定"
command -v rostopic >/dev/null 2>&1 || die "找不到 rostopic"
command -v rosparam >/dev/null 2>&1 || die "找不到 rosparam"

# 有 timeout 就用它兜住"ROS master 没起来时命令挂住"的情况。
TMO=""
command -v timeout >/dev/null 2>&1 && TMO=timeout
rt() { if [ -n "$TMO" ]; then timeout 5 "$@"; else "$@"; fi; }

# ------------------------------------------------------------- 前置检查(1/2) ---
if ! rt rostopic list > "$TMP_DIR/live_topics.txt" 2>"$TMP_DIR/roscore.err"; then
    fail "连不上 ROS master (rostopic list 失败)$([ -s "$TMP_DIR/roscore.err" ] && echo ": $(head -c 200 "$TMP_DIR/roscore.err")")"
    die  "ROS_MASTER_URI=${ROS_MASTER_URI:-<未设置>}。板子上先起 roscore —— 用 deep_bridge/tools/deep_bridge.sh start 的话它会顺手把 roscore 带起来"
fi
ok "ROS master 可达 (ROS_MASTER_URI=${ROS_MASTER_URI:-<默认>})"

has_topic() { grep -qxF "$1" "$TMP_DIR/live_topics.txt"; }

# $1 是 rosbag info 的**文本输出文件**, 不是 bag 本身。
# 两个坑都要防:
#   1. topics 段落有两种排法 —— 第一个话题跟在 "topics:" 后面, 其余各占一行。
#      所以不能按 $1 取话题名, 而是扫到单位词再往前两个字段取 (计数, 话题名)。
#   2. **只有 1 条消息的话题, rosbag info 打印的是单数 "1 msg"**(复数才是 msgs)。
#      2026-10-01 那轮真机数据上, /preset_waypoints 和 /planning/finished 都恰好只有
#      1 条, 这里只认 msgs 就把它们判成 0 条并给了 FAIL —— 包其实是完整的。
topic_count() {  # $1=rosbag info 输出文件 $2=话题名
    awk -v want="$2" '
        /msg/ {
            for (i = 1; i <= NF; ++i) {
                if ($i == "msg" || $i == "msgs" || $i == "msg:" || $i == "msgs:") {
                    c = $(i - 1); t = $(i - 2)
                    if (t ~ /^\// && t == want) { print c; exit }
                    break
                }
            }
        }' "$1"
}

MISSING=(); MISSING_OPT=()
for t in "${ALL_TOPICS[@]}"; do
    has_topic "$t" && continue
    _is_opt=0
    for _o in "${OPT_TOPICS[@]}"; do [ "$_o" = "$t" ] && _is_opt=1; done
    if [ "$_is_opt" -eq 1 ]; then MISSING_OPT+=("$t"); else MISSING+=("$t"); fi
done

NODES=$(rt rosnode list 2>/dev/null || true)
node_alive() { printf '%s\n' "$NODES" | grep -qxF "$1"; }

if rt rosnode list >/dev/null 2>&1; then
    printf '  节点: '
    for n in /scan_planner_node /closed_loop_controller /hand_lio_node /deep_bridge_node; do
        node_alive "$n" && printf '%s✓ ' "$n" || printf '%s✗ ' "$n"
    done
    printf '\n'
fi

# ---------------------------------------------------------- 前置检查: 参数 ---
# 两件最容易"录了但录错东西"的事:
#   1. body_pose_topic 不是 /hand_lio/odom_vehicle —— 那录到的 odom 就不是规划器
#      和控制器实际用的那一路, 这个 bag 的跟踪误差全是错的。
#   2. navi_mode 不是 2 —— 这个最小集里的 /preset_waypoints 在 mode 1/3 下根本
#      没人订阅, 三轮录下来都是空的。
BODY_TOPIC=$(rt rosparam get /body_pose_topic 2>/dev/null | tr -d '\r' || true)
NAVI_MODE=$(rt rosparam get /scan_planner_node/fsm/navi_mode 2>/dev/null | tr -d '\r' || true)
CTRL_LOG_DIR=$(rt rosparam get /closed_loop_controller/log_dir 2>/dev/null | tr -d '\r' || true)

if [ -n "$BODY_TOPIC" ] && [ "$BODY_TOPIC" != "/hand_lio/odom_vehicle" ]; then
    warn "body_pose_topic = '$BODY_TOPIC', 不是 /hand_lio/odom_vehicle。规划器和控制器用的是前者, 本脚本录的是后者 —— 这个 bag 算不出真实的跟踪误差"
elif [ -z "$BODY_TOPIC" ]; then
    warn "读不到 /body_pose_topic (planner 没起?), 无法确认录的 odom 就是它用的那一路"
else
    ok "body_pose_topic = /hand_lio/odom_vehicle (和本脚本录的是同一路)"
fi

if [ -n "$NAVI_MODE" ]; then
    if [ "$NAVI_MODE" = "2" ]; then
        ok "navi_mode = 2 (/preset_waypoints 是任务输入, 和最小集的假设一致)"
    else
        warn "navi_mode = $NAVI_MODE, 不是 2。本最小集按 mode 2 设计 —— mode 1 要录 /move_base_simple/goal, mode 3 要录 /initial_path, 否则一样白录"
    fi
else
    warn "读不到 /scan_planner_node/fsm/navi_mode (planner 没起?)"
fi

if [ "$NO_LOGS" -eq 0 ]; then
    if [ -n "$CTRL_LOG_DIR" ]; then
        ok "控制器逐拍 CSV 目录: $CTRL_LOG_DIR"
    else
        warn "读不到 /closed_loop_controller/log_dir (控制器没起?) —— 那份 CSV 比 bag 里的 /cmd_vel 信息更多, 是这一轮最有用的证据之一"
    fi
fi

# ------------------------------------------------------------ 前置检查: 话题 ---
if [ ${#MISSING[@]} -gt 0 ]; then
    warn "下面这些话题当前不在 rostopic list 里:"
    for t in "${MISSING[@]}"; do
        case "$t" in
            /cmd_vel)               why="闭环控制器没起 (closed_loop_controller)" ;;
            /hand_lio/odom_vehicle) why="hand_lio_node 没起, 或定位服务没起" ;;
            /planning/bspline|/planning/stop|/planning/finished)
                                    why="scan_planner_node 没起, 或它启动时挂掉了" ;;
            /preset_waypoints)      why="scan_planner_node 没起, 或 navi_mode != 2" ;;
            /latest_imu_odom)       why="grodom_ros1 没起 (定位服务), 或它这版不发这条路" ;;
            /lidar_pose)            why="grodom_ros1 没起, 或它这版不发这条路" ;;
            *)                      why="自己确认一下谁负责发它" ;;
        esac
        printf '        %-28s %s\n' "$t" "$why"
    done
    if [ "$STRICT" -eq 1 ]; then
        die "--strict: 缺话题直接退出"
    fi
    warn "缺的话题照样会录(有发布者就会进 bag), 但相应地那部分证据是空的 —— 继续"
else
    ok "六个必需话题都在"
fi
if [ ${#OPT_TOPICS[@]} -gt 0 ]; then
    if [ ${#MISSING_OPT[@]} -gt 0 ]; then
        warn "默认附加的诊断话题当前不在 rostopic list 里: ${MISSING_OPT[*]}"
        warn "  -> 照录, 但相应的诊断数据可能缺失 (不影响必需话题的判定)"
    else
        ok "默认附加的诊断话题都在"
    fi
fi

# /planning/bspline 没人订阅 = 控制器不在, cmd_vel 不会有人发。
if has_topic /planning/bspline; then
    if rt rostopic info /planning/bspline 2>/dev/null | awk '/^Subscribers:/{f=1;next} f&&NF{print;exit}' | grep -q .; then
        ok "/planning/bspline 有订阅者 (控制器在听)"
    else
        warn "/planning/bspline 没有任何订阅者 —— 闭环控制器没在跑, 这一轮不会有 /cmd_vel"
    fi
fi

# ------------------------------------------------------------------ 磁盘 ---
mkdir -p "$OUT_DIR" || die "建不出输出目录 $OUT_DIR"
FREE_MB=$(df -Pk "$OUT_DIR" 2>/dev/null | awk 'NR==2{printf "%d", $4/1024}')
if [ -n "${FREE_MB:-}" ]; then
    if [ "$FREE_MB" -lt 50 ]; then
        die "$OUT_DIR 所在分区只剩 ${FREE_MB} MB, 先清盘"
    elif [ "$FREE_MB" -lt "$MIN_FREE_MB" ]; then
        warn "$OUT_DIR 所在分区只剩 ${FREE_MB} MB (建议 ≥ ${MIN_FREE_MB} MB)。最小集约 0.2~0.5 MB/s, 但 bag 写满盘会把整台机器拖垮"
    else
        ok "$OUT_DIR 可用 ${FREE_MB} MB"
    fi
fi

if pgrep -af '[r]osbag record' >/dev/null 2>&1; then
    warn "已经有 rosbag record 在跑:"
    pgrep -af '[r]osbag record' | sed 's/^/        /'
    warn "两个录制器会各自订阅一遍, 徒增负载也容易搞混文件"
fi

if [ "$DRY_RUN" -eq 1 ]; then
    say "--dry-run: 体检到此为止, 没录。"
    exit 0
fi

# ------------------------------------------------------------------ 会话目录 ---
STAMP=$(date +%Y%m%d_%H%M%S)
SESSION_NAME="nav${LABEL:+_$LABEL}_$STAMP"
SESSION_DIR="$OUT_DIR/$SESSION_NAME"
mkdir -p "$SESSION_DIR"
BAG_PATH="$SESSION_DIR/nav.bag"

cp -f "$TMP_DIR/live_topics.txt" "$SESSION_DIR/topics_at_start.txt" 2>/dev/null || true
[ -n "$NODES" ] && printf '%s\n' "$NODES" > "$SESSION_DIR/nodes_at_start.txt"
rt rosparam dump "$SESSION_DIR/params.yaml" >/dev/null 2>&1 || true

# ------------------------------------------------------------------ 起录 ---
BAG_ARGS=(record -O "$BAG_PATH")
if rosbag record --help 2>&1 | grep -q -- '--tcpnodelay'; then
    BAG_ARGS+=(--tcpnodelay)
fi
if [ "$USE_LZ4" -eq 1 ] && rosbag record --help 2>&1 | grep -q -- '--lz4'; then
    BAG_ARGS+=(--lz4)
elif [ "$USE_LZ4" -eq 1 ]; then
    warn "这个 rosbag 没有 --lz4, 按不压缩录"
fi
[ -n "$DURATION" ] && BAG_ARGS+=("--duration=$DURATION")
BAG_ARGS+=("${ALL_TOPICS[@]}")

START_EPOCH=$(date +%s)
START_ISO=$(date '+%Y-%m-%d %H:%M:%S')

{
    echo "# 会话: $SESSION_NAME"
    echo "# 开始: $START_ISO"
    echo "# 主机: $(hostname 2>/dev/null)  用户: $(id -un 2>/dev/null)"
    echo "# ROS_MASTER_URI: ${ROS_MASTER_URI:-<默认>}"
    echo "# 命令: rosbag ${BAG_ARGS[*]}"
    echo "# 必需话题:"
    printf '#   %s\n' "${REQUIRED_TOPICS[@]}"
    if [ ${#OPT_TOPICS[@]} -gt 0 ]; then
        printf '#   %s\n' "${OPT_TOPICS[@]}"
    fi
    if [ ${#EXTRA_TOPICS[@]} -gt 0 ]; then
        echo "# 额外话题:"
        printf '#   %s\n' "${EXTRA_TOPICS[@]}"
    fi
} > "$SESSION_DIR/session.txt"

say "输出目录: $SESSION_DIR"
rosbag "${BAG_ARGS[@]}" > "$SESSION_DIR/record.log" 2>&1 &
BAG_PID=$!

STOPPED_BY=""
on_signal() {
    STOPPED_BY=${STOPPED_BY:-信号}
    kill -INT "$BAG_PID" 2>/dev/null || true
}
trap on_signal INT TERM

# 给 rosbag 一点时间把 bag 建出来; 立刻退出说明参数被拒或 master 掉了。
sleep 1
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    fail "rosbag record 立刻退出了, 输出如下:"
    sed 's/^/        /' "$SESSION_DIR/record.log"
    die  "没有在录。常见原因: 话题名写错 / 已有同名 bag / ROS master 掉了"
fi

if [ "$STOP_ON_FINISH" -eq 1 ]; then
    if has_topic /planning/finished; then
        ( rostopic echo -n1 /planning/finished > "$TMP_DIR/finished.txt" 2>/dev/null ) &
        FIN_PID=$!
    else
        warn "--stop-on-finish 没生效: /planning/finished 当前没人发, 等不到结束信号。这一轮只能手动 Ctrl-C"
    fi
fi

cat <<EOF

    ================================================================
      ★ 现在开始录了。去网页上点导航 / 下发路线。

      顺序很重要: /preset_waypoints 不 latch 且订阅队列 1, 录制起晚了
      这一轮的任务输入就永久缺失 —— 那样的 bag 要重录。
    ================================================================

EOF

while kill -0 "$BAG_PID" 2>/dev/null; do
    if [ -n "${FIN_PID:-}" ] && ! kill -0 "$FIN_PID" 2>/dev/null; then
        say "收到 /planning/finished, 这一轮结束了, 停止录制"
        STOPPED_BY=${STOPPED_BY:-任务结束}
        kill -INT "$BAG_PID" 2>/dev/null || true
        break
    fi
    sleep 0.5
done

# rosbag 收到 SIGINT 后要写索引、关文件, 等它真的退出。真实的 rosbag record 是
# Python 实现的, 主循环里查标志位, 正常几十毫秒就退; 这里等 10s 是给卡死兜底。
for _ in $(seq 1 40); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 0.25; done
if kill -0 "$BAG_PID" 2>/dev/null; then
    warn "rosbag 10s 还没退出, 发 SIGTERM"
    kill -TERM "$BAG_PID" 2>/dev/null || true
    for _ in $(seq 1 8); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 0.25; done
fi
if kill -0 "$BAG_PID" 2>/dev/null; then
    # 强杀必然留下没写完索引的 bag。别删: rosbag reindex 一般能救回来。
    warn "还是没退, SIGKILL。这个 bag 的索引可能不完整, 后面 rosbag info 失败的话用: rosbag reindex <bag>"
    kill -KILL "$BAG_PID" 2>/dev/null || true
    sleep 0.5
fi
trap - INT TERM
END_EPOCH=$(date +%s)
END_ISO=$(date '+%Y-%m-%d %H:%M:%S')
ELAPSED=$((END_EPOCH - START_EPOCH))
sleep 0.5

# 停止方式要写进 session.txt: "手动 Ctrl-C" 和 "录满时长" 决定这一轮该不该算完整。
if [ -z "$STOPPED_BY" ]; then
    if [ -n "$DURATION" ]; then STOPPED_BY="录满 ${DURATION}s"; else STOPPED_BY="手动 Ctrl-C"; fi
fi
say "录制结束 (${ELAPSED}s, ${STOPPED_BY})"

# ------------------------------------------------------------------ 结论 ---
VERDICT=""
verdict_add() { VERDICT="${VERDICT}$1
"; }

if [ ! -s "$BAG_PATH" ]; then
    fail "没有 bag 文件 ($BAG_PATH)。rosbag 输出:"
    sed 's/^/        /' "$SESSION_DIR/record.log"
    die  "这一轮什么都没录到"
fi

rosbag info "$BAG_PATH" > "$SESSION_DIR/bag_info.txt" 2>&1 || {
    fail "rosbag info 读不了这个 bag (大概率是被强杀, 索引没写完)。原样保留, 可以试: rosbag reindex ${BAG_PATH}.orig"
    sed 's/^/        /' "$SESSION_DIR/record.log" | tail -5
    die  "先别删, 把 $SESSION_DIR 整个留下"
}

BAG_SIZE=$(du -h "$BAG_PATH" 2>/dev/null | cut -f1)
say "bag 大小: ${BAG_SIZE:-?}, 时长: ${ELAPSED}s"
echo
sed 's/^/    /' "$SESSION_DIR/bag_info.txt"
echo

count_of() { local c; c=$(topic_count "$SESSION_DIR/bag_info.txt" "$1"); echo "${c:-0}"; }

check_topic() {  # $1=话题 $2=期望频率(hz, 空=不查) $3=缺了要不要判废
    local t=$1 hz=$2 required=$3 n rate flag
    n=$(count_of "$t")
    if [ "$n" = "0" ]; then
        if [ "$required" = "yes" ]; then
            fail "$t: 0 条 —— 关键数据缺失"
            verdict_add "FAIL  $t: 0 条"
        else
            warn "$t: 0 条"
            verdict_add "warn  $t: 0 条"
        fi
        return
    fi
    if [ -n "$hz" ] && [ "$ELAPSED" -gt 0 ]; then
        rate=$(awk -v n="$n" -v d="$ELAPSED" 'BEGIN{printf "%.1f", n/d}')
        flag=""
        # 期望频率的一半都不到, 基本就是断流/没跑起来
        awk -v r="$rate" -v h="$hz" 'BEGIN{exit !(r < h*0.5)}' && flag="  <-- 明显偏低"
        ok "$t: $n 条 (~${rate} Hz, 期望 ~${hz} Hz)$flag"
        verdict_add "ok    $t: $n 条 (~${rate} Hz, 期望 ~${hz} Hz)$flag"
    else
        ok "$t: $n 条"
        verdict_add "ok    $t: $n 条"
    fi
}

check_topic /cmd_vel               100 yes
check_topic /hand_lio/odom_vehicle 200 yes
check_topic /planning/bspline          "" yes
check_topic /preset_waypoints          "" yes
if [ "$MINIMAL" -eq 0 ]; then
    check_topic /latest_imu_odom                  200 no
    check_topic /lidar_pose                         5 no
    check_topic /hand_lio/odom_vehicle_stream     200 no
    check_topic /hand_lio/odom_fused_shadow        "" no
    check_topic /hand_lio/pose_fusion_diag         "" no
    check_topic /deep_bridge_node/motion_status    "" no
    check_topic /deep_bridge_node/motion_status_raw "" no
fi

# /planning/finished: 整轮任务结束才发一次。0 条不一定是错(可能是中途停的录),
# 但必须说出来, 因为"这一轮到底怎么结束的"是这个 bag 想回答的问题之一。
N_FIN=$(count_of /planning/finished)
if [ "$N_FIN" = "0" ]; then
    warn "/planning/finished: 0 条 —— 录制窗口内整轮任务没有结束(正常到点或急停都会发一条)。是中途停的录, 还是 planner 卡住了?"
    verdict_add "warn  /planning/finished: 0 条 (整轮未结束或录制窗口没覆盖到)"
else
    ok "/planning/finished: $N_FIN 条"
    verdict_add "ok    /planning/finished: $N_FIN 条"
fi

N_STOP=$(count_of /planning/stop)
if [ "$N_STOP" != "0" ]; then
    warn "/planning/stop: $N_STOP 条 —— 这一轮出现过急停信号 (callEmergencyStop)"
    verdict_add "note  /planning/stop: $N_STOP 条 (本轮到过急停)"
fi

# ------------------------------------------------------------------ 配套证据 ---
CSV_SRC="${CTRL_LOG_DIR:-$HOME/scan_planner_track_logs}"
if [ "$NO_LOGS" -eq 0 ]; then
    if [ -d "$CSV_SRC" ]; then
        mkdir -p "$SESSION_DIR/csv"
        # 只收本轮的会话 (开始前 60s 之内动过的都算, 给"录制起晚了"留余量)。
        # -newermt @epoch 是 GNU findutils 的用法, 板子上(Ubuntu)有; 万一没有,
        # 下面那句区分出来, 免得把"筛选失败"说成"控制器没写日志"。
        find "$CSV_SRC" -maxdepth 1 -name 'track_*.csv' -newermt "@$((START_EPOCH - 60))" \
            -exec cp -f {} "$SESSION_DIR/csv/" \; 2>/dev/null || true
        n_csv=$(find "$SESSION_DIR/csv" -name 'track_*.csv' 2>/dev/null | wc -l | tr -d ' ')
        n_all=$(find "$CSV_SRC" -maxdepth 1 -name 'track_*.csv' 2>/dev/null | wc -l | tr -d ' ')
        if [ "$n_csv" -gt 0 ]; then
            ok "控制器逐拍 CSV: $n_csv 份 -> csv/"
            verdict_add "ok    csv/: $n_csv 份控制器逐拍 CSV"
        elif [ "$n_all" -gt 0 ]; then
            warn "$CSV_SRC 里有 $n_all 份 CSV, 但按时间一份都没筛出本轮 —— 时间筛选在本机可能不支持, 自己按时间拷需要的"
            verdict_add "warn  csv/: 目录里有 $n_all 份, 但没筛出本轮"
        else
            warn "$CSV_SRC 里没有 track_*.csv —— 检查 closed_loop_controller/log_dir, 或这一轮没进入过跟踪"
            verdict_add "warn  csv/: 空 (控制器没写跟踪日志)"
        fi
    else
        warn "找不到控制器日志目录 $CSV_SRC"
        verdict_add "warn  csv/: 目录不存在 $CSV_SRC"
    fi

    if command -v journalctl >/dev/null 2>&1; then
        J_ARGS=()
        for u in $JOURNAL_UNITS; do J_ARGS+=(-u "$u"); done
        journalctl "${J_ARGS[@]}" --since "@$START_EPOCH" --until "@$END_EPOCH" --no-pager \
            > "$SESSION_DIR/journal.log" 2>&1 || true
        if [ -s "$SESSION_DIR/journal.log" ] && ! grep -q '^-- No entries --$' "$SESSION_DIR/journal.log"; then
            ok "journal: $(wc -l < "$SESSION_DIR/journal.log" | tr -d ' ') 行 -> journal.log"
            verdict_add "ok    journal.log: $(wc -l < "$SESSION_DIR/journal.log" | tr -d ' ') 行"
        else
            warn "journal 是空的 —— 读不到就用 sudo, 或把用户加进 systemd-journal 组; 也可以现场另开一个终端 journalctl -u deep_bridge -f 存下来"
            verdict_add "warn  journal.log: 空 (权限或 unit 名不对)"
        fi
    else
        warn "这台机器上没有 journalctl —— 底盘侧真值(deep_bridge 的 motion_state/gait/hes/usage_mode)只能现场手动抓"
        verdict_add "warn  journal.log: 无 journalctl"
    fi
fi

{
    echo "# 结束: $END_ISO  (时长 ${ELAPSED}s, 停止方式 ${STOPPED_BY})"
    echo "# bag: $BAG_PATH ($BAG_SIZE)"
    echo "# 每话题条数:"
    sed 's/^/#   /' <<< "$VERDICT"
} >> "$SESSION_DIR/session.txt"

echo
echo "================ 结论 ================"
printf '%s' "$VERDICT" | sed 's/^/  /'
echo "======================================"
echo
if printf '%s' "$VERDICT" | grep -q '^FAIL'; then
    fail "这个 bag 不好用 (上面有 FAIL)。最可能是 /preset_waypoints 录制起晚了, 或者规划器/控制器当时没在跑 —— 修完重录一轮"
else
    ok "可以用。现场都在: $SESSION_DIR"
fi
cat <<EOF

接下来:
  rosbag info "$BAG_PATH"
  离线复算跟踪误差时注意: 控制器的 exec_time 在原地转圈时会冻结
  (closed_loop_controller.cpp:341-350), 所以 desired 位置不等于把墙钟差
  直接代进 bspline; 省事的做法是直接用 csv/ 里那份逐拍日志。
  参数快照在 params.yaml, 复算要的 kp_pos/max_vx/full_scale_* 都在里面。

EOF
