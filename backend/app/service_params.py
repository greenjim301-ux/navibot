"""
「参数配置」页的后端: 读/写某个服务的 ROS 参数 yaml。

只认 config.SERVICE_PARAM_SCHEMAS 里声明过的服务 id 和键名——跟 SYSTEMD_SERVICES
同一个理由, 挡掉"随便传个 key 就能改 yaml"的口子。每个键的类型/范围/可选值也在
那张表里, 这里照着校验。

**写回是按行原地替换值, 不是 yaml 重新序列化。** deep_bridge.yaml 里那些注释
(协议出处、厂商的步态速度范围表、"实测这台本体加密是关掉的"这类坑)信息量比配置
本身还大, 用 PyYAML 读出来再 dump 回去会把它们全部冲掉。所以这里只做**精确替换
value 那一段**, 行内注释、缩进、空行、文件里其它所有内容原样不动。

支持两种写法, 因为实际的 yaml 两种都有:

    (a) 一行式     enable_virtual_obstacles: true   # 行内注释
    (b) 跨行式     blind:
                     0.35 # 盲区半径 [m]
                     # 下面还能接着写好几行纯注释

(b) 是 hand_lio.yaml 里 blind / virtual_obstacle_* 那几个键的写法(值单独一行、
缩进, 后面跟一串解释性注释)。只认第一个"缩进且有实际内容"的行当值; 中间的空行和
纯注释行跳过; 一旦遇到不缩进的行就判定这个键没有标量值(说明它是个 map/list 或者
空值), 不乱猜。

roslaunch XML 也是同一套路子(schema 里 `format: "roslaunch"`), 键就是 name 属性的
完整值, 改的是它同一个标签里的 value= 或 default= :

    <arg   name="max_vel"                        default="0.75"/>
    <param name="grid_map/double_cylinder_radius" value="0.20" />

name 是**精确匹配**的, 所以 `max_vel` 不会误伤同一个文件里的
`<param name="manager/max_vel" value="$(arg max_vel)"/>` —— 那是引用, 不是定义。

值是 `$(arg ...)` / `$(eval ...)` 这类替换表达式时, 读出来当"读不出当前值"(返回
None), 写则直接拒绝: 那个位置存的是一条引用, 拿字面量盖掉会把 launch 文件里
原本的联动关系悄悄拆掉。

列表型的键(vec3 / mat3)另走一套: 定位 `key: [`, 把 `[` 到 `]` 之间的内容整段取出
来按逗号切。写回时整段重新渲染 —— 3x3 矩阵按原样铺成三行、续行缩进对齐到 `[`,
所以值没变的时候写回去跟原文**逐字节相同**(有回归测试盯着这条)。

    # prettier-ignore
    lidar_R_body: [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]
    lidar_t_body: [0.0, 0.0, 0.0]

代价是这两个解析器都**很窄**: yaml 只处理顶层键的标量值、不处理嵌套结构;
roslaunch 只处理写在同一行里的 name/value 属性对。schema 里声明的键如果在文件里
找不到, 直接报错而不是追加一行——追加的键很可能是缩进/命名空间写错了, 静默追加
只会得到一个永远不生效的配置。
"""
import logging
import os
import re
import tempfile
from typing import Any, Dict, List, Optional, Tuple

from . import config

logger = logging.getLogger("navibot.service_params")

# 一行式: `key: value   # 可选行内注释`
# value 捕获到第一个 ` #` 之前(yaml 要求行内注释前有空白), 前后空白不计入。
def _same_line_re(key: str) -> "re.Pattern":
    return re.compile(
        r"^(?P<head>" + re.escape(key) + r"[ \t]*:[ \t]*)"
        r"(?P<value>[^#\s][^#]*?)"
        r"(?P<tail>(?:[ \t]+#.*)?[ \t]*)$"
    )


# 跨行式的第一行: `key:` 后面只剩空白或注释, 值在后面某一行。
def _key_only_re(key: str) -> "re.Pattern":
    return re.compile(r"^" + re.escape(key) + r"[ \t]*:[ \t]*(?:#.*)?$")


# 跨行式的值那一行: 缩进 + 实际内容 + 可选行内注释。
_BLOCK_VALUE_RE = re.compile(
    r"^(?P<head>[ \t]+)"
    r"(?P<value>[^#\s][^#]*?)"
    r"(?P<tail>(?:[ \t]+#.*)?[ \t]*)$"
)


def _locate(lines: List[str], key: str) -> Optional[Tuple[int, str, str, str]]:
    """在 lines(不带换行符)里找 key 的标量值所在的行。

    返回 (行号, head, value, tail) —— head + value + tail 拼回来就是原行, 所以
    改值的时候只换中间那段, 缩进和行内注释原样不动。找不到返回 None。
    """
    same = _same_line_re(key)
    key_only = _key_only_re(key)
    for idx, line in enumerate(lines):
        match = same.match(line)
        if match:
            return idx, match.group("head"), match.group("value"), match.group("tail")
        if not key_only.match(line):
            continue
        # 跨行式: 往后找第一个"缩进且有实际内容"的行。空行和纯注释行跳过;
        # 碰到不缩进的行就说明这个键底下没有标量值(是 map/list 或者空), 放弃。
        for j in range(idx + 1, len(lines)):
            nxt = lines[j]
            if not nxt.strip() or nxt.lstrip().startswith("#"):
                continue
            if not nxt[:1].isspace():
                return None
            block = _BLOCK_VALUE_RE.match(nxt)
            if block:
                return j, block.group("head"), block.group("value"), block.group("tail")
            return None
        return None
    return None


# roslaunch: `<arg name="KEY" ... default="V"/>` 或 `<param name="KEY" ... value="V"/>`。
# name 用精确匹配(带上闭合引号), 否则 `max_vel` 会撞上 `manager/max_vel`。
def _xml_attr_re(key: str, attr: str) -> "re.Pattern":
    return re.compile(
        r'^(?P<head>.*<\s*(?:arg|param)\b[^>]*?\bname\s*=\s*"' + re.escape(key) + r'"'
        r'[^>]*?\b' + attr + r'\s*=\s*")'
        r'(?P<value>[^"]*)'
        r'(?P<tail>".*)$'
    )


def _locate_xml(lines: List[str], key: str) -> Optional[Tuple[int, str, str, str]]:
    """roslaunch 版的 _locate。`<param>` 用 value=, `<arg>` 用 default= —— 两个都
    试一遍, 哪个匹配上用哪个(同一个标签不会同时有这两个属性, 不存在歧义)。"""
    patterns = [_xml_attr_re(key, "value"), _xml_attr_re(key, "default")]
    for idx, line in enumerate(lines):
        for pattern in patterns:
            match = pattern.match(line)
            if match:
                return idx, match.group("head"), match.group("value"), match.group("tail")
    return None


# 替换表达式: $(arg x) / $(eval ...) / $(find pkg)。这种位置存的是引用不是字面量。
_SUBSTITUTION_RE = re.compile(r"\$\(")


def _locator(schema: Dict[str, Any]):
    """按 schema 声明的文件格式挑定位函数。默认 yaml(先接的两个服务都是)。"""
    return _locate_xml if schema.get("format") == "roslaunch" else _locate


LIST_TYPES = ("vec3", "mat3")
_LIST_LEN = {"vec3": 3, "mat3": 9}
# mat3 当旋转矩阵用时的正交性容差(见 _validate_list 的说明)。
_ROTATION_TOL = 1e-3


def _locate_list(lines: List[str], key: str
                  ) -> Optional[Tuple[int, int, str, str, str]]:
    """找 `key: [ ... ]` 这种列表, 方括号可以跨行。

    返回 (起始行, 结束行, head, body, tail):
      head = `key: [` 及其之前的内容, body = 方括号里的全部文本(换行换成空格),
      tail = `]` 之后那一行剩下的内容(通常是行内注释)。
    """
    opener = re.compile(r"^(?P<head>" + re.escape(key) + r"[ \t]*:[ \t]*\[)(?P<rest>.*)$")
    for idx, line in enumerate(lines):
        match = opener.match(line)
        if not match:
            continue
        head = match.group("head")
        chunk = match.group("rest")
        pieces: List[str] = []
        for end in range(idx, len(lines)):
            if end > idx:
                chunk = lines[end]
            close = chunk.find("]")
            if close >= 0:
                pieces.append(chunk[:close])
                return idx, end, head, " ".join(pieces), chunk[close + 1:]
            pieces.append(chunk)
        return None  # 开了方括号但没闭合 —— 文件本身坏了, 不猜
    return None


def _parse_list(spec: Dict[str, Any], body: str) -> Optional[List[float]]:
    """把方括号里的内容切成数字。个数不对/有非数字就返回 None(= 读不出当前值)。"""
    if _SUBSTITUTION_RE.search(body):
        return None
    items = [t.strip() for t in body.split(",")]
    items = [t for t in items if t != ""]
    if len(items) != _LIST_LEN[spec["type"]]:
        return None
    try:
        return [float(t) for t in items]
    except ValueError:
        return None


def _format_list(spec: Dict[str, Any], values: List[float], head: str) -> List[str]:
    """渲染成一行或多行。mat3 铺成三行、续行缩进对齐到 `[` 后面, 跟原文一致。"""
    nums = [_format_scalar({"type": "float"}, v) for v in values]
    if spec["type"] == "vec3":
        return [head + ", ".join(nums) + "]"]
    pad = " " * len(head)
    rows = [", ".join(nums[i:i + 3]) for i in (0, 3, 6)]
    return [head + rows[0] + ",", pad + rows[1] + ",", pad + rows[2] + "]"]


def _validate_list(spec: Dict[str, Any], value: Any) -> List[float]:
    key, kind = spec["key"], spec["type"]
    want = _LIST_LEN[kind]
    if not isinstance(value, (list, tuple)) or len(value) != want:
        raise ValueError(f"{key} 要的是 {want} 个数字, 收到 {value!r}")
    numbers: List[float] = []
    for item in value:
        if isinstance(item, bool):
            raise ValueError(f"{key} 里不能有 true/false")
        try:
            numbers.append(float(item))
        except (TypeError, ValueError):
            raise ValueError(f"{key} 里有解不成数字的项: {item!r}")
    lo, hi = spec.get("min"), spec.get("max")
    for number in numbers:
        if lo is not None and number < lo:
            raise ValueError(f"{key} 的每一项都不能小于 {lo}, 收到 {number}")
        if hi is not None and number > hi:
            raise ValueError(f"{key} 的每一项都不能大于 {hi}, 收到 {number}")

    # mat3 在这两个配置里都是**旋转矩阵**(yaml 里写着"行优先 3x3 旋转矩阵"),
    # hand-lio 直接拿它做坐标变换。随手填 9 个数很容易填出一个不正交的矩阵,
    # 那样算出来的位姿是歪的、而且不会有任何报错 —— 这里挡一道。
    if spec.get("rotation"):
        import numpy as np
        matrix = np.asarray(numbers, dtype=float).reshape(3, 3)
        err = float(np.abs(matrix @ matrix.T - np.eye(3)).max())
        det = float(np.linalg.det(matrix))
        if err > _ROTATION_TOL or abs(det - 1.0) > _ROTATION_TOL:
            raise ValueError(
                f"{key} 必须是旋转矩阵(各行两两正交且模长为 1): "
                f"R·Rᵀ 偏离单位阵 {err:.4f}, 行列式 {det:.4f}(应为 1)"
            )
    return numbers


def _schema(service_id: str) -> Dict[str, Any]:
    schema = config.SERVICE_PARAM_SCHEMAS.get(service_id)
    if schema is None:
        raise ValueError(f"服务 {service_id!r} 没有可配置的参数")
    return schema


def configurable_ids() -> List[str]:
    """哪些服务 id 有参数页。前端据此决定列表里哪些能点。"""
    return list(config.SERVICE_PARAM_SCHEMAS)


def _parse_scalar(spec: Dict[str, Any], raw: str) -> Any:
    """把 yaml 里那一段裸文本按 schema 声明的类型解出来。解不出来返回 None ——
    调用方(read_params)据此如实告诉前端"这个键当前读不出值", 而不是瞎猜一个
    默认值糊过去。"""
    text = raw.strip().strip('"').strip("'")
    # $(arg ...) 这类替换表达式不是字面量, 解不出数值 —— 如实返回 None, 页面
    # 会显示成"读不出当前值", 好过瞎猜一个数。
    if _SUBSTITUTION_RE.search(text):
        return None
    kind = spec["type"]
    try:
        if kind == "bool":
            low = text.lower()
            if low in ("true", "yes", "on", "1"):
                return True
            if low in ("false", "no", "off", "0"):
                return False
            return None
        if kind == "enum":
            value = int(text)
            return value if any(o["value"] == value for o in spec["options"]) else None
        if kind == "float":
            return float(text)
    except ValueError:
        return None
    return None


def _format_scalar(spec: Dict[str, Any], value: Any) -> str:
    """把校验过的值写成 yaml 字面量。"""
    kind = spec["type"]
    if kind == "bool":
        return "true" if value else "false"
    if kind == "enum":
        return str(int(value))
    # float: 必须带小数点。写成 "1" 的话 ROS 会当成 int 加载, 节点那边按 double
    # 取参数就会报类型不符——rospy/roscpp 都不做 int->double 的隐式提升。
    text = "{:g}".format(float(value))
    if "." not in text and "e" not in text and "E" not in text:
        text += ".0"
    return text


def _validate(spec: Dict[str, Any], value: Any) -> Any:
    key, kind = spec["key"], spec["type"]
    if kind in LIST_TYPES:
        return _validate_list(spec, value)
    if kind == "bool":
        if not isinstance(value, bool):
            raise ValueError(f"{key} 要的是 true/false, 收到 {value!r}")
        return value
    if kind == "enum":
        # 前端的 <select> 回传的可能是字符串, 这里统一收敛成 int 再比对。
        try:
            number = int(value)
        except (TypeError, ValueError):
            raise ValueError(f"{key} 要的是整数, 收到 {value!r}")
        allowed = [o["value"] for o in spec["options"]]
        if number not in allowed:
            raise ValueError(f"{key} 只能是 {allowed} 之一, 收到 {number}")
        return number
    if kind == "float":
        if isinstance(value, bool):
            raise ValueError(f"{key} 要的是数字, 收到 {value!r}")
        try:
            number = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{key} 要的是数字, 收到 {value!r}")
        lo, hi = spec.get("min"), spec.get("max")
        if lo is not None and number < lo:
            raise ValueError(f"{key} 不能小于 {lo}, 收到 {number}")
        if hi is not None and number > hi:
            raise ValueError(f"{key} 不能大于 {hi}, 收到 {number}")
        return number
    raise ValueError(f"{key} 的类型声明 {kind!r} 不认识(schema 写错了)")


def get_params(service_id: str) -> Dict[str, Any]:
    """schema + 当前值 + 配置文件路径, 一次给前端。

    文件读不到(路径没配对/还没部署到这台板子)不抛异常: 把 values 全给 None、
    带上 file_error 如实说明, 让页面能显示出"配置文件在哪、为什么读不到", 比
    一个 500 有用得多——多台板子路径各不相同, 这个错是常态不是意外。
    """
    schema = _schema(service_id)
    path = schema["file"]
    specs = schema["params"]

    values: Dict[str, Any] = {spec["key"]: None for spec in specs}
    file_error: Optional[str] = None
    try:
        with open(path, "r", encoding="utf-8") as fh:
            lines = fh.read().splitlines()
    except OSError as e:
        file_error = f"读不到配置文件 {path}: {e}"
        logger.warning("get_params(%s): %s", service_id, file_error)
        lines = []

    locate = _locator(schema)
    present = set()
    for spec in specs:
        if spec["type"] in LIST_TYPES:
            found_list = _locate_list(lines, spec["key"])
            if found_list is not None:
                present.add(spec["key"])
                values[spec["key"]] = _parse_list(spec, found_list[3])
            continue
        found = locate(lines, spec["key"])
        if found is not None:
            present.add(spec["key"])
            values[spec["key"]] = _parse_scalar(spec, found[2])

    # 同一个服务在不同板子上可能跑不同分支, 配置文件的键不一样(比如 deep_bridge 的
    # m20pro 分支没有 use_dtls / usage_mode / full_scale_*)。文件读到了但某个键不在
    # 里面, 说明这个版本根本没有这个参数: 不给前端显示, 免得页面上摆一个改不了、
    # 一改就报"找不到这些键"的空控件。文件读不到时不筛——那时什么都不知道, 照旧
    # 全部列出来、配合 file_error 显示。
    absent: List[str] = []
    if file_error is None:
        absent = [spec["key"] for spec in specs if spec["key"] not in present]
        specs = [spec for spec in specs if spec["key"] in present]
        values = {k: v for k, v in values.items() if k in present}
    specs = [_resolve_conditional(spec, lambda key: locate(lines, key) is not None)
             for spec in specs]

    return {"service_id": service_id, "file": path, "env_var": schema.get("env_var"),
            "file_error": file_error, "params": specs, "values": values, "absent": absent}


def _resolve_conditional(spec: Dict[str, Any], file_has) -> Dict[str, Any]:
    """spec 里的 if_file_has = {"key": K, 其余字段...}: 配置文件里有 K 这个键时, 用
    其余字段覆盖 spec 的同名字段。让提示跟着配置文件的版本走——比如 gait_on_start
    的"换步态要同步改 full_scale_v*"只对有 full_scale_* 的版本成立。"""
    cond = spec.get("if_file_has")
    if not cond:
        return spec
    out = {k: v for k, v in spec.items() if k != "if_file_has"}
    if file_has(cond["key"]):
        out.update({k: v for k, v in cond.items() if k != "key"})
    return out


def write_params(service_id: str, values: Dict[str, Any]) -> Dict[str, Any]:
    """把 values 里给到的键写回 yaml, 返回写完之后重新读出来的 get_params 结果。

    - 只写 schema 里声明过的键, 其余键**直接报错**而不是忽略(前端传了个不认识的
      键, 多半是两边 schema 不同步, 静默吞掉会让人以为改生效了)。
    - 全部校验通过、而且每个键都在文件里找到了, 才动文件 —— 不能出现"改了一半"
      的配置(跟 tools/gen_corridor_walls.py 那次"--replace 删完才报错"是同一个
      教训)。
    - 写临时文件再 os.replace 原子替换, 中途断电/崩溃不会留下半截文件。
    """
    schema = _schema(service_id)
    path = schema["file"]
    by_key = {spec["key"]: spec for spec in schema["params"]}

    unknown = [k for k in values if k not in by_key]
    if unknown:
        raise ValueError(f"不认识的参数: {', '.join(sorted(unknown))}")

    checked = {key: _validate(by_key[key], value) for key, value in values.items()}
    if not checked:
        return get_params(service_id)

    try:
        with open(path, "r", encoding="utf-8") as fh:
            lines = fh.read().splitlines(keepends=True)
    except OSError as e:
        raise RuntimeError(f"读不到配置文件 {path}: {e}") from e

    # 先全文找一遍行号、把每个键的替换内容都算好, 再统一改 —— 边找边改的话,
    # 中途发现某个键不存在时前面几个已经改过了。
    #
    # 一个键可能占**多行**(跨行的 3x3 矩阵), 所以这里记的是 [start, end] 闭区间
    # 和整段替换文本; 替换行数可能跟原来不一样, 所以下面从后往前应用, 免得改完
    # 前面的把后面的行号顶偏。
    stripped = [line.rstrip("\n") for line in lines]
    locate = _locator(schema)
    plan: Dict[str, Tuple[int, int, List[str], str]] = {}
    for key in checked:
        spec = by_key[key]
        if spec["type"] in LIST_TYPES:
            found_list = _locate_list(stripped, key)
            if found_list is None:
                continue
            start, end, head, body, tail = found_list
            rendered = _format_list(spec, checked[key], head)
            rendered[-1] = rendered[-1] + tail
            plan[key] = (start, end, rendered, body)
            continue
        found = locate(stripped, key)
        if found is None:
            continue
        idx, head, old_value, tail = found
        plan[key] = (idx, idx,
                     [head + _format_scalar(spec, checked[key]) + tail], old_value)

    missing = [k for k in checked if k not in plan]
    if missing:
        raise RuntimeError(
            f"配置文件 {path} 里找不到这些键: {', '.join(sorted(missing))}"
        )

    # 原来存的是 $(arg ...) 这类引用, 拿字面量盖掉会把 launch 里原本的联动关系
    # 悄悄拆掉(比如 closed_loop_controller/max_vx 跟着 max_vel 走)。宁可报错。
    referenced = [k for k, (_s, _e, _r, old_value) in plan.items()
                  if _SUBSTITUTION_RE.search(old_value)]
    if referenced:
        raise RuntimeError(
            "这些键当前存的是 $(...) 替换表达式而不是字面量, 不能从这里改"
            f"(会拆掉 launch 文件里的联动): {', '.join(sorted(referenced))}"
        )

    for key, (start, end, rendered, _old) in sorted(
            plan.items(), key=lambda kv: kv[1][0], reverse=True):
        newline = "\n" if lines[end].endswith("\n") else ""
        block = [text + "\n" for text in rendered]
        block[-1] = rendered[-1] + newline
        lines[start:end + 1] = block
        logger.info("参数写回 %s: %s = %s", path, key, checked[key])

    directory = os.path.dirname(path) or "."
    try:
        fd, tmp = tempfile.mkstemp(dir=directory, prefix=".navibot-params-")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                fh.write("".join(lines))
            # 原文件的权限位保留, mkstemp 建出来的是 0600, 直接 replace 会把
            # 文件变成只有后端用户能读——这个 yaml 是 roslaunch 以别的用户身份
            # 读的, 权限一变节点就起不来了。
            try:
                os.chmod(tmp, os.stat(path).st_mode & 0o7777)
            except OSError:
                pass
            os.replace(tmp, path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
    except OSError as e:
        raise RuntimeError(f"写配置文件 {path} 失败(权限?): {e}") from e

    return get_params(service_id)
