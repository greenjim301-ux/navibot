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
    for spec in specs:
        found = locate(lines, spec["key"])
        if found is not None:
            values[spec["key"]] = _parse_scalar(spec, found[2])

    return {"service_id": service_id, "file": path, "env_var": schema.get("env_var"),
            "file_error": file_error, "params": specs, "values": values}


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

    # 先全文找一遍行号, 确认每个要改的键都在, 再统一改 —— 边找边改的话, 中途
    # 发现某个键不存在时前面几个已经改过了。
    stripped = [line.rstrip("\n") for line in lines]
    locate = _locator(schema)
    hits: Dict[str, Tuple[int, str, str, str]] = {}
    for key in checked:
        found = locate(stripped, key)
        if found is not None:
            hits[key] = found
    missing = [k for k in checked if k not in hits]
    if missing:
        raise RuntimeError(
            f"配置文件 {path} 里找不到这些键的标量值: {', '.join(sorted(missing))}"
        )

    # 原来存的是 $(arg ...) 这类引用, 拿字面量盖掉会把 launch 里原本的联动关系
    # 悄悄拆掉(比如 closed_loop_controller/max_vx 跟着 max_vel 走)。宁可报错。
    referenced = [k for k, (_i, _h, old_value, _t) in hits.items()
                  if _SUBSTITUTION_RE.search(old_value)]
    if referenced:
        raise RuntimeError(
            "这些键当前存的是 $(...) 替换表达式而不是字面量, 不能从这里改"
            f"(会拆掉 launch 文件里的联动): {', '.join(sorted(referenced))}"
        )

    for key, (idx, head, _old, tail) in hits.items():
        newline = "\n" if lines[idx].endswith("\n") else ""
        lines[idx] = head + _format_scalar(by_key[key], checked[key]) + tail + newline
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
