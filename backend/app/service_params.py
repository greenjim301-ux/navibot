"""
「参数配置」页的后端: 读/写某个服务的 ROS 参数 yaml。

只认 config.SERVICE_PARAM_SCHEMAS 里声明过的服务 id 和键名——跟 SYSTEMD_SERVICES
同一个理由, 挡掉"随便传个 key 就能改 yaml"的口子。每个键的类型/范围/可选值也在
那张表里, 这里照着校验。

**写回是按行原地替换值, 不是 yaml 重新序列化。** deep_bridge.yaml 里那些注释
(协议出处、厂商的步态速度范围表、"实测这台本体加密是关掉的"这类坑)信息量比配置
本身还大, 用 PyYAML 读出来再 dump 回去会把它们全部冲掉。所以这里只认"顶格
`key: value`"这一种形态, 精确替换 value 那一段, 行内注释、缩进、空行、文件里其它
所有内容原样不动。

代价是这个解析器**很窄**: 只处理顶层(不缩进)的标量键。schema 里声明的键如果在
文件里找不到, 直接报错而不是追加一行——追加的键很可能是缩进/命名空间写错了,
静默追加只会得到一个永远不生效的配置。
"""
import logging
import os
import re
import tempfile
from typing import Any, Dict, List, Optional

from . import config

logger = logging.getLogger("navibot.service_params")

# 顶格的标量键: `key: value   # 可选行内注释`
# value 捕获到第一个 ` #` 之前(yaml 要求行内注释前有空白), 前后空白不计入。
def _line_re(key: str) -> "re.Pattern":
    return re.compile(
        r"^(?P<head>" + re.escape(key) + r"[ \t]*:[ \t]*)"
        r"(?P<value>.*?)"
        r"(?P<tail>(?:[ \t]+#.*)?[ \t]*)$"
    )


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

    for spec in specs:
        pattern = _line_re(spec["key"])
        for line in lines:
            match = pattern.match(line)
            if match:
                values[spec["key"]] = _parse_scalar(spec, match.group("value"))
                break

    return {"service_id": service_id, "file": path, "file_error": file_error,
            "params": specs, "values": values}


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
    hits: Dict[str, int] = {}
    for key in checked:
        pattern = _line_re(key)
        for idx, line in enumerate(lines):
            if pattern.match(line.rstrip("\n")):
                hits[key] = idx
                break
    missing = [k for k in checked if k not in hits]
    if missing:
        raise RuntimeError(
            f"配置文件 {path} 里找不到这些键(只支持顶格的 `key: value`): "
            f"{', '.join(sorted(missing))}"
        )

    for key, idx in hits.items():
        raw = lines[idx]
        newline = "\n" if raw.endswith("\n") else ""
        match = _line_re(key).match(raw.rstrip("\n"))
        # 上面刚匹配过, 这里必中; assert 只是给静态检查/未来改动留个断言。
        assert match is not None
        lines[idx] = (match.group("head") + _format_scalar(by_key[key], checked[key])
                      + match.group("tail") + newline)
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
