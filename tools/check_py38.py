#!/usr/bin/env python3
"""检查代码能不能在 Python 3.8 上跑 (机器上 ROS Noetic 自带的就是 3.8)。

为什么需要这个: 开发机用的是 mamba ros_host 里的 Python 3.12, 写 dict[str, int]、
X | None、asyncio.to_thread 都不会报错, 推到机器上才在 import 阶段炸掉 ——
这类问题本地永远测不出来。CI 没有的情况下, 至少提交前手动跑一次:

    python3 tools/check_py38.py

检查两类:
  1. PEP 585 内建泛型下标 dict[...] / list[...] 和 PEP 604 联合 X | Y。
     只有在**运行时求值**的位置才算问题 —— 模块级变量注解、函数签名、装饰器
     参数(比如 FastAPI 的 response_model=list[X])都会求值。文件顶部有
     `from __future__ import annotations` 时注解变成惰性字符串, 就不算问题,
     但装饰器参数那种非注解位置仍然会炸。
  2. 3.9+ 才有的 stdlib API。
"""
import ast
import pathlib
import sys

BUILTIN_GENERICS = {"dict", "list", "tuple", "set", "frozenset", "type"}
# 3.9+ 才有的 stdlib。用 AST 判断而不是正则: 正则会把注释和文档字符串里提到的
# 名字也算进来 (这个文件自己的规则表就是最好的例子)。
# "模块.属性" 形式
NEW_ATTRS = {
    "asyncio.to_thread": "3.9",
    "functools.cache": "3.9",
    "itertools.pairwise": "3.10",
    "itertools.batched": "3.12",
    "math.lcm": "3.9",
    "math.nextafter": "3.9",
    "typing.ParamSpec": "3.10",
    "typing.TypeAlias": "3.10",
    "datetime.UTC": "3.11",
}
# 裸方法名 (str/bytes 上的新方法, 拿不到接收者类型, 只能按名字报)
NEW_METHODS = {"removeprefix": "3.9", "removesuffix": "3.9"}
# 3.9+ 才有的模块
NEW_MODULES = {"graphlib": "3.9", "zoneinfo": "3.9", "tomllib": "3.11"}
SKIP_DIRS = {"node_modules", ".git", "dist", "__pycache__"}


def has_future_annotations(tree: ast.Module) -> bool:
    return any(
        isinstance(n, ast.ImportFrom) and n.module == "__future__"
        and any(a.name == "annotations" for a in n.names)
        for n in tree.body
    )


def annotation_nodes(tree: ast.Module) -> set:
    """收集所有"注解位置"的节点 id, 这些在 future import 下不会被求值。"""
    out = set()
    for n in ast.walk(tree):
        ann = None
        if isinstance(n, (ast.AnnAssign,)):
            ann = n.annotation
        elif isinstance(n, ast.arg):
            ann = n.annotation
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            ann = n.returns
        if ann is not None:
            out.update(id(x) for x in ast.walk(ann))
    return out


def check(root: pathlib.Path):
    problems = []
    for f in sorted(root.rglob("*.py")):
        if set(f.parts) & SKIP_DIRS:
            continue
        src = f.read_text()
        try:
            tree = ast.parse(src)
        except SyntaxError as e:
            problems.append((f, e.lineno, f"语法错误: {e.msg}"))
            continue

        lazy = has_future_annotations(tree)
        in_annotation = annotation_nodes(tree) if lazy else set()

        for n in ast.walk(tree):
            if id(n) in in_annotation:
                continue
            if isinstance(n, ast.Subscript) and isinstance(n.value, ast.Name) \
                    and n.value.id in BUILTIN_GENERICS:
                problems.append((f, n.lineno, f"PEP 585: {n.value.id}[...] 在 3.8 会 TypeError"))
            if isinstance(n, ast.BinOp) and isinstance(n.op, ast.BitOr):
                for side in (n.left, n.right):
                    if isinstance(side, ast.Constant) and side.value is None:
                        problems.append((f, n.lineno, "PEP 604: X | None 在 3.8 会 TypeError"))
                        break

        for n in ast.walk(tree):
            if isinstance(n, ast.Attribute):
                if isinstance(n.value, ast.Name):
                    dotted = f"{n.value.id}.{n.attr}"
                    if dotted in NEW_ATTRS:
                        problems.append((f, n.lineno, f"{dotted} 需要 Python {NEW_ATTRS[dotted]}+"))
                if n.attr in NEW_METHODS:
                    problems.append((f, n.lineno, f".{n.attr}() 需要 Python {NEW_METHODS[n.attr]}+"))
            elif isinstance(n, ast.Import):
                for a in n.names:
                    top = a.name.split(".")[0]
                    if top in NEW_MODULES:
                        problems.append((f, n.lineno, f"模块 {top} 需要 Python {NEW_MODULES[top]}+"))
            elif isinstance(n, ast.ImportFrom):
                top = (n.module or "").split(".")[0]
                if top in NEW_MODULES:
                    problems.append((f, n.lineno, f"模块 {top} 需要 Python {NEW_MODULES[top]}+"))
    return problems


if __name__ == "__main__":
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    problems = check(root)
    for f, line, msg in problems:
        print(f"{f}:{line}: {msg}")
    print(f"\n{'发现 %d 处 3.8 不兼容' % len(problems) if problems else '未发现 3.8 不兼容问题'}")
    sys.exit(1 if problems else 0)
