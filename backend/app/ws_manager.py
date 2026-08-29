import asyncio
import json
import logging
from typing import Any, Dict, Optional, Set, Union

from fastapi import WebSocket

from . import config

logger = logging.getLogger("navibot.ws")

# 一份已经编码好、可以直接发的待发数据: JSON 消息是文本(str), 膨胀地图/雷达
# 点云这类走 broadcast_binary_threadsafe 的是二进制帧(bytes)。两种都只编码
# 一次, 广播给 N 个客户端时复用同一份, 不会每个连接各自 json.dumps 一遍。
_Payload = Union[str, bytes]


class WebSocketManager:
    """管理前端 WebSocket 连接, 并提供一个线程安全的 broadcast 方法。

    RosBridge 的订阅回调跑在 rospy 的后台线程里 (不是 FastAPI 的 asyncio
    事件循环线程), 所以广播方法要用 run_coroutine_threadsafe 把消息安全地
    丢回事件循环, 不能直接 await。

    按 type 合并/丢帧: 每种消息类型同一时间最多只有一个发送在跑, 新数据来了
    只是替换掉"待发的最新一份", 不会再开一个新任务。膨胀地图这类高频大 payload
    如果客户端(浏览器主线程忙着重建 Three.js 几何体)消费跟不上, ws.send_json
    会卡在 TCP 背压上迟迟不返回——如果每次 broadcast_threadsafe 调用都无条件
    开一个新任务(旧版本就是这么写的), 卡住期间攒起来的任务会越堆越多, 每个都
    拿着一整片点云的引用, 这就是"打开膨胀地图后后端内存一直涨"的根因。现在
    同一 type 最多一个待发送 + 一个在飞, 内存跟客户端快慢无关, 是有界的。

    序列化只做一次: 以前 _send_one 对每个客户端各自调用 ws.send_json(data),
    Starlette 内部会各自 json.dumps 一遍——开着 N 个标签页就是 N 倍重复的
    JSON 编码成本, 还是在唯一的事件循环线程上同步跑, 会卡住其它请求。现在
    broadcast_threadsafe 在丢进 _pending 之前就把 data 序列化成文本, 之后
    每个客户端发的是同一份现成的 str/bytes, 只是 socket 写, 不再重复编码。
    """

    def __init__(self) -> None:
        self._connections: Set[WebSocket] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._pending: Dict[str, _Payload] = {}  # type -> 还没发出去的最新一份(已编码好)
        self._senders: Dict[str, "asyncio.Task[None]"] = {}  # type -> 正在跑的发送任务

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._connections.add(ws)
        logger.info("ws connected, total=%d", len(self._connections))

    def disconnect(self, ws: WebSocket) -> None:
        self._connections.discard(ws)
        logger.info("ws disconnected, total=%d", len(self._connections))

    async def _send_one(self, ws: WebSocket, payload: _Payload) -> None:
        try:
            send = ws.send_bytes(payload) if isinstance(payload, bytes) else ws.send_text(payload)
            await asyncio.wait_for(send, timeout=config.WS_SEND_TIMEOUT_S)
        except Exception:
            self._connections.discard(ws)

    async def _drain(self, msg_type: str) -> None:
        """不断把 self._pending[msg_type] 里最新的一份发出去, 直到没有更新的
        为止。中途来的新数据只是替换 _pending, 不会排队——同一 type 只关心
        "最新状态是什么", 发得慢的话中间几帧丢了也没关系(下一帧本来就是全量替换)。
        """
        while True:
            payload = self._pending.pop(msg_type, None)
            if payload is None:
                del self._senders[msg_type]
                return
            await asyncio.gather(*(self._send_one(ws, payload) for ws in list(self._connections)))

    async def _schedule(self, msg_type: str, payload: _Payload) -> None:
        self._pending[msg_type] = payload
        if msg_type not in self._senders:
            self._senders[msg_type] = asyncio.create_task(self._drain(msg_type))

    def broadcast_threadsafe(self, data: Dict[str, Any]) -> None:
        """可以从任意线程 (包括 rospy 回调线程) 调用。JSON 只在这里(调用方
        线程上)序列化一次, 后面每个客户端复用同一份文本。"""
        if self._loop is None:
            logger.warning("event loop not bound yet, drop message: %s", data)
            return
        msg_type = data.get("type", "")
        text = json.dumps(data, separators=(",", ":"))
        asyncio.run_coroutine_threadsafe(self._schedule(msg_type, text), self._loop)

    def broadcast_binary_threadsafe(self, msg_type: str, payload: bytes) -> None:
        """二进制帧广播(膨胀地图/雷达点云用, 见 route_manager._encode_point_frame),
        复用跟 broadcast_threadsafe 一样的按 type 合并/丢帧机制, 只是发送时走
        ws.send_bytes 而不是 ws.send_text。可以从任意线程调用。"""
        if self._loop is None:
            logger.warning("event loop not bound yet, drop binary message: type=%s", msg_type)
            return
        asyncio.run_coroutine_threadsafe(self._schedule(msg_type, payload), self._loop)
