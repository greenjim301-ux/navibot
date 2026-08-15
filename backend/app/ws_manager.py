import asyncio
import logging
from typing import Any, Dict, Optional, Set

from fastapi import WebSocket

from . import config

logger = logging.getLogger("navibot.ws")


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
    """

    def __init__(self) -> None:
        self._connections: Set[WebSocket] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._pending: Dict[str, Dict[str, Any]] = {}  # type -> 还没发出去的最新一份
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

    async def _send_one(self, ws: WebSocket, data: Dict[str, Any]) -> None:
        try:
            await asyncio.wait_for(ws.send_json(data), timeout=config.WS_SEND_TIMEOUT_S)
        except Exception:
            self._connections.discard(ws)

    async def _drain(self, msg_type: str) -> None:
        """不断把 self._pending[msg_type] 里最新的一份发出去, 直到没有更新的
        为止。中途来的新数据只是替换 _pending, 不会排队——同一 type 只关心
        "最新状态是什么", 发得慢的话中间几帧丢了也没关系(下一帧本来就是全量替换)。
        """
        while True:
            data = self._pending.pop(msg_type, None)
            if data is None:
                del self._senders[msg_type]
                return
            await asyncio.gather(*(self._send_one(ws, data) for ws in list(self._connections)))

    async def _schedule(self, data: Dict[str, Any]) -> None:
        msg_type = data.get("type", "")
        self._pending[msg_type] = data
        if msg_type not in self._senders:
            self._senders[msg_type] = asyncio.create_task(self._drain(msg_type))

    def broadcast_threadsafe(self, data: Dict[str, Any]) -> None:
        """可以从任意线程 (包括 rospy 回调线程) 调用。"""
        if self._loop is None:
            logger.warning("event loop not bound yet, drop message: %s", data)
            return
        asyncio.run_coroutine_threadsafe(self._schedule(data), self._loop)
