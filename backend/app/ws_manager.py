import asyncio
import logging
from typing import Any, Dict, Optional, Set

from fastapi import WebSocket

logger = logging.getLogger("navibot.ws")


class WebSocketManager:
    """管理前端 WebSocket 连接, 并提供一个线程安全的 broadcast 方法。

    RosBridge 的订阅回调跑在 rospy 的后台线程里 (不是 FastAPI 的 asyncio
    事件循环线程), 所以广播方法要用 run_coroutine_threadsafe 把消息安全地
    丢回事件循环, 不能直接 await。
    """

    def __init__(self) -> None:
        self._connections: Set[WebSocket] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._connections.add(ws)
        logger.info("ws connected, total=%d", len(self._connections))

    def disconnect(self, ws: WebSocket) -> None:
        self._connections.discard(ws)
        logger.info("ws disconnected, total=%d", len(self._connections))

    async def _broadcast_async(self, data: Dict[str, Any]) -> None:
        dead = []
        for ws in list(self._connections):
            try:
                await ws.send_json(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._connections.discard(ws)

    def broadcast_threadsafe(self, data: Dict[str, Any]) -> None:
        """可以从任意线程 (包括 rospy 回调线程) 调用。"""
        if self._loop is None:
            logger.warning("event loop not bound yet, drop message: %s", data)
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_async(data), self._loop)
