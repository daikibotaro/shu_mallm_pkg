import asyncio
from typing import Dict, Any, List, Callable, Awaitable, Optional


class MessageBus:
    """エージェント間の通信を管理するメッセージバス"""

    def __init__(self):
        self.subscribers = {}
        self.history = []

    async def publish(self, topic: str, sender: str, message: Any) -> None:
        """メッセージを発行

        Args:
            topic: メッセージトピック
            sender: 送信元エージェント
            message: メッセージ内容
        """
        timestamp = asyncio.get_event_loop().time()

        message_obj = {
            "timestamp": timestamp,
            "topic": topic,
            "sender": sender,
            "content": message
        }

        self.history.append(message_obj)

        if topic in self.subscribers:
            coroutines = [
                subscriber(message_obj)
                for subscriber in self.subscribers[topic]
            ]
            await asyncio.gather(*coroutines)

    def subscribe(self, topic: str, callback: Callable[[Dict[str, Any]], Awaitable[None]]) -> None:
        """トピックにサブスクライブ

        Args:
            topic: サブスクライブするトピック
            callback: メッセージ受信時のコールバック
        """
        if topic not in self.subscribers:
            self.subscribers[topic] = []

        self.subscribers[topic].append(callback)

    def unsubscribe(self, topic: str, callback: Callable[[Dict[str, Any]], Awaitable[None]]) -> None:
        """サブスクリプションを解除

        Args:
            topic: サブスクリプション解除するトピック
            callback: 解除するコールバック
        """
        if topic in self.subscribers and callback in self.subscribers[topic]:
            self.subscribers[topic].remove(callback)

    def get_history(self, topic: Optional[str] = None, limit: int = 100) -> List[Dict[str, Any]]:
        """メッセージ履歴を取得

        Args:
            topic: 特定のトピックのみ取得する場合
            limit: 取得する最大メッセージ数

        Returns:
            メッセージ履歴
        """
        if topic:
            filtered_history = [m for m in self.history if m["topic"] == topic]
            return filtered_history[-limit:]

        return self.history[-limit:]
