from typing import Dict, Any, List, Callable, Awaitable, Optional
import asyncio
import uuid


class DirectChannel:
    """エージェント間の直接通信チャネル"""

    def __init__(self, agent1_name: str, agent2_name: str):
        """
        Args:
            agent1_name: 1つ目のエージェント名
            agent2_name: 2つ目のエージェント名
        """
        self.id = str(uuid.uuid4())
        self.agent1 = agent1_name
        self.agent2 = agent2_name
        self.messages = []
        self.on_message_callbacks = []

    async def send_message(self, sender: str, receiver: str, message: Any) -> bool:
        """メッセージを送信

        Args:
            sender: 送信元エージェント名
            receiver: 受信先エージェント名
            message: メッセージ内容

        Returns:
            送信成功フラグ
        """
        # チャネルの参加者のみメッセージ送信可能
        if sender not in [self.agent1, self.agent2]:
            return False

        # 受信者もチャネル参加者である必要あり
        if receiver not in [self.agent1, self.agent2]:
            return False

        timestamp = asyncio.get_event_loop().time()

        message_obj = {
            "id": str(uuid.uuid4()),
            "timestamp": timestamp,
            "sender": sender,
            "receiver": receiver,
            "content": message
        }

        self.messages.append(message_obj)

        # コールバックを実行
        coroutines = [callback(message_obj) for callback in self.on_message_callbacks]
        await asyncio.gather(*coroutines)

        return True

    def on_message(self, callback: Callable[[Dict[str, Any]], Awaitable[None]]) -> None:
        """メッセージ受信時のコールバックを登録

        Args:
            callback: 実行するコールバック
        """
        self.on_message_callbacks.append(callback)

    def get_history(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """会話履歴を取得

        Args:
            limit: 取得するメッセージ数上限

        Returns:
            メッセージ履歴
        """
        if limit is not None:
            return self.messages[-limit:]

        return self.messages
