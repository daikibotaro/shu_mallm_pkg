from autogen_agentchat.teams import RoundRobinGroupChat as AutoGenGroupChat
from typing import Dict, Any, List, Optional
import asyncio
import json


class GroupChat(AutoGenGroupChat):
    """AutoGenのGroupChatを拡張したクラス"""

    # def __init__(self, agents=None, messages=None, max_round=None,
    #              speaker_selection_method=None, allow_repeat_speaker=None, **kwargs):

    def __init__(
        self,
        participants: List[ChatAgent],
        group_chat_manager_name: str = "manager",
        group_chat_manager_class: type = AutoGenGroupChat,
        termination_condition: Optional[TerminationCondition] = None,
        max_turns: Optional[int] = None,
        **kwargs
    ):

        # message_busとmonitoring_enabledを抽出し、残りのパラメータを親クラスに渡す
        self._message_bus = kwargs.pop("message_bus", None)
        self._monitoring_enabled = kwargs.pop("monitoring_enabled", True)

        # 古いmax_roundsパラメータをmax_roundに変換
        if "max_rounds" in kwargs:
            max_round = kwargs.pop("max_rounds")

        # 親クラスの初期化 - 現在のAPIに合わせたパラメータを使用
        super().__init__(
            participants=participants,
            group_chat_manager_name=group_chat_manager_name,
            group_chat_manager_class=group_chat_manager_class,
            termination_condition=termination_condition,
            max_turns=max_turns,
            **kwargs
        )

        # メトリクスの初期化
        self.metrics = {
            "total_messages": 0,
            "messages_per_agent": {},
            "average_response_time": 0,
            "total_response_time": 0
        }

    # プロパティを定義して外部から設定できるようにする
    @property
    def message_bus(self):
        return self._message_bus

    @message_bus.setter
    def message_bus(self, value):
        self._message_bus = value

    @property
    def monitoring_enabled(self):
        return self._monitoring_enabled

    @monitoring_enabled.setter
    def monitoring_enabled(self, value):
        self._monitoring_enabled = value

    async def send(self, message: Dict[str, Any], sender_name: str) -> None:
        """メッセージ送信の拡張バージョン

        Args:
            message: 送信するメッセージ
            sender_name: 送信元エージェント名
        """
        # 元のGroupChatのsendメソッドを呼び出し
        await super().send(message, sender_name)

        # メッセージバスがあれば発行
        if self.message_bus:
            await self.message_bus.publish(
                topic="group_chat",
                sender=sender_name,
                message=message
            )

        # モニタリングが有効なら指標を更新
        if self.monitoring_enabled:
            self._update_metrics(message, sender_name)

    def _update_metrics(self, message: Dict[str, Any], sender_name: str) -> None:
        """チャットメトリクスを更新"""
        self.metrics["total_messages"] += 1

        if sender_name not in self.metrics["messages_per_agent"]:
            self.metrics["messages_per_agent"][sender_name] = 0

        self.metrics["messages_per_agent"][sender_name] += 1

        # 応答時間の計測（メッセージに時間情報がある場合）
        if "timestamp" in message:
            now = asyncio.get_event_loop().time()
            response_time = now - message["timestamp"]

            self.metrics["total_response_time"] += response_time
            self.metrics["average_response_time"] = (
                self.metrics["total_response_time"] / self.metrics["total_messages"]
            )

    def get_metrics(self) -> Dict[str, Any]:
        """チャットパフォーマンス指標を取得"""
        return self.metrics

    def export_conversation(self, format_type: str = "json") -> str:
        """会話履歴をエクスポート

        Args:
            format_type: エクスポート形式 ("json" or "markdown")

        Returns:
            フォーマットされた会話履歴
        """
        if format_type == "markdown":
            markdown = "# グループチャット会話履歴\n\n"

            for msg in self.messages:
                sender = msg.get("sender", "Unknown")
                content = msg.get("content", "")

                markdown += f"## {sender}\n\n{content}\n\n"

            return markdown

        # デフォルトはJSON
        return json.dumps(self.messages, ensure_ascii=False, indent=2)
