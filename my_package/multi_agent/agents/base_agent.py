from autogen_agentchat.agents import AssistantAgent
from autogen_agentchat.messages import TextMessage
from autogen_core import CancellationToken

from typing import Dict, Any, Optional, List


class BaseAgent(AssistantAgent):
    """全エージェントの基底クラス"""

    def __init__(self, name: str, config: Dict[str, Any]):
        """
        Args:
            name: エージェント名
            config: エージェント設定
        """
        self.role = config.get("role", "assistant")
        self.capabilities = config.get("capabilities", [])

        system_message = self._build_system_message(config)

        super().__init__(
            name=name,
            system_message=system_message,
            llm_config=config.get("llm_config", {"model": "gpt-4o", "temperature": 0}),
        )

    def _build_system_message(self, config: Dict[str, Any]) -> str:
        """システムメッセージを構築

        Args:
            config: エージェント設定

        Returns:
            システムメッセージ
        """
        base_message = config.get("system_message", "")
        capabilities = "\n".join([f"- {cap}" for cap in self.capabilities])

        system_message = f"""
        {base_message}
        
        あなたの役割: {self.role}
        
        以下の機能を持っています:
        {capabilities}
        
        他のエージェントと協力してタスクを完了させてください。
        """

        return system_message

    async def generate_response(self, message: str) -> str:
        # """メッセージに対して応答を生成"""
        # response = await self.on_messages([{"role": "user", "content": message}])
        # return response.message.content if hasattr(response, "message") else ""

        # 新しい on_messages API：Response.chat_message が最終応答
        response = await self.on_messages(
            [TextMessage(content=message, source="user")],
            cancellation_token=CancellationToken()
        )
        return response.chat_message.content
