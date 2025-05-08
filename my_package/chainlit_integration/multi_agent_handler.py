import chainlit as cl
from typing import Dict, Any, List
from chat_handler import ChatHandler
from multi_agent.agent_manager import AgentManager
from chainlit_integration.message_formatter import MessageFormatter


class MultiAgentChatHandler(ChatHandler):
    """マルチエージェント対応のチャットハンドラ"""

    def __init__(self):
        super().__init__()
        self.agent_manager = AgentManager()
        self.message_formatter = MessageFormatter()

    async def initialize(self):
        """初期化処理"""
        # 親クラスの初期化
        await super().initialize()

        # エージェントマネージャーの初期化
        await self.agent_manager.initialize_agents()

        return True

    async def handle_message(self, message: cl.Message):
        """ユーザーメッセージの処理

        Args:
            message: Chainlitメッセージオブジェクト
        """
        user_input = message.content

        # 従来の機能を保持（RAG検索など）
        self.message_history = self.llm.load_message_history()
        if message.content is not None:
            self.message_history.append({"role": "user", "content": message.content})

        # RAGシステムからの関連情報取得
        relevant_info, max_similarity = await self.relevance_finder.get_relevant_info_with_similarity(
            message.content,
            similarity_threshold=self.similarity_threshold
        )

        # マルチエージェントに処理を依頼
        result = await self.agent_manager.process_message(user_input)

        # Chainlitに結果を表示
        thread_msg = cl.Message(content="", author="System")
        await thread_msg.send()

        # 各エージェントからのメッセージを表示
        for agent_name, agent_message in result["messages"]:
            formatted_message = self.message_formatter.format_agent_message(
                agent_name, agent_message
            )
            await cl.Message(
                content=formatted_message,
                author=agent_name,
                parent_id=thread_msg.id
            ).send()

        # 最終結果メッセージ
        final_message = result.get("summary", "処理が完了しました")
        self.message_history.append({"role": "assistant", "content": final_message})

        await cl.Message(content=final_message, author="Nimbus").send()

        # メッセージ履歴を保存
        self.llm.save_message_history(self.message_history)

        # ファイル生成が必要な場合の処理
        await self.file.check_create_file(final_message)
