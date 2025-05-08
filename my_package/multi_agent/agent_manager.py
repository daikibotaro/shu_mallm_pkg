from multi_agent.agents.ui_agent import UIAgent
from multi_agent.agents.planning_agent import PlanningAgent
from multi_agent.agents.knowledge_agent import KnowledgeAgent
from multi_agent.agents.code_agent import CodeAgent
from multi_agent.agents.executor_agent import ExecutorAgent
from multi_agent.communication.message_bus import MessageBus
from multi_agent.communication.group_chat import GroupChat
from multi_agent.communication.direct_channel import DirectChannel
from multi_agent.config.agent_config import AgentConfig

from autogen import GroupChatManager
import asyncio
import logging
from typing import Dict, Any, List, Optional


class AgentManager:
    """マルチエージェントシステムを管理するクラス"""

    def __init__(self, config_path: str = None):
        """
        Args:
            config_path: 設定ファイルのパス
        """
        self.logger = logging.getLogger(__name__)

        # 設定ファイルのパスが指定されていなければデフォルト
        if not config_path:
            config_path = "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/multi_agent/config/config.yaml"

        self.config = AgentConfig(config_path)
        self.message_bus = MessageBus()
        self.agents = {}
        self.direct_channels = {}
        self.group_chat = None
        self.manager = None

        # 依存サービス
        self.relevance_finder = None
        self.file_manager = None

    def set_dependencies(self, relevance_finder=None, file_manager=None):
        """依存サービスを設定

        Args:
            relevance_finder: RAG関連性検索
            file_manager: ファイル管理
        """
        self.relevance_finder = relevance_finder
        self.file_manager = file_manager

    async def initialize_agents(self):
        """エージェントを初期化"""
        # 設定を取得
        llm_defaults = self.config.get_llm_defaults()
        agents_config = self.config.config.get("agents", {})

        # 各エージェントを初期化
        ui_config = agents_config.get("ui", {})
        self.agents["ui"] = UIAgent(ui_config)

        planning_config = agents_config.get("planning", {})
        self.agents["planning"] = PlanningAgent(planning_config)

        knowledge_config = agents_config.get("knowledge", {})
        self.agents["knowledge"] = KnowledgeAgent(knowledge_config)
        if self.relevance_finder:
            await self.agents["knowledge"].initialize(self.relevance_finder)

        code_config = agents_config.get("code", {})
        self.agents["code"] = CodeAgent(code_config)

        executor_config = agents_config.get("executor", {})
        self.agents["executor"] = ExecutorAgent(executor_config)
        if self.file_manager:
            self.agents["executor"].set_file_manager(self.file_manager)

        # 直接通信チャネルを作成
        self._setup_direct_channels()

        # グループチャットを設定
        group_chat_config = self.config.get_group_chat_config()
        agent_list = list(self.agents.values())

        self.group_chat = GroupChat(
            agents=agent_list,
            messages=[],
            max_round=group_chat_config.get("max_rounds", 20),  # max_rounds → max_round に変更
        )

        # message_busプロパティを手動で設定
        self.group_chat.message_bus = self.message_bus
        self.group_chat.monitoring_enabled = group_chat_config.get("enable_monitoring", True)

        self.manager = GroupChatManager(
            groupchat=self.group_chat,
            llm_config=llm_defaults
        )

        self.logger.info("すべてのエージェントが初期化されました")

    def _setup_direct_channels(self):
        """エージェント間の直接通信チャネルをセットアップ"""
        # プランニングとコード生成の間の直接チャネル
        planning_code_channel = DirectChannel(
            agent1_name=self.agents["planning"].name,
            agent2_name=self.agents["code"].name
        )
        self.direct_channels["planning_code"] = planning_code_channel

        # コード生成と実行の間の直接チャネル
        code_executor_channel = DirectChannel(
            agent1_name=self.agents["code"].name,
            agent2_name=self.agents["executor"].name
        )
        self.direct_channels["code_executor"] = code_executor_channel

        # UIと知識の間の直接チャネル
        ui_knowledge_channel = DirectChannel(
            agent1_name=self.agents["ui"].name,
            agent2_name=self.agents["knowledge"].name
        )
        self.direct_channels["ui_knowledge"] = ui_knowledge_channel

    async def process_message(self, message: str) -> Dict[str, Any]:
        """ユーザーメッセージを処理

        Args:
            message: ユーザーメッセージ

        Returns:
            処理結果
        """
        if not self.manager:
            await self.initialize_agents()

        # v0.4のAPIに合わせて修正
        # async_runではなく、on_messagesまたはrunを使用
        chat_result = await self.manager.run(
            messages=[{"role": "user", "content": message}],
            sender=self.agents["ui"].name
        )

        # グループチャットのメトリクスを取得
        metrics = self.group_chat.get_metrics()

        return {
            "messages": chat_result.messages,
            "summary": chat_result.summary if hasattr(chat_result, "summary") else "",
            "status": chat_result.status if hasattr(chat_result, "status") else "completed",
            "metrics": metrics
        }

    async def process_with_workflow(self, message: str) -> Dict[str, Any]:
        """事前定義されたワークフローでユーザーメッセージを処理

        Args:
            message: ユーザーメッセージ

        Returns:
            処理結果
        """
        # 1. UIエージェントがユーザー意図を分析
        intent_data = await self.agents["ui"].analyze_user_intent(message)

        # 2. ナレッジエージェントが関連情報を検索
        knowledge_data = await self.agents["knowledge"].retrieve_knowledge(message)

        # 3. プランニングエージェントがタスク計画を作成
        plan_data = await self.agents["planning"].create_task_plan(
            intent_data, knowledge_data
        )

        # 4. プランを検証
        validation_result = await self.agents["planning"].validate_plan(plan_data)
        final_plan = validation_result.get("improved_plan", plan_data)

        # 5. コードエージェントがコードを生成
        code_result = await self.agents["code"].generate_robot_code(final_plan)

        # 6. コードを検証
        verification = await self.agents["code"].verify_code(code_result["code"])

        # 7. エグゼキュータがコードを実行（条件付き）
        execution_result = {"status": "pending"}

        if not verification.get("has_issues", True):
            # UIエージェントに最終応答をフォーマットしてもらう
            final_response = await self.agents["ui"].format_final_response(
                final_plan, code_result["code"]
            )

            # 最終結果をメッセージバスに発行
            await self.message_bus.publish(
                topic="workflow_result",
                sender="AgentManager",
                message={
                    "intent": intent_data,
                    "knowledge": knowledge_data,
                    "plan": final_plan,
                    "code": code_result,
                    "verification": verification,
                    "execution": execution_result,
                    "final_response": final_response
                }
            )

            return {
                "workflow_complete": True,
                "final_response": final_response,
                "plan": final_plan,
                "code": code_result["code"],
                "execution_status": "ready_for_approval"
            }

        # 問題がある場合はエラーを返す
        return {
            "workflow_complete": False,
            "error": "コード検証に失敗しました",
            "verification_issues": verification.get("review_comments", ""),
            "plan": final_plan,
            "code": code_result["code"],
            "execution_status": "error"
        }
