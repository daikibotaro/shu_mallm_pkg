import unittest
import asyncio
import os
import sys
from my_package.multi_agent.agent_manager import AgentManager
from my_package.multi_agent.config.agent_config import AgentConfig
import tempfile
import yaml


class TestIntegration(unittest.TestCase):
    """統合テスト"""

    async def asyncSetUp(self):
        """非同期のセットアップ"""
        # テスト用設定ファイル作成
        self.config_file = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)

        test_config = {
            "llm_defaults": {
                "model": "gpt-4o",
                "temperature": 0
            },
            "group_chat": {
                "max_rounds": 5,
                "timeout": 60,
                "use_async": True
            },
            "agents": {
                "ui": {
                    "name": "UIAgent",
                    "role": "UIエージェント",
                    "system_message": "ユーザーとの対話を管理します。",
                    "capabilities": ["ユーザー意図の理解"],
                    "llm_config": {"model": "gpt-4o", "temperature": 0.2}
                },
                "planning": {
                    "name": "PlanningAgent",
                    "role": "プランニングエージェント",
                    "system_message": "タスク計画を作成します。",
                    "capabilities": ["タスク分解"],
                    "llm_config": {"model": "gpt-4o", "temperature": 0}
                }
            }
        }

        with open(self.config_file.name, 'w') as f:
            yaml.dump(test_config, f)

        # エージェントマネージャーの初期化
        self.agent_manager = AgentManager(self.config_file.name)
        await self.agent_manager.initialize_agents()

    async def asyncTearDown(self):
        """非同期のクリーンアップ"""
        os.unlink(self.config_file.name)

    async def test_basic_workflow(self):
        """基本的なワークフローのテスト"""
        # 簡単なテストメッセージ
        test_message = "ボルトを組立て工程に持ってきて"

        # ワークフローによる処理
        result = await self.agent_manager.process_with_workflow(test_message)

        # 基本的な検証
        self.assertIn("workflow_complete", result)
        self.assertIn("plan", result)
        self.assertIn("code", result)
        self.assertIn("execution_status", result)

    async def test_group_chat_processing(self):
        """グループチャット処理のテスト"""
        # テストメッセージ
        test_message = "部品棚からM6ボルトを取ってきて"

        # グループチャットによる処理
        result = await self.agent_manager.process_message(test_message)

        # 基本的な検証
        self.assertIn("messages", result)
        self.assertIn("metrics", result)

        # メトリクス検証
        metrics = result["metrics"]
        self.assertIn("total_messages", metrics)
        self.assertGreater(metrics["total_messages"], 0)

# asyncioを使用したテスト実行用クラス


class AsyncioTestCase(unittest.TestCase):
    """asyncioを使用したテストケース基底クラス"""

    def setUp(self):
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.asyncSetUp())

    def tearDown(self):
        self.loop.run_until_complete(self.asyncTearDown())

    async def asyncSetUp(self):
        pass

    async def asyncTearDown(self):
        pass


if __name__ == '__main__':
    unittest.main()
