import unittest
import asyncio
from my_package.multi_agent.agents.base_agent import BaseAgent
from my_package.multi_agent.agents.ui_agent import UIAgent
from my_package.multi_agent.agents.planning_agent import PlanningAgent
from my_package.multi_agent.agents.code_agent import CodeAgent


class TestAgents(unittest.TestCase):
    """エージェントのテスト"""

    def setUp(self):
        # テスト用の設定
        self.base_config = {
            "name": "TestAgent",
            "role": "テストエージェント",
            "system_message": "これはテスト用のエージェントです。",
            "capabilities": ["テスト実行", "検証"],
            "llm_config": {"model": "gpt-4o", "temperature": 0}
        }

    def test_base_agent_creation(self):
        """基本エージェント作成のテスト"""
        agent = BaseAgent("TestAgent", self.base_config)
        self.assertEqual(agent.name, "TestAgent")
        self.assertEqual(agent.role, "テストエージェント")

    def test_ui_agent_creation(self):
        """UIエージェント作成のテスト"""
        ui_config = self.base_config.copy()
        ui_config["name"] = "UIAgent"

        agent = UIAgent(ui_config)
        self.assertEqual(agent.name, "UIAgent")

    def test_planning_agent_creation(self):
        """プランニングエージェント作成のテスト"""
        planning_config = self.base_config.copy()
        planning_config["name"] = "PlanningAgent"

        agent = PlanningAgent(planning_config)
        self.assertEqual(agent.name, "PlanningAgent")

    async def test_code_agent_extract_code(self):
        """コードエージェントのコード抽出テスト"""
        code_config = self.base_config.copy()
        code_config["name"] = "CodeAgent"

        agent = CodeAgent(code_config)

        # テスト用の応答
        response = """
        以下のコードを生成しました：
        
        ```python
        import rospy
        
        def main():
            print("Hello, World!")
            
        if __name__ == "__main__":
            main()
        ```
        
        このコードはシンプルな挨拶を表示します。
        """

        code = agent._extract_code_from_response(response)
        self.assertIn("import rospy", code)
        self.assertIn("main()", code)

        explanation = agent._extract_explanation_from_response(response)
        self.assertIn("シンプルな挨拶", explanation)

# asyncioを使用したテスト実行用ヘルパー


def run_async_test(coro):
    loop = asyncio.get_event_loop()
    return loop.run_until_complete(coro)


if __name__ == '__main__':
    unittest.main()
