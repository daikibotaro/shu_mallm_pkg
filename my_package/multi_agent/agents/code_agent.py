from multi_agent.agents.base_agent import BaseAgent
from typing import Dict, Any, List, Optional
import json


class CodeAgent(BaseAgent):
    """ロボット制御コードの生成を担当するエージェント"""

    def __init__(self, config: Dict[str, Any]):
        super().__init__(name=config.get("name", "CodeAgent"), config=config)

    async def generate_robot_code(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """計画に基づいてロボット制御コードを生成

        Args:
            plan: プランニングエージェントから提供されたタスク計画

        Returns:
            生成されたコードと関連情報
        """
        # プランから必要なステップを抽出
        steps = plan.get("steps", [])
        steps_str = "\n".join([f"{i+1}. {step}" for i, step in enumerate(steps)])

        code_prompt = f"""
        以下のタスク計画に基づいて、ロボット制御のためのPythonコードを生成してください:
        
        タスク計画:
        {steps_str}
        
        安全上の考慮事項:
        {plan.get("safety_considerations", [])}
        
        失敗時の対応:
        {plan.get("fallback_plans", [])}
        
        次の要件を満たすコードを生成してください:
        1. ROS環境で動作すること
        2. function_class.pyで定義されているクラス（RobotController、DisplayManager、TextToSpeech）を使用
        3. エラーハンドリングを含めること
        4. コメントで各ステップを明確にすること
        5. コードはPython3形式で完全な実行可能形式にすること
        
        コード全体を提供し、重要な部分についての説明を添えてください。
        """

        response = await self.generate_response(code_prompt)

        # コードブロックの抽出
        code = self._extract_code_from_response(response)
        explanation = self._extract_explanation_from_response(response)

        return {
            "code": code,
            "explanation": explanation
        }

    def _extract_code_from_response(self, response: str) -> str:
        """応答からコードブロックを抽出"""
        if "```python" in response and "```" in response:
            start = response.find("```python") + len("```python")
            end = response.rfind("```")
            if start < end:
                return response[start:end].strip()

        # フォールバック: Python風のコードを探す
        if "import rospy" in response:
            lines = response.split('\n')
            code_lines = []
            in_code_block = False

            for line in lines:
                if "import rospy" in line or "import os" in line:
                    in_code_block = True

                if in_code_block:
                    code_lines.append(line)

                if in_code_block and line.strip() == "" and len(code_lines) > 10:
                    break

            return "\n".join(code_lines)

        return ""

    def _extract_explanation_from_response(self, response: str) -> str:
        """応答から説明部分を抽出"""
        if "```python" in response and "```" in response:
            end = response.rfind("```")
            after_code = response[end + 3:].strip()
            if after_code:
                return after_code

            start = response.find("```python")
            before_code = response[:start].strip()
            if before_code:
                return before_code

        return ""

    async def verify_code(self, code: str) -> Dict[str, Any]:
        """生成されたコードを検証

        Args:
            code: 生成されたPythonコード

        Returns:
            検証結果と改善点
        """
        verify_prompt = f"""
        以下のロボット制御コードをレビューし、問題点や改善点を特定してください:
        
        ```python
        {code}
        ```
        
        以下の観点から検証:
        1. 構文エラー
        2. 論理エラー
        3. エラーハンドリングの完全性
        4. ROSの適切な使用
        5. メモリ/リソースの問題
        6. 安全上の問題
        
        問題を特定し、必要に応じて修正案を提供してください。
        """

        response = await self.generate_response(verify_prompt)

        # 問題があるかどうかの判定（簡略化）
        has_issues = "問題" in response or "エラー" in response or "修正" in response

        return {
            "has_issues": has_issues,
            "review_comments": response,
            "improved_code": code if not has_issues else None
        }
