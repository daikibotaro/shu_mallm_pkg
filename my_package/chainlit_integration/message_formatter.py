from typing import Dict, Any, Optional
import re


class MessageFormatter:
    """エージェントメッセージをChainlit用にフォーマット"""

    def __init__(self):
        self.agent_colors = {
            "UIAgent": "blue",
            "PlanningAgent": "green",
            "KnowledgeAgent": "purple",
            "CodeAgent": "orange",
            "ExecutorAgent": "red",
            "System": "gray"
        }

    def format_agent_message(self, agent_name: str, message: str) -> str:
        """エージェントメッセージをフォーマット

        Args:
            agent_name: エージェント名
            message: メッセージ内容

        Returns:
            フォーマットされたメッセージ
        """
        # エージェントに合わせた色を選択
        color = self.agent_colors.get(agent_name, "black")

        # Markdownフォーマットでエージェント名と色付きセクション
        formatted = f"### {agent_name}\n\n"

        # コードブロックが含まれる場合は保持
        if "```" in message:
            # コードブロックを分割して処理
            parts = re.split(r'(```(?:python)?\n.*?\n```)', message, flags=re.DOTALL)
            for i, part in enumerate(parts):
                if part.startswith("```") and part.endswith("```"):
                    # コードブロックはそのまま
                    formatted += part + "\n\n"
                else:
                    # テキスト部分は装飾
                    formatted += part + "\n"
        else:
            # テキスト部分のみの場合
            formatted += message

        return formatted

    def format_system_message(self, message: str) -> str:
        """システムメッセージをフォーマット

        Args:
            message: メッセージ内容

        Returns:
            フォーマットされたメッセージ
        """
        return f"**システム**: {message}"

    def format_conversation_summary(self, summary: Dict[str, Any]) -> str:
        """会話サマリーをフォーマット

        Args:
            summary: 会話サマリー

        Returns:
            フォーマットされたサマリー
        """
        formatted = "## 会話サマリー\n\n"

        if "final_plan" in summary:
            formatted += "### 最終プラン\n\n"
            steps = summary.get("final_plan", {}).get("steps", [])
            for i, step in enumerate(steps):
                formatted += f"{i+1}. {step}\n"
            formatted += "\n"

        if "code_status" in summary:
            code_status = summary.get("code_status", "")
            formatted += f"### コード生成状況: {code_status}\n\n"

        if "execution_status" in summary:
            execution_status = summary.get("execution_status", "")
            formatted += f"### 実行状況: {execution_status}\n\n"

        return formatted
