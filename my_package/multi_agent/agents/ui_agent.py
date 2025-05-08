from multi_agent.agents.base_agent import BaseAgent
from typing import Dict, Any, List, Optional


class UIAgent(BaseAgent):
    """ユーザーとのインタラクションを管理するエージェント"""

    def __init__(self, config: Dict[str, Any]):
        super().__init__(name=config.get("name", "UIAgent"), config=config)

    async def analyze_user_intent(self, message: str) -> Dict[str, Any]:
        """ユーザーの意図を分析

        Args:
            message: ユーザーメッセージ

        Returns:
            分析結果（意図、エンティティなど）
        """
        analysis_prompt = f"""
        以下のユーザーメッセージを分析し、意図とエンティティを特定してください:
        
        ユーザーメッセージ: {message}
        
        分析結果:
        - タスクタイプ: [移動系/取得系/配布系/確認系/その他]
        - 主要なエンティティ: [対象物や場所のリスト]
        - 条件や制約: [タスクに関連する条件]
        """

        response = await self.generate_response(analysis_prompt)

        # 応答を解析して構造化データを作成
        # （簡略化のため、実際の解析コードは省略）

        return {
            "task_type": "移動系",  # 例
            "entities": ["部品棚", "ボルト"],  # 例
            "constraints": ["M8サイズ", "2パレット分"]  # 例
        }

    async def format_final_response(self, plan: str, code: str) -> str:
        """最終的な応答をフォーマット

        Args:
            plan: 作成されたプラン
            code: 生成されたコード

        Returns:
            フォーマットされた応答
        """
        response = f"""
        タスクプランが作成されました。以下の手順で実行します：
        
        {plan}
        
        生成されたコード:
        ```python
        {code}
        ```
        
        実行してもよろしいですか？
        """

        return response
