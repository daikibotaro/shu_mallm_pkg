from multi_agent.agents.base_agent import BaseAgent
from typing import Dict, Any, List, Optional
import json


class PlanningAgent(BaseAgent):
    """タスクプランニングを担当するエージェント"""

    def __init__(self, config: Dict[str, Any]):
        super().__init__(name=config.get("name", "PlanningAgent"), config=config)

    async def create_task_plan(self, intent_data: Dict[str, Any], knowledge: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """タスクプランを作成

        Args:
            intent_data: UIエージェントから提供された意図データ
            knowledge: ナレッジエージェントから提供された知識

        Returns:
            タスクプラン（ステップ、条件など）
        """
        # 知識ベースからのデータがあれば活用
        knowledge_prompt = ""
        if knowledge and "relevant_examples" in knowledge:
            knowledge_prompt = "以下の類似タスク例を参考にしてください:\n"
            for i, example in enumerate(knowledge["relevant_examples"]):
                knowledge_prompt += f"例{i+1}: {example}\n"

        plan_prompt = f"""
        以下の情報に基づいてロボットタスクの詳細な計画を作成してください：
        
        タスクタイプ: {intent_data.get('task_type')}
        対象エンティティ: {', '.join(intent_data.get('entities', []))}
        制約条件: {', '.join(intent_data.get('constraints', []))}
        
        {knowledge_prompt}
        
        タスク計画:
        1. ステップごとの詳細な動作
        2. 必要なチェックポイントとフィードバック
        3. 安全上の考慮事項
        4. 失敗時の対応策
        
        JSON形式で提供してください。
        """

        response = await self.generate_response(plan_prompt)

        # 応答をJSON形式で解析（エラーハンドリング付き）
        try:
            plan_data = json.loads(response)
        except json.JSONDecodeError:
            # JSON変換に失敗した場合、テキスト応答から構造を抽出
            plan_data = self._extract_plan_from_text(response)

        return plan_data

    def _extract_plan_from_text(self, text: str) -> Dict[str, Any]:
        """テキスト応答からプラン構造を抽出（フォールバック）"""
        # 簡略化のためのダミー実装
        steps = []
        for line in text.split('\n'):
            if line.strip().startswith(('1.', '2.', '3.', '4.', '5.')):
                steps.append(line.strip())

        return {
            "steps": steps,
            "checkpoints": [],
            "safety_considerations": [],
            "fallback_plans": []
        }

    async def validate_plan(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """プランを検証

        Args:
            plan: 作成されたプラン

        Returns:
            検証結果と（必要に応じて）修正されたプラン
        """
        validation_prompt = f"""
        以下のロボットタスク計画を検証し、潜在的な問題や改善点を特定してください:
        
        計画:
        {json.dumps(plan, ensure_ascii=False, indent=2)}
        
        以下の観点から検証:
        - 完全性: すべての必要なステップが含まれているか
        - 安全性: 安全上の懸念事項はすべて対処されているか
        - 効率性: より効率的な実行方法はあるか
        - ロバスト性: 失敗時の対応は十分か
        
        検証結果とともに、必要に応じて修正された計画を提供してください。
        """

        response = await self.generate_response(validation_prompt)

        # 応答の処理（簡略化）
        return {
            "is_valid": True,  # 実際には応答から判断
            "improved_plan": plan,  # 実際には応答から抽出
            "validation_notes": response
        }
