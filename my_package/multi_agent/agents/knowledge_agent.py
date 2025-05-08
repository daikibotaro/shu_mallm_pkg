from multi_agent.agents.base_agent import BaseAgent
from typing import Dict, Any, List, Optional
import asyncio


class KnowledgeAgent(BaseAgent):
    """RAGシステムとの連携を担当するエージェント"""

    def __init__(self, config: Dict[str, Any]):
        super().__init__(name=config.get("name", "KnowledgeAgent"), config=config)
        self.relevance_finder = None
        self.similarity_threshold = config.get("similarity_threshold", 0.7)

    async def initialize(self, relevance_finder):
        """RAGシステムの参照を設定"""
        self.relevance_finder = relevance_finder

    async def retrieve_knowledge(self, user_query: str) -> Dict[str, Any]:
        """関連知識を取得

        Args:
            user_query: ユーザークエリ

        Returns:
            関連知識（類似事例、参考情報など）
        """
        if not self.relevance_finder:
            return {"error": "RAGシステムが初期化されていません"}

        # RAGシステムから関連情報を取得
        relevant_info, max_similarity = await self.relevance_finder.get_relevant_info_with_similarity(
            user_query, self.similarity_threshold
        )

        # 関連情報がなければLLMに推論を依頼
        if not relevant_info:
            inference_prompt = f"""
            以下のユーザークエリに関連する知識や考慮事項を推論してください:
            
            クエリ: {user_query}
            
            考慮すべき点:
            - このタスクで注意すべき安全上の問題
            - 必要なリソースや前提条件
            - 想定されるチャレンジと解決策
            """

            inference = await self.generate_response(inference_prompt)

            return {
                "source": "inference",
                "relevant_examples": [],
                "inferred_knowledge": inference,
                "similarity_score": 0
            }

        return {
            "source": "knowledge_base",
            "relevant_examples": relevant_info,
            "similarity_score": max_similarity
        }

    async def analyze_knowledge_gaps(self, query: str, retrieved_info: Dict[str, Any]) -> Dict[str, Any]:
        """知識ギャップを分析

        Args:
            query: ユーザークエリ
            retrieved_info: 取得した知識

        Returns:
            ギャップ分析結果
        """
        gap_prompt = f"""
        以下のユーザークエリと取得された知識を分析し、不足している情報や知識ギャップを特定してください:
        
        クエリ: {query}
        
        取得された知識:
        {retrieved_info}
        
        以下の点について分析:
        1. 不足している重要情報
        2. 曖昧な点
        3. 確認が必要な仮定
        4. 追加すべき考慮事項
        """

        response = await self.generate_response(gap_prompt)

        return {
            "identified_gaps": response,
            "requires_clarification": "曖昧" in response or "確認" in response
        }
