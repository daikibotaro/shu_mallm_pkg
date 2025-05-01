from sklearn.metrics.pairwise import cosine_similarity
import numpy as np
from typing import List, Tuple


class RelevanceFinder:
    def __init__(self, vectorizer, knowledge_base, embedding_store):
        self.vectorizer = vectorizer
        self.knowledge_base = knowledge_base
        self.embedding_store = embedding_store
        self.embedding_matrix = None
        self.texts = None

    async def prepare(self):
        scenarios = self.knowledge_base.get_scenarios()
        self.texts = []
        embeddings = []

        for scenario_id, info in scenarios.items():
            text = info["request"]
            embedding = await self.embedding_store.get_or_create_embedding(
                text, scenario_id, self.vectorizer
            )
            self.texts.append(text)
            embeddings.append(embedding)

        self.embedding_matrix = np.array(embeddings)

        # requests = self.knowledge_base.get_requests()
        # self.embedding = np.array([await self.vectorizer.get_embedding(request) for request in requests])

    async def get_relevant_info_with_similarity(self, query: str, similarity_threshold: float) -> Tuple[List[str], float]:
        query_embedding = await self.vectorizer.get_embedding(query)
        similarities = cosine_similarity([query_embedding], self.embedding_matrix)[0]

        relevant_indices = [i for i, sim in enumerate(similarities) if sim >= similarity_threshold]

        scenarios = self.knowledge_base.get_scenarios()
        relevant_info = []
        for i, (scenario_id, info) in enumerate(scenarios.items()):
            if i in relevant_indices:
                task_info = f"## 要求: \n{info['request']}\n## 最終的な動作手順: \n{info['plan']}"
                if 'condition' in info:
                    task_info += f"\n## この要求において追加された特別な条件: \n{info['condition']}"
                relevant_info.append(task_info)

        max_similarity = np.max(similarities) if similarities.size > 0 else 0

        return relevant_info, max_similarity

    async def update_embeddings(self, scenarios):
        for scenario_id, info in scenarios.items():
            text = info["request"]
            embedding = await self.vectorizer.get_embedding(text)
            self.embedding_store.update_embedding(scenario_id, text, embedding)

        await self.prepare()

    async def print_debug_info(self, query):
        query_embedding = await self.vectorizer.get_embedding(query)
        similarities = cosine_similarity([query_embedding], self.embedding_matrix)[0]

        # print("Debug Information:")
        # print(f"Query: {query}")
        # print("Tokenized Query:", self.vectorizer.tokenizer(query))
        print("Similarities:", similarities)
        # print("Vocabulary:", self.vectorizer.get_feature_names())
        # print("Query Vector:", query_vector.toarray())
        # print("Knowledge Base Vectors:")
        # print(self.question_vectors.toarray())

        # print("Tokenized Knowledge Base:")
        # for request in self.knowledge_base.get_requests():
        #     print(f"  {request}: {self.vectorizer.tokenizer(request)}")
