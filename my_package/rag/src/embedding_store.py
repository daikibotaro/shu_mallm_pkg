import json
import os
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class StoredEmbedding:
    text: str
    embedding: List[float]
    scenario_id: str


class EmbeddingStore:
    def __init__(self, store_path: str = "rag/data/embeddings.json"):
        """Initialize the embedding store with a path to the storage file."""
        self.store_path = store_path
        self.embeddings: Dict[str, StoredEmbedding] = {}
        self._load_embeddings()

    def _load_embeddings(self) -> None:
        """Load existing embeddings from file if it exists."""
        if os.path.exists(self.store_path):
            try:
                with open(self.store_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    for scenario_id, item in data.items():
                        self.embeddings[scenario_id] = StoredEmbedding(
                            text=item['text'],
                            embedding=item['embedding'],
                            scenario_id=scenario_id
                        )
            except Exception as e:
                print(f"Error loading embeddings: {e}")
                self.embeddings = {}

    def _save_embeddings(self) -> None:
        """Save current embeddings to file."""
        os.makedirs(os.path.dirname(self.store_path), exist_ok=True)
        data = {
            scenario_id: {
                'text': emb.text,
                'embedding': emb.embedding,
                'scenario_id': emb.scenario_id
            }
            for scenario_id, emb in self.embeddings.items()
        }
        with open(self.store_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    async def get_or_create_embedding(self, text: str, scenario_id: str, vectorizer) -> List[float]:
        """Get existing embedding or create new one if not exists."""
        if scenario_id in self.embeddings and self.embeddings[scenario_id].text == text:
            return self.embeddings[scenario_id].embedding

        # Create new embedding
        embedding = await vectorizer.get_embedding(text)
        self.embeddings[scenario_id] = StoredEmbedding(
            text=text,
            embedding=embedding,
            scenario_id=scenario_id
        )
        self._save_embeddings()
        return embedding

    def get_all_embeddings(self) -> List[Tuple[str, List[float]]]:
        """Return all stored embeddings with their texts."""
        return [(emb.text, emb.embedding) for emb in self.embeddings.values()]

    def update_embedding(self, scenario_id: str, text: str, embedding: List[float]) -> None:
        """Update or add a new embedding."""
        self.embeddings[scenario_id] = StoredEmbedding(
            text=text,
            embedding=embedding,
            scenario_id=scenario_id
        )
        self._save_embeddings()

    def delete_embedding(self, scenario_id: str) -> None:
        """Delete an embedding if it exists."""
        if scenario_id in self.embeddings:
            del self.embeddings[scenario_id]
            self._save_embeddings()

    def clear_all_embeddings(self) -> None:
        """Clear all stored embeddings."""
        self.embeddings = {}
        self._save_embeddings()
