import yaml
import os
from typing import Dict, Any, Optional


class AgentConfig:
    """エージェント設定を管理するクラス"""

    def __init__(self, config_path: str):
        """
        Args:
            config_path: 設定ファイルのパス
        """
        self.config_path = config_path
        self.config = self._load_config()

    def _load_config(self) -> Dict[str, Any]:
        """設定ファイルを読み込む

        Returns:
            設定辞書
        """
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"設定ファイルが見つかりません: {self.config_path}")

        with open(self.config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        return config

    def get_agent_config(self, agent_name: str) -> Optional[Dict[str, Any]]:
        """特定のエージェントの設定を取得

        Args:
            agent_name: エージェント名

        Returns:
            エージェント設定
        """
        agents_config = self.config.get("agents", {})
        for agent_key, agent_config in agents_config.items():
            if agent_config.get("name") == agent_name:
                return agent_config

        return None

    def get_group_chat_config(self) -> Dict[str, Any]:
        """グループチャット設定を取得

        Returns:
            グループチャット設定
        """
        return self.config.get("group_chat", {})

    def get_llm_defaults(self) -> Dict[str, Any]:
        """LLMのデフォルト設定を取得

        Returns:
            LLM設定
        """
        return self.config.get("llm_defaults", {})

    def save_config(self, config: Dict[str, Any]) -> None:
        """設定を保存

        Args:
            config: 保存する設定
        """
        with open(self.config_path, 'w', encoding='utf-8') as f:
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

        self.config = config
