class ConversationManager:
    def __init__(self):
        self.conversation_history = []
        self.important_info = {}

    def save_conversation(self, user_input: str, system_response: str):
        self.conversation_history.append({"user": user_input, "system": system_response})

    def extract_important_info(self, user_input: str) -> dict:
        # 例として、ネジに関する情報を抽出
        if "ネジ" in user_input and "保管場所" in user_input:
            self.important_info["ネジ"] = "部品棚"
        return self.important_info

    def get_conversation_history(self) -> list:
        return self.conversation_history

    def load_history(self, file_path: str):
        # ファイルから会話履歴を読み込む
        pass

    def save_history(self, file_path: str):
        # 会話履歴をファイルに保存する
        pass


class KnowledgeBase:
    def __init__(self):
        self.data = {}

    def add_item(self, item_name: str, location: str):
        self.data[item_name] = location

    def update_item_location(self, item_name: str, location: str):
        self.data[item_name] = location

    def get_item_location(self, item_name: str) -> str:
        return self.data.get(item_name, "不明")

    def load_data(self, file_path: str):
        # ファイルからデータを読み込む
        pass

    def save_data(self, file_path: str):
        # データをファイルに保存する
        pass


class TaskPlanner:
    def __init__(self, conversation_manager: ConversationManager, knowledge_base: KnowledgeBase):
        self.conversation_manager = conversation_manager
        self.knowledge_base = knowledge_base

    def plan_task(self, user_input: str) -> str:
        # タスクを計画する
        context = self.check_context()
        if "ネジ" in user_input:
            location = context.get("ネジ", "場所が不明")
            return f"ネジは{location}にあります。"
        return "指示が不明確です。"

    def generate_code(self, task_description: str) -> str:
        # タスク説明に基づいてコードを生成する
        return f"# コード生成: {task_description}"

    def check_context(self) -> dict:
        # 会話履歴と知識ベースをチェックし、コンテキストを取得する
        return {**self.conversation_manager.important_info, **self.knowledge_base.data}
