import json
import re
import os


class KnowledgeBaseUpdater:
    def __init__(self):
        self.knowledge_base_path = "/home/hiratalab/catkin_ws/src/shu_task_planning/my_package/rag/data/knowledge_base.json"
        self.history_file = "/home/hiratalab/catkin_ws/src/shu_task_planning/history/message_history.json"

    def extract_task_info(self):
        # 会話履歴を読み込む
        with open(self.history_file, 'r') as file:
            message_history = json.load(file)
        length = len(message_history)

        # ユーザーの最初の要求を取得
        request = message_history[1]["content"]

        plan_sequence = message_history[int(length - 5)]["content"]
        pattern = r"(^\d\.\s.*|^\s*-\s.*$)"
        matches = re.findall(pattern, plan_sequence, re.MULTILINE)
        plan = "\n".join(matches).replace("-", "・")

        # 条件を取得（直前のメッセージ）
        condition = None
        for msg in reversed(message_history):
            if msg["role"] == "assistant" and msg["content"].startswith('- '):
                condition = msg["content"]
                break

        return request, plan, condition

    def create_new_scenario(self, request, plan, condition=None):
        """
        新しいシナリオエントリを作成
        """
        scenario = {
            "request": request,
            "plan": plan
        }

        if condition and condition.lower() != "なし":
            scenario["condition"] = condition

        return scenario

    def load_knowledge_base(self):
        """
        knowledge_base.jsonを読み込む
        """
        try:
            with open(self.knowledge_base_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            return {}

    def save_knowledge_base(self, data):
        """
        knowledge_base.jsonに保存
        """
        # ディレクトリが存在することを確認
        os.makedirs(os.path.dirname(self.knowledge_base_path), exist_ok=True)

        with open(self.knowledge_base_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)

    async def add_scenario(self):
        """
        新しいシナリオをknowledge baseに追加
        """
        # 要求と計画を抽出
        request, plan, condition = self.extract_task_info()

        if not plan:
            print("Warning: No plan was extracted from the assistant's response")
            return False

        # 現在のknowledge baseを読み込む
        knowledge_base = self.load_knowledge_base()

        # 新しいシナリオの番号を決定
        next_scenario_num = len(knowledge_base) + 1
        scenario_key = f"scenario{next_scenario_num}"

        # 新しいシナリオを作成
        new_scenario = self.create_new_scenario(request, plan, condition)

        # knowledge baseに追加
        knowledge_base[scenario_key] = new_scenario

        # 保存
        self.save_knowledge_base(knowledge_base)

        return True

    def update_existing_scenario(self, scenario_key, user_message, assistant_response):
        """
        既存のシナリオを更新
        """
        request, plan, condition = self.extract_task_info()

        if not plan:
            print("Warning: No plan was extracted from the assistant's response")
            return False

        knowledge_base = self.load_knowledge_base()

        if scenario_key not in knowledge_base:
            print(f"Warning: Scenario {scenario_key} not found in knowledge base")
            return False

        knowledge_base[scenario_key] = self.create_new_scenario(request, plan, condition)
        self.save_knowledge_base(knowledge_base)

        return True
