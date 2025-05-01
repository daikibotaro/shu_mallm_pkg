import json


class KnowledgeBase:
    def __init__(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as f:
            self.data = json.load(f)

    def get_scenarios(self):
        return self.data

    def get_requests(self):
        return [info["request"] for info in self.data.values()]

    def get_conditions(self):
        return [info.get("condition", "") for info in self.data.values()]
