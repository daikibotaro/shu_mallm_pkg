from datetime import datetime
import json
from collections import Counter


class TaskTracker:
    def __init__(self, history_file="task_usage_history.json"):
        self.history_file = history_file
        self.usage_data = self._load_history()
        self.current_month = datetime.now().strftime("%Y-%m")

    def _load_history(self):
        try:
            with open(self.history_file, 'r', encoding='utf-8')as f:
                return json.load(f)
        except FileNotFoundError:
            return {"monthly_usage": {}, "task_patterns": {}}

    def _save_history(self):
        with open(self.history_file, 'w', encoding='utf-8') as f:
            json.dump(self.usage_data, f, ensure_ascii=False, indent=4)

    def track_task(self, task_content):
        # 月間使用回数の更新
        if self.current_month not in self.usage_data["monthly_usage"]:
            self.usage_data["monthly_usage"][self.current_month] = {}

        if task_content not in self.usage_data["monthly_usage"][self.current_month]:
            self.usage_data["monthly_usage"][self.current_month][task_content] = 0

        self.usage_data["monthly_usage"][self.current_month][task_content] += 1

        # タスクパターンの保存・更新
        if task_content not in self.usage_data["task_patterns"]:
            self.usage_data["task_patterns"][task_content] = {
                "total_usage": 0,
                "first_used": datetime.now().isoformat(),
                "last_used": datetime.now().isoformat()
            }

        self.usage_data["task_patterns"][task_content]["total_usage"] += 1
        self.usage_data["task_patterns"][task_content]["last_used"] = datetime.now().isoformat()

        self._save_history()

    def get_popular_tasks(self, limit=4):
        if not self.usage_data["task_patterns"]:
            return []

        sorted_tasks = sorted(
            self.usage_data["task_patterns"].items(),
            key=lambda x: x[1]["total_usage"],
            reverse=True
        )

        return [(task, data["total_usage"])for task, data in sorted_tasks[:limit]]

    def get_recent_popular_tasks(self, months=1, limit=4):
        """直近の期間でよく使用されるタスクを取得"""
        recent_usage = Counter()

        for month, tasks in self.usage_data["monthly_usage"].items():
            if month >= self.current_month:  # 簡易的な比較
                for task, count in tasks.items():
                    recent_usage[task] += count

        return recent_usage.most_common(limit)

    def generate_task_suggestions(self):
        """タスク提案を生成"""
        popular_all_time = self.get_popular_tasks()
        popular_recent = self.get_recent_popular_tasks()

        # 全期間と最近の人気タスクを組み合わせて提案を生成
        suggestions = []
        seen_tasks = set()

        # 最近の人気タスクを優先
        for task, count in popular_recent:
            if task not in seen_tasks:
                suggestions.append({
                    "task": task,
                    "recent_usage": count,
                    "is_recent": True
                })
                seen_tasks.add(task)

        # 全期間の人気タスクを追加
        for task, count in popular_all_time:
            if task not in seen_tasks:
                suggestions.append({
                    "task": task,
                    "total_usage": count,
                    "is_recent": False
                })
                seen_tasks.add(task)

        return suggestions[:5]  # 最大5つの提案を返す
