from openai import AsyncOpenAI
import time
import subprocess
import json
from rich import print
from llm_processor import LLMProcessor
from task_usage_tracker import TaskTracker
from rag.src.knowledge_base_updater import KnowledgeBaseUpdater
from display_manager import DisplayManager
import os


class FileManager():
    def __init__(self):
        self.client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY_SHU"))
        self.display = DisplayManager()
        self.llm = LLMProcessor()
        self.task_tracker = TaskTracker()

        self.history_file = "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/history/message_history.json"
        self.output_file = "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/task_history.txt"

    @staticmethod
    def load_text(path):
        with open(path) as f:
            text = f.read()
        return text

    def remove_until_target(self, input_str, target):
        index = str(input_str).find(target)
        if index != -1:
            output_str = input_str[index + len(target):]
            return output_str
        else:
            return input_str

    def remove_after_target(self, input_str, target):
        index = str(input_str).find(target)
        if index != -1:
            output_str = input_str[:index]
            return output_str
        else:
            return input_str

    def get_code_block(self, input):
        output = self.remove_until_target(input, "```python")
        output = self.remove_after_target(output, "```")
        return output

    def save_task_history(self):
        with open(self.history_file, 'r') as file:
            message_history = json.load(file)
        length = len(message_history)
        result = message_history[int(length - 1)]["content"]
        result = self.get_code_block(result)

        # with open(self.output_file, "a", encoding="utf-8") as file:
        #     file.write("task:" + task + "\n\n")
        #     file.write("result:\n" + result + "\n\n")

    def save_task_usage_history(self):
        self.message_history = self.llm.load_message_history()
        request = self.message_history[1]["content"]
        self.task_tracker.track_task(request)

    async def get_condition(self):
        self.message_history = self.llm.load_message_history()
        context = "この要求に対して、会話で与えられた追加の情報や制約を箇条書きで書いてください。各項目の先頭には'- 'をつけてください。追加情報がない場合は'なし'と返してください。"
        self.message_history.append({"role": "user", "content": context})

        # LLMからの応答を取得
        response = await self.client.chat.completions.create(
            model="gpt-4o-mini",
            messages=self.message_history,
            temperature=0
        )

        condition = response.choices[0].message.content

        # 応答が"なし"でない場合のみ、会話履歴に追加
        if condition.strip().lower() != "なし":
            self.message_history.append({"role": "assistant", "content": condition})
            self.llm.save_message_history(self.message_history)

        # 条件を返す（KnowledgeBaseUpdaterで使用）
        return condition

    async def update_knowledge_base(self):
        updater = KnowledgeBaseUpdater()
        success = await updater.add_scenario()
        if success:
            print("\n[green]Knowledge baseが更新されました[/green]")

        else:
            print("\n[yellow]Knowledge baseの更新に失敗しました[/yellow]")

    async def check_create_file(self, input):
        """Create a file if message contains a python script.

        Args:
            input (str): message from LLM.
        """

        input = input.strip()
        output = self.get_code_block(input)
        output = output.strip()
        if len(output) == len(input):
            pass
        else:
            time.sleep(5)
            execution = self.display.ask_yes_or_no("実行してもよろしいですか？")
            if execution:
                self.save_task_history()
                self.save_task_usage_history()
                await self.get_condition()
                await self.update_knowledge_base()
                file_path = "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/scripts/result.py"
                with open(file_path, "w") as f:
                    f.write(output)
                print("\n[bold yellow]ファイルを作成しました！[/bold yellow]")
                print("\n[bold green_yellow]ファイルを実行します......[/bold green_yellow]")
                # ロボットなしでも動作確認ができるようにコメントアウト中
                # subprocess.run(["chmod", "+x", file_path])
                # subprocess.run(["rosrun", "shu_mallm_pkg", "result.py"])
                return True
            else:
                print("キャンセルされました")
