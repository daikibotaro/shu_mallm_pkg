from openai import OpenAI
import os


class ChatManager:
    def __init__(self):
        OpenAI.api_key = os.getenv("OPENAI_API_KEY_SHU")
        self.client = OpenAI()
        self.messages = [
            {"role": "system", "content": "あなたはロボットの動作手順を提案することができます。与えられた情報を参考にして、ユーザーの要求を達成する動作手順を提案して。"}
        ]

    def add_message(self, role, content):
        self.messages.append({"role": role, "content": content})

    def get_response(self):
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=self.messages,
        )
        return response.choices[0].message.content
