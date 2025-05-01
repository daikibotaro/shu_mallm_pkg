from openai import OpenAI
import base64
import json
from rich import print


class LLMProcessor:
    def __init__(self):
        self.client = OpenAI()
        self.history_file = "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/history/message_history.json"

    def generate_response(self, text, model):
        response = self.client.chat.completions.create(
            model=model,
            messages=[
                {"role": "user", "content": text}
            ],
        )
        print(response.choices[0].message.content)
        return response.choices[0].message.content

    def create_message_dictionary(self, role, content):
        return {"role": role, "content": content}

    def save_message_history(self, message_history):
        file_path = self.history_file
        with open(file_path, 'w') as file:
            json.dump(message_history, file, ensure_ascii=False, indent=4)
        # print(f"Message history saved to {file_path}")

    def add_message_history(self, role, content):
        file_path = self.history_file

        with open(file_path, 'r') as file:
            message_history = json.load(file)
        # print(f"Message history loaded from {file_path}")

        new_dic = self.create_message_dictionary(role, content)

        message_history.append(new_dic)

        self.save_message_history(message_history)

    def load_message_history(self):
        file_path = self.history_file
        with open(file_path, 'r') as file:
            message_history = json.load(file)
        # print(f"Message history loaded from {file_path}")
        return message_history

    def generate_response_history(self, text):
        messages = self.load_message_history()
        messages.append(self.create_message_dictionary("user", text))
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=messages
        )
        print(response.choices[0].message.content)
        return response.choices[0].message.content

    def encode_image(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    def AI_vision(self, image_filename, text):
        add = "回答は必ず日本語で行ってください．出力に句読点以外の記号を含めずに回答してください．"
        base64_image = self.encode_image(image_filename)
        message_text = {"type": "text", "text": text + add}
        message_image = {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/png;base64,{base64_image}",
                "detail": "low"
            }
        }
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "user", "content": [message_text, message_image]}
            ],
            # temperature=0.0,
        )

        explanation = response.choices[0].message.content
        completion_tokens, prompt_usage, total_usage = response.usage.completion_tokens, response.usage.prompt_tokens, response.usage.total_tokens
        print(explanation)
        print(f"\ncompletion_tokens: {completion_tokens}  prompt_tokens: {prompt_usage}  total usage: {total_usage}")

        self.add_message_history("user", [message_text, message_image])
        self.add_message_history("assistant", explanation)

        return explanation
