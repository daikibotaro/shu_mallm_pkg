from rich import print
import chainlit as cl
from file_manager import FileManager
from llm_processor import LLMProcessor
from task_usage_tracker import TaskTracker
from openai import AsyncOpenAI
import os
from rag.src.knowledge_base import KnowledgeBase
from rag.src.vectorizer import Vectorizer
from rag.src.relevance_finder import RelevanceFinder
from rag.src.embedding_store import EmbeddingStore


class ChatHandler():
    def __init__(self):
        self.client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY_SHU"))
        self.llm = LLMProcessor()
        self.file = FileManager()
        self.task_tracker = TaskTracker()
        self.llm_model = "gpt-4o"
        self.contain_image = False

        self.similarity_threshold = 0.70
        self.knowledge_base = KnowledgeBase('rag/data/knowledge_base.json')
        self.vectorizer = Vectorizer(self.client)
        self.embedding_store = EmbeddingStore()
        self.relevance_finder = RelevanceFinder(self.vectorizer, self.knowledge_base, self.embedding_store)
        self.message_history = None

    async def initialize(self):
        try:
            await self.relevance_finder.prepare()
            print("Knowledge base embeddings prepared successfully")
            # 他の初期化処理があれば追加

            if self.message_history is None:
                self.message_history = []

            return True

        except Exception as e:
            print(f"Error during initialization: {e}")
            raise

    async def chat_profile(self):
        return [
            cl.ChatProfile(
                name="Hirata-Lab",
                markdown_description="Our System",
                icon="http://localhost:5000/favicon.png",
            ),
            cl.ChatProfile(
                name="GPT-3.5",
                markdown_description="The underlying LLM model is **GPT-3.5**.",
                icon="https://picsum.photos/200",
            ),
            cl.ChatProfile(
                name="GPT-4o-mini",
                markdown_description="The underlying LLM model is **GPT-4o-mini**.",
                icon="https://picsum.photos/250",
            ),
            cl.ChatProfile(
                name="GPT-4o",
                markdown_description="The underlying LLM model is **GPT-4o**.",
                icon="https://picsum.photos/300",
            ),
            cl.ChatProfile(
                name="GPT-5-ultra",
                markdown_description="The underlying LLM model is **GPT-5-ultra**.",
                icon="https://picsum.photos/350",
            ),
        ]

    async def set_starters(self):
        static_starters = [
            cl.Starter(
                label="受入れの写真を入手",
                message="受入れに移動して写真を撮ってきてください。撮影後は休憩室に戻ってきて写真を表示してください。",
                icon="./public/favicon.png",
            ),
            cl.Starter(
                label="受入れからフレームを部品棚に運搬",
                message="受入れにあるフレームを、部品棚に運んでください。",
                icon="./public/favicon.png",
            ),
            cl.Starter(
                label="ネジを組立て工程に持ってくる",
                message="部品棚に移動して、ネジを載せてもらってください。載せてもらったら組立て工程に届けてください。",
                icon="./public/favicon.png",
            ),
            cl.Starter(
                label="看板回収",
                message="各工程を回って看板を荷台に載せてもらってください。回収が終わったら休憩室に運んでください。",
                icon="./public/favicon.png",
            ),
        ]

        # 使用頻度に基づく動的スターターの生成
        dynamic_suggestions = self.task_tracker.generate_task_suggestions()
        dynamic_starters = []

        for suggestion in dynamic_suggestions:
            task = suggestion["task"]
            is_recent = suggestion.get("is_recent", False)
            count = suggestion.get("recent_usage" if is_recent else "total_usage", 0)

            if not any(starter.message == task for starter in static_starters):
                label = f"よく使うタスク: {task}" if is_recent else f"人気のタスク: {task}"
                dynamic_starters.append(
                    cl.Starter(
                        label=label,
                        message=task,
                        icon="./public/favicon.png",
                    )
                )

        return static_starters + dynamic_starters

    async def start_chat(self):
        global contain_image
        contain_image = False

        try:
            chat_profile = cl.user_session.get("chat_profile")
            if chat_profile is None:
                chat_profile = "Hirata-Lab"

            system = self.file.load_text("./Instruction.txt")
            start_message = "You are a helpful assistant."

            # プロファイルに応じた設定
            profile_settings = {
                "Hirata-Lab": ("gpt-4o", system),
                "GPT-3.5": ("gpt-3.5-turbo", start_message),
                "GPT-4o-mini": ("gpt-4o-mini", start_message),
                "GPT-4o": ("gpt-4o", start_message),
                "GPT-5-ultra": ("gpt-4o-mini", start_message)
            }

            if chat_profile in profile_settings:
                self.llm_model, message = profile_settings[chat_profile]

                # メッセージ履歴の初期化
                initial_message = self.llm.create_message_dictionary("system", message)
                cl.user_session.set("message_history", [initial_message])
                self.llm.save_message_history([initial_message])

                if chat_profile != "Hirata-Lab":
                    await cl.Message(content=f"starting chat using the {chat_profile} chat profile").send()
            else:
                print(f"未知のチャットプロファイル: {chat_profile}")
                # デフォルト設定を使用
                self.llm_model = "gpt-4o"
                initial_message = self.llm.create_message_dictionary("system", system)
                cl.user_session.set("message_history", [initial_message])
                self.llm.save_message_history([initial_message])

        except Exception as e:
            print(f"チャット開始時にエラーが発生しました: {e}")
            # エラー発生時のフォールバック処理
            self.llm_model = "gpt-4o"
            initial_message = self.llm.create_message_dictionary("system", system)
            cl.user_session.set("message_history", [initial_message])
            self.llm.save_message_history([initial_message])

    async def handle_message(self, message):
        global contain_image
        self.message_history = self.llm.load_message_history()

        if message.content is not None:
            if message.elements:
                contain_image = True
                self.llm_model = "gpt-4o"
                image_path = message.elements[0].path
                base64_image = self.llm.encode_image(image_path)
                message_text = {"type": "text", "text": message.content}
                message_image = {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{base64_image}",
                        "detail": "low"
                    }
                }
                self.message_history.append({"role": "user", "content": [message_text, message_image]})
            else:
                self.message_history.append({"role": "user", "content": message.content})
        else:
            # contentがNoneの場合のエラーハンドリング
            print("エラー: メッセージのcontentがNoneです")

        if contain_image:
            print(f"[bold bright_red]会話履歴に画像が含まれています！！！[/bold bright_red]")
            self.llm_model = "gpt-4o"

        print(f"\n[purple]User: {message.content}[/purple]")

        # RAGで新しく追加
        await self.relevance_finder.print_debug_info(message.content)

        # 関連情報の取得と類似度のチェック
        relevant_info, max_similarity = await self.relevance_finder.get_relevant_info_with_similarity(
            message.content,
            similarity_threshold=self.similarity_threshold
        )
        if relevant_info:
            context = "対話履歴から類似した要求に対する動作生成が見つかりました．以下の情報を参考にしてください．参考にした場合は，手順確認の際に，過去の類似した手順を参考にしたことをユーザーに伝えてください．：\n" + "\n\n".join(relevant_info)
            self.message_history.append({"role": "system", "content": context})
            print(f"\n[green]最大類似度: {max_similarity:.4f} - 関連情報を追加しました[/green]:{context}")
        else:
            print(f"\n[yellow]最大類似度: {max_similarity:.4f} - 関連性が低いため、追加情報は提供しません[/yellow]")

        self.message_history = [msg for msg in self.message_history if msg['content'] is not None]

        msg = cl.Message(content="", author="ニンバスくん")
        await msg.send()

        stream = await self.client.chat.completions.create(
            messages=self.message_history,
            stream=True,
            stream_options={"include_usage": True},
            model=self.llm_model,
            temperature=0
        )

        async for part in stream:
            if part.choices and part.choices[0].delta:
                token = part.choices[0].delta.content or ""
                if token:
                    await msg.stream_token(token)

        response = msg.content
        print(f"\n[cyan3]Assistant:[/cyan3]{response}")

        completion_tokens, prompt_usage, total_usage = part.usage.completion_tokens, part.usage.prompt_tokens, part.usage.total_tokens
        print(f"\ncompletion_tokens: {completion_tokens}  prompt_tokens: {prompt_usage}  total usage: {total_usage}")

        self.message_history.append({"role": "assistant", "content": msg.content})
        await msg.update()

        self.llm.save_message_history(self.message_history)
        await self.file.check_create_file(response)
