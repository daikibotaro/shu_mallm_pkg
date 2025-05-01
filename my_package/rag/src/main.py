from knowledge_base import KnowledgeBase
from vectorizer import Vectorizer
from relevance_finder import RelevanceFinder
from chat_manager import ChatManager


def main():
    knowledge_base = KnowledgeBase('../data/knowledge_base.json')
    vectorizer = Vectorizer()
    relevance_finder = RelevanceFinder(vectorizer, knowledge_base)
    relevance_finder.prepare()
    chat_manager = ChatManager()

    while True:
        user_input = input("あなた：")
        relevant_info = relevance_finder.get_relevant_info(user_input)

        context = "以下の情報を参考にしてください：\n" + "\n\n".join(relevant_info)

        chat_manager.add_message("user", user_input)
        chat_manager.add_message("system", context)

        assistant_message = chat_manager.get_response()
        print("AI: " + assistant_message)

        chat_manager.add_message("assistant", assistant_message)


if __name__ == "__main__":
    main()
