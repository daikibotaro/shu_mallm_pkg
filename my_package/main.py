import chainlit as cl
from chat_handler import ChatHandler
from chainlit_integration.multi_agent_handler import MultiAgentChatHandler
from chainlit_runner import ChainlitRunner


class ChatHandlerManager:
    def __init__(self):
        self._handler = None
        self._use_multi_agent = True  # マルチエージェントモードを有効化

    async def get_handler(self):
        if self._handler is None:
            if self._use_multi_agent:
                self._handler = MultiAgentChatHandler()
            else:
                self._handler = ChatHandler()
            await self._handler.initialize()
        return self._handler


manager = ChatHandlerManager()


@cl.on_chat_start
async def start():
    chat_handler = await manager.get_handler()
    await chat_handler.start_chat()


@cl.set_chat_profiles
async def chat_profile():
    chat_handler = await manager.get_handler()
    return await chat_handler.chat_profile()


@cl.set_starters
async def set_starters():
    chat_handler = await manager.get_handler()
    return await chat_handler.set_starters()


@cl.on_message
async def main(message: cl.Message):
    chat_handler = await manager.get_handler()
    await chat_handler.handle_message(message)


if __name__ == "__main__":
    print("Run Chainlit with Multi-Agent System!")
    ChainlitRunner.run_chainlit()
