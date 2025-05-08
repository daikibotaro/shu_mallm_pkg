import unittest
import asyncio
from my_package.multi_agent.communication.message_bus import MessageBus
from my_package.multi_agent.communication.direct_channel import DirectChannel


class TestCommunication(unittest.TestCase):
    """通信機能のテスト"""

    def setUp(self):
        self.message_bus = MessageBus()
        self.direct_channel = DirectChannel("Agent1", "Agent2")

    async def test_message_bus_publish_subscribe(self):
        """メッセージバスの発行/購読テスト"""
        received_messages = []

        # メッセージ受信コールバック
        async def on_message(message):
            received_messages.append(message)

        # トピックをサブスクライブ
        self.message_bus.subscribe("test_topic", on_message)

        # メッセージを発行
        await self.message_bus.publish("test_topic", "Sender", "Test Message")

        # 非同期処理を待機
        await asyncio.sleep(0.1)

        # 検証
        self.assertEqual(len(received_messages), 1)
        self.assertEqual(received_messages[0]["sender"], "Sender")
        self.assertEqual(received_messages[0]["content"], "Test Message")

    async def test_direct_channel_send(self):
        """直接通信チャネルのテスト"""
        received_messages = []

        # メッセージ受信コールバック
        async def on_message(message):
            received_messages.append(message)

        # コールバックを登録
        self.direct_channel.on_message(on_message)

        # メッセージを送信
        result = await self.direct_channel.send_message("Agent1", "Agent2", "Direct Message")

        # 非同期処理を待機
        await asyncio.sleep(0.1)

        # 検証
        self.assertTrue(result)
        self.assertEqual(len(received_messages), 1)
        self.assertEqual(received_messages[0]["sender"], "Agent1")
        self.assertEqual(received_messages[0]["receiver"], "Agent2")
        self.assertEqual(received_messages[0]["content"], "Direct Message")

    async def test_direct_channel_unauthorized_sender(self):
        """未許可の送信者の直接通信チャネルテスト"""
        # 未登録のエージェントからメッセージ送信
        result = await self.direct_channel.send_message("UnknownAgent", "Agent2", "Forbidden Message")

        # 検証
        self.assertFalse(result)
        self.assertEqual(len(self.direct_channel.get_history()), 0)

    async def test_message_history(self):
        """メッセージ履歴テスト"""
        # メッセージを複数発行
        await self.message_bus.publish("topic1", "Sender1", "Message 1")
        await self.message_bus.publish("topic2", "Sender2", "Message 2")
        await self.message_bus.publish("topic1", "Sender3", "Message 3")

        # 全履歴を取得
        all_history = self.message_bus.get_history()
        self.assertEqual(len(all_history), 3)

        # トピックでフィルタ
        topic1_history = self.message_bus.get_history("topic1")
        self.assertEqual(len(topic1_history), 2)
        self.assertEqual(topic1_history[0]["content"], "Message 1")
        self.assertEqual(topic1_history[1]["content"], "Message 3")

# asyncioを使用したテスト実行用ヘルパー


def run_async_test(coro):
    loop = asyncio.get_event_loop()
    return loop.run_until_complete(coro)


if __name__ == '__main__':
    unittest.main()
