#!/usr/bin/env python3

import rospy
from function_class import RobotController, DisplayManager, TextToSpeech


def main():
    rospy.init_node('get_book')
    controller = RobotController()
    display = DisplayManager()
    tts = TextToSpeech()

    # 初期位置を取得
    initial_position = controller.get_robot_position()

    # リビングルームへ移動
    if not controller.move_to_location("リビングルーム"):
        rospy.logerr("リビングルームへの移動に失敗しました。")
        return False

    # メッセージを伝達
    tts.talk("本を置いてください")
    display.display_message("本を置いてください")

    # ベッドに移動
    if not controller.move_to_location("ベッド"):
        rospy.logerr("ベッドへの移動に失敗しました。")
        return False

    # メッセージを伝達
    tts.talk("本をお持ちしました")
    display.display_message("本をお持ちしました")

    # 初期位置に戻る
    if not controller.return_to_initial_position(initial_position):
        rospy.logerr("初期位置への移動に失敗しました。")
        return False


if __name__ == '__main__':
    if not main():
        rospy.logerr("タスクの実行に失敗しました。")
