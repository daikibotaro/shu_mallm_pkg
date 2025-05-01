#!/usr/bin/env python3

import rospy
from function_class import RobotController, DisplayManager, TextToSpeech


def main():
    rospy.init_node('fetch_snack')
    controller = RobotController()
    display = DisplayManager()
    tts = TextToSpeech()

    # 初期位置を取得
    initial_position = controller.get_robot_position()

    # 休憩室へ移動
    if not controller.move_to_location("休憩室"):
        rospy.logerr("休憩室への移動に失敗しました。")
        return False

    # 軽食を載せてもらうようにメッセージを伝達
    tts.talk("軽食を荷台に載せていただけますか？")
    display.display_message("軽食を荷台に載せていただけますか？")

    # 初期位置に戻る
    if not controller.return_to_initial_position(initial_position):
        rospy.logerr("初期位置への移動に失敗しました。")
        return False


if __name__ == '__main__':
    main()