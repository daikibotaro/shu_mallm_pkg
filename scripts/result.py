#!/usr/bin/env python3

import rospy
from function_class import RobotController, DisplayManager, TextToSpeech


def main():
    rospy.init_node('fetch_m8_bolts')
    controller = RobotController()
    display = DisplayManager()
    tts = TextToSpeech()

    # 初期位置を取得
    initial_position = controller.get_robot_position()

    # 部品棚へ移動
    if not controller.move_to_location("部品棚"):
        rospy.logerr("部品棚への移動に失敗しました。")
        return False

    # M8ボルトを2パレット分置いてもらうように呼びかけ
    tts.talk("M8ボルトを2パレット分、荷台に乗せていただけますか？")
    display.display_message("M8ボルトを2パレット分、荷台に乗せていただけますか？")

    # 検査工程に移動
    if not controller.move_to_location("検査工程"):
        rospy.logerr("検査工程への移動に失敗しました。")
        return False

    # メッセージを伝達
    tts.talk("M8ボルトを2パレット分お持ちしました")
    display.display_message("M8ボルトを2パレット分お持ちしました")

    # 初期位置に戻る
    if not controller.return_to_initial_position(initial_position):
        rospy.logerr("初期位置への移動に失敗しました。")
        return False


if __name__ == '__main__':
    main()
